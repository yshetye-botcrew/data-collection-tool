// dds_recorder — subscribe to DDS topics and write every message to a single
// MCAP file as raw CDR (a replayable, lossless bag).
//
//   dds_recorder [--config record.json] [--output out.mcap] [--domain N]
//                [--all] [--no-shm] [--shm-mb 1024]
//
// Topics come from the config "topics" list and/or automatic DDS discovery
// (default when no list is given, or --all / "discover": true).
//
// Two DDS transports are used, mirroring the foxglove-aggregator:
//   * UDP            — normal topics (odometry, tf, imu, …)
//   * SHM data-sharing — bounded zero-copy types (PointCloud2_*MB_, images),
//                        which are NOT delivered over UDP.
// The subscriber QoS (reliability/durability) is matched to each discovered
// writer — a reliable reader will not match a best-effort writer, which is why
// best-effort lidar clouds record 0 messages otherwise.
//
#include <dds_mcap/cdr_codec.hpp>
#include <dds_mcap/topic_discovery.hpp>

#include <message_manager/fastdds_transport.hpp>

#include <mcap/writer.hpp>
#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

using message_manager::FastDDSTransport;

namespace {

std::atomic<bool> g_running{true};
void handleSignal(int) { g_running = false; }

std::uint64_t nowNanos() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

std::string parseArg(int argc, char** argv, const std::string& flag,
                     const std::string& fallback) {
  for (int i = 1; i + 1 < argc; ++i)
    if (flag == argv[i]) return argv[i + 1];
  return fallback;
}
bool hasFlag(int argc, char** argv, const std::string& flag) {
  for (int i = 1; i < argc; ++i)
    if (flag == argv[i]) return true;
  return false;
}

// DDS discovery reports the mangled wire name; ROS 2 maps a topic "/foo" to the
// DDS topic "rt/foo". The message_manager transport re-applies that prefix
// itself, so we must hand it the demangled ROS name. Returns empty for the
// service-style prefixes (rq/ rr/ rs/), which aren't record-able topics.
std::string ddsTopicToRos(const std::string& dds) {
  if (dds.rfind("rt/", 0) == 0) return dds.substr(2);  // "rt/test/x" -> "/test/x"
  if (dds.rfind("rq/", 0) == 0 || dds.rfind("rr/", 0) == 0 ||
      dds.rfind("rs/", 0) == 0)
    return {};
  return dds;  // not ROS-mangled; pass through unchanged
}

struct Channel {
  mcap::ChannelId id = 0;
  std::uint64_t count = 0;
  std::string topic;
};

// SHM keep_last depth. Each bounded zero-copy reader/writer reserves a
// data-sharing pool of depth × max_sample_size in /dev/shm (a 32MB cloud type
// → ~depth×32MB), so keep this small to avoid filling tmpfs across runs.
constexpr std::size_t kShmDepth = 2;      // SHM keep_last depth (limits /dev/shm use)
constexpr std::size_t kUdpDepth = 16;     // generous buffering for normal topics

class BagRecorder {
 public:
  BagRecorder(FastDDSTransport& udp, FastDDSTransport* shm, mcap::McapWriter& writer)
      : udp_(udp), shm_(shm), writer_(writer) {}

  void setExcludes(std::set<std::string> ex) { excludes_ = std::move(ex); }

  // Subscribe to `topic` (type given by name) with QoS matching the writer.
  void addTopic(const std::string& topic, const std::string& type_name,
                bool reliable, bool transient_local, bool quiet_unknown) {
    const auto resolved = dds_mcap::resolveTypeByName(type_name);
    if (!resolved) {
      if (!quiet_unknown)
        std::cerr << "  [skip] " << topic << ": unknown type '" << type_name
                  << "' (not in SupportedTypes)\n";
      return;
    }

    const bool shm = dds_mcap::isZeroCopyShmType(resolved->type);
    FastDDSTransport* transport = shm ? shm_ : &udp_;
    if (shm && !shm_) {
      if (!quiet_unknown)
        std::cerr << "  [skip] " << topic
                  << ": needs SHM transport (disabled with --no-shm)\n";
      return;
    }

    Channel* ch = nullptr;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (excludes_.count(topic)) return;
      if (!recorded_.insert(topic).second) return;

      mcap::SchemaId schema_id;
      if (auto it = schema_ids_.find(resolved->dds_name); it != schema_ids_.end()) {
        schema_id = it->second;
      } else {
        const std::string schema_name =
            resolved->schema_name.empty() ? resolved->dds_name : resolved->schema_name;
        mcap::Schema schema(schema_name, "ros2msg", std::string_view{});
        writer_.addSchema(schema);
        schema_id = schema.id;
        schema_ids_[resolved->dds_name] = schema_id;
      }
      // Self-describing metadata: type + QoS so the player can republish faithfully.
      mcap::KeyValueMap metadata{
          {"dds_type", resolved->dds_name},
          {"reliability", reliable ? "reliable" : "best_effort"},
          {"durability", transient_local ? "transient_local" : "volatile"},
      };
      mcap::Channel channel(topic, "cdr", schema_id, metadata);
      writer_.addChannel(channel);
      channels_.push_back(Channel{channel.id, 0, topic});
      ch = &channels_.back();
    }

    FastDDSTransport::SubConfig sub;
    sub.qos_depth = shm ? kShmDepth : kUdpDepth;
    sub.reliable = reliable;
    sub.transient_local = transient_local;

    transport->registerSubscriber(
        topic, resolved->type,
        [this, ch](const void* msg, const std::type_index& t) {
          auto bytes = dds_mcap::serializeCDR(msg, t);
          if (!bytes) return;
          const std::uint64_t ts = nowNanos();
          mcap::Message m;
          m.channelId = ch->id;
          m.logTime = ts;
          m.publishTime = ts;
          m.data = reinterpret_cast<const std::byte*>(bytes->data());
          m.dataSize = bytes->size();
          std::lock_guard<std::mutex> lk(mtx_);
          m.sequence = static_cast<std::uint32_t>(ch->count);
          if (writer_.write(m).ok()) ch->count++;
        },
        message_manager::SubscriberOptions{}.transport<FastDDSTransport>(sub));

    std::cout << "  + recording " << topic << "  (" << resolved->schema_name
              << (shm ? ", shm" : "")
              << (reliable ? ", reliable" : ", best_effort") << ")\n";
  }

  std::size_t channelCount() {
    std::lock_guard<std::mutex> lk(mtx_);
    return channels_.size();
  }
  std::uint64_t totalMessages() {
    std::lock_guard<std::mutex> lk(mtx_);
    std::uint64_t t = 0;
    for (const auto& c : channels_) t += c.count;
    return t;
  }
  void printSummary() {
    std::lock_guard<std::mutex> lk(mtx_);
    std::uint64_t total = 0;
    for (const auto& c : channels_) {
      std::cout << "  " << c.topic << ": " << c.count << " msgs\n";
      total += c.count;
    }
    std::cout << "total: " << total << " messages across " << channels_.size()
              << " topics\n";
  }
  void closeWriter() {
    std::lock_guard<std::mutex> lk(mtx_);
    writer_.close();
  }

 private:
  FastDDSTransport& udp_;
  FastDDSTransport* shm_;
  mcap::McapWriter& writer_;
  std::mutex mtx_;
  std::deque<Channel> channels_;
  std::set<std::string> recorded_;
  std::set<std::string> excludes_;
  std::unordered_map<std::string, mcap::SchemaId> schema_ids_;
};

}  // namespace

int main(int argc, char** argv) {
  const std::string config_path = parseArg(argc, argv, "--config", "");
  nlohmann::json cfg = nlohmann::json::object();
  if (!config_path.empty()) {
    try {
      std::ifstream f(config_path);
      if (!f) throw std::runtime_error("cannot open config: " + config_path);
      f >> cfg;
    } catch (const std::exception& e) {
      std::cerr << "config error: " << e.what() << "\n";
      return 2;
    }
  }

  const int domain_id = std::stoi(
      parseArg(argc, argv, "--domain", std::to_string(cfg.value("domain_id", 0))));
  const std::string output =
      parseArg(argc, argv, "--output", cfg.value("output", "recording.mcap"));
  const std::string compression = cfg.value("compression", "zstd");
  const bool no_shm = hasFlag(argc, argv, "--no-shm");
  const std::uint32_t shm_mb = static_cast<std::uint32_t>(
      std::stoul(parseArg(argc, argv, "--shm-mb",
                          std::to_string(cfg.value("shm_segment_mb", 1024)))));

  const bool has_topics = cfg.contains("topics") && cfg["topics"].is_array() &&
                          !cfg["topics"].empty();
  const bool discover = hasFlag(argc, argv, "--all") ||
                        cfg.value("discover", false) || !has_topics;

  // ---- DDS transports: UDP (always) + SHM (for zero-copy bounded types) ----
  FastDDSTransport::Config udp_cfg;
  udp_cfg.domain_id = domain_id;
  udp_cfg.participant_name = "dds_recorder";
  FastDDSTransport udp(udp_cfg);
  if (!udp.initialize()) {
    std::cerr << "failed to initialize UDP transport\n";
    return 1;
  }

  std::unique_ptr<FastDDSTransport> shm;
  if (!no_shm) {
    FastDDSTransport::Config shm_cfg;
    shm_cfg.domain_id = domain_id;
    shm_cfg.participant_name = "dds_recorder_shm";
    shm_cfg.enable_shm = true;
    shm_cfg.shm_segment_size = shm_mb * 1024u * 1024u;
    shm = std::make_unique<FastDDSTransport>(shm_cfg);
    if (!shm->initialize()) {
      std::cerr << "failed to initialize SHM transport (continuing UDP-only)\n";
      shm.reset();
    } else {
      std::cout << "SHM transport enabled (" << shm_mb << " MB segment)\n";
    }
  }

  // ---- MCAP writer ----
  mcap::McapWriterOptions opts("ros2");
  if (compression == "none")
    opts.compression = mcap::Compression::None;
  else if (compression == "lz4")
    opts.compression = mcap::Compression::Lz4;
  else
    opts.compression = mcap::Compression::Zstd;
  mcap::McapWriter writer;
  if (const auto st = writer.open(output, opts); !st.ok()) {
    std::cerr << "failed to open mcap '" << output << "': " << st.message << "\n";
    return 1;
  }

  BagRecorder recorder(udp, shm.get(), writer);
  if (cfg.contains("exclude") && cfg["exclude"].is_array()) {
    std::set<std::string> ex;
    for (const auto& e : cfg["exclude"]) ex.insert(e.get<std::string>());
    recorder.setExcludes(std::move(ex));
  }

  // ---- explicit topics from config ----
  if (has_topics) {
    for (const auto& entry : cfg["topics"]) {
      const std::string topic = entry.value("topic", "");
      const std::string type = entry.value("type", "");
      if (topic.empty() || type.empty()) continue;
      bool reliable = true, transient_local = false;
      if (entry.contains("qos")) {
        reliable = entry["qos"].value("reliable", true);
        transient_local = entry["qos"].value("transient_local", false);
      }
      recorder.addTopic(topic, type, reliable, transient_local,
                        /*quiet_unknown=*/false);
    }
  }

  // ---- automatic discovery ----
  dds_mcap::TopicDiscovery discovery(
      [&recorder](const dds_mcap::DiscoveredWriter& w) {
        const std::string topic = ddsTopicToRos(w.topic);
        if (topic.empty()) return;
        recorder.addTopic(topic, w.type_name, w.reliable, w.transient_local,
                          /*quiet_unknown=*/true);
      });
  if (discover) {
    if (!discovery.start(domain_id)) {
      std::cerr << "failed to start topic discovery\n";
      return 1;
    }
    std::cout << "auto-discovery ON — recording every known type on domain "
              << domain_id << "\n";
  }

  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);
  std::cout << "writing " << output << "  (domain " << domain_id
            << ") — Ctrl-C to stop\n";

  while (g_running) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "\r  " << recorder.totalMessages() << " messages across "
              << recorder.channelCount() << " topics   " << std::flush;
  }

  std::cout << "\nstopping…\n";
  discovery.stop();
  recorder.closeWriter();
  udp.shutdown();
  if (shm) shm->shutdown();
  recorder.printSummary();
  std::cout << "-> " << output << "\n";
  return 0;
}
