// dds_player — read an MCAP bag recorded by dds_recorder and re-publish every
// message back onto DDS, preserving inter-message timing. Any node subscribing
// to the original topics (e.g. terrain mapping) sees the replayed traffic as if
// it were live.
//
//   dds_player <bag.mcap> [--domain N] [--rate R] [--loop] [--warmup SECS]
//              [--no-shm] [--shm-mb 1024] [--topics /a,/b]
//
// Mirrors the recorder: bounded zero-copy types (PointCloud2_*MB_, images) are
// published over an SHM transport; everything else over UDP. Publisher QoS
// (reliability/durability) is restored from the channel metadata so consumers
// match exactly.
//
#include <dds_mcap/cdr_codec.hpp>

#include <message_manager/fastdds_transport.hpp>

#include <mcap/reader.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <typeindex>
#include <unordered_map>

using message_manager::FastDDSTransport;

namespace {

std::atomic<bool> g_running{true};
void handleSignal(int) { g_running = false; }

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

std::string metaValue(const mcap::KeyValueMap& m, const std::string& key,
                      const std::string& fallback) {
  auto it = m.find(key);
  return it == m.end() ? fallback : it->second;
}

// Split a comma-separated topic list ("/a,/b,/c") into a set. Empty tokens are
// dropped, so "" yields an empty set (meaning "no filter — replay everything").
std::set<std::string> splitCsv(const std::string& s) {
  std::set<std::string> out;
  for (std::size_t start = 0; start <= s.size();) {
    const auto comma = s.find(',', start);
    const auto end = comma == std::string::npos ? s.size() : comma;
    if (end > start) out.insert(s.substr(start, end - start));
    if (comma == std::string::npos) break;
    start = comma + 1;
  }
  return out;
}

struct Publisher {
  FastDDSTransport* transport = nullptr;
  message_manager::PublisherHandle handle = 0;
  std::type_index type = typeid(void);
};

// SHM keep_last depth. Kept small: each bounded zero-copy writer reserves a
// data-sharing pool of depth × max_sample_size in /dev/shm, so depth 32MB ×
// many cloud topics fills tmpfs fast. 2 is plenty for sequential playback.
constexpr std::size_t kShmDepth = 2;
constexpr std::size_t kUdpDepth = 16;

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2 || argv[1][0] == '-') {
    std::cerr << "usage: dds_player <bag.mcap> [--domain N] [--rate R] "
                 "[--loop] [--warmup SECS] [--no-shm] [--shm-mb 1024] "
                 "[--topics /a,/b]\n";
    return 2;
  }
  const std::string bag = argv[1];
  const int domain_id = std::stoi(parseArg(argc, argv, "--domain", "0"));
  const double rate = std::stod(parseArg(argc, argv, "--rate", "1.0"));
  const double warmup = std::stod(parseArg(argc, argv, "--warmup", "1.0"));
  const bool loop = hasFlag(argc, argv, "--loop");
  const bool no_shm = hasFlag(argc, argv, "--no-shm");
  const std::uint32_t shm_mb = static_cast<std::uint32_t>(
      std::stoul(parseArg(argc, argv, "--shm-mb", "1024")));
  // Optional include-list: only channels whose topic is in this set get a
  // publisher (and get replayed). Empty => no filter, replay every topic.
  const std::set<std::string> only_topics =
      splitCsv(parseArg(argc, argv, "--topics", ""));

  // ---- open bag ----
  mcap::McapReader reader;
  if (const auto st = reader.open(bag); !st.ok()) {
    std::cerr << "failed to open '" << bag << "': " << st.message << "\n";
    return 1;
  }
  if (const auto st = reader.readSummary(mcap::ReadSummaryMethod::AllowFallbackScan);
      !st.ok()) {
    std::cerr << "failed to read summary: " << st.message << "\n";
    return 1;
  }

  // ---- DDS transports: UDP + SHM ----
  FastDDSTransport::Config udp_cfg;
  udp_cfg.domain_id = domain_id;
  udp_cfg.participant_name = "dds_player";
  FastDDSTransport udp(udp_cfg);
  if (!udp.initialize()) {
    std::cerr << "failed to initialize UDP transport\n";
    return 1;
  }
  std::unique_ptr<FastDDSTransport> shm;
  if (!no_shm) {
    FastDDSTransport::Config shm_cfg;
    shm_cfg.domain_id = domain_id;
    shm_cfg.participant_name = "dds_player_shm";
    shm_cfg.enable_shm = true;
    shm_cfg.shm_segment_size = shm_mb * 1024u * 1024u;
    shm = std::make_unique<FastDDSTransport>(shm_cfg);
    if (!shm->initialize()) {
      std::cerr << "failed to initialize SHM transport (continuing UDP-only)\n";
      shm.reset();
    }
  }

  // ---- create a publisher per channel ----
  std::unordered_map<mcap::ChannelId, Publisher> pubs;
  for (const auto& [cid, ch] : reader.channels()) {
    // --topics include-list: drop channels not requested (no publisher created,
    // so the replay loop skips their messages too — see pubs.find below).
    if (!only_topics.empty() && !only_topics.count(ch->topic)) continue;

    std::optional<std::type_index> type;
    const std::string dds_type = metaValue(ch->metadata, "dds_type", "");
    if (!dds_type.empty())
      type = message_manager::getTypeIndexFromDDSName(dds_type);
    if (!type) {
      const auto* schema =
          ch->schemaId ? reader.schema(ch->schemaId).get() : nullptr;
      if (schema)
        if (auto r = dds_mcap::resolveTypeByName(schema->name)) type = r->type;
    }
    if (!type) {
      std::cerr << "  [skip] " << ch->topic << ": cannot resolve message type\n";
      continue;
    }

    const bool is_shm = dds_mcap::isZeroCopyShmType(*type);
    FastDDSTransport* transport = is_shm ? shm.get() : &udp;
    if (is_shm && !shm) {
      std::cerr << "  [skip] " << ch->topic
                << ": needs SHM transport (disabled with --no-shm)\n";
      continue;
    }

    FastDDSTransport::PubConfig pub;
    pub.qos_depth = is_shm ? kShmDepth : kUdpDepth;
    pub.reliable = metaValue(ch->metadata, "reliability", "reliable") == "reliable";
    pub.transient_local =
        metaValue(ch->metadata, "durability", "volatile") == "transient_local";

    const auto handle = transport->registerPublisher(
        ch->topic, *type,
        message_manager::PublisherOptions{}.transport<FastDDSTransport>(pub));
    if (!handle) {
      std::cerr << "  [skip] " << ch->topic << ": advertise failed\n";
      continue;
    }
    pubs.emplace(cid, Publisher{transport, handle, *type});
    std::cout << "  publishing " << ch->topic << (is_shm ? "  (shm)" : "") << "\n";
  }

  if (pubs.empty()) {
    std::cerr << "no publishable channels in bag\n";
    return 2;
  }

  // Clean shutdown on Ctrl-C so DDS releases its /dev/shm data-sharing
  // segments — otherwise they leak and eventually fill tmpfs.
  std::signal(SIGINT, handleSignal);
  std::signal(SIGTERM, handleSignal);

  std::cout << "warmup " << warmup << "s (waiting for subscribers)…\n";
  std::this_thread::sleep_for(std::chrono::duration<double>(warmup));

  auto onProblem = [](const mcap::Status& s) {
    std::cerr << "  mcap: " << s.message << "\n";
  };

  std::uint64_t total = 0;
  do {
    bool have_first = false;
    std::uint64_t first_log = 0;
    auto wall_start = std::chrono::steady_clock::now();

    // Replay in ascending log-time order. The bag's physical order may differ
    // from log-time order (e.g. after re-stamping log_time = header.stamp on a
    // recording that was written with transport backlog); FileOrder would then
    // replay out of order. The reader uses the message index to merge to time order.
    mcap::ReadMessageOptions ropts;
    ropts.readOrder = mcap::ReadMessageOptions::ReadOrder::LogTimeOrder;
    auto view = reader.readMessages(onProblem, ropts);
    for (const auto& mv : view) {
      if (!g_running) break;
      auto it = pubs.find(mv.channel->id);
      if (it == pubs.end()) continue;

      if (!have_first) {
        first_log = mv.message.logTime;
        wall_start = std::chrono::steady_clock::now();
        have_first = true;
      }
      const double offset_ns =
          static_cast<double>(mv.message.logTime - first_log) / rate;
      std::this_thread::sleep_until(
          wall_start +
          std::chrono::nanoseconds(static_cast<std::int64_t>(offset_ns)));

      dds_mcap::withDeserialized(
          reinterpret_cast<const std::uint8_t*>(mv.message.data),
          static_cast<std::uint32_t>(mv.message.dataSize), it->second.type,
          [&](const void* sample, const std::type_index& t) {
            it->second.transport->publishImpl(it->second.handle, sample, t);
          });
      ++total;
    }
    std::cout << "  replayed " << total << " messages\n";
  } while (loop && g_running);

  reader.close();
  udp.shutdown();
  if (shm) shm->shutdown();
  std::cout << "done.\n";
  return 0;
}
