// Loopback test helper: subscribes to the replayed topics and checks the
// values arrive intact and in order. Exits 0 on success, 1 on mismatch.
//
//   --expect N   messages expected per topic
//   --seconds S  listen window before judging
//   --shm        also expect the SHM best-effort bounded cloud (/test/shmcloud)
//
#include <message_manager/bounded_pointclouds.hpp>
#include <message_manager/fastdds_transport.hpp>
#include <message_manager/sensor_msgs.hpp>
#include <message_manager/std_msgs.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using message_manager::FastDDSTransport;

static std::string arg(int c, char** v, const std::string& f, const std::string& d) {
  for (int i = 1; i + 1 < c; ++i)
    if (f == v[i]) return v[i + 1];
  return d;
}
static bool flag(int c, char** v, const std::string& f) {
  for (int i = 1; i < c; ++i)
    if (f == v[i]) return true;
  return false;
}

int main(int argc, char** argv) {
  const int domain = std::stoi(arg(argc, argv, "--domain", "0"));
  const int expect = std::stoi(arg(argc, argv, "--expect", "20"));
  const double seconds = std::stod(arg(argc, argv, "--seconds", "12"));
  const bool shm = flag(argc, argv, "--shm");

  FastDDSTransport::Config cfg;
  cfg.domain_id = domain;
  cfg.participant_name = "test_subscriber";
  FastDDSTransport tp(cfg);
  if (!tp.initialize()) { std::cerr << "init failed\n"; return 1; }

  std::mutex mtx;
  std::vector<double> values;
  std::vector<std::uint32_t> clouds;
  std::vector<std::uint32_t> shmclouds;
  bool cloud_ok = true, shm_ok = true;

  tp.subscribe<std_msgs::msg::dds_::Float64_>(
      "/test/value", [&](const std_msgs::msg::dds_::Float64_& m) {
        std::lock_guard<std::mutex> lk(mtx);
        values.push_back(m.data());
      });

  tp.subscribe<sensor_msgs::msg::dds_::PointCloud2_>(
      "/test/cloud", [&](const sensor_msgs::msg::dds_::PointCloud2_& m) {
        std::lock_guard<std::mutex> lk(mtx);
        const auto& d = m.data();
        const std::uint8_t exp = static_cast<std::uint8_t>((m.width() - 1) & 0xFF);
        if (d.size() != static_cast<std::size_t>(m.width()) * 4) cloud_ok = false;
        for (auto b : d) if (b != exp) { cloud_ok = false; break; }
        clouds.push_back(m.width());
      });

  std::unique_ptr<FastDDSTransport> shmtp;
  if (shm) {
    FastDDSTransport::Config scfg;
    scfg.domain_id = domain;
    scfg.participant_name = "test_subscriber_shm";
    scfg.enable_shm = true;
    scfg.shm_segment_size = 64u * 1024 * 1024;
    shmtp = std::make_unique<FastDDSTransport>(scfg);
    if (!shmtp->initialize()) { std::cerr << "shm init failed\n"; return 1; }
    shmtp->subscribe<sensor_msgs::msg::dds_::PointCloud2_64KB_>(
        "/test/shmcloud",
        [&](const sensor_msgs::msg::dds_::PointCloud2_64KB_& m) {
          std::lock_guard<std::mutex> lk(mtx);
          const std::uint8_t exp = static_cast<std::uint8_t>((m.width() - 1) & 0xFF);
          if (m.data()[0] != exp || m.data()[100] != exp) shm_ok = false;
          shmclouds.push_back(m.width());
        },
        {.qos_depth = 1, .reliable = false});
  }

  std::cout << "listening " << seconds << "s on domain " << domain << "…\n";
  std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  tp.shutdown();
  if (shmtp) shmtp->shutdown();

  std::lock_guard<std::mutex> lk(mtx);
  std::cout << "RECV value=" << values.size() << " cloud=" << clouds.size();
  if (shm) std::cout << " shmcloud=" << shmclouds.size();
  std::cout << "\n";

  bool ok = true;
  auto need = [&](const char* what, std::size_t got) {
    if (static_cast<int>(got) != expect) {
      std::cerr << "FAIL: expected " << expect << " " << what << ", got " << got << "\n";
      ok = false;
    }
  };
  need("Float64", values.size());
  for (int i = 0; ok && i < expect; ++i)
    if (values[i] != static_cast<double>(i)) {
      std::cerr << "FAIL: value[" << i << "]=" << values[i] << "\n"; ok = false;
    }
  need("PointCloud2", clouds.size());
  if (!cloud_ok) { std::cerr << "FAIL: PointCloud2 payload mismatch\n"; ok = false; }
  if (shm) {
    need("SHM cloud", shmclouds.size());
    if (!shm_ok) { std::cerr << "FAIL: SHM cloud payload mismatch\n"; ok = false; }
  }

  if (ok) std::cout << "PASS: all messages round-tripped intact\n";
  return ok ? 0 : 1;
}
