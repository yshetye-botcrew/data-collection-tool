// Loopback test helper: publishes a deterministic sequence so the
// recorder/player round-trip can be verified byte-for-byte.
//
//   /test/value     std_msgs/Float64        (UDP, reliable)   data = i
//   /test/cloud     sensor_msgs/PointCloud2  (UDP, reliable)  width = i+1
//
// With --shm it also mimics the robot lidar: a bounded zero-copy cloud over
// SHM with BEST-EFFORT QoS — the exact pattern that recorded 0 messages before
// QoS-matching + an SHM transport were added.
//
//   /test/shmcloud  sensor_msgs/PointCloud2_64KB_  (SHM, best-effort)
//
#include <message_manager/bounded_pointclouds.hpp>
#include <message_manager/fastdds_transport.hpp>
#include <message_manager/sensor_msgs.hpp>
#include <message_manager/std_msgs.hpp>

#include <array>
#include <chrono>
#include <iostream>
#include <memory>
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
  const int count = std::stoi(arg(argc, argv, "--count", "20"));
  const double warmup = std::stod(arg(argc, argv, "--warmup", "1.5"));
  const bool shm = flag(argc, argv, "--shm");

  FastDDSTransport::Config cfg;
  cfg.domain_id = domain;
  cfg.participant_name = "test_publisher";
  FastDDSTransport tp(cfg);
  if (!tp.initialize()) { std::cerr << "init failed\n"; return 1; }

  auto value_pub = tp.advertise<std_msgs::msg::dds_::Float64_>("/test/value");
  auto cloud_pub = tp.advertise<sensor_msgs::msg::dds_::PointCloud2_>("/test/cloud");

  // SHM, best-effort, depth 1 — the robot lidar pattern.
  std::unique_ptr<FastDDSTransport> shmtp;
  message_manager::PublisherHandle shm_pub = 0;
  if (shm) {
    FastDDSTransport::Config scfg;
    scfg.domain_id = domain;
    scfg.participant_name = "test_publisher_shm";
    scfg.enable_shm = true;
    scfg.shm_segment_size = 64u * 1024 * 1024;
    shmtp = std::make_unique<FastDDSTransport>(scfg);
    if (!shmtp->initialize()) { std::cerr << "shm init failed\n"; return 1; }
    shm_pub = shmtp->advertise<sensor_msgs::msg::dds_::PointCloud2_64KB_>(
        "/test/shmcloud", {.qos_depth = 1, .reliable = false});
  }

  std::this_thread::sleep_for(std::chrono::duration<double>(warmup));

  for (int i = 0; i < count; ++i) {
    std_msgs::msg::dds_::Float64_ f;
    f.data(static_cast<double>(i));
    tp.publish(value_pub, f);

    sensor_msgs::msg::dds_::PointCloud2_ pc;
    pc.height(1);
    pc.width(static_cast<std::uint32_t>(i + 1));
    pc.data(std::vector<std::uint8_t>(static_cast<std::size_t>(i + 1) * 4,
                                      static_cast<std::uint8_t>(i & 0xFF)));
    tp.publish(cloud_pub, pc);

    if (shm) {
      auto sc = std::make_unique<sensor_msgs::msg::dds_::PointCloud2_64KB_>();
      sc->height(1);
      sc->width(static_cast<std::uint32_t>(i + 1));
      std::array<std::uint8_t, 65536> buf{};
      buf[0] = static_cast<std::uint8_t>(i & 0xFF);
      buf[100] = static_cast<std::uint8_t>(i & 0xFF);
      sc->data(buf);
      shmtp->publish(shm_pub, *sc);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  std::cout << "published " << count << " msgs"
            << (shm ? " (incl. SHM best-effort cloud)" : "") << "\n";
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  tp.shutdown();
  if (shmtp) shmtp->shutdown();
  return 0;
}
