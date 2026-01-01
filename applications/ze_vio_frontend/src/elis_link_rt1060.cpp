// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <gflags/gflags.h>
#include <ze/common/logging.hpp>
#include <ze/vio_frontend/frontend_api.hpp>
#include <ze/vio_frontend/frontend_base.hpp>

#include <dvs_msgs/Event.h>

#ifdef ELIS_LINK_ENABLED
#include <elisUltimateLink/keypoint_service.hpp>
#include <elisUltimateLink/synchronizer_bridge.hpp>
#endif

#include <atomic>
#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <signal.h>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

DEFINE_string(elis_rt1060_port, "/dev/ttyACM0",
              "Serial device for RT1060 telemetry (e.g. /dev/ttyACM0).");
DEFINE_int32(elis_rt1060_baud, 115200,
             "Baud rate (ignored by USB CDC but required by the serial API).");
DEFINE_string(elis_rt1060_keypoint_source, "elis_code",
              "Keypoint source: elis_code (SNN) or firmware.");
DEFINE_int32(elis_rt1060_max_frames, 0,
             "Maximum number of frames to process (0 = run until interrupted).");
DEFINE_int32(elis_rt1060_queue_size, 2,
             "Max number of RT1060 slices queued between reader and frontend.");
DEFINE_bool(elis_rt1060_drop_empty_raw, true,
            "Drop RT1060 packets that contain no raw events.");

DECLARE_bool(vio_use_elis_link);
DECLARE_bool(vio_elis_use_cached_packets);

namespace {

constexpr uint16_t kMagic = 0xABCD;
constexpr uint8_t kFlagHasRaw = 0x01;
constexpr uint8_t kFlagHasKp = 0x02;
constexpr uint8_t kFlagHasImu = 0x04;
constexpr uint8_t kFlagHasAccel = 0x08;
constexpr size_t kHeaderSize = 14;

volatile sig_atomic_t g_should_stop = 0;

void handle_signal(int)
{
  g_should_stop = 1;
}

inline bool stop_requested()
{
  return g_should_stop != 0;
}

struct Rt1060Packet
{
  uint8_t version = 0;
  uint8_t flags = 0;
  uint16_t seq = 0;
  uint32_t ts_us = 0;
  uint16_t n_raw = 0;
  uint16_t n_kp = 0;
  uint8_t kp_desc_len = 0;
  std::vector<uint8_t> kp_bytes;
  std::vector<uint8_t> raw_bytes;
};

struct SliceData
{
  ze::EventArrayPtr events;
  elis_link::KeypointPacket packet;
  bool has_packet = false;
  int64_t stamp_ns = 0;
};

uint16_t read_le_u16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t read_le_u32(const uint8_t* data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}

speed_t to_speed(int baud)
{
  switch (baud)
  {
  case 9600: return B9600;
  case 19200: return B19200;
  case 38400: return B38400;
  case 57600: return B57600;
  case 115200: return B115200;
  case 230400: return B230400;
  default: return B115200;
  }
}

class SerialPort
{
public:
  SerialPort(const std::string& port, int baud)
  {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0)
    {
      throw std::runtime_error("Failed to open serial port: " + port);
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
    {
      ::close(fd_);
      throw std::runtime_error("Failed to get serial attributes.");
    }

    cfsetospeed(&tty, to_speed(baud));
    cfsetispeed(&tty, to_speed(baud));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0)
    {
      ::close(fd_);
      throw std::runtime_error("Failed to set serial attributes.");
    }
  }

  ~SerialPort()
  {
    if (fd_ >= 0)
    {
      ::close(fd_);
    }
  }

  bool read_exact(uint8_t* buf, size_t n) const
  {
    size_t offset = 0;
    while (offset < n)
    {
      if (stop_requested())
      {
        return false;
      }
      ssize_t r = ::read(fd_, buf + offset, n - offset);
      if (r > 0)
      {
        offset += static_cast<size_t>(r);
        continue;
      }
      if (r == 0)
      {
        continue;
      }
      if (errno == EINTR)
      {
        if (stop_requested())
        {
          return false;
        }
        continue;
      }
      return false;
    }
    return true;
  }

private:
  int fd_ = -1;
};

bool sync_to_magic(const SerialPort& port)
{
  uint8_t b0 = 0;
  if (!port.read_exact(&b0, 1))
  {
    return false;
  }
  while (true)
  {
    if (stop_requested())
    {
      return false;
    }
    uint8_t b1 = 0;
    if (!port.read_exact(&b1, 1))
    {
      return false;
    }
    const uint16_t magic = static_cast<uint16_t>(b0) | (static_cast<uint16_t>(b1) << 8);
    if (magic == kMagic)
    {
      return true;
    }
    b0 = b1;
  }
}

bool read_packet(const SerialPort& port, Rt1060Packet* out)
{
  if (!sync_to_magic(port))
  {
    return false;
  }

  uint8_t hdr[kHeaderSize] = {0};
  if (!port.read_exact(hdr, sizeof(hdr)))
  {
    return false;
  }

  Rt1060Packet pkt;
  pkt.version = hdr[0];
  pkt.flags = hdr[1];
  pkt.seq = read_le_u16(hdr + 2);
  pkt.ts_us = read_le_u32(hdr + 4);
  pkt.n_raw = read_le_u16(hdr + 8);
  pkt.n_kp = read_le_u16(hdr + 10);
  pkt.kp_desc_len = hdr[12];

  if (pkt.flags & kFlagHasKp)
  {
    const size_t kp_stride = static_cast<size_t>(8 + pkt.kp_desc_len);
    const size_t kp_bytes = static_cast<size_t>(pkt.n_kp) * kp_stride;
    pkt.kp_bytes.resize(kp_bytes);
    if (!port.read_exact(pkt.kp_bytes.data(), kp_bytes))
    {
      return false;
    }
  }
  if (pkt.flags & kFlagHasRaw)
  {
    const size_t raw_bytes = static_cast<size_t>(pkt.n_raw) * 6;
    pkt.raw_bytes.resize(raw_bytes);
    if (!port.read_exact(pkt.raw_bytes.data(), raw_bytes))
    {
      return false;
    }
  }
  if (pkt.flags & kFlagHasAccel)
  {
    uint8_t accel[6] = {0};
    if (!port.read_exact(accel, sizeof(accel)))
    {
      return false;
    }
  }
  if (pkt.flags & kFlagHasImu)
  {
    uint8_t imu[6] = {0};
    if (!port.read_exact(imu, sizeof(imu)))
    {
      return false;
    }
  }

  uint8_t tail[2] = {0};
  if (!port.read_exact(tail, sizeof(tail)))
  {
    return false;
  }

  *out = std::move(pkt);
  return true;
}

bool decode_events(const Rt1060Packet& pkt, ze::EventArrayPtr* events_out, int64_t* stamp_ns_out)
{
  if (pkt.raw_bytes.empty())
  {
    return false;
  }

  const size_t count = pkt.raw_bytes.size() / 6;
  const uint8_t* data = pkt.raw_bytes.data();

  uint64_t total_dt = 0;
  for (size_t i = 0; i < count; ++i)
  {
    const uint16_t pol_dt = read_le_u16(data + 4);
    total_dt += static_cast<uint64_t>(pol_dt & 0x7FFF);
    data += 6;
  }

  int64_t base_us = static_cast<int64_t>(pkt.ts_us) - static_cast<int64_t>(total_dt);
  if (base_us < 0)
  {
    base_us = 0;
  }

  auto events = std::make_shared<ze::EventArray>();
  events->reserve(count);

  int64_t t_us = base_us;
  data = pkt.raw_bytes.data();
  for (size_t i = 0; i < count; ++i)
  {
    const uint16_t x = read_le_u16(data);
    const uint16_t y = read_le_u16(data + 2);
    const uint16_t pol_dt = read_le_u16(data + 4);
    const uint16_t dt = static_cast<uint16_t>(pol_dt & 0x7FFF);
    t_us += static_cast<int64_t>(dt);

    dvs_msgs::Event ev;
    ev.x = x;
    ev.y = y;
    ev.ts.fromNSec(static_cast<uint64_t>(t_us) * 1000ull);
    ev.polarity = static_cast<bool>((pol_dt >> 15) & 0x1);
    events->push_back(ev);
    data += 6;
  }

  if (events->empty())
  {
    return false;
  }

  if (stamp_ns_out)
  {
    *stamp_ns_out = events->back().ts.toNSec();
  }
  if (events_out)
  {
    *events_out = std::move(events);
  }
  return true;
}

bool decode_keypoints(const Rt1060Packet& pkt, elis_link::KeypointPacket* packet_out)
{
  if (pkt.kp_bytes.empty())
  {
    return false;
  }
  const size_t stride = static_cast<size_t>(8 + pkt.kp_desc_len);
  if (stride == 0 || (pkt.kp_bytes.size() % stride) != 0)
  {
    return false;
  }

  const size_t count = pkt.kp_bytes.size() / stride;
  elis_link::KeypointPacket packet;
  packet.x_px.reserve(count);
  packet.y_px.reserve(count);
  packet.strength.reserve(count);
  packet.track_ids.reserve(count);
  if (pkt.kp_desc_len > 0)
  {
    packet.descriptors.reserve(count * pkt.kp_desc_len);
  }

  const uint8_t* data = pkt.kp_bytes.data();
  for (size_t i = 0; i < count; ++i)
  {
    const uint16_t x = read_le_u16(data);
    const uint16_t y = read_le_u16(data + 2);
    const uint16_t score = read_le_u16(data + 4);
    const uint16_t track_id = read_le_u16(data + 6);

    packet.x_px.push_back(static_cast<float>(x));
    packet.y_px.push_back(static_cast<float>(y));
    packet.strength.push_back(static_cast<float>(score));
    packet.track_ids.push_back(static_cast<int32_t>(track_id));

    if (pkt.kp_desc_len > 0)
    {
      const uint8_t* desc = data + 8;
      packet.descriptors.insert(packet.descriptors.end(), desc, desc + pkt.kp_desc_len);
    }

    data += stride;
  }

  packet.descriptor_stride = pkt.kp_desc_len;
  packet.processing_time_ms = 0.0;
  packet.detection_mode = "rt1060_fw";
  if (packet_out)
  {
    *packet_out = std::move(packet);
  }
  return true;
}

void process_event_slice(ze::FrontendBase& frontend,
                         ze::EventArrayPtr events_ptr)
{
  if (!events_ptr || events_ptr->empty())
  {
    return;
  }

  const int64_t t_start_ns = events_ptr->front().ts.toNSec();
  const int64_t t_end_ns = events_ptr->back().ts.toNSec();
  const int64_t t_mid_ns = (t_start_ns + t_end_ns) / 2;

  static int64_t last_imu_end_ns = -1;
  const int64_t t_imu_start_ns = (last_imu_end_ns >= 0) ? last_imu_end_ns : t_start_ns;
  last_imu_end_ns = t_end_ns;

  ze::ImuStamps imu_stamps(3);
  imu_stamps << t_imu_start_ns, t_mid_ns, t_end_ns;

  ze::ImuAccGyrContainer imu_accgyr(6, 3);
  imu_accgyr.setZero();

  std::vector<ze::ImuStamps> imu_stamps_vec{imu_stamps};
  std::vector<ze::ImuAccGyrContainer> imu_accgyr_vec{imu_accgyr};

  ze::StampedEventArray stamped_events{t_end_ns, std::move(events_ptr)};
  frontend.addData(stamped_events, imu_stamps_vec, imu_accgyr_vec);
}

}  // namespace

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  struct sigaction sa{};
  sa.sa_handler = handle_signal;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

#ifndef ELIS_LINK_ENABLED
  std::cerr << "[RT1060] ELIS_LINK_ENABLED not set; rebuild with elis_link_core available.\n";
  return 1;
#else
  const bool use_firmware = (FLAGS_elis_rt1060_keypoint_source == "firmware");
  if (use_firmware)
  {
    FLAGS_vio_elis_use_cached_packets = true;
  }

  ze::VisualOdometry vo;
  FLAGS_vio_use_events_and_images = false;
  FLAGS_vio_use_events = true;
  if (!FLAGS_vio_use_elis_link)
  {
    LOG(WARNING) << "[RT1060] Enabling --vio_use_elis_link for this run.";
    FLAGS_vio_use_elis_link = true;
  }
  vo.initialize();
  vo.frontend()->stage_ = ze::FrontendStage::Initializing;

  vo.registerResultCallback([&](const int64_t timestamp_ns,
                                const Eigen::Quaterniond& orientation,
                                const Eigen::Vector3d& position,
                                const ze::FrontendStage stage,
                                const uint32_t num_tracked_features) {
    std::cout << "[RT1060] t_ns=" << timestamp_ns
              << " stage=" << static_cast<int>(stage)
              << " keypoints=" << num_tracked_features
              << " p=[" << position.transpose() << "]\n";
  });

  SerialPort port(FLAGS_elis_rt1060_port, FLAGS_elis_rt1060_baud);
  std::cout << "[RT1060] Listening on " << FLAGS_elis_rt1060_port << "\n";

  std::atomic<bool> stop{false};
  std::atomic<int> frames_done{0};
  std::atomic<int> frames_enqueued{0};
  std::atomic<int> slices_dropped{0};

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<SliceData> slice_queue;

  auto enqueue_slice = [&](SliceData slice) {
    const int queue_limit = std::max(1, FLAGS_elis_rt1060_queue_size);
    {
      std::lock_guard<std::mutex> lock(queue_mutex);
      while (static_cast<int>(slice_queue.size()) >= queue_limit)
      {
        slice_queue.pop_front();
        ++slices_dropped;
      }
      slice_queue.push_back(std::move(slice));
    }
    queue_cv.notify_one();
  };

  std::thread worker([&]() {
    while (!stop.load())
    {
      SliceData slice;
      {
        std::unique_lock<std::mutex> lock(queue_mutex);
        queue_cv.wait(lock, [&]() { return stop.load() || !slice_queue.empty(); });
        if (stop.load() && slice_queue.empty())
        {
          break;
        }
        slice = std::move(slice_queue.front());
        slice_queue.pop_front();
      }

      if (use_firmware && slice.has_packet)
      {
        elis_link::cache_packet(slice.stamp_ns, std::move(slice.packet));
      }
      process_event_slice(*vo.frontend(), std::move(slice.events));

      const int n = ++frames_done;
      if (FLAGS_elis_rt1060_max_frames > 0 && n >= FLAGS_elis_rt1060_max_frames)
      {
        stop.store(true);
        queue_cv.notify_all();
        break;
      }
    }
  });

  while (!stop.load())
  {
    if (stop_requested())
    {
      stop.store(true);
      queue_cv.notify_all();
      break;
    }
    Rt1060Packet pkt;
    if (!read_packet(port, &pkt))
    {
      if (stop_requested())
      {
        stop.store(true);
        queue_cv.notify_all();
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    ze::EventArrayPtr events;
    int64_t stamp_ns = 0;
    if (!decode_events(pkt, &events, &stamp_ns))
    {
      if (FLAGS_elis_rt1060_drop_empty_raw)
      {
        continue;
      }
    }

    SliceData slice;
    slice.events = std::move(events);
    slice.stamp_ns = stamp_ns;
    if (use_firmware)
    {
      elis_link::KeypointPacket packet;
      if (decode_keypoints(pkt, &packet))
      {
        slice.packet = std::move(packet);
        slice.has_packet = true;
      }
    }

    if (!slice.events || slice.events->empty())
    {
      continue;
    }

    enqueue_slice(std::move(slice));
    const int n = ++frames_enqueued;
    if (FLAGS_elis_rt1060_max_frames > 0 && n >= FLAGS_elis_rt1060_max_frames)
    {
      stop.store(true);
      queue_cv.notify_all();
      break;
    }
  }

  stop.store(true);
  queue_cv.notify_all();
  worker.join();

  std::cout << "[RT1060] Done. Frames processed: " << frames_done.load()
            << " (enqueued=" << frames_enqueued.load()
            << ", dropped=" << slices_dropped.load() << ")\n";

  if (vo.frontend())
  {
    vo.frontend()->shutdown();
  }
  return 0;
#endif
}
