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
#include <cctype>
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
DEFINE_string(elis_rt1060_ts_anchor, "end",
              "Timestamp anchor for compact raw: start or end.");
DEFINE_bool(elis_rt1060_allow_xy_only_synth, false,
            "Allow RAW_XY_ONLY packets to synthesize events (test mode).");
DEFINE_int32(elis_rt1060_xy_only_window_us, 3000,
             "Synthetic window (us) for RAW_XY_ONLY event timestamps.");
DEFINE_int32(elis_rt1060_xy_only_polarity, 1,
             "Synthetic polarity for RAW_XY_ONLY events (0 or 1).");

DECLARE_bool(vio_use_elis_link);
DECLARE_bool(vio_elis_use_cached_packets);
DECLARE_bool(vio_elis_require_cached_packets);
DECLARE_int32(vio_elis_track_ttl_frames);

namespace {

constexpr uint8_t kMagicBytes[2] = {0xCD, 0xAB};
constexpr size_t kMagicSize = 2;
constexpr uint8_t kFlagHasRaw = 0x01;
constexpr uint8_t kFlagHasKp = 0x02;
constexpr uint8_t kFlagHasImu = 0x04;
constexpr uint8_t kFlagHasAccel = 0x08;
constexpr uint8_t kFlagRawXYOnly = 0x10;
constexpr uint8_t kFlagImuTs = 0x20;
constexpr uint8_t kFlagKpId = 0x40;
constexpr size_t kHeaderSize = 14;
constexpr size_t kTailSize = 2;
constexpr size_t kMaxKeypoints = 512;
constexpr size_t kMaxRawEvents = 16384;
constexpr size_t kMaxDescLen = 128;
constexpr size_t kMaxPacketBytes = 128 * 1024;

volatile sig_atomic_t g_should_stop = 0;

void handle_signal(int)
{
  g_should_stop = 1;
}

inline bool stop_requested()
{
  return g_should_stop != 0;
}

enum class Rt1060TsAnchor
{
  Start,
  End
};

Rt1060TsAnchor parseRt1060TsAnchor(const std::string& value)
{
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  if (lowered == "start")
  {
    return Rt1060TsAnchor::Start;
  }
  if (lowered == "end")
  {
    return Rt1060TsAnchor::End;
  }
  LOG(WARNING) << "Unknown --elis_rt1060_ts_anchor=" << value
               << " (expected start|end); defaulting to end.";
  return Rt1060TsAnchor::End;
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
  bool raw_xy_only = false;
  bool kp_has_id = false;
  bool has_imu_ts = false;
  uint32_t imu_ts_us = 0;
  std::vector<uint8_t> kp_bytes;
  std::vector<uint8_t> raw_bytes;
};

struct Rt1060DecodeConfig
{
  bool allow_xy_only_synth = false;
  int xy_only_window_us = 3000;
  bool xy_only_polarity = true;
  Rt1060TsAnchor ts_anchor = Rt1060TsAnchor::End;
};

struct Rt1060Stats
{
  uint64_t packets_parsed = 0;
  uint64_t checksum_failures = 0;
  uint64_t resyncs = 0;
  uint64_t bytes_received = 0;
};

enum class ParseResult
{
  NeedMore,
  Parsed,
  Resync
};

uint8_t xor_checksum(const uint8_t* data, size_t len)
{
  uint8_t c = 0;
  for (size_t i = 0; i < len; ++i)
  {
    c ^= data[i];
  }
  return c;
}

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
    tty.c_cc[VMIN] = 0;
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

  ssize_t read_some(uint8_t* buf, size_t n) const
  {
    while (true)
    {
      if (stop_requested())
      {
        return 0;
      }
      const ssize_t r = ::read(fd_, buf, n);
      if (r >= 0)
      {
        return r;
      }
      if (errno == EINTR)
      {
        continue;
      }
      if (errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return 0;
      }
      return -1;
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

ParseResult try_parse_packet(std::vector<uint8_t>& buffer,
                             Rt1060Packet* out,
                             Rt1060Stats* stats)
{
  if (buffer.size() < kMagicSize)
  {
    return ParseResult::NeedMore;
  }

  auto it = std::search(buffer.begin(), buffer.end(),
                        std::begin(kMagicBytes), std::end(kMagicBytes));
  if (it == buffer.end())
  {
    if (!buffer.empty())
    {
      if (buffer.back() == kMagicBytes[0])
      {
        buffer.erase(buffer.begin(), buffer.end() - 1);
      }
      else
      {
        buffer.clear();
      }
    }
    return ParseResult::NeedMore;
  }

  if (it != buffer.begin())
  {
    buffer.erase(buffer.begin(), it);
    if (stats)
    {
      stats->resyncs++;
    }
  }

  if (buffer.size() < (kMagicSize + kHeaderSize))
  {
    return ParseResult::NeedMore;
  }

  const uint8_t* hdr = buffer.data() + kMagicSize;
  const uint8_t version = hdr[0];
  const uint8_t flags = hdr[1];
  const uint16_t seq = read_le_u16(hdr + 2);
  const uint32_t ts_us = read_le_u32(hdr + 4);
  const uint16_t n_raw = read_le_u16(hdr + 8);
  const uint16_t n_kp = read_le_u16(hdr + 10);
  const uint8_t kp_desc_len = hdr[12];

  const bool has_raw = (flags & kFlagHasRaw) != 0;
  const bool has_kp = (flags & kFlagHasKp) != 0;
  const bool has_imu = (flags & kFlagHasImu) != 0;
  const bool has_accel = (flags & kFlagHasAccel) != 0;
  const bool raw_xy_only = (flags & kFlagRawXYOnly) != 0;
  const bool has_imu_ts = (flags & kFlagImuTs) != 0;
  const bool kp_has_id = (flags & kFlagKpId) != 0;

  if (n_kp > kMaxKeypoints || n_raw > kMaxRawEvents || kp_desc_len > kMaxDescLen)
  {
    buffer.erase(buffer.begin());
    if (stats)
    {
      stats->resyncs++;
    }
    return ParseResult::Resync;
  }

  const size_t kp_stride = has_kp ? (static_cast<size_t>(kp_has_id ? 10 : 8) + kp_desc_len) : 0u;
  const size_t kp_bytes = has_kp ? (static_cast<size_t>(n_kp) * kp_stride) : 0u;
  const size_t raw_stride = raw_xy_only ? 4u : 6u;
  const size_t raw_bytes = has_raw ? (static_cast<size_t>(n_raw) * raw_stride) : 0u;
  size_t payload_len = kp_bytes + raw_bytes;
  if (has_accel)
  {
    payload_len += 6u;
  }
  if (has_imu)
  {
    payload_len += 6u;
  }
  if (has_imu_ts)
  {
    payload_len += 4u;
  }

  const size_t packet_len = kMagicSize + kHeaderSize + payload_len + kTailSize;
  if (packet_len > kMaxPacketBytes)
  {
    buffer.erase(buffer.begin());
    if (stats)
    {
      stats->resyncs++;
    }
    return ParseResult::Resync;
  }

  if (buffer.size() < packet_len)
  {
    return ParseResult::NeedMore;
  }

  const size_t checksum_offset = kMagicSize + kHeaderSize + payload_len;
  const uint8_t expected_checksum = buffer[checksum_offset];
  const uint8_t computed_checksum =
      xor_checksum(buffer.data(), kMagicSize + kHeaderSize + payload_len);
  if (expected_checksum != computed_checksum)
  {
    if (stats)
    {
      stats->checksum_failures++;
      stats->resyncs++;
    }
    buffer.erase(buffer.begin());
    return ParseResult::Resync;
  }

  Rt1060Packet pkt;
  pkt.version = version;
  pkt.flags = flags;
  pkt.seq = seq;
  pkt.ts_us = ts_us;
  pkt.n_raw = n_raw;
  pkt.n_kp = n_kp;
  pkt.kp_desc_len = kp_desc_len;
  pkt.raw_xy_only = raw_xy_only;
  pkt.kp_has_id = kp_has_id;
  pkt.has_imu_ts = has_imu_ts;

  size_t offset = kMagicSize + kHeaderSize;
  if (has_kp && kp_bytes > 0)
  {
    pkt.kp_bytes.assign(
        buffer.begin() + static_cast<std::vector<uint8_t>::difference_type>(offset),
        buffer.begin() + static_cast<std::vector<uint8_t>::difference_type>(offset + kp_bytes));
    offset += kp_bytes;
  }
  if (has_raw && raw_bytes > 0)
  {
    pkt.raw_bytes.assign(
        buffer.begin() + static_cast<std::vector<uint8_t>::difference_type>(offset),
        buffer.begin() + static_cast<std::vector<uint8_t>::difference_type>(offset + raw_bytes));
    offset += raw_bytes;
  }
  if (has_accel)
  {
    offset += 6u;
  }
  if (has_imu)
  {
    offset += 6u;
  }
  if (has_imu_ts)
  {
    if (offset + 4u <= buffer.size())
    {
      pkt.imu_ts_us = read_le_u32(buffer.data() + offset);
    }
    offset += 4u;
  }

  buffer.erase(buffer.begin(),
               buffer.begin() + static_cast<std::vector<uint8_t>::difference_type>(packet_len));
  if (out)
  {
    *out = std::move(pkt);
  }
  return ParseResult::Parsed;
}

bool decode_events(const Rt1060Packet& pkt,
                   const Rt1060DecodeConfig& cfg,
                   ze::EventArrayPtr* events_out,
                   int64_t* stamp_ns_out)
{
  if (pkt.raw_bytes.empty())
  {
    return false;
  }

  if (pkt.raw_xy_only)
  {
    if (!cfg.allow_xy_only_synth)
    {
      static int warned_xy_only = 0;
      if (warned_xy_only++ < 3)
      {
        LOG(WARNING) << "[RT1060] RAW_XY_ONLY packet ignored (synth disabled).";
      }
      return false;
    }

    static int warned_xy_only_synth = 0;
    if (warned_xy_only_synth++ < 1)
    {
      LOG(WARNING) << "[RT1060] RAW_XY_ONLY synth mode enabled; "
                   << "timestamps are synthesized over a fixed window.";
    }

    const size_t count = pkt.raw_bytes.size() / 4;
    if (count == 0)
    {
      return false;
    }
    const uint8_t* data = pkt.raw_bytes.data();

    auto events = std::make_shared<ze::EventArray>();
    events->reserve(count);

    int64_t t_end_us = static_cast<int64_t>(pkt.ts_us);
    int64_t window_us = static_cast<int64_t>(cfg.xy_only_window_us);
    if (window_us < 0)
    {
      window_us = 0;
    }
    int64_t t_start_us = t_end_us - window_us;
    if (t_start_us < 0)
    {
      t_start_us = 0;
    }
    const int64_t span_us = t_end_us - t_start_us;
    const int64_t steps = (count > 1) ? static_cast<int64_t>(count - 1) : 0;
    const int64_t step_us = (steps > 0) ? (span_us / steps) : 0;
    const int64_t rem_us = (steps > 0) ? (span_us % steps) : 0;

    const bool pol = cfg.xy_only_polarity;
    for (size_t i = 0; i < count; ++i)
    {
      const uint16_t x = read_le_u16(data);
      const uint16_t y = read_le_u16(data + 2);
      int64_t t_us = (count == 1)
        ? t_end_us
        : (t_start_us + step_us * static_cast<int64_t>(i) +
           std::min<int64_t>(static_cast<int64_t>(i), rem_us));
      if (t_us < 0)
      {
        t_us = 0;
      }

      dvs_msgs::Event ev;
      ev.x = x;
      ev.y = y;
      ev.ts.fromNSec(static_cast<uint64_t>(t_us) * 1000ull);
      ev.polarity = pol;
      events->push_back(ev);
      data += 4;
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

  const size_t count = pkt.raw_bytes.size() / 6;
  const uint8_t* data = pkt.raw_bytes.data();

  uint64_t total_dt = 0;
  for (size_t i = 0; i < count; ++i)
  {
    const uint16_t pol_dt = read_le_u16(data + 4);
    total_dt += static_cast<uint64_t>(pol_dt & 0x7FFF);
    data += 6;
  }

  int64_t base_us = static_cast<int64_t>(pkt.ts_us);
  if (cfg.ts_anchor == Rt1060TsAnchor::End)
  {
    base_us -= static_cast<int64_t>(total_dt);
  }
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
  const size_t base = pkt.kp_has_id ? 10u : 8u;
  const size_t stride = base + static_cast<size_t>(pkt.kp_desc_len);
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

  if (!pkt.kp_has_id)
  {
    static int warned_no_id = 0;
    if (warned_no_id++ < 3)
    {
      LOG(WARNING) << "[RT1060] Firmware keypoints missing KP_ID flag; "
                   << "using per-slice indices as track IDs.";
    }
  }

  const uint8_t* data = pkt.kp_bytes.data();
  for (size_t i = 0; i < count; ++i)
  {
    const uint16_t x = read_le_u16(data);
    const uint16_t y = read_le_u16(data + 2);
    const uint16_t score = read_le_u16(data + 4);
    const uint16_t angle = read_le_u16(data + 6);
    (void)angle;
    int32_t track_id = static_cast<int32_t>(i);
    size_t desc_offset = 8;
    if (pkt.kp_has_id)
    {
      track_id = static_cast<int32_t>(read_le_u16(data + 8));
      desc_offset = 10;
    }

    packet.x_px.push_back(static_cast<float>(x));
    packet.y_px.push_back(static_cast<float>(y));
    packet.strength.push_back(static_cast<float>(score));
    packet.track_ids.push_back(track_id);

    if (pkt.kp_desc_len > 0)
    {
      const uint8_t* desc = data + desc_offset;
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
    FLAGS_vio_elis_require_cached_packets = true;
    if (FLAGS_vio_elis_track_ttl_frames <= 0)
    {
      FLAGS_vio_elis_track_ttl_frames = 2;
      LOG(WARNING) << "[RT1060] Enabling --vio_elis_track_ttl_frames=2 for firmware keypoints.";
    }
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

  Rt1060DecodeConfig decode_cfg;
  decode_cfg.allow_xy_only_synth = FLAGS_elis_rt1060_allow_xy_only_synth;
  decode_cfg.xy_only_window_us = FLAGS_elis_rt1060_xy_only_window_us;
  decode_cfg.xy_only_polarity = FLAGS_elis_rt1060_xy_only_polarity != 0;
  decode_cfg.ts_anchor = parseRt1060TsAnchor(FLAGS_elis_rt1060_ts_anchor);

  Rt1060Stats stats;
  std::vector<uint8_t> buffer;
  buffer.reserve(8192);
  std::vector<uint8_t> scratch(4096);

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
    const ssize_t n_read = port.read_some(scratch.data(), scratch.size());
    if (n_read < 0)
    {
      LOG(ERROR) << "[RT1060] Serial read failed.";
      stop.store(true);
      queue_cv.notify_all();
      break;
    }
    if (n_read > 0)
    {
      buffer.insert(buffer.end(), scratch.begin(), scratch.begin() + n_read);
      stats.bytes_received += static_cast<uint64_t>(n_read);
    }
    else
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    while (!stop.load())
    {
      Rt1060Packet pkt;
      const ParseResult res = try_parse_packet(buffer, &pkt, &stats);
      if (res == ParseResult::Parsed)
      {
        stats.packets_parsed++;
        if (FLAGS_elis_rt1060_drop_empty_raw && !(pkt.flags & kFlagHasRaw))
        {
          continue;
        }

        ze::EventArrayPtr events;
        int64_t stamp_ns = 0;
        if (!decode_events(pkt, decode_cfg, &events, &stamp_ns))
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
          else
          {
            static int missing_kp_log_count = 0;
            if (missing_kp_log_count++ < 5)
            {
              LOG(WARNING) << "[RT1060] Firmware packet missing keypoints (seq="
                           << pkt.seq << ", ts_us=" << pkt.ts_us << ").";
            }
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
        continue;
      }
      if (res == ParseResult::Resync)
      {
        continue;
      }
      break;
    }
  }

  stop.store(true);
  queue_cv.notify_all();
  worker.join();

  std::cout << "[RT1060] Done. Frames processed: " << frames_done.load()
            << " (enqueued=" << frames_enqueued.load()
            << ", dropped=" << slices_dropped.load() << ")\n";
  std::cout << "[RT1060] Parser stats: packets=" << stats.packets_parsed
            << " checksum_failures=" << stats.checksum_failures
            << " resyncs=" << stats.resyncs
            << " bytes=" << stats.bytes_received << "\n";

  if (vo.frontend())
  {
    vo.frontend()->shutdown();
  }
  return 0;
#endif
}
