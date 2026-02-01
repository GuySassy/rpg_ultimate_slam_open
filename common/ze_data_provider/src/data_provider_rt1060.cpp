// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Modified: Robotics and Perception Group

#include <ze/data_provider/data_provider_rt1060.hpp>
#include <ze/data_provider/rt1060_parser.hpp>

#include <ze/common/logging.hpp>

#include <dvs_msgs/Event.h>

#ifdef ELIS_LINK_ENABLED
#include <elisUltimateLink/keypoint_service.hpp>
#include <elisUltimateLink/synchronizer_bridge.hpp>
#endif

#include <algorithm>
#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <termios.h>
#include <unistd.h>

namespace ze {
constexpr uint8_t kRawFmtCompact = 0;
constexpr uint8_t kRawFmtFull = 1;

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
constexpr size_t kMaxPacketBytes = 256 * 1024;
// RT1060 protocol: accel in g*1024, gyro in LSM6DSO raw (8.75 mdps/LSB).
constexpr double kAccelScale = 9.81 / 1024.0;
constexpr double kGyroScale = 0.00875 * (3.14159265358979323846 / 180.0);

uint16_t read_le_u16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8);
}

int16_t read_le_i16(const uint8_t* data)
{
  return static_cast<int16_t>(read_le_u16(data));
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


uint8_t xor_checksum(const uint8_t* data, size_t len)
{
  uint8_t c = 0;
  for (size_t i = 0; i < len; ++i)
  {
    c ^= data[i];
  }
  return c;
}

class SerialPort
{
public:
  SerialPort(const std::string& device, int baud, volatile bool& running)
    : running_(running)
  {
    fd_ = ::open(device.c_str(), O_RDONLY | O_NOCTTY);
    if (fd_ < 0)
    {
      throw std::runtime_error("Failed to open serial device " + device);
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0)
    {
      ::close(fd_);
      throw std::runtime_error("Failed to read serial attributes.");
    }

    cfsetospeed(&tty, to_speed(baud));
    cfsetispeed(&tty, to_speed(baud));

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

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
    if (!running_)
    {
      return 0;
    }
    while (running_)
    {
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
    return 0;
  }

  bool read_exact(uint8_t* buf, size_t n) const
  {
    size_t offset = 0;
    while (offset < n && running_)
    {
      ssize_t r = ::read(fd_, buf + offset, n - offset);
      if (r > 0)
      {
        offset += static_cast<size_t>(r);
        continue;
      }
      if (r == 0)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (errno == EINTR)
      {
        continue;
      }
      return false;
    }
    return offset == n;
  }

private:
  volatile bool& running_;
  int fd_ = -1;
};

Rt1060ParseResult try_parse_packet(std::vector<uint8_t>& buffer,
                                   Rt1060Packet* out,
                                   Rt1060Stats* stats,
                                   const Rt1060ParseConfig& cfg)
{
  if (buffer.size() < kMagicSize)
  {
    return Rt1060ParseResult::NeedMore;
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
    return Rt1060ParseResult::NeedMore;
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
    return Rt1060ParseResult::NeedMore;
  }

  const uint8_t* hdr = buffer.data() + kMagicSize;
  const uint8_t version = hdr[0];
  const uint8_t flags = hdr[1];
  const uint16_t seq = read_le_u16(hdr + 2);
  const uint32_t ts_us = read_le_u32(hdr + 4);
  const uint16_t n_raw = read_le_u16(hdr + 8);
  const uint16_t n_kp = read_le_u16(hdr + 10);
  const uint8_t kp_desc_len = hdr[12];
  uint8_t raw_format = hdr[13];
  raw_format = rt1060NormalizeRawFormat(raw_format, cfg.raw_format);

  const bool has_raw = ((flags & kFlagHasRaw) != 0) || (n_raw > 0);
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
    return Rt1060ParseResult::Resync;
  }

  const size_t kp_base = (version >= 2) ? 10u : 8u;
  const size_t kp_stride = has_kp
    ? (kp_base + static_cast<size_t>(kp_has_id ? 2 : 0) + kp_desc_len)
    : 0u;
  const size_t kp_bytes = has_kp ? (static_cast<size_t>(n_kp) * kp_stride) : 0u;
  const size_t raw_stride = rt1060RawStride(raw_xy_only, raw_format);
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
    return Rt1060ParseResult::Resync;
  }

  if (buffer.size() < packet_len)
  {
    return Rt1060ParseResult::NeedMore;
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
    return Rt1060ParseResult::Resync;
  }

  Rt1060Packet pkt;
  pkt.version = version;
  pkt.flags = flags;
  pkt.seq = seq;
  pkt.ts_us = ts_us;
  pkt.n_raw = n_raw;
  pkt.n_kp = n_kp;
  pkt.kp_desc_len = kp_desc_len;
  pkt.raw_format = raw_format;
  pkt.raw_xy_only = raw_xy_only;
  pkt.kp_has_id = kp_has_id;
  pkt.has_accel = has_accel;
  pkt.has_imu = has_imu;
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
    pkt.accel_raw[0] = read_le_i16(buffer.data() + offset);
    pkt.accel_raw[1] = read_le_i16(buffer.data() + offset + 2);
    pkt.accel_raw[2] = read_le_i16(buffer.data() + offset + 4);
    offset += 6u;
  }
  if (has_imu)
  {
    pkt.gyro_raw[0] = read_le_i16(buffer.data() + offset);
    pkt.gyro_raw[1] = read_le_i16(buffer.data() + offset + 2);
    pkt.gyro_raw[2] = read_le_i16(buffer.data() + offset + 4);
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
  return Rt1060ParseResult::Parsed;
}

bool decode_events(const Rt1060Packet& pkt,
                   const Rt1060DecodeConfig& cfg,
                   EventArrayPtr* events_out,
                   int64_t* stamp_ns_out,
                   Rt1060EventDebug* debug_out)
{
  if (debug_out)
  {
    debug_out->count = 0;
    debug_out->total_dt = 0;
    debug_out->base_us = static_cast<int64_t>(pkt.ts_us);
    debug_out->end_us = static_cast<int64_t>(pkt.ts_us);
    debug_out->xy_only = pkt.raw_xy_only;
  }

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

    auto events = std::make_shared<EventArray>();
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
    if (debug_out)
    {
      debug_out->count = count;
      debug_out->total_dt = static_cast<uint64_t>(span_us);
      debug_out->base_us = t_start_us;
      debug_out->end_us = t_end_us;
    }
    return true;
  }

  const size_t raw_stride = rt1060RawStride(pkt.raw_xy_only, pkt.raw_format);
  if (raw_stride == 0)
  {
    return false;
  }
  const size_t count = pkt.raw_bytes.size() / raw_stride;
  const uint8_t* data = pkt.raw_bytes.data();

  auto events = std::make_shared<EventArray>();
  events->reserve(count);

  if (pkt.raw_format == kRawFmtFull)
  {
    constexpr int64_t kTickHz = 24000000;
    constexpr int64_t kNsPerSec = 1000000000;
    int64_t total_dt_ns = 0;
    for (size_t i = 0; i < count; ++i)
    {
      const uint16_t dt_ticks = read_le_u16(data + 4);
      const int64_t dt_ns = (static_cast<int64_t>(dt_ticks) * kNsPerSec + kTickHz / 2) / kTickHz;
      total_dt_ns += dt_ns;
      data += raw_stride;
    }

    int64_t base_ns = static_cast<int64_t>(pkt.ts_us) * 1000;
    if (cfg.ts_anchor == Rt1060TsAnchor::End)
    {
      base_ns -= total_dt_ns;
    }
    if (base_ns < 0)
    {
      base_ns = 0;
    }

    int64_t t_ns = base_ns;
    data = pkt.raw_bytes.data();
    for (size_t i = 0; i < count; ++i)
    {
      const uint16_t raw_word = read_le_u16(data);
      const uint16_t dt_ticks = read_le_u16(data + 4);
      const int64_t dt_ns = (static_cast<int64_t>(dt_ticks) * kNsPerSec + kTickHz / 2) / kTickHz;
      t_ns += dt_ns;

      dvs_msgs::Event ev;
      ev.polarity = static_cast<bool>(raw_word & 0x1);
      ev.x = read_le_u16(data + 8);
      ev.y = read_le_u16(data + 10);
      ev.ts.fromNSec(static_cast<uint64_t>(t_ns));
      events->push_back(ev);
      data += raw_stride;
    }

    if (debug_out)
    {
      debug_out->count = count;
      debug_out->total_dt = static_cast<uint64_t>((total_dt_ns + 500) / 1000);
      debug_out->base_us = base_ns / 1000;
      debug_out->end_us = t_ns / 1000;
    }
  }
  else
  {
    uint64_t total_dt = 0;
    for (size_t i = 0; i < count; ++i)
    {
      const uint16_t pol_dt = read_le_u16(data + 4);
      total_dt += static_cast<uint64_t>(pol_dt & 0x7FFF);
      data += raw_stride;
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
      data += raw_stride;
    }

    if (debug_out)
    {
      debug_out->count = count;
      debug_out->total_dt = total_dt;
      debug_out->base_us = base_us;
      debug_out->end_us = t_us;
    }
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

#ifdef ELIS_LINK_ENABLED
bool decode_keypoints(const Rt1060Packet& pkt, elis_link::KeypointPacket* packet_out)
{
  if (pkt.kp_bytes.empty())
  {
    return false;
  }
  const bool score_u32 = (pkt.version >= 2);
  const size_t base = score_u32 ? 10u : 8u;
  const size_t stride = base + static_cast<size_t>(pkt.kp_has_id ? 2 : 0)
                        + static_cast<size_t>(pkt.kp_desc_len);
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
    const float score = score_u32
      ? static_cast<float>(read_le_u32(data + 4))
      : static_cast<float>(read_le_u16(data + 4));
    size_t offset = score_u32 ? 8u : 6u;
    int32_t track_id = static_cast<int32_t>(i);
    if (pkt.kp_has_id)
    {
      track_id = static_cast<int32_t>(read_le_u16(data + offset));
      offset += 2u;
    }
    const uint16_t angle = read_le_u16(data + offset);
    (void)angle;
    offset += 2u;
    const size_t desc_offset = offset;

    packet.x_px.push_back(static_cast<float>(x));
    packet.y_px.push_back(static_cast<float>(y));
    packet.strength.push_back(score);
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
#endif

} // namespace

uint8_t rt1060NormalizeRawFormat(uint8_t raw_format, Rt1060RawFormat override)
{
  switch (override)
  {
  case Rt1060RawFormat::Auto:
    break;
  case Rt1060RawFormat::Compact:
    return kRawFmtCompact;
  case Rt1060RawFormat::Full:
    return kRawFmtFull;
  default:
    break;
  }

  if (raw_format == kRawFmtFull)
  {
    return kRawFmtFull;
  }
  if (raw_format != kRawFmtCompact)
  {
    static int warned_unknown = 0;
    if (warned_unknown++ < 3)
    {
      LOG(WARNING) << "[RT1060] Unknown raw_format=" << static_cast<int>(raw_format)
                   << "; defaulting to compact.";
    }
  }
  return kRawFmtCompact;
}

size_t rt1060RawStride(bool raw_xy_only, uint8_t raw_format)
{
  if (raw_xy_only)
  {
    return 4u;
  }
  if (raw_format == kRawFmtFull)
  {
    return 12u;
  }
  return 6u;
}

Rt1060ParseResult rt1060TryParsePacket(std::vector<uint8_t>& buffer,
                                       Rt1060Packet* out,
                                       Rt1060Stats* stats,
                                       const Rt1060ParseConfig& cfg)
{
  return try_parse_packet(buffer, out, stats, cfg);
}

bool rt1060DecodeEvents(const Rt1060Packet& pkt,
                        const Rt1060DecodeConfig& cfg,
                        EventArrayPtr* events_out,
                        int64_t* stamp_ns_out,
                        Rt1060EventDebug* debug_out)
{
  return decode_events(pkt, cfg, events_out, stamp_ns_out, debug_out);
}

struct DataProviderRt1060::SliceData
{
  EventArrayPtr events;
  int64_t stamp_ns = 0;
  bool has_imu = false;
  std::vector<int64_t> imu_stamps_ns;
  std::vector<Vector3> acc;
  std::vector<Vector3> gyr;
#ifdef ELIS_LINK_ENABLED
  elis_link::KeypointPacket packet;
  bool has_packet = false;
#endif
};

DataProviderRt1060::DataProviderRt1060(const std::string& port,
                                       int baud,
                                       bool use_firmware_keypoints,
                                       bool drop_empty_raw,
                                       int queue_size,
                                       bool allow_xy_only_synth,
                                       int xy_only_window_us,
                                       int xy_only_polarity,
                                       Rt1060TsAnchor ts_anchor,
                                       Rt1060RawFormat raw_format,
                                       int log_stats_interval_s,
                                       bool debug_packets,
                                       int debug_every_n_packets,
                                       const std::string& debug_log_path,
                                       int debug_log_max_lines,
                                       int debug_log_flush_every,
                                       size_t imu_count)
  : DataProviderBase(DataProviderType::Rt1060)
  , port_(port)
  , baud_(baud)
  , use_firmware_keypoints_(use_firmware_keypoints)
  , drop_empty_raw_(drop_empty_raw)
  , imu_count_(imu_count)
  , queue_size_(std::max(1, queue_size))
  , allow_xy_only_synth_(allow_xy_only_synth)
  , xy_only_window_us_(std::max(0, xy_only_window_us))
  , xy_only_polarity_(xy_only_polarity != 0)
  , ts_anchor_(ts_anchor)
  , raw_format_(raw_format)
  , log_stats_interval_s_(std::max(0, log_stats_interval_s))
  , debug_packets_(debug_packets)
  , debug_every_n_packets_(std::max(1, debug_every_n_packets))
  , debug_log_path_(debug_log_path)
  , debug_log_max_lines_(debug_log_max_lines)
  , debug_log_flush_every_(std::max(1, debug_log_flush_every))
{
  if (imu_count_ > 1u)
  {
    LOG(WARNING) << "[RT1060] Only one IMU is supported; clamping --num_imus to 1.";
    imu_count_ = 1u;
  }
  reader_thread_ = std::thread(&DataProviderRt1060::readerLoop, this);
}

DataProviderRt1060::~DataProviderRt1060()
{
  shutdown();
  if (reader_thread_.joinable())
  {
    reader_thread_.join();
  }
}

bool DataProviderRt1060::ok() const
{
  if (!running_)
  {
    return pending_.load() > 0;
  }
  if (reader_done_.load())
  {
    return pending_.load() > 0;
  }
  return true;
}

void DataProviderRt1060::enqueueSlice(SliceData slice)
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    int dropped = 0;
    while (static_cast<int>(queue_.size()) >= queue_size_)
    {
      queue_.pop_front();
      ++dropped;
    }
    if (dropped > 0)
    {
      queue_drops_.fetch_add(static_cast<uint64_t>(dropped), std::memory_order_relaxed);
    }
    queue_.push_back(std::move(slice));
    pending_.store(static_cast<int>(queue_.size()));
  }
  queue_cv_.notify_one();
}

bool DataProviderRt1060::spinOnce()
{
  SliceData slice;
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait_for(lock, std::chrono::milliseconds(50), [&]() {
      return !running_ || !queue_.empty() || reader_done_.load();
    });
    if (!running_ && queue_.empty())
    {
      return false;
    }
    if (queue_.empty())
    {
      return running_;
    }
    slice = std::move(queue_.front());
    queue_.pop_front();
    pending_.store(static_cast<int>(queue_.size()));
  }

#ifdef ELIS_LINK_ENABLED
  if (use_firmware_keypoints_ && slice.has_packet)
  {
    elis_link::cache_packet(slice.stamp_ns, std::move(slice.packet));
  }
#endif

  if (slice.has_imu)
  {
    if (imu_callback_)
    {
      const size_t n = slice.imu_stamps_ns.size();
      for (size_t i = 0; i < n; ++i)
      {
        const Vector3& acc = (i < slice.acc.size()) ? slice.acc[i] : slice.acc.back();
        const Vector3& gyr = (i < slice.gyr.size()) ? slice.gyr[i] : slice.gyr.back();
        imu_callback_(slice.imu_stamps_ns[i], acc, gyr, 0u);
      }
    }
    else
    {
      static int warned_no_imu_cb = 0;
      if (warned_no_imu_cb++ < 1)
      {
        LOG(WARNING) << "[RT1060] IMU data received but no IMU callback registered. "
                        "Set --num_imus>0 to enable IMU processing.";
      }
    }
  }

  if (dvs_callback_ && slice.events)
  {
    dvs_callback_(slice.stamp_ns, slice.events, 0u);
  }
  return true;
}

void DataProviderRt1060::readerLoop()
{
  bool warned_missing_elis = false;
  try
  {
    SerialPort port(port_, baud_, running_);
    LOG(INFO) << "[RT1060] DataProvider listening on " << port_;

    Rt1060DecodeConfig decode_cfg;
    decode_cfg.allow_xy_only_synth = allow_xy_only_synth_;
    decode_cfg.xy_only_window_us = xy_only_window_us_;
    decode_cfg.xy_only_polarity = xy_only_polarity_;
    decode_cfg.ts_anchor = ts_anchor_;

    Rt1060ParseConfig parse_cfg;
    parse_cfg.raw_format = raw_format_;

    Rt1060Stats stats;
    std::vector<uint8_t> buffer;
    buffer.reserve(8192);
    std::vector<uint8_t> scratch(4096);

    auto last_log = std::chrono::steady_clock::now();
    uint64_t last_packets = 0;
    uint64_t last_events = 0;
    uint64_t last_bytes = 0;
    uint64_t debug_packets = 0;
    int64_t last_stamp_ns = -1;
    uint64_t debug_log_packets = 0;

    std::ofstream debug_log;
    bool debug_log_active = false;
    int debug_log_lines = 0;
    int debug_log_since_flush = 0;
    const int debug_log_flush_every = std::max(1, debug_log_flush_every_);
    const int debug_log_max_lines = debug_log_max_lines_;

    if (!debug_log_path_.empty())
    {
      debug_log.open(debug_log_path_.c_str(), std::ios::out | std::ios::trunc);
      if (!debug_log.is_open())
      {
        LOG(WARNING) << "[RT1060] Failed to open debug log at " << debug_log_path_;
      }
      else
      {
        debug_log_active = true;
        debug_log << "tag,seq,ts_us,flags,raw_xy_only,raw_format,n_raw,raw_stride,raw_count_calc,"
                     "n_kp,kp_desc_len,kp_has_id,imu_ts,imu_ts_us,raw_bytes,events,"
                     "base_us,end_us,total_dt,stamp_ns,checksum_failures,resyncs,queue_drops,"
                     "non_monotonic,status\n";
        LOG(INFO) << "[RT1060] Debug log enabled: " << debug_log_path_;
      }
    }

    auto write_debug_line = [&](const char* tag,
                                const Rt1060Packet& pkt,
                                const Rt1060EventDebug& info,
                                int64_t stamp_ns,
                                bool non_monotonic,
                                const char* status) {
      if (!debug_log_active)
      {
        return;
      }
      if (debug_log_max_lines > 0 && debug_log_lines >= debug_log_max_lines)
      {
        debug_log_active = false;
        LOG(WARNING) << "[RT1060] Debug log max lines reached; disabling file output.";
        return;
      }

      const size_t raw_stride = rt1060RawStride(pkt.raw_xy_only, pkt.raw_format);
      const size_t raw_count_calc = (raw_stride > 0u)
        ? (pkt.raw_bytes.size() / raw_stride)
        : 0u;

      debug_log << tag << ','
                << pkt.seq << ','
                << pkt.ts_us << ",0x"
                << std::hex << static_cast<int>(pkt.flags) << std::dec << ','
                << static_cast<int>(pkt.raw_xy_only) << ','
                << static_cast<int>(pkt.raw_format) << ','
                << pkt.n_raw << ','
                << raw_stride << ','
                << raw_count_calc << ','
                << pkt.n_kp << ','
                << static_cast<int>(pkt.kp_desc_len) << ','
                << static_cast<int>(pkt.kp_has_id) << ','
                << static_cast<int>(pkt.has_imu_ts) << ','
                << pkt.imu_ts_us << ','
                << pkt.raw_bytes.size() << ','
                << info.count << ','
                << info.base_us << ','
                << info.end_us << ','
                << info.total_dt << ','
                << stamp_ns << ','
                << stats.checksum_failures << ','
                << stats.resyncs << ','
                << queue_drops_.load(std::memory_order_relaxed) << ','
                << static_cast<int>(non_monotonic) << ','
                << status << '\n';

      ++debug_log_lines;
      ++debug_log_since_flush;
      if (debug_log_since_flush >= debug_log_flush_every)
      {
        debug_log.flush();
        debug_log_since_flush = 0;
      }
    };

    while (running_)
    {
      const ssize_t n = port.read_some(scratch.data(), scratch.size());
      if (n < 0)
      {
        LOG(ERROR) << "[RT1060] Serial read failed.";
        break;
      }
      if (n > 0)
      {
        buffer.insert(buffer.end(), scratch.begin(), scratch.begin() + n);
        stats.bytes_received += static_cast<uint64_t>(n);
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }

      while (running_)
      {
        Rt1060Packet pkt;
        const Rt1060ParseResult res = rt1060TryParsePacket(buffer, &pkt, &stats, parse_cfg);
        if (res == Rt1060ParseResult::Parsed)
        {
          stats.packets_parsed++;
          if (debug_packets_)
          {
            static int warned_flags = 0;
            if (warned_flags++ < 50)
            {
              LOG(INFO) << "[RT1060][hdr] seq=" << pkt.seq
                        << " flags=0x" << std::hex << static_cast<int>(pkt.flags) << std::dec
                        << " n_raw=" << pkt.n_raw
                        << " raw_bytes=" << pkt.raw_bytes.size()
                        << " raw_format=" << static_cast<int>(pkt.raw_format)
                        << " raw_xy_only=" << pkt.raw_xy_only
                        << " imu_ts=" << pkt.has_imu_ts
                        << " imu_ts_us=" << pkt.imu_ts_us;
            }
          }
          if (debug_log_active)
          {
            ++debug_log_packets;
          }
          if (drop_empty_raw_ && !(pkt.flags & kFlagHasRaw) &&
              (pkt.n_raw == 0 || pkt.raw_bytes.empty()))
          {
            if (pkt.has_accel || pkt.has_imu)
            {
              const real_t accel_scale = static_cast<real_t>(kAccelScale);
              const real_t gyro_scale = static_cast<real_t>(kGyroScale);
              Vector3 acc = Vector3::Zero();
              Vector3 gyr = Vector3::Zero();
              if (pkt.has_accel)
              {
                acc.x() = static_cast<real_t>(pkt.accel_raw[0]) * accel_scale;
                acc.y() = static_cast<real_t>(pkt.accel_raw[1]) * accel_scale;
                acc.z() = static_cast<real_t>(pkt.accel_raw[2]) * accel_scale;
              }
              if (pkt.has_imu)
              {
                gyr.x() = static_cast<real_t>(pkt.gyro_raw[0]) * gyro_scale;
                gyr.y() = static_cast<real_t>(pkt.gyro_raw[1]) * gyro_scale;
                gyr.z() = static_cast<real_t>(pkt.gyro_raw[2]) * gyro_scale;
              }
              uint32_t imu_ts_us = pkt.has_imu_ts ? pkt.imu_ts_us : pkt.ts_us;
              if (imu_ts_us == 0u)
              {
                imu_ts_us = pkt.ts_us;
              }
              imu_assigner_.addSample(static_cast<int64_t>(imu_ts_us) * 1000, acc, gyr);
              if (debug_log_active)
              {
                Rt1060EventDebug imu_info;
                imu_info.count = 0;
                imu_info.total_dt = 0;
                imu_info.base_us = static_cast<int64_t>(pkt.ts_us);
                imu_info.end_us = static_cast<int64_t>(pkt.ts_us);
                imu_info.xy_only = pkt.raw_xy_only;
                write_debug_line("imu_only", pkt, imu_info, -1, false, "imu_only");
              }
            }
            else if (debug_log_active)
            {
              Rt1060EventDebug drop_info;
              drop_info.count = 0;
              drop_info.total_dt = 0;
              drop_info.base_us = static_cast<int64_t>(pkt.ts_us);
              drop_info.end_us = static_cast<int64_t>(pkt.ts_us);
              drop_info.xy_only = pkt.raw_xy_only;
              write_debug_line("drop", pkt, drop_info, -1, false, "drop_no_raw_flag");
            }
            continue;
          }

          EventArrayPtr events;
          int64_t stamp_ns = -1;
          Rt1060EventDebug debug_info;
          const bool debug_enabled = debug_packets_;
          const bool want_debug_info = debug_enabled || debug_log_active || (imu_count_ > 0u);
          const bool decoded = rt1060DecodeEvents(
            pkt,
            decode_cfg,
            &events,
            &stamp_ns,
            want_debug_info ? &debug_info : nullptr);
          if (!decoded)
          {
            if (debug_log_active)
            {
              const char* reason = "drop_decode";
              if (pkt.raw_xy_only && !allow_xy_only_synth_)
              {
                reason = "drop_xy_only";
              }
              else if (pkt.raw_bytes.empty())
              {
                reason = "drop_empty_raw_bytes";
              }
              write_debug_line("drop", pkt, debug_info, -1, false, reason);
            }
            continue;
          }
          if (!events || events->empty())
          {
            if (debug_log_active)
            {
              write_debug_line("drop", pkt, debug_info, stamp_ns, false, "drop_empty_events");
            }
            continue;
          }
          stats.events_emitted += static_cast<uint64_t>(events->size());

          SliceData slice;
          slice.events = std::move(events);
          // Match frontend cached-packet lookup key: events.back().ts.toNSec().
          slice.stamp_ns = stamp_ns;
          if (pkt.has_accel || pkt.has_imu)
          {
            const real_t accel_scale = static_cast<real_t>(kAccelScale);
            const real_t gyro_scale = static_cast<real_t>(kGyroScale);
            Vector3 acc = Vector3::Zero();
            Vector3 gyr = Vector3::Zero();
            if (pkt.has_accel)
            {
              acc.x() = static_cast<real_t>(pkt.accel_raw[0]) * accel_scale;
              acc.y() = static_cast<real_t>(pkt.accel_raw[1]) * accel_scale;
              acc.z() = static_cast<real_t>(pkt.accel_raw[2]) * accel_scale;
            }
            if (pkt.has_imu)
            {
              gyr.x() = static_cast<real_t>(pkt.gyro_raw[0]) * gyro_scale;
              gyr.y() = static_cast<real_t>(pkt.gyro_raw[1]) * gyro_scale;
              gyr.z() = static_cast<real_t>(pkt.gyro_raw[2]) * gyro_scale;
            }
            uint32_t imu_ts_us = pkt.has_imu_ts ? pkt.imu_ts_us : pkt.ts_us;
            if (imu_ts_us == 0u)
            {
              imu_ts_us = pkt.ts_us;
            }
            imu_assigner_.addSample(static_cast<int64_t>(imu_ts_us) * 1000, acc, gyr);
          }

          if (!imu_assigner_.empty())
          {
            int64_t start_ns = last_event_stamp_ns_;
            if (start_ns < 0)
            {
              if (debug_info.total_dt > 0)
              {
                start_ns = stamp_ns - static_cast<int64_t>(debug_info.total_dt) * 1000;
              }
              else
              {
                start_ns = stamp_ns - 2000000;
              }
            }
            if (start_ns < 0)
            {
              start_ns = 0;
            }
            Rt1060ImuAssigned assigned = imu_assigner_.assign(start_ns, stamp_ns);
            if (assigned.stamps_ns.size() >= 2)
            {
              slice.has_imu = true;
              slice.imu_stamps_ns = std::move(assigned.stamps_ns);
              slice.acc = std::move(assigned.acc);
              slice.gyr = std::move(assigned.gyr);
            }
          }

          last_event_stamp_ns_ = stamp_ns;

          const bool non_monotonic = (last_stamp_ns >= 0 && stamp_ns <= last_stamp_ns);
          if (debug_enabled && non_monotonic)
          {
            LOG(WARNING) << "[RT1060][debug] Non-monotonic slice stamp: prev_ns="
                         << last_stamp_ns << " curr_ns=" << stamp_ns
                         << " seq=" << pkt.seq << " ts_us=" << pkt.ts_us
                         << " n_raw=" << pkt.n_raw
                         << " raw_xy_only=" << pkt.raw_xy_only
                         << " base_us=" << debug_info.base_us
                         << " end_us=" << debug_info.end_us
                         << " total_dt=" << debug_info.total_dt;
          }

          if (debug_log_active && non_monotonic)
          {
            write_debug_line("non_monotonic", pkt, debug_info, stamp_ns, true, "non_monotonic");
          }

          if (debug_log_active &&
              (debug_log_packets % static_cast<uint64_t>(debug_every_n_packets_) == 0))
          {
            write_debug_line("packet", pkt, debug_info, stamp_ns, non_monotonic, "ok");
          }

          if (debug_enabled)
          {
            ++debug_packets;
            if (debug_packets % static_cast<uint64_t>(debug_every_n_packets_) == 0)
            {
              LOG(INFO) << "[RT1060][debug] seq=" << pkt.seq
                        << " ts_us=" << pkt.ts_us
                        << " flags=0x" << std::hex << static_cast<int>(pkt.flags) << std::dec
                        << " raw_xy_only=" << pkt.raw_xy_only
                        << " raw_format=" << static_cast<int>(pkt.raw_format)
                        << " n_raw=" << pkt.n_raw
                        << " n_kp=" << pkt.n_kp
                        << " kp_desc_len=" << static_cast<int>(pkt.kp_desc_len)
                        << " kp_has_id=" << pkt.kp_has_id
                        << " imu_ts=" << pkt.has_imu_ts
                        << " imu_ts_us=" << pkt.imu_ts_us
                        << " raw_bytes=" << pkt.raw_bytes.size()
                        << " events=" << debug_info.count
                        << " base_us=" << debug_info.base_us
                        << " end_us=" << debug_info.end_us
                        << " total_dt=" << debug_info.total_dt
                        << " stamp_ns=" << stamp_ns;
            }
          }

#ifdef ELIS_LINK_ENABLED
          if (use_firmware_keypoints_)
          {
            slice.has_packet = decode_keypoints(pkt, &slice.packet);
          }
#else
          if (use_firmware_keypoints_ && !warned_missing_elis)
          {
            warned_missing_elis = true;
            LOG(WARNING) << "[RT1060] Firmware keypoints requested but ELIS_LINK_ENABLED is not set; ignoring.";
          }
#endif

          last_stamp_ns = stamp_ns;
          enqueueSlice(std::move(slice));
          continue;
        }
        if (res == Rt1060ParseResult::Resync)
        {
          continue;
        }
        break;
      }

      if (log_stats_interval_s_ > 0)
      {
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - last_log).count();
        if (elapsed >= static_cast<double>(log_stats_interval_s_))
        {
          const uint64_t curr_queue_drops = queue_drops_.load(std::memory_order_relaxed);
          const uint64_t packets = stats.packets_parsed - last_packets;
          const uint64_t events = stats.events_emitted - last_events;
          const uint64_t bytes = stats.bytes_received - last_bytes;
          const double packets_rate = packets / elapsed;
          const double events_rate = events / elapsed;
          const double bytes_rate = bytes / elapsed;

          LOG(INFO) << std::fixed << std::setprecision(1)
                    << "[RT1060] stats packets/s=" << packets_rate
                    << " events/s=" << events_rate
                    << " bytes/s=" << bytes_rate
                    << " checksum_failures=" << stats.checksum_failures
                    << " resyncs=" << stats.resyncs
                    << " queue_drops=" << curr_queue_drops;

          last_log = now;
          last_packets = stats.packets_parsed;
          last_events = stats.events_emitted;
          last_bytes = stats.bytes_received;
        }
      }
    }
  }
  catch (const std::exception& e)
  {
    LOG(ERROR) << "[RT1060] DataProvider failed: " << e.what();
  }

  reader_done_.store(true);
  queue_cv_.notify_all();
}

} // namespace ze
