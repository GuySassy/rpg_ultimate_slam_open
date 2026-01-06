// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Modified: Robotics and Perception Group

#include <ze/data_provider/data_provider_rt1060.hpp>

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
#include <iostream>
#include <stdexcept>
#include <vector>
#include <termios.h>
#include <unistd.h>

namespace ze {
namespace {

constexpr uint16_t kMagic = 0xABCD;
constexpr uint8_t kFlagHasRaw = 0x01;
constexpr uint8_t kFlagHasKp = 0x02;
constexpr uint8_t kFlagHasImu = 0x04;
constexpr uint8_t kFlagHasAccel = 0x08;
constexpr size_t kHeaderSize = 14;

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

bool sync_to_magic(const SerialPort& port, volatile bool& running)
{
  uint8_t b0 = 0;
  if (!port.read_exact(&b0, 1))
  {
    return false;
  }
  while (running)
  {
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
  return false;
}

bool read_packet(const SerialPort& port, Rt1060Packet* out, volatile bool& running)
{
  if (!sync_to_magic(port, running))
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

bool decode_events(const Rt1060Packet& pkt, EventArrayPtr* events_out, int64_t* stamp_ns_out)
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

  auto events = std::make_shared<EventArray>();
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

#ifdef ELIS_LINK_ENABLED
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
#endif

} // namespace

struct DataProviderRt1060::SliceData
{
  EventArrayPtr events;
  int64_t stamp_ns = 0;
#ifdef ELIS_LINK_ENABLED
  elis_link::KeypointPacket packet;
  bool has_packet = false;
#endif
};

DataProviderRt1060::DataProviderRt1060(const std::string& port,
                                       int baud,
                                       bool use_firmware_keypoints,
                                       bool drop_empty_raw,
                                       int queue_size)
  : DataProviderBase(DataProviderType::Rt1060)
  , port_(port)
  , baud_(baud)
  , use_firmware_keypoints_(use_firmware_keypoints)
  , drop_empty_raw_(drop_empty_raw)
  , queue_size_(std::max(1, queue_size))
{
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
    while (static_cast<int>(queue_.size()) >= queue_size_)
    {
      queue_.pop_front();
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

    while (running_)
    {
      Rt1060Packet pkt;
      if (!read_packet(port, &pkt, running_))
      {
        if (!running_)
        {
          break;
        }
        continue;
      }

      if (drop_empty_raw_ && !(pkt.flags & kFlagHasRaw))
      {
        continue;
      }

      EventArrayPtr events;
      int64_t stamp_ns = 0;
      if (!decode_events(pkt, &events, &stamp_ns))
      {
        continue;
      }
      if (!events || events->empty())
      {
        continue;
      }

      SliceData slice;
      slice.events = std::move(events);
      slice.stamp_ns = stamp_ns;

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

      enqueueSlice(std::move(slice));
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
