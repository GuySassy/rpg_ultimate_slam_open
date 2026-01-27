// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Modified: Robotics and Perception Group

#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include <ze/common/types.hpp>
#include <ze/data_provider/data_provider_rt1060.hpp>

namespace ze {

struct Rt1060ParseConfig
{
  Rt1060RawFormat raw_format = Rt1060RawFormat::Auto;
};

struct Rt1060DecodeConfig
{
  bool allow_xy_only_synth = false;
  int xy_only_window_us = 3000;
  bool xy_only_polarity = true;
  Rt1060TsAnchor ts_anchor = Rt1060TsAnchor::End;
};

struct Rt1060Packet
{
  uint8_t version = 0;
  uint8_t flags = 0;
  uint16_t seq = 0;
  uint32_t ts_us = 0;
  uint16_t n_raw = 0;
  uint16_t n_kp = 0;
  uint8_t kp_desc_len = 0;
  uint8_t raw_format = 0;
  bool raw_xy_only = false;
  bool kp_has_id = false;
  bool has_accel = false;
  bool has_imu = false;
  bool has_imu_ts = false;
  uint32_t imu_ts_us = 0;
  std::array<int16_t, 3> accel_raw{{0, 0, 0}};
  std::array<int16_t, 3> gyro_raw{{0, 0, 0}};
  std::vector<uint8_t> kp_bytes;
  std::vector<uint8_t> raw_bytes;
};

struct Rt1060Stats
{
  uint64_t packets_parsed = 0;
  uint64_t checksum_failures = 0;
  uint64_t resyncs = 0;
  uint64_t bytes_received = 0;
  uint64_t queue_drops = 0;
  uint64_t events_emitted = 0;
};

struct Rt1060EventDebug
{
  size_t count = 0;
  uint64_t total_dt = 0;
  int64_t base_us = -1;
  int64_t end_us = -1;
  bool xy_only = false;
};

enum class Rt1060ParseResult
{
  NeedMore,
  Parsed,
  Resync
};

uint8_t rt1060NormalizeRawFormat(uint8_t raw_format, Rt1060RawFormat override);
size_t rt1060RawStride(bool raw_xy_only, uint8_t raw_format);

Rt1060ParseResult rt1060TryParsePacket(std::vector<uint8_t>& buffer,
                                       Rt1060Packet* out,
                                       Rt1060Stats* stats,
                                       const Rt1060ParseConfig& cfg);

bool rt1060DecodeEvents(const Rt1060Packet& pkt,
                        const Rt1060DecodeConfig& cfg,
                        EventArrayPtr* events_out,
                        int64_t* stamp_ns_out,
                        Rt1060EventDebug* debug_out);

} // namespace ze
