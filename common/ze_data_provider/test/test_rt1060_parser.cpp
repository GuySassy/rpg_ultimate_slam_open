// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Modified: Robotics and Perception Group

#include <ze/common/test_entrypoint.hpp>
#include <ze/data_provider/rt1060_parser.hpp>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace {

constexpr uint8_t kMagic0 = 0xCD;
constexpr uint8_t kMagic1 = 0xAB;
constexpr uint8_t kFlagHasRaw = 0x01;
constexpr uint8_t kFlagRawXYOnly = 0x10;

void append_le_u16(std::vector<uint8_t>& buf, uint16_t value)
{
  buf.push_back(static_cast<uint8_t>(value & 0xFF));
  buf.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

void append_le_u32(std::vector<uint8_t>& buf, uint32_t value)
{
  buf.push_back(static_cast<uint8_t>(value & 0xFF));
  buf.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
  buf.push_back(static_cast<uint8_t>((value >> 16) & 0xFF));
  buf.push_back(static_cast<uint8_t>((value >> 24) & 0xFF));
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

std::vector<uint8_t> build_packet(uint16_t seq,
                                  uint32_t ts_us,
                                  uint16_t n_raw,
                                  bool raw_xy_only,
                                  uint8_t raw_format,
                                  const std::vector<uint8_t>& raw_bytes)
{
  uint8_t flags = kFlagHasRaw;
  if (raw_xy_only)
  {
    flags |= kFlagRawXYOnly;
  }

  std::vector<uint8_t> buf;
  buf.reserve(2 + 14 + raw_bytes.size() + 2);
  buf.push_back(kMagic0);
  buf.push_back(kMagic1);
  buf.push_back(1);  // version
  buf.push_back(flags);
  append_le_u16(buf, seq);
  append_le_u32(buf, ts_us);
  append_le_u16(buf, n_raw);
  append_le_u16(buf, 0);  // n_kp
  buf.push_back(0);       // kp_desc_len
  buf.push_back(raw_format);
  buf.insert(buf.end(), raw_bytes.begin(), raw_bytes.end());

  const uint8_t checksum = xor_checksum(buf.data(), buf.size());
  buf.push_back(checksum);
  buf.push_back(0);  // pad
  return buf;
}

void append_compact_event(std::vector<uint8_t>& raw,
                          uint16_t x,
                          uint16_t y,
                          bool polarity,
                          uint16_t dt_us)
{
  const uint16_t pol_dt = static_cast<uint16_t>((polarity ? 0x8000 : 0x0000) | (dt_us & 0x7FFF));
  append_le_u16(raw, x);
  append_le_u16(raw, y);
  append_le_u16(raw, pol_dt);
}

void append_full_event(std::vector<uint8_t>& raw,
                       uint16_t raw_word,
                       uint16_t dt_ticks,
                       uint16_t x,
                       uint16_t y)
{
  append_le_u16(raw, raw_word);
  append_le_u16(raw, 0);  // unused
  append_le_u16(raw, dt_ticks);
  append_le_u16(raw, 0);  // pol (unused)
  append_le_u16(raw, x);
  append_le_u16(raw, y);
}

}  // namespace

TEST(Rt1060Parser, CompactRawEndAnchor)
{
  using namespace ze;

  std::vector<uint8_t> raw;
  append_compact_event(raw, 10, 20, true, 10);
  append_compact_event(raw, 11, 21, false, 20);

  std::vector<uint8_t> buffer = build_packet(1, 1000, 2, false, 0, raw);

  Rt1060ParseConfig parse_cfg;
  parse_cfg.raw_format = Rt1060RawFormat::Auto;
  Rt1060Packet pkt;
  Rt1060Stats stats;

  EXPECT_EQ(rt1060TryParsePacket(buffer, &pkt, &stats, parse_cfg),
            Rt1060ParseResult::Parsed);
  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(pkt.n_raw, 2);
  EXPECT_EQ(pkt.raw_format, 0);

  Rt1060DecodeConfig decode_cfg;
  decode_cfg.ts_anchor = Rt1060TsAnchor::End;
  EventArrayPtr events;
  int64_t stamp_ns = -1;
  Rt1060EventDebug debug;

  ASSERT_TRUE(rt1060DecodeEvents(pkt, decode_cfg, &events, &stamp_ns, &debug));
  ASSERT_TRUE(events);
  ASSERT_EQ(events->size(), 2u);
  EXPECT_EQ((*events)[0].x, 10);
  EXPECT_EQ((*events)[0].y, 20);
  EXPECT_TRUE((*events)[0].polarity);
  EXPECT_EQ((*events)[0].ts.toNSec(), 980000ull);
  EXPECT_EQ((*events)[1].x, 11);
  EXPECT_EQ((*events)[1].y, 21);
  EXPECT_FALSE((*events)[1].polarity);
  EXPECT_EQ((*events)[1].ts.toNSec(), 1000000ull);
  EXPECT_EQ(static_cast<uint64_t>(stamp_ns), 1000000ull);
}

TEST(Rt1060Parser, FullRawEndAnchor)
{
  using namespace ze;

  std::vector<uint8_t> raw;
  append_full_event(raw, 0x0001, 24, 5, 6);
  append_full_event(raw, 0x0000, 48, 7, 8);

  std::vector<uint8_t> buffer = build_packet(2, 1000, 2, false, 1, raw);

  Rt1060ParseConfig parse_cfg;
  parse_cfg.raw_format = Rt1060RawFormat::Auto;
  Rt1060Packet pkt;
  Rt1060Stats stats;

  EXPECT_EQ(rt1060TryParsePacket(buffer, &pkt, &stats, parse_cfg),
            Rt1060ParseResult::Parsed);
  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(pkt.n_raw, 2);
  EXPECT_EQ(pkt.raw_format, 1);

  Rt1060DecodeConfig decode_cfg;
  decode_cfg.ts_anchor = Rt1060TsAnchor::End;
  EventArrayPtr events;
  int64_t stamp_ns = -1;
  Rt1060EventDebug debug;

  ASSERT_TRUE(rt1060DecodeEvents(pkt, decode_cfg, &events, &stamp_ns, &debug));
  ASSERT_TRUE(events);
  ASSERT_EQ(events->size(), 2u);
  EXPECT_EQ((*events)[0].x, 5);
  EXPECT_EQ((*events)[0].y, 6);
  EXPECT_TRUE((*events)[0].polarity);
  EXPECT_EQ((*events)[0].ts.toNSec(), 998000ull);
  EXPECT_EQ((*events)[1].x, 7);
  EXPECT_EQ((*events)[1].y, 8);
  EXPECT_FALSE((*events)[1].polarity);
  EXPECT_EQ((*events)[1].ts.toNSec(), 1000000ull);
  EXPECT_EQ(static_cast<uint64_t>(stamp_ns), 1000000ull);
}

TEST(Rt1060Parser, FullRawOverride)
{
  using namespace ze;

  std::vector<uint8_t> raw;
  append_full_event(raw, 0x0001, 24, 1, 2);

  std::vector<uint8_t> buffer = build_packet(3, 500, 1, false, 0, raw);

  Rt1060ParseConfig parse_cfg;
  parse_cfg.raw_format = Rt1060RawFormat::Full;
  Rt1060Packet pkt;
  Rt1060Stats stats;

  EXPECT_EQ(rt1060TryParsePacket(buffer, &pkt, &stats, parse_cfg),
            Rt1060ParseResult::Parsed);
  EXPECT_TRUE(buffer.empty());
  EXPECT_EQ(pkt.raw_format, 1);

  Rt1060DecodeConfig decode_cfg;
  decode_cfg.ts_anchor = Rt1060TsAnchor::End;
  EventArrayPtr events;
  int64_t stamp_ns = -1;
  Rt1060EventDebug debug;

  ASSERT_TRUE(rt1060DecodeEvents(pkt, decode_cfg, &events, &stamp_ns, &debug));
  ASSERT_TRUE(events);
  ASSERT_EQ(events->size(), 1u);
  EXPECT_EQ((*events)[0].x, 1);
  EXPECT_EQ((*events)[0].y, 2);
  EXPECT_TRUE((*events)[0].polarity);
  EXPECT_EQ((*events)[0].ts.toNSec(), 500000ull);
}

ZE_UNITTEST_ENTRYPOINT
