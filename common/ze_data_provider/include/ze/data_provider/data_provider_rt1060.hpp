// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Modified: Robotics and Perception Group

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <ze/data_provider/data_provider_base.hpp>

namespace ze {

enum class Rt1060TsAnchor {
  Start,
  End
};

//! RT1060 serial data provider (events + optional firmware keypoints).
class DataProviderRt1060 : public DataProviderBase
{
public:
  DataProviderRt1060(const std::string& port,
                     int baud,
                     bool use_firmware_keypoints,
                     bool drop_empty_raw,
                     int queue_size,
                     bool allow_xy_only_synth,
                     int xy_only_window_us,
                     int xy_only_polarity,
                     Rt1060TsAnchor ts_anchor,
                     int log_stats_interval_s,
                     bool debug_packets,
                     int debug_every_n_packets,
                     const std::string& debug_log_path,
                     int debug_log_max_lines,
                     int debug_log_flush_every);
  ~DataProviderRt1060() override;

  bool spinOnce() override;
  bool ok() const override;

  size_t imuCount() const override { return 0u; }
  size_t cameraCount() const override { return 0u; }
  size_t dvsCount() const override { return 1u; }

private:
  struct SliceData;

  void readerLoop();
  void enqueueSlice(SliceData slice);

  std::string port_;
  int baud_ = 115200;
  bool use_firmware_keypoints_ = false;
  bool drop_empty_raw_ = true;
  int queue_size_ = 2;
  bool allow_xy_only_synth_ = false;
  int xy_only_window_us_ = 3000;
  bool xy_only_polarity_ = true;
  Rt1060TsAnchor ts_anchor_ = Rt1060TsAnchor::End;
  int log_stats_interval_s_ = 5;
  bool debug_packets_ = false;
  int debug_every_n_packets_ = 200;
  std::string debug_log_path_;
  int debug_log_max_lines_ = 2000;
  int debug_log_flush_every_ = 20;

  std::thread reader_thread_;
  std::atomic<bool> reader_done_{false};
  std::atomic<int> pending_{0};
  std::atomic<uint64_t> queue_drops_{0};

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<SliceData> queue_;
};

} // namespace ze
