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

//! RT1060 serial data provider (events + optional firmware keypoints).
class DataProviderRt1060 : public DataProviderBase
{
public:
  DataProviderRt1060(const std::string& port,
                     int baud,
                     bool use_firmware_keypoints,
                     bool drop_empty_raw,
                     int queue_size);
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

  std::thread reader_thread_;
  std::atomic<bool> reader_done_{false};
  std::atomic<int> pending_{0};

  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<SliceData> queue_;
};

} // namespace ze
