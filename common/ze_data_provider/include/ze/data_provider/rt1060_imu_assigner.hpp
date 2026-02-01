// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Modified: Robotics and Perception Group

#pragma once

#include <cstdint>
#include <deque>
#include <vector>

#include <ze/common/types.hpp>

namespace ze {

struct Rt1060ImuSample
{
  int64_t stamp_ns = 0;
  Vector3 acc = Vector3::Zero();
  Vector3 gyr = Vector3::Zero();
};

struct Rt1060ImuAssigned
{
  std::vector<int64_t> stamps_ns;
  std::vector<Vector3> acc;
  std::vector<Vector3> gyr;
};

class Rt1060ImuAssigner
{
public:
  explicit Rt1060ImuAssigner(int64_t min_margin_ns = 2000000,
                            int64_t max_margin_ns = 20000000);

  void addSample(int64_t stamp_ns, const Vector3& acc, const Vector3& gyr);

  Rt1060ImuAssigned assign(int64_t start_ns, int64_t end_ns);

  size_t bufferSize() const { return samples_.size(); }
  bool empty() const { return samples_.empty(); }

private:
  int64_t estimatePeriodNs() const;
  int64_t computeMarginNs() const;
  void appendAssigned(Rt1060ImuAssigned& out, const Rt1060ImuSample& sample) const;

  std::deque<Rt1060ImuSample> samples_;
  int64_t min_margin_ns_ = 0;
  int64_t max_margin_ns_ = 0;
};

} // namespace ze
