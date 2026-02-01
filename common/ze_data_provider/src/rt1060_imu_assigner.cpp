// Copyright (c) 2015-2016, ETH Zurich, Wyss Zurich, Zurich Eye
// All rights reserved.
//
// Modified: Robotics and Perception Group

#include <ze/data_provider/rt1060_imu_assigner.hpp>

#include <algorithm>
#include <vector>

namespace ze {
namespace {

constexpr int64_t kDefaultPeriodNs = 1000000; // 1 ms

} // namespace

Rt1060ImuAssigner::Rt1060ImuAssigner(int64_t min_margin_ns, int64_t max_margin_ns)
  : min_margin_ns_(std::max<int64_t>(0, min_margin_ns))
  , max_margin_ns_(std::max<int64_t>(0, max_margin_ns))
{
}

void Rt1060ImuAssigner::addSample(int64_t stamp_ns, const Vector3& acc, const Vector3& gyr)
{
  Rt1060ImuSample sample;
  sample.stamp_ns = stamp_ns;
  sample.acc = acc;
  sample.gyr = gyr;

  if (samples_.empty() || stamp_ns >= samples_.back().stamp_ns)
  {
    samples_.push_back(sample);
    return;
  }

  auto it = std::upper_bound(samples_.begin(), samples_.end(), stamp_ns,
                             [](int64_t value, const Rt1060ImuSample& s) {
                               return value < s.stamp_ns;
                             });
  samples_.insert(it, sample);
}

int64_t Rt1060ImuAssigner::estimatePeriodNs() const
{
  if (samples_.size() < 2)
  {
    return kDefaultPeriodNs;
  }

  const size_t max_pairs = std::min<size_t>(samples_.size() - 1, 10);
  std::vector<int64_t> deltas;
  deltas.reserve(max_pairs);

  const size_t start = samples_.size() - 1 - max_pairs;
  for (size_t i = start + 1; i < samples_.size(); ++i)
  {
    const int64_t prev = samples_[i - 1].stamp_ns;
    const int64_t curr = samples_[i].stamp_ns;
    const int64_t dt = curr - prev;
    if (dt > 0)
    {
      deltas.push_back(dt);
    }
  }

  if (deltas.empty())
  {
    return kDefaultPeriodNs;
  }

  const size_t mid = deltas.size() / 2;
  std::nth_element(deltas.begin(), deltas.begin() + mid, deltas.end());
  return deltas[mid];
}

int64_t Rt1060ImuAssigner::computeMarginNs() const
{
  const int64_t period_ns = estimatePeriodNs();
  int64_t margin_ns = std::max(min_margin_ns_, 2 * period_ns);
  if (max_margin_ns_ > 0)
  {
    margin_ns = std::min(margin_ns, max_margin_ns_);
  }
  return margin_ns;
}

void Rt1060ImuAssigner::appendAssigned(Rt1060ImuAssigned& out,
                                       const Rt1060ImuSample& sample) const
{
  if (!out.stamps_ns.empty() && out.stamps_ns.back() == sample.stamp_ns)
  {
    return;
  }
  out.stamps_ns.push_back(sample.stamp_ns);
  out.acc.push_back(sample.acc);
  out.gyr.push_back(sample.gyr);
}

Rt1060ImuAssigned Rt1060ImuAssigner::assign(int64_t start_ns, int64_t end_ns)
{
  Rt1060ImuAssigned out;
  if (samples_.empty())
  {
    return out;
  }
  if (start_ns < 0)
  {
    start_ns = 0;
  }
  if (end_ns < start_ns)
  {
    return out;
  }

  const int64_t margin_ns = computeMarginNs();

  int before_idx = -1;
  int after_idx = -1;
  int keep_idx = -1;

  for (size_t i = 0; i < samples_.size(); ++i)
  {
    const int64_t stamp = samples_[i].stamp_ns;
    if (stamp <= start_ns)
    {
      before_idx = static_cast<int>(i);
    }
    if (stamp <= end_ns)
    {
      keep_idx = static_cast<int>(i);
    }
    if (after_idx < 0 && stamp > end_ns)
    {
      after_idx = static_cast<int>(i);
    }
  }

  if (before_idx >= 0)
  {
    const int64_t dt = start_ns - samples_[before_idx].stamp_ns;
    if (dt >= 0 && dt <= margin_ns)
    {
      appendAssigned(out, samples_[before_idx]);
    }
  }

  for (size_t i = 0; i < samples_.size(); ++i)
  {
    const int64_t stamp = samples_[i].stamp_ns;
    if (stamp > start_ns && stamp <= end_ns)
    {
      appendAssigned(out, samples_[i]);
    }
  }

  if (after_idx >= 0)
  {
    const int64_t dt = samples_[after_idx].stamp_ns - end_ns;
    if (dt >= 0 && dt <= margin_ns)
    {
      appendAssigned(out, samples_[after_idx]);
    }
  }

  if (keep_idx >= 0)
  {
    std::deque<Rt1060ImuSample> next;
    next.push_back(samples_[static_cast<size_t>(keep_idx)]);
    for (size_t i = static_cast<size_t>(keep_idx) + 1; i < samples_.size(); ++i)
    {
      next.push_back(samples_[i]);
    }
    samples_.swap(next);
  }

  return out;
}

} // namespace ze
