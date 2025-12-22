// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <ze/common/types.hpp>
#include <ze/vio_common/landmark_handle.hpp>
#include <ze/vio_common/landmark_table.hpp>

namespace ze {

struct ElisLinkHandleStats
{
  std::size_t unique_track_ids = 0u;
  std::size_t allocated_handles = 0u;

  std::size_t missing_new = 0u;
  std::size_t missing_invalid_handle = 0u;
  std::size_t missing_inactive = 0u;
  std::size_t missing_not_stored = 0u;

  std::size_t healed_invalid_handle = 0u;
};

// Ensures every unique track_id has a valid, stored LandmarkHandle associated
// with it in track_to_handle. If a mapping exists but the handle is invalid
// (e.g. version was overwritten to 0/1), we try to heal it using the slot's
// current version in the LandmarkTable; otherwise, we allocate a fresh handle.
ElisLinkHandleStats ensureElisLandmarkHandles(
    const std::vector<int32_t>& track_ids,
    LandmarkTable& landmarks,
    std::unordered_map<int32_t, LandmarkHandle>& track_to_handle,
    uint32_t nframe_seq,
    const Seed& seed_prototype);

}  // namespace ze

