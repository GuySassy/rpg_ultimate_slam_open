// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <ze/vio_frontend/elis_link_handles.hpp>

#include <algorithm>
#include <sstream>
#include <string>
#include <unordered_set>

#include <ze/common/logging.hpp>
#include <ze/vio_common/landmark_types.hpp>

namespace ze {

namespace {

inline bool canHealFromSlot(const LandmarkHandle h)
{
  return h.slot() < LandmarkTable::c_capacity_;
}

inline void ensure_slot_version_valid(LandmarkTable& landmarks, const uint32_t slot)
{
  auto& versions = landmarks.versions();
  auto& v = versions[slot];
  if (v < c_landmark_version_min_valid)
  {
    v = c_landmark_version_min_valid;
  }
}

std::string format_track_id_sample(const std::vector<int32_t>& ids, const size_t max_count)
{
  std::ostringstream oss;
  const size_t count = std::min(ids.size(), max_count);
  for (size_t i = 0; i < count; ++i)
  {
    if (i > 0)
    {
      oss << ',';
    }
    oss << ids[i];
  }
  if (ids.size() > count)
  {
    oss << "...";
  }
  return oss.str();
}

std::string format_handle_sample(const LandmarkHandles& handles, const size_t max_count)
{
  std::ostringstream oss;
  const size_t count = std::min(handles.size(), max_count);
  for (size_t i = 0; i < count; ++i)
  {
    if (i > 0)
    {
      oss << ',';
    }
    const LandmarkHandle& h = handles[i];
    oss << h.slot() << ":" << static_cast<int>(h.version())
        << "(" << static_cast<uint32_t>(h.handle) << ")";
  }
  if (handles.size() > count)
  {
    oss << "...";
  }
  return oss.str();
}

size_t count_inactive_slots(const LandmarkTable& landmarks)
{
  size_t count = 0;
  for (const auto& type : landmarks.types())
  {
    if (isLandmarkInactive(type))
    {
      ++count;
    }
  }
  return count;
}

}  // namespace

ElisLinkHandleStats ensureElisLandmarkHandles(
    const std::vector<int32_t>& track_ids,
    LandmarkTable& landmarks,
    std::unordered_map<int32_t, LandmarkHandle>& track_to_handle,
    const uint32_t nframe_seq,
    const Seed& seed_prototype)
{
  ElisLinkHandleStats stats;

  std::vector<int32_t> unique_track_ids;
  unique_track_ids.reserve(track_ids.size());
  std::unordered_set<int32_t> seen;
  seen.reserve(track_ids.size());
  for (const int32_t track_id : track_ids)
  {
    if (seen.insert(track_id).second)
    {
      unique_track_ids.push_back(track_id);
    }
  }
  stats.unique_track_ids = unique_track_ids.size();

  std::vector<int32_t> missing_track_ids;
  missing_track_ids.reserve(unique_track_ids.size());

  for (const int32_t track_id : unique_track_ids)
  {
    auto it = track_to_handle.find(track_id);
    if (it == track_to_handle.end())
    {
      missing_track_ids.push_back(track_id);
      ++stats.missing_new;
      continue;
    }

    LandmarkHandle& h = it->second;
    if (h.slot() >= LandmarkTable::c_capacity_)
    {
      missing_track_ids.push_back(track_id);
      ++stats.missing_invalid_handle;
      continue;
    }
    if (!isValidLandmarkHandle(h))
    {
      if (canHealFromSlot(h))
      {
        const uint32_t slot = h.slot();
        ensure_slot_version_valid(landmarks, slot);
        const LandmarkHandle candidate(slot, landmarks.versions()[slot]);
        if (isValidLandmarkHandle(candidate) && landmarks.isStored(candidate, true) &&
            !isLandmarkInactive(landmarks.typeAtSlot(slot)))
        {
          h = candidate;
          ++stats.healed_invalid_handle;
          continue;
        }
      }

      missing_track_ids.push_back(track_id);
      ++stats.missing_invalid_handle;
      continue;
    }

    if (isLandmarkInactive(landmarks.type(h)))
    {
      missing_track_ids.push_back(track_id);
      ++stats.missing_inactive;
      continue;
    }

    if (!landmarks.isStored(h, true))
    {
      // A valid handle with a mismatching version is unsafe to keep using: it
      // may alias a different landmark at the same slot. Reallocate.
      missing_track_ids.push_back(track_id);
      ++stats.missing_not_stored;
      continue;
    }
  }

  if (missing_track_ids.empty())
  {
    return stats;
  }

  const size_t available_slots = count_inactive_slots(landmarks);
  if (missing_track_ids.size() > available_slots)
  {
    const size_t dropped = missing_track_ids.size() - available_slots;
    static int shortage_log_count = 0;
    if (shortage_log_count++ < 5)
    {
      LOG(ERROR) << "[ELIS] Not enough landmark slots: missing=" << missing_track_ids.size()
                 << " available=" << available_slots
                 << " track_to_handle=" << track_to_handle.size()
                 << " capacity=" << LandmarkTable::c_capacity_
                 << " dropping=" << dropped;
    }
    missing_track_ids.resize(available_slots);
    if (missing_track_ids.empty())
    {
      return stats;
    }
  }

  const LandmarkHandles new_handles =
      landmarks.getNewLandmarkHandles(static_cast<uint32_t>(missing_track_ids.size()), nframe_seq);
  // Be explicit: we requested one handle per missing track id.
  stats.allocated_handles = missing_track_ids.size();
  const bool size_mismatch = new_handles.size() != missing_track_ids.size();
  if (size_mismatch)
  {
    LOG(ERROR) << "[ELIS] Landmark handle allocation mismatch: requested "
               << missing_track_ids.size() << " got " << new_handles.size();
    static int mismatch_log_count = 0;
    if (mismatch_log_count++ < 5)
    {
      const size_t dupes = track_ids.size() - unique_track_ids.size();
      LOG(ERROR) << "[ELIS] Handle allocation debug: input_tracks=" << track_ids.size()
                 << " unique=" << unique_track_ids.size()
                 << " dupes=" << dupes
                 << " missing=" << missing_track_ids.size()
                 << " track_to_handle=" << track_to_handle.size()
                 << " nframe_seq=" << nframe_seq
                 << " capacity=" << LandmarkTable::c_capacity_
                 << " sizeof(LandmarkHandle)=" << sizeof(LandmarkHandle)
                 << " sizeof(LandmarkHandles)=" << sizeof(LandmarkHandles)
                 << " sizeof(std::vector<int32_t>)=" << sizeof(std::vector<int32_t>)
                 << " missing_ids_sample=[" << format_track_id_sample(missing_track_ids, 8) << "]"
                 << " new_handles_sample=[" << format_handle_sample(new_handles, 8) << "]";
    }
  }

  const size_t assign_count = std::min(missing_track_ids.size(), new_handles.size());
  for (size_t i = 0; i < assign_count; ++i)
  {
    const int32_t track_id = missing_track_ids[i];
    LandmarkHandle h = new_handles[i];

    if (h.slot() >= LandmarkTable::c_capacity_)
    {
      LOG(ERROR) << "[ELIS] Invalid landmark slot " << h.slot()
                 << " (capacity=" << LandmarkTable::c_capacity_ << ", handle=" << h
                 << ") for track_id=" << track_id;
      ++stats.missing_invalid_handle;
      continue;
    }

    // Defensive: make sure the table's version is never below the minimum valid
    // value. If a slot's version becomes 0/1 (e.g. due to a stale build or
    // legacy handle packing), downstream validity checks and handle reuse will
    // fail and cause perpetual reallocations.
    if (canHealFromSlot(h))
    {
      const uint32_t slot = h.slot();
      ensure_slot_version_valid(landmarks, slot);
      h = LandmarkHandle(slot, landmarks.versions()[slot]);
      if (isLandmarkInactive(landmarks.typeAtSlot(slot)))
      {
        landmarks.typeAtSlot(slot) = LandmarkType::Seed;
      }
    }

    // Defensive: if the returned handle is not valid/stored (e.g. due to a bad
    // version), try to reconstruct the slot's current handle from the table.
    if (!isValidLandmarkHandle(h) || !landmarks.isStored(h, true))
    {
      if (canHealFromSlot(h))
      {
        const LandmarkHandle candidate = landmarks.getHandleAtSlot(h.slot());
        if (isValidLandmarkHandle(candidate) && landmarks.isStored(candidate, true) &&
            !isLandmarkInactive(landmarks.type(candidate)))
        {
          h = candidate;
        }
      }
    }

    track_to_handle[track_id] = h;
    landmarks.seed(h) = seed_prototype;
  }

  if (size_mismatch && new_handles.size() < missing_track_ids.size())
  {
    LOG(ERROR) << "[ELIS] Missing " << (missing_track_ids.size() - new_handles.size())
               << " handles; leaving those track_ids unmapped this frame.";
  }

  return stats;
}

}  // namespace ze
