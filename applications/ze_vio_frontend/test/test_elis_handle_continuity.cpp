#include <gtest/gtest.h>

#include <ze/vio_frontend/elis_link_handles.hpp>

TEST(ElisLinkHandles, AllocatesAndKeepsStableHandles)
{
  ze::LandmarkTable landmarks;
  std::unordered_map<int32_t, ze::LandmarkHandle> track_to_handle;
  ze::Seed seed;
  seed << 0.1, 0.2, 1.0, 1.0;

  const std::vector<int32_t> track_ids = {1, 2, 3};
  const ze::ElisLinkHandleStats s0 =
      ze::ensureElisLandmarkHandles(track_ids, landmarks, track_to_handle, 0u, seed);

  EXPECT_EQ(s0.unique_track_ids, 3u);
  EXPECT_EQ(s0.allocated_handles, 3u);
  ASSERT_EQ(track_to_handle.size(), 3u);

  const ze::LandmarkHandle h1 = track_to_handle.at(1);
  const ze::LandmarkHandle h2 = track_to_handle.at(2);
  const ze::LandmarkHandle h3 = track_to_handle.at(3);
  EXPECT_TRUE(ze::isValidLandmarkHandle(h1));
  EXPECT_TRUE(ze::isValidLandmarkHandle(h2));
  EXPECT_TRUE(ze::isValidLandmarkHandle(h3));
  EXPECT_NE(h1, h2);
  EXPECT_NE(h1, h3);
  EXPECT_NE(h2, h3);

  const ze::ElisLinkHandleStats s1 =
      ze::ensureElisLandmarkHandles(track_ids, landmarks, track_to_handle, 1u, seed);
  EXPECT_EQ(s1.unique_track_ids, 3u);
  EXPECT_EQ(s1.allocated_handles, 0u);
  EXPECT_EQ(track_to_handle.at(1), h1);
  EXPECT_EQ(track_to_handle.at(2), h2);
  EXPECT_EQ(track_to_handle.at(3), h3);
}

TEST(ElisLinkHandles, HealsInvalidHandleVersion)
{
  ze::LandmarkTable landmarks;
  std::unordered_map<int32_t, ze::LandmarkHandle> track_to_handle;
  ze::Seed seed;
  seed << 0.1, 0.2, 1.0, 1.0;

  const std::vector<int32_t> track_ids = {42};
  (void)ze::ensureElisLandmarkHandles(track_ids, landmarks, track_to_handle, 0u, seed);
  const ze::LandmarkHandle original = track_to_handle.at(42);
  ASSERT_TRUE(ze::isValidLandmarkHandle(original));

  ze::LandmarkHandle corrupted = original;
  corrupted.setVersion(ze::c_landmark_version_invalid);
  track_to_handle[42] = corrupted;
  ASSERT_FALSE(ze::isValidLandmarkHandle(track_to_handle.at(42)));

  const ze::ElisLinkHandleStats s1 =
      ze::ensureElisLandmarkHandles(track_ids, landmarks, track_to_handle, 1u, seed);
  EXPECT_EQ(s1.healed_invalid_handle, 1u);
  EXPECT_EQ(s1.allocated_handles, 0u);
  EXPECT_EQ(track_to_handle.at(42), original);
}

TEST(ElisLinkHandles, DeduplicatesTrackIds)
{
  ze::LandmarkTable landmarks;
  std::unordered_map<int32_t, ze::LandmarkHandle> track_to_handle;
  ze::Seed seed;
  seed << 0.1, 0.2, 1.0, 1.0;

  const std::vector<int32_t> track_ids = {7, 7, 7, 8, 8};
  const ze::ElisLinkHandleStats s0 =
      ze::ensureElisLandmarkHandles(track_ids, landmarks, track_to_handle, 0u, seed);
  EXPECT_EQ(s0.unique_track_ids, 2u);
  EXPECT_EQ(s0.allocated_handles, 2u);
  ASSERT_EQ(track_to_handle.size(), 2u);
  EXPECT_TRUE(ze::isValidLandmarkHandle(track_to_handle.at(7)));
  EXPECT_TRUE(ze::isValidLandmarkHandle(track_to_handle.at(8)));
  EXPECT_NE(track_to_handle.at(7), track_to_handle.at(8));
}

TEST(ElisLinkHandles, AllocatesWhenSlotIsOutOfRange)
{
  ze::LandmarkTable landmarks;
  std::unordered_map<int32_t, ze::LandmarkHandle> track_to_handle;
  ze::Seed seed;
  seed << 0.1, 0.2, 1.0, 1.0;

  ze::LandmarkHandle bad;
  bad.setSlot(ze::LandmarkTable::c_capacity_ + 10u);
  bad.setVersion(ze::c_landmark_version_deleted);
  track_to_handle[9] = bad;

  const std::vector<int32_t> track_ids = {9};
  const ze::ElisLinkHandleStats s0 =
      ze::ensureElisLandmarkHandles(track_ids, landmarks, track_to_handle, 0u, seed);
  EXPECT_EQ(s0.missing_invalid_handle, 1u);
  EXPECT_EQ(s0.allocated_handles, 1u);
  EXPECT_TRUE(ze::isValidLandmarkHandle(track_to_handle.at(9)));
  EXPECT_TRUE(landmarks.isStored(track_to_handle.at(9), true));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
