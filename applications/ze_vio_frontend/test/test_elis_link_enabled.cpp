#include <gtest/gtest.h>

#include <elisUltimateLink/frame_populator.hpp>
#include <elisUltimateLink/keypoint_service.hpp>

// Smoke-test that the optional ElisUltimateLink bridge is present and usable.
TEST(ElisLinkEnabled, PacketToFrameFeatureData)
{
#ifdef ELIS_LINK_ENABLED
  elis_link::KeypointPacket pkt;
  pkt.x_px = {1.f, 2.f};
  pkt.y_px = {3.f, 4.f};
  pkt.strength = {0.5f, 0.6f};
  pkt.track_ids = {7, 8};
  pkt.descriptor_stride = 4;
  pkt.descriptors = {1, 2, 3, 4, 5, 6, 7, 8};

  elis_link::FramePopulationConfig cfg;
  cfg.descriptor_length = 8;
  elis_link::FrameFeatureData data = elis_link::make_frame_feature_data(pkt, cfg);

  EXPECT_EQ(data.keypoints_px.cols(), 2);
  EXPECT_EQ(data.descriptor_stride, cfg.descriptor_length);
  EXPECT_EQ(data.descriptors.rows(), cfg.descriptor_length);
  EXPECT_EQ(data.descriptors.cols(), 2);
  EXPECT_EQ(data.track_ids.size(), 2u);
#else
  GTEST_SKIP() << "ELIS_LINK_ENABLED not set; skipping.";
#endif
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
