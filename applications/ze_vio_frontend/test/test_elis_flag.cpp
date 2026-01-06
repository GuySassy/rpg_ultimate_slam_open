// This code is released only for internal evaluation.
// No commercial use, editing or copying is allowed.

#include <gtest/gtest.h>

#include <ze/vio_frontend/frontend_base.hpp>
#include <ze/vio_frontend/frontend_gflags.hpp>
#include <ze/vio_common/frame.hpp>

#ifdef ELIS_LINK_ENABLED
#include <elisUltimateLink/frame_populator.hpp>
#include <elisUltimateLink/landmark_mapping.hpp>
#include <elisUltimateLink/keypoint_service.hpp>
#include <elisUltimateLink/synchronizer_bridge.hpp>
#endif

namespace ze {

class FrontendElisFlagTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    FLAGS_vio_use_elis_link = false;
  }
};

#ifdef ELIS_LINK_ENABLED

TEST_F(FrontendElisFlagTest, FlagOffDoesNotTouchFrame)
{
  FrontendBase frontend;
  FLAGS_vio_use_elis_link = false;

  // Minimal frame with capacity for 2 features.
  Image8uC1::Ptr img = std::make_shared<Image8uC1>(Eigen::Vector2i(10, 10));
  Frame frame(img, 2);
  frame.ensureFeatureCapacity(2);
  frame.num_features_ = 0;

  EXPECT_EQ(frame.num_features_, 0);
}

TEST_F(FrontendElisFlagTest, FlagOnPopulatesFrameFromPacket)
{
  FrontendBase frontend;
  FLAGS_vio_use_elis_link = true;

  // Build a synthetic packet.
  elis_link::KeypointPacket pkt;
  pkt.x_px = {1.f, 2.f};
  pkt.y_px = {3.f, 4.f};
  pkt.strength = {0.5f, 0.6f};
  pkt.track_ids = {5, 6};
  pkt.descriptor_stride = 32;
  pkt.descriptors.resize(pkt.descriptor_stride * pkt.x_px.size());
  for (std::size_t j = 0; j < pkt.descriptor_stride; ++j)
  {
    pkt.descriptors[j] = static_cast<uint8_t>(j);
    pkt.descriptors[pkt.descriptor_stride + j] = static_cast<uint8_t>(j + 1);
  }

  elis_link::FramePopulationConfig cfg;
  elis_link::FrameFeatureData data = elis_link::make_frame_feature_data(pkt, cfg);

  Image8uC1::Ptr img = std::make_shared<Image8uC1>(Eigen::Vector2i(10, 10));
  Frame frame(img, 2);
  frame.ensureFeatureCapacity(2);

  // Populate the frame manually using the data produced above.
  frame.px_vec_.leftCols(2) = data.keypoints_px;
  frame.score_vec_.head(2) = data.strengths;
  frame.angle_vec_.head(2).setZero();
  frame.level_vec_.head(2).setZero();
  frame.type_vec_.head(2).setZero();
  frame.descriptors_.resize(data.descriptors.rows(), 2);
  frame.descriptors_.leftCols(2) = data.descriptors;
  frame.num_features_ = 2;

  EXPECT_EQ(frame.num_features_, 2);
  EXPECT_FLOAT_EQ(frame.px_vec_(0, 0), 1.f);
  EXPECT_FLOAT_EQ(frame.px_vec_(1, 1), 4.f);
  EXPECT_EQ(frame.descriptors_.rows(), 64);
  EXPECT_EQ(frame.descriptors_.cols(), 2);
  EXPECT_EQ(frame.descriptors_(0, 0), 0);
  EXPECT_EQ(frame.descriptors_(31, 0), 31);
  EXPECT_EQ(frame.descriptors_(32, 0), 0);
}

#endif  // ELIS_LINK_ENABLED

}  // namespace ze
