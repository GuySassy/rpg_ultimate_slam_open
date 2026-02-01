#include <ze/common/test_entrypoint.hpp>
#include <ze/data_provider/rt1060_imu_assigner.hpp>

namespace ze {

TEST(Rt1060ImuAssignerTest, NonOverlappingConsumption)
{
  Rt1060ImuAssigner assigner(0, 0);
  Vector3 acc = Vector3::Zero();
  Vector3 gyr = Vector3::Zero();

  assigner.addSample(0, acc, gyr);
  assigner.addSample(10, acc, gyr);
  assigner.addSample(20, acc, gyr);
  assigner.addSample(30, acc, gyr);
  assigner.addSample(40, acc, gyr);

  Rt1060ImuAssigned first = assigner.assign(10, 30);
  ASSERT_EQ(first.stamps_ns.size(), 4u);
  EXPECT_EQ(first.stamps_ns[0], 10);
  EXPECT_EQ(first.stamps_ns[1], 20);
  EXPECT_EQ(first.stamps_ns[2], 30);
  EXPECT_EQ(first.stamps_ns[3], 40);

  ASSERT_EQ(assigner.bufferSize(), 2u);

  Rt1060ImuAssigned second = assigner.assign(30, 40);
  ASSERT_EQ(second.stamps_ns.size(), 2u);
  EXPECT_EQ(second.stamps_ns[0], 30);
  EXPECT_EQ(second.stamps_ns[1], 40);

  ASSERT_EQ(assigner.bufferSize(), 1u);
  EXPECT_EQ(assigner.assign(40, 40).stamps_ns.size(), 1u);
}

TEST(Rt1060ImuAssignerTest, BracketingSamplesNotConsumed)
{
  Rt1060ImuAssigner assigner(5, 5);
  Vector3 acc = Vector3::Zero();
  Vector3 gyr = Vector3::Zero();

  assigner.addSample(0, acc, gyr);
  assigner.addSample(5, acc, gyr);
  assigner.addSample(10, acc, gyr);
  assigner.addSample(15, acc, gyr);

  Rt1060ImuAssigned first = assigner.assign(5, 10);
  ASSERT_EQ(first.stamps_ns.size(), 3u);
  EXPECT_EQ(first.stamps_ns[0], 5);
  EXPECT_EQ(first.stamps_ns[1], 10);
  EXPECT_EQ(first.stamps_ns[2], 15);

  ASSERT_EQ(assigner.bufferSize(), 2u);

  Rt1060ImuAssigned second = assigner.assign(10, 15);
  ASSERT_EQ(second.stamps_ns.size(), 2u);
  EXPECT_EQ(second.stamps_ns[0], 10);
  EXPECT_EQ(second.stamps_ns[1], 15);
}

TEST(Rt1060ImuAssignerTest, OutOfOrderInsertSorted)
{
  Rt1060ImuAssigner assigner(0, 0);
  Vector3 acc = Vector3::Zero();
  Vector3 gyr = Vector3::Zero();

  assigner.addSample(20, acc, gyr);
  assigner.addSample(0, acc, gyr);
  assigner.addSample(10, acc, gyr);

  Rt1060ImuAssigned out = assigner.assign(0, 20);
  ASSERT_EQ(out.stamps_ns.size(), 3u);
  EXPECT_EQ(out.stamps_ns[0], 0);
  EXPECT_EQ(out.stamps_ns[1], 10);
  EXPECT_EQ(out.stamps_ns[2], 20);
}

} // namespace ze

ZE_UNITTEST_ENTRYPOINT
