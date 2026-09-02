#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "stage_ros2/laser_scan_merger.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kOneDegree = kPi / 180.0;

std::vector<float> empty_scan() { return std::vector<float>(361, 40.0F); }

} // namespace

TEST(LaserScanMerger, KeepsFrontScannerInVirtualLaserFrame) {
  auto merged_ranges = empty_scan();
  std::vector<float> merged_intensities(merged_ranges.size(), 0.0F);

  stage_ros2::merge_laser_scan({0.0, 0.0, 0.0, 40.0}, {0.0}, {2.0}, {0.5}, -kPi,
                               kOneDegree, merged_ranges, merged_intensities);

  EXPECT_FLOAT_EQ(merged_ranges[180], 2.0F);
  EXPECT_FLOAT_EQ(merged_intensities[180], 0.5F);
}

TEST(LaserScanMerger, ProjectsRearScannerReturnsIntoFrontFrame) {
  auto merged_ranges = empty_scan();
  std::vector<float> merged_intensities(merged_ranges.size(), 0.0F);

  stage_ros2::merge_laser_scan({-1.0, 0.0, kPi, 40.0}, {0.0}, {2.0}, {0.75},
                               -kPi, kOneDegree, merged_ranges,
                               merged_intensities);

  EXPECT_NEAR(merged_ranges[360], 3.0F, 1e-6F);
  EXPECT_FLOAT_EQ(merged_intensities[360], 0.75F);
}

TEST(LaserScanMerger, IgnoresNoReturnAtPhysicalRangeLimit) {
  auto merged_ranges = empty_scan();
  std::vector<float> merged_intensities(merged_ranges.size(), 0.0F);

  stage_ros2::merge_laser_scan({-0.74011, -0.50011, kPi, 40.0}, {0.0}, {40.0},
                               {0.0}, -kPi, kOneDegree, merged_ranges,
                               merged_intensities);

  for (const float range : merged_ranges) {
    EXPECT_FLOAT_EQ(range, 40.0F);
  }
}

TEST(LaserScanMerger, RetainsNearestReturnInAnOutputBin) {
  auto merged_ranges = empty_scan();
  std::vector<float> merged_intensities(merged_ranges.size(), 0.0F);

  stage_ros2::merge_laser_scan({0.0, 0.0, 0.0, 40.0}, {0.0}, {3.0}, {0.25},
                               -kPi, kOneDegree, merged_ranges,
                               merged_intensities);
  stage_ros2::merge_laser_scan({0.0, 0.0, 0.0, 40.0}, {0.0}, {1.5}, {0.9}, -kPi,
                               kOneDegree, merged_ranges, merged_intensities);

  EXPECT_FLOAT_EQ(merged_ranges[180], 1.5F);
  EXPECT_FLOAT_EQ(merged_intensities[180], 0.9F);
}
