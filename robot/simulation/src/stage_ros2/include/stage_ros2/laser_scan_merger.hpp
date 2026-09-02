#ifndef STAGE_ROS2__LASER_SCAN_MERGER_HPP_
#define STAGE_ROS2__LASER_SCAN_MERGER_HPP_

#include <vector>

#include "stage_ros2/visibility.h"

namespace stage_ros2 {

struct LaserScanMount {
  double x;
  double y;
  double yaw;
  double range_max;
};

STAGE_ROS2_PACKAGE_PUBLIC
void merge_laser_scan(const LaserScanMount &mount,
                      const std::vector<double> &bearings,
                      const std::vector<double> &ranges,
                      const std::vector<double> &intensities, double angle_min,
                      double angle_increment, std::vector<float> &merged_ranges,
                      std::vector<float> &merged_intensities);

} // namespace stage_ros2

#endif // STAGE_ROS2__LASER_SCAN_MERGER_HPP_
