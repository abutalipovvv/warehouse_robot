#include "stage_ros2/laser_scan_merger.hpp"

#include <algorithm>
#include <cmath>

void stage_ros2::merge_laser_scan(const LaserScanMount &mount,
                                  const std::vector<double> &bearings,
                                  const std::vector<double> &ranges,
                                  const std::vector<double> &intensities,
                                  double angle_min, double angle_increment,
                                  std::vector<float> &merged_ranges,
                                  std::vector<float> &merged_intensities) {
  if (angle_increment <= 0.0 || merged_ranges.empty() ||
      merged_ranges.size() != merged_intensities.size()) {
    return;
  }

  const size_t sample_count = std::min(bearings.size(), ranges.size());
  const double cos_yaw = std::cos(mount.yaw);
  const double sin_yaw = std::sin(mount.yaw);

  for (size_t index = 0; index < sample_count; ++index) {
    const double sensor_range = ranges[index];
    // Stage reports range_max when a ray has no return. Projecting that
    // endpoint from an offset scanner would create a false obstacle ring.
    if (!std::isfinite(sensor_range) || sensor_range < 0.0 ||
        sensor_range >= mount.range_max) {
      continue;
    }

    const double sensor_x = sensor_range * std::cos(bearings[index]);
    const double sensor_y = sensor_range * std::sin(bearings[index]);
    const double laser_x = mount.x + cos_yaw * sensor_x - sin_yaw * sensor_y;
    const double laser_y = mount.y + sin_yaw * sensor_x + cos_yaw * sensor_y;
    const double projected_range = std::hypot(laser_x, laser_y);
    const double projected_angle = std::atan2(laser_y, laser_x);
    const long output_index =
        std::lround((projected_angle - angle_min) / angle_increment);

    if (output_index < 0 ||
        static_cast<size_t>(output_index) >= merged_ranges.size() ||
        projected_range >= merged_ranges[output_index]) {
      continue;
    }

    merged_ranges[output_index] = static_cast<float>(projected_range);
    merged_intensities[output_index] =
        index < intensities.size() ? static_cast<float>(intensities[index])
                                   : 0.0F;
  }
}
