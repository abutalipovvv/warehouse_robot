#include <stage_ros2/stage_node.hpp>

#include "stage_ros2/laser_scan_merger.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
#include <memory>

#define TOPIC_LASER "scan"
#define FRAME_LASER "laser"

using std::placeholders::_1;

StageNode::Vehicle::Ranger::Ranger(unsigned int id, Stg::ModelRanger *m,
                                   std::shared_ptr<Vehicle> &v)
    : initialized_(false), id_(id), model(m), vehicle(v) {}

unsigned int StageNode::Vehicle::Ranger::id() const { return id_; }
void StageNode::Vehicle::Ranger::init(bool add_id_to_topic) {
  if (initialized_)
    return;
  model->Subscribe();
  topic_name = vehicle->topic_name_space_ + TOPIC_LASER;
  frame_id = vehicle->frame_name_space_ + FRAME_LASER;
  if (add_id_to_topic) {
    topic_name += std::to_string(id());
    frame_id += std::to_string(id());
  }

  pub = vehicle->ros_node()->create_publisher<sensor_msgs::msg::LaserScan>(
      topic_name, 10);
  initialized_ = true;
}

bool StageNode::Vehicle::Ranger::prepare_msg() {
  if (msg) {
    return true;
  }
  const auto &sensors = model->GetSensors();
  if (sensors.empty()) {
    return false;
  }
  if (sensors.size() == 1) {
    const auto &sensor = sensors.front();
    if (sensor.ranges.empty() || sensor.sample_count < 2) {
      return false;
    }
    msg = std::make_shared<sensor_msgs::msg::LaserScan>();
    msg->angle_min = -sensor.fov / 2.0;
    msg->angle_max = sensor.fov / 2.0;
    msg->angle_increment =
        sensor.fov / static_cast<double>(sensor.sample_count - 1);
    msg->range_min = sensor.range.min;
    msg->range_max = sensor.range.max;
    msg->ranges.resize(sensor.ranges.size());
    msg->intensities.resize(sensor.ranges.size(), 0.0F);
    msg->header.frame_id = frame_id;
    return true;
  }

  double finest_increment = std::numeric_limits<double>::infinity();
  double range_min = std::numeric_limits<double>::infinity();
  double range_max = 0.0;
  for (const auto &sensor : sensors) {
    if (sensor.ranges.empty()) {
      return false;
    }
    if (sensor.sample_count > 1) {
      finest_increment =
          std::min(finest_increment,
                   sensor.fov / static_cast<double>(sensor.sample_count - 1));
    }
    range_min = std::min(range_min, sensor.range.min);
    range_max = std::max(range_max, sensor.range.max);
  }
  if (!std::isfinite(finest_increment) || finest_increment <= 0.0) {
    return false;
  }

  constexpr double kPi = 3.14159265358979323846;
  const auto intervals =
      std::max(1L, std::lround(2.0 * kPi / finest_increment));

  msg = std::make_shared<sensor_msgs::msg::LaserScan>();
  msg->angle_min = -kPi;
  msg->angle_max = kPi;
  msg->angle_increment = 2.0 * kPi / static_cast<double>(intervals);
  msg->range_min = range_min;
  msg->range_max = range_max;
  msg->ranges.resize(static_cast<size_t>(intervals) + 1, msg->range_max);
  msg->intensities.resize(msg->ranges.size(), 0.0F);
  msg->header.frame_id = frame_id;

  return true;
}

bool StageNode::Vehicle::Ranger::prepare_tf() {

  transform = std::make_shared<geometry_msgs::msg::TransformStamped>();

  Stg::Pose pose = model->GetPose();
  tf2::Quaternion quternion;
  quternion.setRPY(0.0, 0.0, pose.a);
  tf2::Transform txLaser = tf2::Transform(
      quternion,
      tf2::Vector3(pose.x, pose.y,
                   vehicle->positionmodel->GetGeom().size.z + pose.z));
  *transform = create_transform_stamped(txLaser, vehicle->node()->sim_time_,
                                        vehicle->frame_id_base_link_, frame_id);
  if (vehicle->node()->use_static_transformations_) {
    vehicle->tf_static_broadcaster_->sendTransform(*transform);
  }
  return true;
}

void StageNode::Vehicle::Ranger::publish_msg() {
  // Guard
  if (!initialized_)
    return;

  if (prepare_msg()) {
    msg->header.stamp = vehicle->node()->sim_time_;
    const auto &sensors = model->GetSensors();
    if (sensors.size() == 1) {
      const auto &sensor = sensors.front();
      for (size_t index = 0; index < sensor.ranges.size(); ++index) {
        msg->ranges[index] = sensor.ranges[index];
        msg->intensities[index] = index < sensor.intensities.size()
                                      ? sensor.intensities[index]
                                      : 0.0F;
      }
      pub->publish(*msg);
      return;
    }

    std::fill(msg->ranges.begin(), msg->ranges.end(), msg->range_max);
    std::fill(msg->intensities.begin(), msg->intensities.end(), 0.0F);

    for (const auto &sensor : sensors) {
      const stage_ros2::LaserScanMount mount{sensor.pose.x, sensor.pose.y,
                                             sensor.pose.a, sensor.range.max};
      stage_ros2::merge_laser_scan(
          mount, sensor.bearings, sensor.ranges, sensor.intensities,
          msg->angle_min, msg->angle_increment, msg->ranges, msg->intensities);
    }
    pub->publish(*msg);
  }
}

void StageNode::Vehicle::Ranger::publish_tf() {
  if (prepare_tf()) {

    if (vehicle->node()->use_static_transformations_) {
      return;
    }

    // use tf publsiher only if use_static_transformations_ is false
    transform->header.stamp = vehicle->node()->sim_time_;
    vehicle->tf_broadcaster_->sendTransform(*transform);
  }
}
