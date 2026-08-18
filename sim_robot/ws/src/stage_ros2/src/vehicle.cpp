#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <memory>
#include <filesystem>

#define TOPIC_TF "tf"
#define TOPIC_TF_STATIC "tf_static"
#define TOPIC_ODOM "odom"
#define TOPIC_GROUND_TRUTH "ground_truth"
#define TOPIC_IMU "imu"
#define TOPIC_CMD_VEL "cmd_vel"

using std::placeholders::_1;

StageNode::Vehicle::Vehicle(
    size_t id, const Stg::Pose &pose, const std::string &name,
    StageNode *node)
    : initialized_(false),
      id_(id),
      initial_pose_(pose),
      name_(name),
      node_(node),
      rng_(static_cast<std::mt19937::result_type>(0x6d2b79f5u + (id * 2654435761u))),
      imu_initialized_(false),
      imu_yaw_bias_(0.0),
      imu_angular_velocity_bias_(0.0),
      odom_yaw_offset_(0.0),
      odom_world_yaw_offset_(0.0),
      imu_odom_initialized_(false),
      imu_odom_x_(0.0),
      imu_odom_y_(0.0),
      imu_odom_yaw_(0.0)
{
}

double StageNode::Vehicle::sample_noise(double stddev)
{
  if (stddev <= 0.0) {
    return 0.0;
  }
  std::normal_distribution<double> distribution(0.0, stddev);
  return distribution(rng_);
}

void StageNode::Vehicle::ensure_imu_initialized()
{
  if (imu_initialized_) {
    return;
  }
  imu_yaw_bias_ = sample_noise(node_->imu_yaw_noise_stddev_);
  imu_angular_velocity_bias_ = sample_noise(node_->imu_angular_velocity_noise_stddev_ * 0.2);
  imu_initialized_ = true;
}

size_t StageNode::Vehicle::id() const
{
  return id_;
}
void StageNode::Vehicle::soft_reset()
{
  positionmodel->SetPose(this->initial_pose_);
  reset_odom();
}

void StageNode::Vehicle::reset_odom()
{
  positionmodel->SetSpeed(0.0, 0.0, 0.0);
  positionmodel->SetOdom(Stg::Pose(0.0, 0.0, 0.0, 0.0));
  positionmodel->SetStall(false);
  ensure_imu_initialized();

  const Stg::Pose gpose = positionmodel->GetGlobalPose();
  odom_yaw_offset_ = Stg::normalize(gpose.a + imu_yaw_bias_);
  odom_world_yaw_offset_ = gpose.a;
  imu_odom_initialized_ = true;
  imu_odom_x_ = 0.0;
  imu_odom_y_ = 0.0;
  imu_odom_yaw_ = 0.0;
  time_last_pose_update_ = rclcpp::Time(0, 0);
  time_last_cmd_received_ = node_->sim_time_;
  timeout_cmd_ = rclcpp::Time(0, 0);
  global_pose_.reset();
  body_velocity_.reset();
  msg_odom_ = nav_msgs::msg::Odometry();
  msg_imu_ = sensor_msgs::msg::Imu();

  // Planar odometry covariance. Z/roll/pitch are intentionally unobserved.
  msg_odom_.pose.covariance[0] = 0.0025;
  msg_odom_.pose.covariance[7] = 0.0025;
  msg_odom_.pose.covariance[14] = 1e6;
  msg_odom_.pose.covariance[21] = 1e6;
  msg_odom_.pose.covariance[28] = 1e6;
  msg_odom_.pose.covariance[35] =
    node_->imu_yaw_noise_stddev_ * node_->imu_yaw_noise_stddev_;
  msg_odom_.twist.covariance[0] = 0.0004;
  msg_odom_.twist.covariance[7] = 0.0004;
  msg_odom_.twist.covariance[14] = 1e6;
  msg_odom_.twist.covariance[21] = 1e6;
  msg_odom_.twist.covariance[28] = 1e6;
  msg_odom_.twist.covariance[35] =
    node_->imu_angular_velocity_noise_stddev_ *
    node_->imu_angular_velocity_noise_stddev_;
}

const std::string &StageNode::Vehicle::name() const
{
  return name_;
}

void StageNode::Vehicle::init(bool use_topic_prefixes, bool use_one_tf_tree)
{
  if (initialized_)
    return;

  time_last_pose_update_ = rclcpp::Time(0, 0);
  time_last_cmd_received_ = rclcpp::Time(0, 0);
  timeout_cmd_ = rclcpp::Time(0, 0);

  topic_name_space_ = std::string();
  frame_name_space_ = std::string();
  if (use_topic_prefixes == true)
  {
    topic_name_space_ = name() + "/";
  }
  if (use_one_tf_tree)
  {
    frame_name_space_ = name() + "/";
    topic_name_tf_ = std::string("/") + TOPIC_TF;
    topic_name_tf_static_ =  std::string("/") + TOPIC_TF_STATIC;
  } else {
    topic_name_tf_ = topic_name_space_ + TOPIC_TF;
    topic_name_tf_static_ = topic_name_space_ + TOPIC_TF_STATIC;
  }

  frame_id_base_link_ = frame_name_space_ + node_->frame_id_base_link_name_;
  frame_id_odom_ = frame_name_space_ + node_->frame_id_odom_name_;
  frame_id_world_ = frame_name_space_ + node_->frame_id_world_name_;
  frame_id_imu_ = frame_name_space_ + node_->frame_id_imu_name_;

  topic_name_odom_ = topic_name_space_ + TOPIC_ODOM;
  topic_name_ground_truth_ = topic_name_space_ + TOPIC_GROUND_TRUTH;
  topic_name_imu_ = topic_name_space_ + TOPIC_IMU;
  topic_name_cmd_ = topic_name_space_ + TOPIC_CMD_VEL;

  tf_static_broadcaster_ = std::make_shared<stage_ros2::StaticTransformBroadcaster>(node_, topic_name_tf_static_.c_str());
  tf_broadcaster_ = std::make_shared<stage_ros2::TransformBroadcaster>(node_, topic_name_tf_.c_str());

  pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>(topic_name_odom_, 10);
  pub_ground_truth_ =
      node_->create_publisher<nav_msgs::msg::Odometry>(topic_name_ground_truth_, 10);
  if (node_->publish_imu_) {
    pub_imu_ = node_->create_publisher<sensor_msgs::msg::Imu>(topic_name_imu_, 10);
  }
  sub_cmd_ =
      node_->create_subscription<geometry_msgs::msg::Twist>(
          topic_name_cmd_, 10,
          std::bind(&StageNode::Vehicle::callback_cmd, this, _1));

  positionmodel->Subscribe();
  // Stage initializes est_pose from the model, but the IMU heading offset
  // must be initialized explicitly as well. Without this, a non-zero spawn
  // yaw leaks directly into odom->base_link on the first frame.
  reset_odom();

  for (std::shared_ptr<Ranger> ranger : rangers_)
  {
    ranger->init(rangers_.size() > 1);
  }

  for (std::shared_ptr<Camera> camera : cameras_)
  {
    camera->init(rangers_.size() > 1);
  }
  initialized_ = true;
}

void StageNode::Vehicle::publish_msg()
{
  // Guard
  if (!initialized_)
    return;

  Stg::Pose gpose = positionmodel->GetGlobalPose();
  tf2::Quaternion q_gpose;
  q_gpose.setRPY(0.0, 0.0, gpose.a);
  tf2::Transform gt(q_gpose, tf2::Vector3(gpose.x, gpose.y, 0.0));

  double dt = 0.0;
  if (time_last_pose_update_ != rclcpp::Time(0, 0)) {
    dt = (node_->sim_time_ - time_last_pose_update_).seconds();
  }

  Stg::Velocity gvel(0, 0, 0, 0);
  double delta_world_x = 0.0;
  double delta_world_y = 0.0;
  bool has_pose_delta = false;
  if (global_pose_)
  {
    if (dt > 0)
    {
      delta_world_x = gpose.x - global_pose_->x;
      delta_world_y = gpose.y - global_pose_->y;
      has_pose_delta = true;
      gvel = Stg::Velocity(
          delta_world_x / dt,
          delta_world_y / dt,
          (gpose.z - global_pose_->z) / dt,
          Stg::normalize(gpose.a - global_pose_->a) / dt);
    }
    *global_pose_ = gpose;
  }
  else
  {
    // There are no previous readings, adding current pose...
    global_pose_ = std::make_shared<Stg::Pose>(gpose);
  }

  ensure_imu_initialized();

  // ModelPosition::GetVelocity() returns the requested velocity even when
  // Stage has stopped the body on a collision. Derive sensor motion from the
  // actual pose delta so odometry and IMU stop with the physical robot.
  const double heading_cos = std::cos(gpose.a);
  const double heading_sin = std::sin(gpose.a);
  const Stg::Velocity body_motion(
    (gvel.x * heading_cos) + (gvel.y * heading_sin),
    (-gvel.x * heading_sin) + (gvel.y * heading_cos),
    gvel.z,
    gvel.a);

  double linear_acceleration_x = 0.0;
  double linear_acceleration_y = 0.0;
  if (body_velocity_ && dt > 0.0) {
    // REP-103 body-frame acceleration: derivative in a rotating frame needs
    // the omega x velocity term. This matters while the robot follows arcs.
    linear_acceleration_x =
      ((body_motion.x - body_velocity_->x) / dt) - (body_motion.a * body_motion.y);
    linear_acceleration_y =
      ((body_motion.y - body_velocity_->y) / dt) + (body_motion.a * body_motion.x);
  }
  if (body_velocity_) {
    *body_velocity_ = body_motion;
  } else {
    body_velocity_ = std::make_shared<Stg::Velocity>(body_motion);
  }

  const double imu_yaw = Stg::normalize(gpose.a + imu_yaw_bias_);
  const double relative_imu_yaw = Stg::normalize(imu_yaw - odom_yaw_offset_);
  if (node_->use_imu_for_odom_yaw_) {
    if (!imu_odom_initialized_) {
      imu_odom_x_ = 0.0;
      imu_odom_y_ = 0.0;
      imu_odom_yaw_ = relative_imu_yaw;
      imu_odom_initialized_ = true;
    } else {
      if (has_pose_delta) {
        // Express the collision-constrained world displacement in the odom
        // frame fixed at reset. This keeps translation and IMU heading in one
        // coordinate system without integrating a stale velocity command.
        const double origin_cos = std::cos(odom_world_yaw_offset_);
        const double origin_sin = std::sin(odom_world_yaw_offset_);
        imu_odom_x_ +=
          (delta_world_x * origin_cos) + (delta_world_y * origin_sin);
        imu_odom_y_ +=
          (-delta_world_x * origin_sin) + (delta_world_y * origin_cos);
      }
      imu_odom_yaw_ = relative_imu_yaw;
    }
  } else {
    imu_odom_x_ = positionmodel->est_pose.x;
    imu_odom_y_ = positionmodel->est_pose.y;
    imu_odom_yaw_ = positionmodel->est_pose.a;
    imu_odom_initialized_ = true;
  }
  const double imu_angular_velocity_z =
    body_motion.a + imu_angular_velocity_bias_ +
    sample_noise(node_->imu_angular_velocity_noise_stddev_);

  msg_imu_.header.stamp = node_->sim_time_;
  msg_imu_.header.frame_id = frame_id_imu_;
  msg_imu_.orientation = createQuaternionMsgFromYaw(imu_yaw);
  msg_imu_.angular_velocity.x = 0.0;
  msg_imu_.angular_velocity.y = 0.0;
  msg_imu_.angular_velocity.z = imu_angular_velocity_z;
  msg_imu_.linear_acceleration.x =
    linear_acceleration_x + sample_noise(node_->imu_linear_acceleration_noise_stddev_);
  msg_imu_.linear_acceleration.y =
    linear_acceleration_y + sample_noise(node_->imu_linear_acceleration_noise_stddev_);
  // REP-103/145 ENU specific force: a stationary, upright accelerometer
  // measures +g on Z. robot_localization is configured to remove gravity.
  msg_imu_.linear_acceleration.z =
    9.80665 + sample_noise(node_->imu_linear_acceleration_noise_stddev_);
  msg_imu_.orientation_covariance.fill(0.0);
  msg_imu_.angular_velocity_covariance.fill(0.0);
  msg_imu_.linear_acceleration_covariance.fill(0.0);
  msg_imu_.orientation_covariance[0] = 1e6;
  msg_imu_.orientation_covariance[4] = 1e6;
  msg_imu_.orientation_covariance[8] =
    node_->imu_yaw_noise_stddev_ * node_->imu_yaw_noise_stddev_;
  msg_imu_.angular_velocity_covariance[0] = 1e6;
  msg_imu_.angular_velocity_covariance[4] = 1e6;
  msg_imu_.angular_velocity_covariance[8] =
    node_->imu_angular_velocity_noise_stddev_ * node_->imu_angular_velocity_noise_stddev_;
  msg_imu_.linear_acceleration_covariance[0] =
    node_->imu_linear_acceleration_noise_stddev_ * node_->imu_linear_acceleration_noise_stddev_;
  msg_imu_.linear_acceleration_covariance[4] =
    node_->imu_linear_acceleration_noise_stddev_ * node_->imu_linear_acceleration_noise_stddev_;
  msg_imu_.linear_acceleration_covariance[8] =
    node_->imu_linear_acceleration_noise_stddev_ *
    node_->imu_linear_acceleration_noise_stddev_;

  if (node_->publish_imu_ && pub_imu_) {
    pub_imu_->publish(msg_imu_);
  }

  // Publish one self-consistent planar odometry estimate.
  msg_odom_.pose.pose.position.x = imu_odom_x_;
  msg_odom_.pose.pose.position.y = imu_odom_y_;
  msg_odom_.pose.pose.orientation = createQuaternionMsgFromYaw(imu_odom_yaw_);
  msg_odom_.twist.twist.linear.x = body_motion.x;
  msg_odom_.twist.twist.linear.y = body_motion.y;
  // Odometry twist describes the simulated chassis motion. Keep IMU noise on
  // /imu only; leaking gyro noise into /odom makes a stationary robot appear
  // to move and destabilizes status/localization consumers.
  msg_odom_.twist.twist.angular.z = body_motion.a;
  msg_odom_.header.frame_id = frame_id_odom_;
  msg_odom_.header.stamp = node_->sim_time_;
  msg_odom_.child_frame_id = frame_id_base_link_;

  pub_odom_->publish(msg_odom_);

  if (node_->publish_ground_truth_ && pub_ground_truth_) {
    nav_msgs::msg::Odometry ground_truth_msg;
    ground_truth_msg.pose.pose.position.x = gt.getOrigin().x();
    ground_truth_msg.pose.pose.position.y = gt.getOrigin().y();
    ground_truth_msg.pose.pose.position.z = gt.getOrigin().z();
    ground_truth_msg.pose.pose.orientation.x = gt.getRotation().x();
    ground_truth_msg.pose.pose.orientation.y = gt.getRotation().y();
    ground_truth_msg.pose.pose.orientation.z = gt.getRotation().z();
    ground_truth_msg.pose.pose.orientation.w = gt.getRotation().w();
    ground_truth_msg.twist.twist.linear.x = gvel.x;
    ground_truth_msg.twist.twist.linear.y = gvel.y;
    ground_truth_msg.twist.twist.linear.z = gvel.z;
    ground_truth_msg.twist.twist.angular.z = gvel.a;
    ground_truth_msg.header.frame_id = frame_id_world_;
    ground_truth_msg.header.stamp = node_->sim_time_;
    pub_ground_truth_->publish(ground_truth_msg);
  }
  time_last_pose_update_ = node_->sim_time_;
}
void StageNode::Vehicle::publish_tf()
{

  // broadcast odometry transform
  tf2::Quaternion quaternion = tf2::Quaternion(
      msg_odom_.pose.pose.orientation.x,
      msg_odom_.pose.pose.orientation.y,
      msg_odom_.pose.pose.orientation.z,
      msg_odom_.pose.pose.orientation.w);
  tf2::Transform transform(quaternion,
                           tf2::Vector3(msg_odom_.pose.pose.position.x, msg_odom_.pose.pose.position.y, 0.0));
  tf_broadcaster_->sendTransform(
      create_transform_stamped(
          transform, node_->sim_time_,
          frame_id_odom_,
          frame_id_base_link_));
}

void StageNode::Vehicle::check_watchdog_timeout()
{

  if ((timeout_cmd_ != rclcpp::Time(0, 0)) && (node_->sim_time_ > timeout_cmd_))
  {
    Stg::Velocity v = positionmodel->GetVelocity();
    // stopping makes only sense if the vehicle drives
    if (!positionmodel->GetVelocity().IsZero())
    {
      this->positionmodel->SetSpeed(0.0, 0.0, 0.0);
      RCLCPP_INFO(node_->get_logger(), "watchdog timeout on %s", name().c_str());
    }
  }
}
void StageNode::Vehicle::callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  std::scoped_lock lock(node_->msg_lock);
  if (!std::isfinite(msg->linear.x) || !std::isfinite(msg->angular.z)) {
    this->positionmodel->SetSpeed(0.0, 0.0, 0.0);
    timeout_cmd_ = rclcpp::Time(0, 0);
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000,
      "Rejected non-finite cmd_vel for %s", name().c_str());
    return;
  }
  const double linear = std::clamp(
    msg->linear.x,
    -node_->max_command_linear_speed_,
    node_->max_command_linear_speed_);
  const double angular = std::clamp(
    msg->angular.z,
    -node_->max_command_angular_speed_,
    node_->max_command_angular_speed_);
  this->positionmodel->SetSpeed(
      linear,
      0.0,
      angular);
  time_last_cmd_received_ = node_->sim_time_;
  timeout_cmd_ = time_last_cmd_received_ + node_->base_watchdog_timeout_;
}
