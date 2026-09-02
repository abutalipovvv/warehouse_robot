#include <stage_ros2/robot_domain_map.hpp>
#include <stage_ros2/stage_node.hpp>

#include <chrono>
#include <filesystem>
#include <memory>
#include <set>

StageNode::StageNode(rclcpp::NodeOptions options)
: Node("stage_ros2", options), base_watchdog_timeout_(0, 0)
{
  tf_broadcaster_stage_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  declare_parameters();
}

StageNode::~StageNode()
{
}

void StageNode::declare_parameters()
{
  this->set_parameter(rclcpp::Parameter("use_sim_time", true));
  auto param_desc_enable_gui = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_enable_gui.description = "Enable GUI!";
  this->declare_parameter<bool>("enable_gui", true, param_desc_enable_gui);

  auto param_desc_enforce_prefixes = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_enforce_prefixes.description =
    "on true it enforces prefixes on topic even with one robot";
  this->declare_parameter<bool>("enforce_prefixes", false, param_desc_enforce_prefixes);


  auto param_desc_one_tf_tree = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_one_tf_tree.description =
    "On true: all tfs are publishe on /tf and /tf_static!";
  this->declare_parameter<bool>("one_tf_tree", false, param_desc_one_tf_tree);

  auto param_desc_use_static_transformations_ = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_use_static_transformations_.description =
    "use static transformations for sensor frames!";
  this->declare_parameter<bool>(
    "use_static_transformations", true,
    param_desc_use_static_transformations_);

  auto param_desc_watchdog_timeout = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_watchdog_timeout.description =
    "timeout after which a vehicle stopps if no command is received!";
  this->declare_parameter<double>("base_watchdog_timeout", 0.5, param_desc_watchdog_timeout);

  auto param_desc_max_linear_speed = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_max_linear_speed.description = "maximum absolute differential-drive linear command in m/s";
  this->declare_parameter<double>("max_command_linear_speed", 1.0, param_desc_max_linear_speed);

  auto param_desc_max_angular_speed = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_max_angular_speed.description = "maximum absolute differential-drive angular command in rad/s";
  this->declare_parameter<double>("max_command_angular_speed", 1.5, param_desc_max_angular_speed);

  auto param_desc_is_depth_canonical = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_is_depth_canonical.description = "USE depth canonical!";
  this->declare_parameter<bool>("is_depth_canonical", true, param_desc_is_depth_canonical);

  auto param_desc_publish_ground_truth = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_publish_ground_truth.description = "publishes on true a ground truth tf!";
  this->declare_parameter<bool>("publish_ground_truth", true, param_desc_publish_ground_truth);

  auto param_desc_publish_imu = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_publish_imu.description = "publishes on true a simulated imu topic!";
  this->declare_parameter<bool>("publish_imu", true, param_desc_publish_imu);

  auto param_desc_use_imu_for_odom_yaw = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_use_imu_for_odom_yaw.description =
    "on true the simulated odom yaw is stabilized with imu heading and gyro";
  this->declare_parameter<bool>("use_imu_for_odom_yaw", true, param_desc_use_imu_for_odom_yaw);

  auto param_desc_robot_domain_map = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_robot_domain_map.description =
      "Comma-separated Stage model/robot_id to ROS domain mapping, for example "
      "robot11=11,robot12=12. Each mapped robot publishes unprefixed topics in "
      "its own domain.";
  this->declare_parameter<std::string>("robot_domain_map", "",
                                       param_desc_robot_domain_map);

  auto param_desc_world_file = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_world_file.description = "USE model names!";
  this->declare_parameter<std::string>("world_file", "cave.world", param_desc_world_file);

  auto param_desc_frame_id_odom_name_ = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_frame_id_odom_name_.description =
    "odom frame name or postfix in case of multiple robots";
  this->declare_parameter<std::string>("frame_id_odom", "odom", param_desc_frame_id_odom_name_);

  auto param_desc_frame_id_world_name_ = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_frame_id_world_name_.description = "world frame name for ground truth odom data";
  this->declare_parameter<std::string>("frame_id_world", "world", param_desc_frame_id_world_name_);

  auto param_desc_frame_id_base_link_name_ = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_frame_id_base_link_name_.description =
    "base link frame name or postfix in case of multiple robots";
  this->declare_parameter<std::string>(
    "frame_id_base_link", "base_link",
    param_desc_frame_id_base_link_name_);

  auto param_desc_frame_id_imu_name_ = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_frame_id_imu_name_.description =
    "imu frame name or postfix in case of multiple robots";
  this->declare_parameter<std::string>(
    "frame_id_imu", "imu_link",
    param_desc_frame_id_imu_name_);

  auto param_desc_imu_yaw_noise = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_imu_yaw_noise.description = "constant yaw bias stddev for simulated imu in rad";
  this->declare_parameter<double>("imu_yaw_noise_stddev", 0.01, param_desc_imu_yaw_noise);

  auto param_desc_imu_angular_velocity_noise = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_imu_angular_velocity_noise.description =
    "white noise stddev for simulated imu angular velocity in rad/s";
  this->declare_parameter<double>(
    "imu_angular_velocity_noise_stddev", 0.02,
    param_desc_imu_angular_velocity_noise);

  auto param_desc_imu_linear_acceleration_noise = rcl_interfaces::msg::ParameterDescriptor{};
  param_desc_imu_linear_acceleration_noise.description =
    "white noise stddev for simulated imu linear acceleration in m/s^2";
  this->declare_parameter<double>(
    "imu_linear_acceleration_noise_stddev", 0.05,
    param_desc_imu_linear_acceleration_noise);
}

void StageNode::update_parameters()
{
  double base_watchdog_timeout_sec{0.5};
  this->get_parameter("enable_gui", this->enable_gui_);
  this->get_parameter("enforce_prefixes", this->enforce_prefixes_);
  this->get_parameter("one_tf_tree", this->one_tf_tree_);
  this->get_parameter("base_watchdog_timeout", base_watchdog_timeout_sec);
  this->get_parameter("max_command_linear_speed", this->max_command_linear_speed_);
  this->get_parameter("max_command_angular_speed", this->max_command_angular_speed_);
  this->base_watchdog_timeout_ = rclcpp::Duration::from_seconds(base_watchdog_timeout_sec);
  this->get_parameter("is_depth_canonical", this->isDepthCanonical_);
  this->get_parameter("publish_ground_truth", this->publish_ground_truth_);
  this->get_parameter("publish_imu", this->publish_imu_);
  this->get_parameter("use_imu_for_odom_yaw", this->use_imu_for_odom_yaw_);
  this->get_parameter("robot_domain_map", this->robot_domain_map_config_);
  parse_robot_domain_map();
  this->get_parameter("frame_id_odom", this->frame_id_odom_name_);
  this->get_parameter("frame_id_world", this->frame_id_world_name_);
  this->get_parameter("frame_id_base_link", this->frame_id_base_link_name_);
  this->get_parameter("frame_id_imu", this->frame_id_imu_name_);
  this->get_parameter("imu_yaw_noise_stddev", this->imu_yaw_noise_stddev_);
  this->get_parameter(
    "imu_angular_velocity_noise_stddev",
    this->imu_angular_velocity_noise_stddev_);
  this->get_parameter(
    "imu_linear_acceleration_noise_stddev",
    this->imu_linear_acceleration_noise_stddev_);

  this->get_parameter("world_file", this->world_file_);
  if (!std::filesystem::exists(this->world_file_)) {
    RCLCPP_FATAL(
      this->get_logger(), "The world file %s does not exist.",
      this->world_file_.c_str());
    exit(0);
  }

  if (this->one_tf_tree_){
    RCLCPP_WARN(
      this->get_logger(), "The parameter one_tf_tree is set but deprecated and will be removed in later versions");
  }

  callback_update_parameters();

  using namespace std::chrono_literals;
  timer_update_parameter_ =
    this->create_wall_timer(1000ms, std::bind(&StageNode::callback_update_parameters, this));
}

void StageNode::parse_robot_domain_map()
{
  robot_domain_map_ =
    stage_ros2::parse_robot_domain_map(robot_domain_map_config_);
}

void StageNode::callback_update_parameters()
{
  double base_watchdog_timeout_sec;
  this->get_parameter("base_watchdog_timeout", base_watchdog_timeout_sec);
  this->base_watchdog_timeout_ = rclcpp::Duration::from_seconds(base_watchdog_timeout_sec);
  this->get_parameter("max_command_linear_speed", this->max_command_linear_speed_);
  this->get_parameter("max_command_angular_speed", this->max_command_angular_speed_);

  this->get_parameter("use_static_transformations", use_static_transformations_);

  this->get_parameter("publish_ground_truth", this->publish_ground_truth_);
  this->get_parameter("publish_imu", this->publish_imu_);
  this->get_parameter("use_imu_for_odom_yaw", this->use_imu_for_odom_yaw_);
  this->get_parameter("frame_id_imu", this->frame_id_imu_name_);
  this->get_parameter("imu_yaw_noise_stddev", this->imu_yaw_noise_stddev_);
  this->get_parameter(
    "imu_angular_velocity_noise_stddev",
    this->imu_angular_velocity_noise_stddev_);
  this->get_parameter(
    "imu_linear_acceleration_noise_stddev",
    this->imu_linear_acceleration_noise_stddev_);
  // RCLCPP_INFO(this->get_logger(), "callback_update_parameter");
}

/**
 * Is called only ones after the simulation starts with each model
 * The function fills the vehicle vector with pointers to the stage models
 * @param mod stage model
 * @param node pointer to this class
*/
int StageNode::callback_init_stage_model(Stg::Model * mod, StageNode * node)
{
  if (dynamic_cast<Stg::ModelPosition *>(mod)) {
    Stg::ModelPosition * position = dynamic_cast<Stg::ModelPosition *>(mod);
    RCLCPP_INFO(node->get_logger(), "New Vehicle \"%s\"", mod->TokenStr().c_str());
    auto vehicle = std::make_shared<Vehicle>(
      node->vehicles_.size(),
      position->GetGlobalPose(), mod->TokenStr(), node);
    node->vehicles_.push_back(vehicle);
    vehicle->positionmodel = position;
  }

  if (dynamic_cast<Stg::ModelRanger *>(mod)) {
    Stg::ModelPosition * parent = dynamic_cast<Stg::ModelPosition *>(mod->Parent());
    for (std::shared_ptr<Vehicle> vehcile: node->vehicles_) {
      if (parent == vehcile->positionmodel) {
        auto ranger =
          std::make_shared<Vehicle::Ranger>(
          vehcile->rangers_.size() + 1,
          dynamic_cast<Stg::ModelRanger *>(mod), vehcile);
        vehcile->rangers_.push_back(ranger);
      }
    }
  }
  if (dynamic_cast<Stg::ModelCamera *>(mod)) {
    Stg::ModelPosition * parent = dynamic_cast<Stg::ModelPosition *>(mod->Parent());
    for (std::shared_ptr<Vehicle> vehcile: node->vehicles_) {
      if (parent == vehcile->positionmodel) {
        auto camera =
          std::make_shared<Vehicle::Camera>(
          vehcile->cameras_.size() + 1,
          dynamic_cast<Stg::ModelCamera *>(mod), vehcile);
        vehcile->cameras_.push_back(camera);
      }
    }
  }
  return 0;
}

int StageNode::callback_update_stage_world(Stg::World * world, StageNode * node)
{
  // We return false to indicate that we want to be called again (an
  // odd convention, but that's the way that Stage works).
  if (!rclcpp::ok()) {
    RCLCPP_INFO(node->get_logger(), "rclcpp::ok() is false. Quitting.");
    node->world->QuitAll();
    return 1;
  }

  std::scoped_lock lock(node->msg_lock);


  node->sim_time_ = rclcpp::Time(world->SimTimeNow() * 1e3);
  // We're not allowed to publish clock==0, because it used as a special
  // value in parts of ROS, #4027.
  if (int(node->sim_time_.nanoseconds()) == 0) {
    RCLCPP_DEBUG(
      node->get_logger(), "Skipping initial simulation step, to avoid publishing clock==0");
    return 0;
  }
  // loop on the robot models
  for (size_t r = 0; r < node->vehicles_.size(); ++r) {
    auto vehicle = node->vehicles_[r];
    vehicle->check_watchdog_timeout();
    vehicle->publish_msg();
    vehicle->publish_tf();

    // loop on the ranger devices for the current robot
    for (auto ranger: vehicle->rangers_) {
      ranger->publish_msg();
      ranger->publish_tf();
    }


    // loop on the camera devices for the current robot
    for (auto camera: vehicle->cameras_) {
      camera->publish_msg();
      camera->publish_tf();
    }
  }
  rosgraph_msgs::msg::Clock clock_msg;
  clock_msg.clock = node->sim_time_;
  if (node->robot_domain_map_.empty()) {
    node->clock_pub_->publish(clock_msg);
  } else {
    for (const auto & vehicle : node->vehicles_) {
      vehicle->publish_clock(clock_msg);
    }
  }
  return 0;
}

bool StageNode::cb_reset_srv(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  RCLCPP_INFO(this->get_logger(), "Resetting stage!");
  for (auto vehicle: this->vehicles_) {
    vehicle->soft_reset();
  }
  return true;
}

bool StageNode::cb_reset_odom_srv(
  const std_srvs::srv::Empty::Request::SharedPtr,
  std_srvs::srv::Empty::Response::SharedPtr)
{
  std::scoped_lock lock(this->msg_lock);
  RCLCPP_INFO(this->get_logger(), "Resetting odometry to zero!");
  for (auto vehicle: this->vehicles_) {
    vehicle->reset_odom();
  }
  return true;
}

void StageNode::init(int argc, char ** argv)
{

  this->sim_time_ = rclcpp::Time(0, 0);
  update_parameters();


  // initialize the libstage
  Stg::Init(&argc, &argv);

  if (this->enable_gui_) {
    this->world = new Stg::WorldGui(600, 400, "Stage (ROS)");
  } else {
    this->world = new Stg::World();
  }

  this->world->Load(world_file_.c_str());
  this->world->AddUpdateCallback((Stg::world_callback_t)callback_update_stage_world, this);
  this->world->ForEachDescendant((Stg::model_callback_t)callback_init_stage_model, this);
}

// Subscribe to models of interest.  Currently, we find and subscribe
// to the first 'laser' model and the first 'position' model.  Returns
// 0 on success (both models subscribed), -1 otherwise.
//
// Eventually, we should provide a general way to map stage models onto ROS
// topics, similar to Player .cfg files.
int StageNode::SubscribeModels()
{
  if (!robot_domain_map_.empty()) {
    std::set<std::string> world_robot_ids;
    for (const auto & vehicle : vehicles_) {
      world_robot_ids.emplace(vehicle->name());
      if (robot_domain_map_.count(vehicle->name()) == 0) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Stage model '%s' is missing from robot_domain_map",
          vehicle->name().c_str());
        return -1;
      }
    }
    for (const auto & mapping : robot_domain_map_) {
      if (world_robot_ids.count(mapping.first) == 0) {
        RCLCPP_ERROR(
          this->get_logger(),
          "robot_domain_map contains '%s', but the world has no Stage model with that name",
          mapping.first.c_str());
        return -1;
      }
    }
    if (enforce_prefixes_ || one_tf_tree_) {
      RCLCPP_WARN(
        this->get_logger(),
        "robot_domain_map uses isolated ROS domains; enforce_prefixes and one_tf_tree are ignored");
    }
  }

  for (std::shared_ptr<Vehicle> vehicle: this->vehicles_) {
    if (robot_domain_map_.empty()) {
      // Legacy mode: Stage model names prefix topics when the world has
      // multiple vehicles.
      const bool use_topic_prefix = this->enforce_prefixes_ || (vehicles_.size() > 1);
      vehicle->init(use_topic_prefix, this->one_tf_tree_);
    } else {
      const size_t domain_id = robot_domain_map_.at(vehicle->name());
      vehicle->init(false, false, &domain_id);
      RCLCPP_INFO(
        this->get_logger(),
        "Robot '%s': ROS_DOMAIN_ID=%zu, topics=/cmd_vel,/scan,/odom,/imu,/tf",
        vehicle->name().c_str(), domain_id);
    }
  }

  // Legacy mode has one clock in the Stage node's domain. Domain-isolated
  // vehicles create their own /clock publisher in Vehicle::init_ros_domain().
  if (robot_domain_map_.empty()) {
    clock_pub_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  }

  // In domain-isolated mode each robot has its own /reset_* services. Keep
  // whole-world administration under /stage so it cannot collide with a
  // robot service if the Stage supervisor shares that robot's ROS domain.
  const std::string reset_positions_service =
    robot_domain_map_.empty() ? "reset_positions" : "/stage/reset_positions";
  const std::string reset_odom_service =
    robot_domain_map_.empty() ? "reset_odom" : "/stage/reset_odom";

  srv_reset_ = this->create_service<std_srvs::srv::Empty>(
    reset_positions_service,
    [this](const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response)
    {this->cb_reset_srv(request, response);});

  srv_reset_odom_ = this->create_service<std_srvs::srv::Empty>(
    reset_odom_service,
    [this](const std_srvs::srv::Empty::Request::SharedPtr request,
    std_srvs::srv::Empty::Response::SharedPtr response)
    {this->cb_reset_odom_srv(request, response);});

  return 0;
}

bool StageNode::UpdateWorld()
{
  return this->world->UpdateAll();
}

// helper functions
geometry_msgs::msg::TransformStamped StageNode::create_transform_stamped(
  const tf2::Transform & in,
  const rclcpp::Time & timestamp, const std::string & frame_id, const std::string & child_frame_id)
{
  geometry_msgs::msg::TransformStamped out;
  out.header.stamp = timestamp;
  out.header.frame_id = frame_id;
  out.child_frame_id = child_frame_id;
  out.transform.translation.x = in.getOrigin().getX();
  out.transform.translation.y = in.getOrigin().getY();
  out.transform.translation.z = in.getOrigin().getZ();
  out.transform.rotation.w = in.getRotation().getW();
  out.transform.rotation.x = in.getRotation().getX();
  out.transform.rotation.y = in.getRotation().getY();
  out.transform.rotation.z = in.getRotation().getZ();
  return out;
}

geometry_msgs::msg::Quaternion StageNode::createQuaternionMsgFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}
