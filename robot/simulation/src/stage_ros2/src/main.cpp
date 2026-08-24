#include <chrono>
#include <memory>
#include <thread>

#include "stage_ros2/stage_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<StageNode>(rclcpp::NodeOptions());
  node->init(argc - 1, argv);
  if (node->SubscribeModels() != 0) {exit(-1);}
  std::thread t = std::thread([&node]() {rclcpp::spin(node);});
  node->world->Start();

  if (node->world->IsGUI()) {
    // WorldGui schedules updates against wall time itself.
    Stg::World::Run();
  } else {
    // The upstream non-GUI World::Run() is an unthrottled tight loop. That
    // makes simulated time run thousands of times faster than ROS consumers,
    // so scans, TF and odometry are dropped before AMCL can use them. Keep a
    // headless simulation at real time, just like the default GUI speedup=1.
    const auto simulation_step =
      std::chrono::microseconds(node->world->sim_interval);
    auto next_update = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !Stg::World::UpdateAll()) {
      next_update += simulation_step;
      const auto now = std::chrono::steady_clock::now();
      if (now > next_update + simulation_step) {
        // Do not attempt a burst of catch-up iterations after a slow update.
        next_update = now;
      }
      std::this_thread::sleep_until(next_update);
    }
  }

  rclcpp::shutdown();
  if (t.joinable()) {
    t.join();
  }
  return 0;
}
