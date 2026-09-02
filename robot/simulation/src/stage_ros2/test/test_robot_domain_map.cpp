#include <gtest/gtest.h>

#include <stdexcept>

#include "stage_ros2/robot_domain_map.hpp"

TEST(RobotDomainMap, ParsesRobotIdsAndDomains)
{
  const auto result = stage_ros2::parse_robot_domain_map(" robot11 = 11,robot12=12, robot-13=232 ");

  ASSERT_EQ(result.size(), 3u);
  EXPECT_EQ(result.at("robot11"), 11u);
  EXPECT_EQ(result.at("robot12"), 12u);
  EXPECT_EQ(result.at("robot-13"), 232u);
}

TEST(RobotDomainMap, EmptyConfigurationKeepsLegacyMode)
{
  EXPECT_TRUE(stage_ros2::parse_robot_domain_map("").empty());
  EXPECT_TRUE(stage_ros2::parse_robot_domain_map(" , ").empty());
}

TEST(RobotDomainMap, RejectsMalformedEntries)
{
  EXPECT_THROW(stage_ros2::parse_robot_domain_map("robot11"), std::invalid_argument);
  EXPECT_THROW(stage_ros2::parse_robot_domain_map("robot11=abc"), std::invalid_argument);
  EXPECT_THROW(stage_ros2::parse_robot_domain_map("robot11=233"), std::invalid_argument);
  EXPECT_THROW(stage_ros2::parse_robot_domain_map("=11"), std::invalid_argument);
  EXPECT_THROW(stage_ros2::parse_robot_domain_map("11robot=11"), std::invalid_argument);
  EXPECT_THROW(stage_ros2::parse_robot_domain_map("r=11"), std::invalid_argument);
}

TEST(RobotDomainMap, RejectsDuplicateRobotIdsAndDomains)
{
  EXPECT_THROW(stage_ros2::parse_robot_domain_map("robot11=11,robot11=12"), std::invalid_argument);
  EXPECT_THROW(stage_ros2::parse_robot_domain_map("robot11=11,robot12=11"), std::invalid_argument);
}
