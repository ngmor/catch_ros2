#include <catch2/catch_session.hpp>
#include "catch2_ros/arguments.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  // Split input ROS args from args that will be input to Catch
  const auto split = catch2_ros::SplitROSArgs{argc, argv};

  // Init ROS
  rclcpp::init(split.argc(), split.argv());

  // Run Catch session
  int result = Catch::Session().run(split.argc_without_ros(), split.argv_without_ros() );

  // Shutdown ROS
  rclcpp::shutdown();

  return result;
}