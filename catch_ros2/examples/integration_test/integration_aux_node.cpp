/// @file A dummy ROS 2 node that provides a service for the test node
/// to check.

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("integration_aux_node");
  auto service = node->create_service<std_srvs::srv::Empty>(
    "test_service",
    [](
      const std_srvs::srv::Empty::Request::SharedPtr,
      std_srvs::srv::Empty::Response::SharedPtr
    ) {}
  );
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
