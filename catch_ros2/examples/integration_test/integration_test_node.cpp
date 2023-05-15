/// @file This is an example ROS 2 node that checks assertions using Catch2.
/// It simply checks if the "test_service" service is available at least once
/// during the duration of the test.

#include <catch2/catch_test_macros.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

TEST_CASE("example_integration_test", "[integration]") {
  // Create a simple client node to check if the auxiliary node
  // has a service available
  auto node = rclcpp::Node::make_shared("integration_test_node");

  // Declare a parameter on the node
  // (the default catch_ros2 node main will allow ROS arguments
  // like parameters to be passed to nodes in test files)
  node->declare_parameter<double>("test_duration");

  // Get value of the parameter
  // This line will cause a runtime error if a value
  // for the "test_duration" parameter is not passed to the node
  const auto TEST_DURATION =
    node->get_parameter("test_duration").get_parameter_value().get<double>();

  // Create a client for the service we're looking for
  auto client = node->create_client<std_srvs::srv::Empty>("test_service");


  rclcpp::Time start_time = rclcpp::Clock().now();

  bool service_found = false;

  // Keep test running only for the length of the "test_duration" parameter
  // (in seconds)
  while (
    rclcpp::ok() &&
    ((rclcpp::Clock().now() - start_time) < rclcpp::Duration::from_seconds(TEST_DURATION))
  )
  {
    // Repeatedly check for the dummy service until its found
    if (client->wait_for_service(0s)) {
      service_found = true;
      break;
    }

    rclcpp::spin_some(node);
  }

  // Test assertions - check that the dummy node was found
  CHECK(service_found);
}
