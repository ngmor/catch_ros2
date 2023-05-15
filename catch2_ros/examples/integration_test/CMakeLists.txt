# This CMakeLists.txt only include the lines needed to set up this test
# in your own custom ROS 2 package. Other necessary CMake commands
# (such as the boilerplate ROS 2 items added when using ros2 pkg create)
# are omitted.

# DO NOT INCLUDE ITEMS BETWEEN THESE LINES
# in your typical usage of catch2_ros. They are only included here so
# this file can be include in the top level CMakeLists.txt
set(catch2_ros_DIR ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/catch2_ros_add_integration_test.cmake)
# DO NOT INCLUDE ITEMS BETWEEN THESE LINES

# THIS LINE SHOULD BE UNCOMMENTED - it is only commented here
# so this file can be included in the top level CMakeLists.txt
# find_package(catch2_ros REQUIRED) # UNCOMMENT
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)


##########################
## INTEGRATION AUX NODE ##
##########################
# This could be any node in your ROS 2 package
# Likely you are looking to test some functionality of this node
add_executable(integration_aux_node
  examples/integration_test/integration_aux_node.cpp
)
ament_target_dependencies(integration_aux_node
  rclcpp std_srvs
)

install(TARGETS
  integration_aux_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  include(CTest)

  ###########################
  ## INTEGRATION TEST NODE ##
  ###########################
  # This is the node in which integration tests occur
  add_executable(integration_test_node
    examples/integration_test/integration_test_node.cpp
  )
  # The link libraries call links this node with catch2::catch2_ros_with_node_main
  # to get the default integration test node main function
  # TODO - add link
  target_link_libraries(integration_test_node
    catch2_ros::catch2_ros_with_node_main
  )
  ament_target_dependencies(integration_test_node
    rclcpp std_srvs
  )
  install(TARGETS
    integration_test_node
    DESTINATION lib/${PROJECT_NAME}
  )

  #################
  ## LAUNCH FILE ##
  #################
  # This is the launch file that will be used to run the integration test
  install(FILES
    examples/integration_test/example_integration_test.launch.py
    DESTINATION share/${PROJECT_NAME}
  )

  ######################
  ## INTEGRATION TEST ##
  ######################
  # This CMake function allows the integration test to be run
  # when running "colcon test".
  # See here for more information:
  # TODO add link
  catch2_ros_add_integration_test(ExampleIntegration_Test
    LAUNCH_FILE example_integration_test.launch.py
  )

endif()