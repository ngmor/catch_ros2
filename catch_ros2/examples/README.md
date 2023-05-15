# Example Usage
These directories contain examples of how to use the `catch_ros2` library. This README provides an overview, but more details can be found in the source code.

To build these examples, use the following from the root of your workspace:

```
colcon build --cmake-args -DCATCH_ROS2_BUILD_EXAMPLES=TRUE
```

There are two basic ways to use this framework:
1. [Integration testing](#integration-testing)
2. [Unit testing](#unit-testing)

You can also [customize further](#using-a-custom-main) if you need further control.

## Integration Testing
Integration testing is an important way to ensure different elements of your system work together as desired. In ROS, we generally want to test that our nodes behave correctly, which may difficult with typical unit testing.

ROS 2 already contains a [framework for writing integration tests in Python](https://github.com/ros2/launch/tree/rolling/launch_testing) which may be more than sufficient for most users. However, to my knowledge I have not seen an easy way to create integration tests in C++ or more specifically with Catch2. The primary motivation of this package is to provide a framework for just that.

For a launch integration test in this framework you need 4 components:
1. [Node(s) under test](#nodes-under-test)
2. [Testing node](#testing-node)
3. [Launch file](#launch-file)
4. [CMake setup](#integration-cmake-setup)

### Node(s) under test
These are the nodes in your package whose functionality you'd like to test. Nothing special has to be done with these nodes to test them with this framework. This example has [`integration_aux_node`](integration_test/integration_aux_node.cpp) as the node under test.

### Testing node
This node is specifically written with Catch2 test cases and assertions to test the functionality of other nodes. The example has [`integration_test_node`](integration_test/integration_test_node.cpp) as the testing node.

If it is linked to `catch_ros2::catch_ros2_with_node_main` (see [CMake setup](#integration-cmake-setup)) it does not require a `main` function and can be run just like any other ROS 2 node (`ros2 run ${package_name} ${executable_name}`). Tests can be executed manually by just running the node like this.

If more precise functionality beyond the [default node main](../src/main.cpp) of this package is desired, see [using a custom main](#using-a-custom-main).

### Launch file
To automate the integration test, a [Python launch file](integration_test/example_integration_test.launch.py) can be used to launch all the necessary nodes for the test, including the test node.

This launch file is the same as any other ROS 2 Python launch file, but the `launch_catch_ros2` Python module provides a few useful classes:
- `Catch2LaunchDescription` - a wrapper around the typical `launch.LaunchDescription` that includes a required "result_file" argument where the results of the test will be output.
- `Catch2IntegrationTestNode` - a wrapper around the typical `launch_ros.actions.Node` that passes the "result_file" argument to Catch2 (as per the default catch_ros2 main function) and shuts down all nodes on exit. This should be used to launch the [testing node](#testing-node). Only one should be used per integration test.

### Integration CMake setup
Most of the [CMake required for this integration test](integration_test/CMakeLists.txt) is pretty standard for a ROS 2 package (though the ROS 2 boilerplate CMake is not shown in this example). Nodes are built/installed and the launch file is installed to the package's share directory.

A few items are of note:
```
target_link_libraries(integration_test_node
  catch_ros2::catch_ros2_with_node_main
)
```
This links the integration test node with the version of `catch_ros2` that includes the [default main for writing nodes](../src/main.cpp). Only one version of the `catch_ros2` library need be linked to a target, depending on the need for a default main. See [using a custom main](#using-a-custom-main).

```
catch_ros2_add_integration_test(ExampleIntegration_Test
  LAUNCH_FILE example_integration_test.launch.py
)
```
This custom CMake function is only necessary if the test should be run automatically whenever `colcon test` is run. See [here](../cmake/catch_ros2_add_integration_test.cmake) for documentation on the function.

## Unit Testing
TODO

## Using a custom main
TODO
TODO - link to custom main section of Catch2