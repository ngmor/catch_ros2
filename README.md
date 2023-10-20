# catch_ros2
A lightweight wrapper around the [Catch2](https://github.com/catchorg/Catch2/) testing framework for use with ROS 2.

This can be used for unit testing of ROS 2 related functionality or for integration tests to test the functionality of nodes in C++.

## Installation
To install Debian packages of `catch_ros2` simply run the following command (assuming your environment is properly [set up for installing Debian packages](https://docs.ros.org/en/rolling/Installation/Ubuntu-Install-Debians.html)):
```
apt install ros-${ROS_DISTRO}-catch-ros2
```
Debian packages are currently available for Humble, Iron, and Rolling ROS 2 distributions.

To build the package from source, clone it into the `src` directory of your workspace root directory (`ws`). Then from `ws` use `colcon build`, like any other ROS 2 package.

## Usage
See the [examples](examples/) section for guides on usage.

## Catch2 Version
Currently this repository uses the [amalgamated version of Catch2](https://github.com/catchorg/Catch2/blob/devel/docs/migrate-v2-to-v3.md#how-to-migrate-projects-from-v2-to-v3) as a simple way of vendoring v3.3.2 of Catch2. Ubuntu currently only provides Catch2 v2, which ROS 2 depends on, so we've made the decision to vendor v3 to be able to use the newest features. In the future, this approach may be abandoned for vendoring Catch2 v3 in its multiple header form.

In order to update the repository for a future release of Catch2 v3, simply copy the amalgamated [header](https://github.com/catchorg/Catch2/blob/devel/extras/catch_amalgamated.hpp) and [source](https://github.com/catchorg/Catch2/blob/devel/extras/catch_amalgamated.cpp) files from the Catch2 repository and replace those [header](include/catch_amalgamated.hpp) and [source](src/catch_amalgamated.cpp) files in this repository. The [default main](src/default_main.cpp) may also need to be updated.

## License
This package is released under the Apache-2.0 License. Catch2 is licensed under the BSL-1.0 License (see their repository for more details).