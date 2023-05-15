cmake_minimum_required(VERSION 3.8)
project(catch2_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##################
## DEPENDENCIES ##
##################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Catch2 REQUIRED)
find_package(rclcpp REQUIRED)

###########
## Build ##
###########

# Base Library
add_library(${PROJECT_NAME} SHARED 
  src/arguments.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp Catch2
)

# Library with Catch2 default main
# https://stackoverflow.com/questions/34234446/cmake-is-it-possible-to-build-an-executable-from-only-static-libraries-and-no-s
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/null.cpp "")

add_library(${PROJECT_NAME}_with_main ${CMAKE_CURRENT_BINARY_DIR}/null.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME}_with_main ALIAS ${PROJECT_NAME}_with_main)
target_link_libraries(${PROJECT_NAME}_with_main PUBLIC
  ${PROJECT_NAME} Catch2::Catch2WithMain
)
set_target_properties(${PROJECT_NAME}_with_main
  PROPERTIES
  OUTPUT_NAME "${PROJECT_NAME}_with_main"
)

# Library with custom main for running nodes
add_library(${PROJECT_NAME}_with_node_main
  src/main.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME}_with_node_main ALIAS ${PROJECT_NAME}_with_node_main)
target_link_libraries(${PROJECT_NAME}_with_node_main PUBLIC
  ${PROJECT_NAME} Catch2
)
set_target_properties(${PROJECT_NAME}_with_node_main
  PROPERTIES
  OUTPUT_NAME "${PROJECT_NAME}_with_node_main"
)

#############
## Install ##
#############

# install targets
install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_with_main ${PROJECT_NAME}_with_node_main
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install include directories
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Install launch utilities
ament_python_install_package(launch_${PROJECT_NAME} PACKAGE_DIR launch_${PROJECT_NAME})

# Install custom test runner
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies() # TODO uncomment
endif()

# Examples
if (CATCH2_ROS_BUILD_EXAMPLES)
  include(examples/integration_test/CMakeLists.txt)
endif()

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_with_main ${PROJECT_NAME}_with_node_main)

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  Catch2
  rclcpp
)

ament_package(CONFIG_EXTRAS
  cmake/catch2_ros_add_integration_test.cmake
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)