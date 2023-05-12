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

###########
## Build ##
###########

add_library(${PROJECT_NAME} SHARED 
  src/arguments.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

#############
## Install ##
#############

# install targets
install(TARGETS ${PROJECT_NAME}
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

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
  Catch2
)

ament_package(CONFIG_EXTRAS
  cmake/catch2_ros_add_integration_test.cmake
)

install(
  DIRECTORY cmake
  DESTINATION share/${PROJECT_NAME}
)