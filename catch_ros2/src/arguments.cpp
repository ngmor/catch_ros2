// Copyright 2023 Nick Morales.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "catch_ros2/arguments.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sstream>

namespace catch_ros2
{
SimulateArgs::SimulateArgs(const std::string & args)
{
  // Generate a stringstream from input string
  std::stringstream ss {args};
  std::string s;

  // Parse string, separating at spaces
  // Store data in string vector
  while (std::getline(ss, s, ' ')) {
    args_.push_back(s);
  }

  // Generate argv_vec_
  generate_argv_vec_();
}

SimulateArgs::SimulateArgs(const std::vector<std::string> args)
: args_{args}
{
  // Generate argv_vec_
  generate_argv_vec_();
}

int SimulateArgs::argc() const {return argv_vec_.size() - 1;}

const char * const * SimulateArgs::argv() const {return argv_vec_.data();}

void SimulateArgs::generate_argv_vec_()
{
  // Create char* vector to hold pointers to each element of the argument vector
  for (const auto & arg : args_) {
    argv_vec_.push_back((char *)arg.data());
  }
  argv_vec_.push_back(nullptr);
}


SplitROSArgs::SplitROSArgs(const int argc, const char * const * argv)
: argc_{argc},
  argv_{argv},
  args_without_ros_{SimulateArgs{rclcpp::remove_ros_arguments(argc_, argv_)}}
{}

int SplitROSArgs::argc() const {return argc_;}

int SplitROSArgs::argc_without_ros() const {return args_without_ros_.argc();}

const char * const * SplitROSArgs::argv() const {return argv_;}

const char * const * SplitROSArgs::argv_without_ros() const {return args_without_ros_.argv();}
}
