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

/// @file - Example test with a custom main

#include "catch_ros2/catch_ros2.hpp"

using catch_ros2::tokenize;
using Catch::Matchers::Equals;

TEST_CASE("tokenize", "[tokenize]") {
  SECTION("no quotes") {
    const auto result = tokenize("tokenize me please!");
    CHECK_THAT(result.at(0), Equals("tokenize"));
    CHECK_THAT(result.at(1), Equals("me"));
    CHECK_THAT(result.at(2), Equals("please!"));
  }

  SECTION("quotes") {
    const auto result = tokenize("here is a string \"with some\" quotations \"in it\"");
    CHECK_THAT(result.at(0), Equals("here"));
    CHECK_THAT(result.at(1), Equals("is"));
    CHECK_THAT(result.at(2), Equals("a"));
    CHECK_THAT(result.at(3), Equals("string"));
    CHECK_THAT(result.at(4), Equals("with some"));
    CHECK_THAT(result.at(5), Equals("quotations"));
    CHECK_THAT(result.at(6), Equals("in it"));
  }
}

// Example main can be specified like so. For more information see
// https://github.com/catchorg/Catch2/blob/devel/docs/own-main.md
int main(int argc, char * argv[])
{
  // your setup ...

  int result = Catch::Session().run(argc, argv);

  // your clean-up...

  return result;
}
