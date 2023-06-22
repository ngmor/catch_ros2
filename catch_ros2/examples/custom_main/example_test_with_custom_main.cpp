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

/// @file - Example test with a custom main. See main function at bottom.

#include <vector>
#include <string>
#include "catch_ros2/catch_ros2.hpp"

using catch_ros2::tokenize;
using catch_ros2::SimulateArgs;
using Catch::Matchers::Equals;

TEST_CASE("tokenize", "[tokenize]")
{
  SECTION("no quotes")
  {
    const auto result = tokenize("tokenize me please!");
    CHECK_THAT(result.at(0), Equals("tokenize"));
    CHECK_THAT(result.at(1), Equals("me"));
    CHECK_THAT(result.at(2), Equals("please!"));
  }
  SECTION("quotes")
  {
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

TEST_CASE("SimulateArgs", "[SimulateArgs]")
{
  // Examples of different constructors of SimulateArgs
  SECTION("string, default executable path")
  {
    SimulateArgs args {"-c some -o args.o"};
    CHECK(args.argc() == 5);
    CHECK_THAT(*(args.argv() + 0), Equals("/path/to/executable"));
    CHECK_THAT(*(args.argv() + 1), Equals("-c"));
    CHECK_THAT(*(args.argv() + 2), Equals("some"));
    CHECK_THAT(*(args.argv() + 3), Equals("-o"));
    CHECK_THAT(*(args.argv() + 4), Equals("args.o"));
  }
  SECTION("vector, default executable path")
  {
    const std::vector<std::string> arg_vec {"argument with space", "Klaatu barada nikto"};
    SimulateArgs args {arg_vec};
    CHECK(args.argc() == 3);
    CHECK_THAT(*(args.argv() + 0), Equals("/path/to/executable"));
    CHECK_THAT(*(args.argv() + 1), Equals("argument with space"));
    CHECK_THAT(*(args.argv() + 2), Equals("Klaatu barada nikto"));
  }
  SECTION("string, omit executable path")
  {
    // Second boolean argument omits the default executable path
    // from the simulated arguments
    SimulateArgs args {"Houston we have \"a problem\"", true};
    CHECK(args.argc() == 4);
    CHECK_THAT(*(args.argv() + 0), Equals("Houston"));
    CHECK_THAT(*(args.argv() + 1), Equals("we"));
    CHECK_THAT(*(args.argv() + 2), Equals("have"));
    CHECK_THAT(*(args.argv() + 3), Equals("a problem"));
  }
  SECTION("vector, omit executable path")
  {
    const std::vector<std::string> arg_vec {"--ros-args", "-p", "rate:=100"};
    // Second boolean argument omits the default executable path
    // from the simulated arguments
    SimulateArgs args {arg_vec, true};
    CHECK(args.argc() == 3);
    CHECK_THAT(*(args.argv() + 0), Equals("--ros-args"));
    CHECK_THAT(*(args.argv() + 1), Equals("-p"));
    CHECK_THAT(*(args.argv() + 2), Equals("rate:=100"));
  }
  SECTION("string, custom exectable path")
  {
    SimulateArgs args {"/millenium/falcon", "--kessel-run-parsecs 12"};
    CHECK(args.argc() == 3);
    CHECK_THAT(*(args.argv() + 0), Equals("/millenium/falcon"));
    CHECK_THAT(*(args.argv() + 1), Equals("--kessel-run-parsecs"));
    CHECK_THAT(*(args.argv() + 2), Equals("12"));
  }
  SECTION("vector, custom executable path")
  {
    SimulateArgs args {
      "/bin/reach",
      std::vector<std::string>({"--current-objective", "survive"})
    };
    CHECK(args.argc() == 3);
    CHECK_THAT(*(args.argv() + 0), Equals("/bin/reach"));
    CHECK_THAT(*(args.argv() + 1), Equals("--current-objective"));
    CHECK_THAT(*(args.argv() + 2), Equals("survive"));
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
