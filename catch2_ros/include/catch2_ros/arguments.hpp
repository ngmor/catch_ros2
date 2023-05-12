#ifndef CATCH2_ROS_ARGUMENTS_INCLUDE_GUARD_H
#define CATCH2_ROS_ARGUMENTS_ARGS_INCLUDE_GUARD_H
/// @file
/// @brief Utilities for handling command line arguments

#include <vector>
#include <string>

namespace catch2_ros
{
  /// @brief A class for generating simulated argc/argv values from an input string
  /// https://stackoverflow.com/questions/39883433/create-argc-argv-in-the-code
  /// TODO - improve so this can handle quotations in the input string properly
  class SimulateArgs
  {
  public:
    /// @brief Parse string into data structures that can be used to generate simulated argc/argvs
    /// @param args - command string to parse into arguments
    SimulateArgs(const std::string & args);

    /// @brief generate argc
    /// @return argc, argument count
    int argc() const;

    /// @brief generate argv
    /// @return pointer to first argument data
    const char* const * argv() const;

  private:
    /// @brief string argument data
    std::vector<std::string> args_ {};

    /// @brief pointers to string argument data
    std::vector<char*> argv_vec_ {};
  };

  /// @brief a class for splitting ROS arguments out of input arguments
  class SplitROSArgs
  {
  public:
    
    /// @brief find where ROS Args start
    /// @param argc - argument count
    /// @param argv - pointer to first argument data
    /// TODO improve so the end of ROS args can also be detected (--)
    SplitROSArgs(const int argc, const char* const* argv);

    /// @brief return original argc
    /// @return original argc
    int argc() const;

    /// @brief return argc if ROS args are taken out
    /// @return argc if ROS args are taken out
    int argc_without_ros() const;

    /// @brief return original argv, can also be used when ROS args are taken out
    /// @return original argv
    const char* const* argv() const;

  private:
    /// @brief original argc
    const int argc_;

    /// @brief original argv
    const char* const* argv_;

    /// @brief the index where ros args start
    int ndx_ros_args_start_;
  };
}

#endif