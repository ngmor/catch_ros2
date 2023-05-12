#include "catch2_ros/arguments.hpp"
#include <sstream>

namespace catch2_ros
{
    SimulateArgs::SimulateArgs(const std::string & args)
    {
      // Generate a stringstream from input string
      std::stringstream ss {args};
      std::string s;

      // Parse string, separating at spaces
      // Store data in string vector
      while (std::getline(ss, s, ' '))
      {
        args_.push_back(s);
      }

      // Create char* vector to hold pointers to each element of the argument vector
      for (const auto & arg : args_)
      {
        argv_vec_.push_back((char*)arg.data());
      }
      argv_vec_.push_back(nullptr);
    }

    int SimulateArgs::argc() const {return argv_vec_.size() - 1;}

    const char* const * SimulateArgs::argv() const {return argv_vec_.data();}

    SplitROSArgs::SplitROSArgs(const int argc, const char* const* argv):
      argc_{argc}, argv_{argv}, ndx_ros_args_start_{argc_}
    {
      // Find start of ros args
      for (int i = 0; i < argc_; i++)
      {
        if (static_cast<std::string>(argv_[i]) == "--ros-args")
        {
          ndx_ros_args_start_ = i;
          break;
        }
      }
    }

    int SplitROSArgs::argc() const {return argc_;}

    int SplitROSArgs::argc_without_ros() const {return ndx_ros_args_start_;}

    const char* const* SplitROSArgs::argv() const {return argv_;}
}