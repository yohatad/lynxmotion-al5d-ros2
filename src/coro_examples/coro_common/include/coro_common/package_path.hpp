#pragma once

#include <cctype>
#include <cstdlib>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace coro_common
{

/// ROS 2 replacement for ros::package::getPath(). Returns the installed share directory of
/// `package` (where the example packages install their data/ folder), or an empty string
/// when the package cannot be found -- the same contract as the ROS 1 call.
inline std::string packagePath(const std::string & package)
{
  try {
    return ament_index_cpp::get_package_share_directory(package);
  } catch (const std::exception &) {
    return std::string();
  }
}

/// Directory (with trailing slash) holding the data files of `package`: its installed share
/// directory + "/data/", unless the environment variable CORO_DATA_DIR_<PACKAGE> (package
/// name upper-cased, for example CORO_DATA_DIR_MODULE4) names another directory. The override
/// lets the tests and users run the programs on their own configuration without editing the
/// installed files. Returns an empty string if the package is unknown and there is no override.
inline std::string dataDirectory(const std::string & package)
{
  std::string variable = "CORO_DATA_DIR_";
  for (char c : package) {
    variable += static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  }
  const char * override_dir = std::getenv(variable.c_str());
  std::string directory;
  if (override_dir != nullptr && override_dir[0] != '\0') {
    directory = override_dir;
  } else {
    directory = packagePath(package);
    if (directory.empty()) {
      return directory;
    }
    directory += "/data";
  }
  if (directory.back() != '/') {
    directory += '/';
  }
  return directory;
}

}  // namespace coro_common
