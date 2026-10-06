#include "lynxmotion_al5d_description/cli_args.hpp"

#include <cmath>
#include <stdexcept>

namespace lynxmotion_al5d
{

std::string spawnUsage()
{
  return "Usage: ros2 run lynxmotion_al5d_description spawn_brick -c color [-n name] "
         "[-x x] [-y y] [-z z] [-R roll] [-P pitch] [-Y yaw]\n"
         "  color: red | green | blue; positions in metres, angles in radians";
}

std::string killUsage()
{
  return "Usage: ros2 run lynxmotion_al5d_description kill_brick name";
}

SpawnParse parseSpawnArgs(const std::vector<std::string> & args)
{
  SpawnParse result;
  auto fail = [&result](const std::string & message) {
      result.status = ParseStatus::Error;
      result.error = message;
      return result;
    };

  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string & flag = args[i];
    if (flag == "-h" || flag == "--help") {
      result.status = ParseStatus::Help;
      return result;
    }
    if (flag.size() != 2 || flag[0] != '-') {
      return fail("unexpected argument '" + flag + "'");
    }
    if (i + 1 >= args.size()) {
      return fail("option '" + flag + "' needs a value");
    }
    const std::string & value = args[++i];
    if (flag == "-c") {
      result.args.color = value;
    } else if (flag == "-n") {
      result.args.name = value;
    } else {
      double * target = nullptr;
      switch (flag[1]) {
        case 'x': target = &result.args.x; break;
        case 'y': target = &result.args.y; break;
        case 'z': target = &result.args.z; break;
        case 'R': target = &result.args.roll; break;
        case 'P': target = &result.args.pitch; break;
        case 'Y': target = &result.args.yaw; break;
        default: return fail("unknown option '" + flag + "'");
      }
      try {
        std::size_t consumed = 0;
        *target = std::stod(value, &consumed);
        if (consumed != value.size() || !std::isfinite(*target)) {
          throw std::invalid_argument(value);
        }
      } catch (const std::exception &) {
        return fail("'" + value + "' is not a valid number for option '" + flag + "'");
      }
    }
  }
  if (result.args.color.empty()) {
    return fail("the brick colour (-c) is required");
  }
  result.status = ParseStatus::Ok;
  return result;
}

KillParse parseKillArgs(const std::vector<std::string> & args)
{
  KillParse result;
  if (args.size() == 1 && (args[0] == "-h" || args[0] == "--help")) {
    result.status = ParseStatus::Help;
    return result;
  }
  if (args.size() != 1) {
    result.error = "exactly one brick name is required";
    return result;
  }
  result.name = args[0];
  result.status = ParseStatus::Ok;
  return result;
}

}  // namespace lynxmotion_al5d
