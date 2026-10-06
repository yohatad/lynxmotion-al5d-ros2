#pragma once

#include <string>
#include <vector>

namespace lynxmotion_al5d
{

enum class ParseStatus {Ok, Help, Error};

struct SpawnArgs
{
  std::string color;
  std::string name;
  double x{0.0}, y{0.0}, z{0.0};
  double roll{0.0}, pitch{0.0}, yaw{0.0};
};

struct SpawnParse
{
  ParseStatus status{ParseStatus::Error};
  std::string error;
  SpawnArgs args;
};

struct KillParse
{
  ParseStatus status{ParseStatus::Error};
  std::string error;
  std::string name;
};

/// `args` excludes the program name:  -c color [-n name] [-x x] [-y y] [-z z] [-R r] [-P p] [-Y y]
SpawnParse parseSpawnArgs(const std::vector<std::string> & args);
/// `args` excludes the program name:  name
KillParse parseKillArgs(const std::vector<std::string> & args);

std::string spawnUsage();
std::string killUsage();

}  // namespace lynxmotion_al5d
