#include <gtest/gtest.h>

#include <fstream>
#include <string>

#include "coro_common/package_path.hpp"

TEST(PackagePath, FindsInstalledPackage)
{
  const std::string path = coro_common::packagePath("coro_common");
  ASSERT_FALSE(path.empty());
  std::ifstream marker(path + "/package.xml");
  EXPECT_TRUE(marker.good()) << "package.xml should be installed next to " << path;
}

TEST(PackagePath, UnknownPackageGivesEmptyString)
{
  EXPECT_TRUE(coro_common::packagePath("no_such_package_xyz").empty());
}

TEST(PackagePath, DoesNotThrowOnEmptyName)
{
  EXPECT_NO_THROW(coro_common::packagePath(""));
}

#include <cstdlib>

TEST(DataDirectory, DefaultsToInstalledDataFolderWithTrailingSlash)
{
  unsetenv("CORO_DATA_DIR_CORO_COMMON");
  const std::string dir = coro_common::dataDirectory("coro_common");
  ASSERT_FALSE(dir.empty());
  EXPECT_EQ(dir.back(), '/');
  EXPECT_NE(dir.find("/share/coro_common/data/"), std::string::npos) << dir;
}

TEST(DataDirectory, EnvironmentVariableOverrides)
{
  setenv("CORO_DATA_DIR_CORO_COMMON", "/tmp/my_data", 1);
  EXPECT_EQ(coro_common::dataDirectory("coro_common"), "/tmp/my_data/");
  setenv("CORO_DATA_DIR_CORO_COMMON", "/tmp/my_data/", 1);
  EXPECT_EQ(coro_common::dataDirectory("coro_common"), "/tmp/my_data/");
  unsetenv("CORO_DATA_DIR_CORO_COMMON");
}

TEST(DataDirectory, EmptyOverrideIsIgnored)
{
  setenv("CORO_DATA_DIR_CORO_COMMON", "", 1);
  EXPECT_NE(coro_common::dataDirectory("coro_common").find("coro_common"), std::string::npos);
  unsetenv("CORO_DATA_DIR_CORO_COMMON");
}

TEST(DataDirectory, UnknownPackageWithoutOverrideIsEmpty)
{
  unsetenv("CORO_DATA_DIR_NO_SUCH_PACKAGE_XYZ");
  EXPECT_TRUE(coro_common::dataDirectory("no_such_package_xyz").empty());
}
