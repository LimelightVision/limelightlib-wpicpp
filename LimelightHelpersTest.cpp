
#include "gtest/gtest.h"

#include "LimelightHelpers.h"

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}

TEST(LimelightHelpersTest, BasicTest) {
    EXPECT_TRUE(static_cast<bool>(LimelightHelpers::getLimelightNTTable("limelight")));
}