#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "tmr_rostest");
  ros::NodeHandle nh("tmr_rostest");

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}