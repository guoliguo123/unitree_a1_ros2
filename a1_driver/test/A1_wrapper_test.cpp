// Copyright 2015 Open Source Robotics Foundation, Inc.
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
#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "a1_driver/A1_wrapper.hpp"
// Test 1 : Setup mode
TEST(A1WrapperTest, SetupModeTest) {
  bool ret;
  A1Wrapper wrapper = A1Wrapper(HIGH_LEVEL);
  uint8_t mode = 2;
  ret = wrapper.set_mode(mode);
  EXPECT_EQ(mode, wrapper.highCmd.mode);
  EXPECT_EQ(ret, true);
  mode = 3;
  ret = wrapper.set_mode(mode);
  EXPECT_EQ(ret, false);
}

// Test 2 : Setup velocity
TEST(A1WrapperTest, SetupVelTest) {
  A1Wrapper wrapper = A1Wrapper(STARTUP_SPORT_MODE, HIGH_LEVEL);
  // Test 2.1: Setup velocity
  bool ret;
  float forwardSpeed = 0.2;
  float sideSpeed = 0.2;
  float rotateSpeed = 0.2;
  ret = wrapper.set_velocity(forwardSpeed, sideSpeed, rotateSpeed);
  EXPECT_EQ(forwardSpeed, wrapper.highCmd.forwardSpeed);
  EXPECT_EQ(sideSpeed, wrapper.highCmd.sideSpeed);
  EXPECT_EQ(rotateSpeed, wrapper.highCmd.rotateSpeed);
  EXPECT_EQ(rotateSpeed, wrapper.highCmd.rotateSpeed);
  EXPECT_EQ(CMD_SET_MODE_WALK, wrapper.highCmd.mode);
  EXPECT_EQ(ret, true);
  // Test 2.2: Clear set
  ret = wrapper.set_velocity(0.0, 0.0, 0.0);
  EXPECT_EQ(0.0, wrapper.highCmd.forwardSpeed);
  EXPECT_EQ(0.0, wrapper.highCmd.sideSpeed);
  EXPECT_EQ(0.0, wrapper.highCmd.rotateSpeed);
  EXPECT_EQ(CMD_SET_MODE_FORCE_STAND, wrapper.highCmd.mode);
  EXPECT_EQ(ret, true);
  // Test 2.3: Check vaild
  float set_speed;
  for (int i = 0; i < 2; i++) {
    if (i == 0) {
      set_speed = 1.2;
    } else {
      set_speed = -1.2;
    }
    ret = wrapper.set_velocity(set_speed, 0.0, 0.0);
    EXPECT_EQ(ret, false);
    ret = wrapper.set_velocity(0.0, set_speed, 0.0);
    EXPECT_EQ(ret, false);
    ret = wrapper.set_velocity(0.0, 0.0, set_speed);
    EXPECT_EQ(ret, false);

    // Test 3: Setup pose
    float yaw = 0.3;
    float pitch = 0.3;
    float roll = 0.3;
    float bodyHeight = 0.5;
    ret = wrapper.set_pose(yaw, pitch, roll, bodyHeight);
    EXPECT_EQ(yaw, wrapper.highCmd.yaw);
    EXPECT_EQ(pitch, wrapper.highCmd.pitch);
    EXPECT_EQ(roll, wrapper.highCmd.roll);
    EXPECT_EQ(bodyHeight, wrapper.highCmd.bodyHeight);
    EXPECT_EQ(ret, true);

    ret = wrapper.set_pose(1.2, 1.2, 1.2, 1.2);
    EXPECT_EQ(ret, false);
    ret = wrapper.set_pose(-1.2, -1.2, -1.2, -1.2);
    EXPECT_EQ(ret, false);

    ret = wrapper.set_pose(0.0, 0.0, 0.0, 0.0);
    EXPECT_EQ(ret, true);
  }
}
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  // initialize ROS
  rclcpp::init(argc, argv);
  bool all_successful = RUN_ALL_TESTS();
  // shutdown ROS
  rclcpp::shutdown();
  return all_successful;
}
