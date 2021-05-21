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

#include "A1_ros_test.hpp"
// TEST 1: set mode
TEST(A1RosTest, SetModeTest) {
  bool ret;
  TestNode client(CMD_SET_MODE);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    ret = client.client_set_mode(2);
    EXPECT_EQ(ret, true);
  }
}
// TEST 2: set velocity
TEST(A1RosTest, PubVelTest) {
  bool ret;
  TestNode pub(CMD_SET_VEL);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    ret = pub.pub_velocity(0.1, 0.1, 0.1);
    EXPECT_EQ(ret, true);
  }
}
// TEST 3: set pose
TEST(A1RosTest, PubPoseTest) {
  TestNode pub(CMD_SET_POSE);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    pub.pub_pose(0.1, 0.1, 0.1, 0.1);
  }
}
#if 0
// TEST 4: get cartesian msg
TEST(A1RosTest, GetCartesian) {
  bool ret;
  TestNode client(CMD_GET_CARTESIAN);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    ret = client.client_node_get_cartesian_msg();
    EXPECT_EQ(ret, true);
  }
}

// TEST 5: get high state
TEST(A1RosTest, GetHigh) {
  bool ret;
  TestNode client(CMD_GET_HIGH_STATE);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    ret = client.client_node_get_high_state();
    EXPECT_EQ(ret, true);
  }
}
// TEST 6: get low state
TEST(A1RosTest, GetLow) {
  bool ret;
  TestNode client(CMD_GET_LOW_STATE);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    ret = client.client_node_get_low_state();
    EXPECT_EQ(ret, true);
  }
}

// TEST 7: get imu msg
TEST(A1RosTest, GetImu) {
  bool ret;
  TestNode client(CMD_GET_IMU);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    ret = client.client_node_get_imu_msg();
    EXPECT_EQ(ret, true);
  }
}
#endif
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  // initialize ROS
  rclcpp::init(argc, argv);
  startTestPthread start("a1_node", HIGH_LEVEL);
  bool all_successful = RUN_ALL_TESTS();
  // shutdown ROS
  rclcpp::shutdown();
  return all_successful;
}
