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
// 1.test set velocity
TEST(A1RosTest, SetVelTest) {
  TestRosNode testRos;
  auto ros_thread = std::thread(
    [&]() {
      std::cout << "start ros" << std::endl;
      bool result = testRos.TestRosInit();
      EXPECT_EQ(result, true);
    });

  TestNode pub_vel(CMD_SET_VEL);
  int cnt = 2;
  while (cnt--) {
    float forwardSpeed = 0.2;
    float sideSpeed = 0.2;
    float rotateSpeed = 0.2;
    testRos.TestResult.vel.in_test_cnt++;
    testRos.TestResult.vel.in_forwardSpeed = forwardSpeed;
    testRos.TestResult.vel.in_sideSpeed = sideSpeed;
    testRos.TestResult.vel.in_rotateSpeed = rotateSpeed;
    bool ret = pub_vel.pub_velocity(forwardSpeed, sideSpeed, rotateSpeed);
    EXPECT_EQ(ret, true);
    pub_vel.wait_time(2);
  }
  sleep(1);
  TestNode set_pose(CMD_SET_POSE);
  cnt = 2;
  while (cnt--) {
    float yaw = 0.2;
    float pitch = 0.2;
    float roll = 0.2;
    float bodyHeight = 0.2;

    testRos.TestResult.pose.in_yaw = yaw;
    testRos.TestResult.pose.in_pitch = pitch;
    testRos.TestResult.pose.in_roll = roll;
    testRos.TestResult.pose.in_bodyHeight = bodyHeight;
    std::cout << "test begin " << std::endl;
    set_pose.pub_pose(yaw, pitch, roll, bodyHeight);
    std::cout << "test over" << std::endl;
    set_pose.wait_time(2);
  }
  sleep(1);
  rclcpp::shutdown();
  ros_thread.join();
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
