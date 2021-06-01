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
  bool ret = true;
  auto pub_thread = std::thread(
    [&]() {
      std::cout << "8888888" << std::endl;
      auto A1_node = rclcpp::Node::make_shared("A1_node");
      auto vel_sub = A1_node->create_subscription<geometry_msgs::msg::Twist>(
        ROS2_TOPIC_SET_VELOCITY, 10,
        [this](geometry_msgs::msg::Twist::UniquePtr msg) {
          RCLCPP_INFO(
            rclcpp::get_logger("rcv_vel"),
            "forwardSpeed[%0.2f],sideSpeed[%0.2f],rotateSpeed[%0.2f]",
            msg->linear.x, msg->linear.y, msg->angular.z);
        });
      rclcpp::spin(A1_node);
    });

  unsigned int pid;
#if 0
  pid = fork();
  if (pid < 0) {
    return;
  } else if (pid == 0) {
    std::cout << "start child process 1" << std::endl;
    startTestPthread start("a1_node", HIGH_LEVEL);
    std::cout << "start child process 2" << std::endl;
    start.pthreadLoop();
  }
#endif
#if 0
  std::cout << "parent process " << std::endl;
  // startTestPthread start("a1_node", HIGH_LEVEL);
  TestNode client(CMD_SET_MODE);
  int cnt = 2;
  while (cnt--) {
    client.wait_time(1);
    ret = client.client_set_mode(2);
    EXPECT_EQ(ret, true);
    client.wait_time(1);
    // uint8_t mode = start.a1_ros.wrapper.highCmd.mode;
    // EXPECT_EQ(mode, 2);
  }
#endif
  TestNode pub_vel(CMD_SET_VEL);
  int cnt = 2;
  while (cnt--) {
    float forwardSpeed = 0.2;
    float sideSpeed = 0.2;
    float rotateSpeed = 0.2;
    ret = pub_vel.pub_velocity(forwardSpeed, sideSpeed, rotateSpeed);
    EXPECT_EQ(ret, true);
    pub_vel.wait_time(2);
    // EXPECT_EQ(forwardSpeed, start.a1_ros.wrapper.highCmd.forwardSpeed);
    // EXPECT_EQ(sideSpeed, start.a1_ros.wrapper.highCmd.sideSpeed);
    // EXPECT_EQ(rotateSpeed, start.a1_ros.wrapper.highCmd.rotateSpeed);
  }
  rclcpp::shutdown();
  pub_thread.join();
#if 0
  TestNode pub(CMD_SET_POSE);
  cnt = 2;
  while (cnt--) {
    float yaw = 0.3;
    float pitch = 0.3;
    float roll = 0.3;
    float bodyHeight = 0.3;
    pub.pub_pose(yaw, pitch, roll, bodyHeight);
    pub.wait_time(2);
    // EXPECT_EQ(yaw, start.a1_ros.wrapper.highCmd.yaw);
    // EXPECT_EQ(pitch, start.a1_ros.wrapper.highCmd.pitch);
    // EXPECT_EQ(roll, start.a1_ros.wrapper.highCmd.roll);
    // EXPECT_EQ(bodyHeight, start.a1_ros.wrapper.highCmd.bodyHeight);
  }
#endif
#if 0
  std::cout << "kill child process " << std::endl;
  int result = kill(pid, 9);
  std::cout << "result =  " << result << std::endl;
#endif
}
#if 0
// TEST 5: get high state
TEST(A1RosTest, GetHigh) {
  bool ret;
  TestNode client(CMD_GET_HIGH_STATE);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    ret = client.client_node_get_high_state();
    EXPECT_EQ(ret, false);
  }
}


// TEST 2: set velocity
TEST(A1RosTest, PubVelTest) {
  bool ret;
  startTestPthread start("a1_node", HIGH_LEVEL);
  sleep(1);
  TestNode pub(CMD_SET_VEL);
  int cnt = 1;
  while (cnt--) {
    sleep(1);
    ret = pub.pub_velocity(0.1, 0.1, 0.1);
    EXPECT_EQ(ret, true);
  }
}

// TEST 3: set pose
TEST(A1RosTest, PubPoseTest) {
  TestNode pub(CMD_SET_POSE);
  int cnt = 1;
  while (cnt--) {
    sleep(1);
    pub.pub_pose(0.1, 0.1, 0.1, 0.1);
  }
}

// TEST 4: get cartesian msg
TEST(A1RosTest, GetCartesian) {
  bool ret;
  TestNode client(CMD_GET_CARTESIAN);
  int cnt = 3;
  while (cnt--) {
    sleep(1);
    ret = client.client_node_get_cartesian_msg();
    EXPECT_EQ(ret, false);
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
    EXPECT_EQ(ret, false);
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
    EXPECT_EQ(ret, false);
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
    EXPECT_EQ(ret, false);
  }
}

#endif
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  // initialize ROS
  rclcpp::init(argc, argv);
  // startTestPthread start("a1_node", HIGH_LEVEL);
  bool all_successful = RUN_ALL_TESTS();
  // shutdown ROS
  rclcpp::shutdown();
  return all_successful;
}
