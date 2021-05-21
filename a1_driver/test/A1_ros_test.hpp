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
#ifndef A1_ROS_TEST_HPP_
#define A1_ROS_TEST_HPP_
#include <boost/thread/thread.hpp>
#include <gtest/gtest.h>
#include <string>
#include <memory>
#include "a1_driver/A1_ros.hpp"
#include "a1_driver/A1_wrapper.hpp"

enum command
{
  CMD_SET_MODE,
  CMD_SET_VEL,
  CMD_SET_POSE,
  CMD_GET_HIGH_STATE,
  CMD_GET_LOW_STATE,
  CMD_GET_IMU,
  CMD_GET_CARTESIAN
};
using namespace std::chrono_literals;

class startTestPthread
{
public:
  startTestPthread(std::string node_name, int level)
  : a1_ros(node_name, level)
  {
    t = boost::thread(boost::bind(&startTestPthread::pthreadLoop, this));
  }
  void pthreadLoop()
  {
    a1_ros.node_init();
  }
  A1ROS a1_ros;
  boost::thread t;
};

class TestNode
{
public:
  explicit TestNode(uint8_t command);
  bool client_set_mode(uint8_t mode);
  bool pub_velocity(float forwardSpeed, float sideSpeed, float rotateSpeed);
  void pub_pose(float yaw, float pitch, float roll, float bodyHeight);
  bool client_node_get_high_state();
  bool client_node_get_low_state();
  bool client_node_get_imu_msg();
  bool client_node_get_cartesian_msg();

private:
  rclcpp::Client<a1_msgs::srv::Mode>::SharedPtr mode_client;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr walk_pub;
  rclcpp::Publisher<a1_msgs::msg::Pose>::SharedPtr pose_pub;
  rclcpp::Client<a1_msgs::srv::HighState>::SharedPtr high_state_client;
  rclcpp::Client<a1_msgs::srv::LowState>::SharedPtr lowState_client;
  rclcpp::Client<a1_msgs::srv::Imu>::SharedPtr imu_client;
  rclcpp::Client<a1_msgs::srv::Cartesian>::SharedPtr cartesian_client;
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::TimerBase::SharedPtr timer_;
  uint8_t set_mode;
  size_t count_ = 5;
};

TestNode::TestNode(uint8_t command)
{
  switch (command) {
    case CMD_SET_MODE:
      node = rclcpp::Node::make_shared("server_client");
      mode_client = node->create_client<a1_msgs::srv::Mode>(ROS2_SERVICE_SET_MODE);
      break;
    case CMD_SET_VEL:
      node = rclcpp::Node::make_shared("set_vel");
      walk_pub = node->create_publisher<geometry_msgs::msg::Twist>(ROS2_TOPIC_SET_VELOCITY, 10);
      break;
    case CMD_SET_POSE:
      node = rclcpp::Node::make_shared("set_pose");
      pose_pub = node->create_publisher<a1_msgs::msg::Pose>(ROS2_TOPIC_SET_POSE, 10);
      break;
    case CMD_GET_HIGH_STATE:
      node = rclcpp::Node::make_shared("High_State");
      high_state_client = node->create_client<a1_msgs::srv::HighState>(
        ROS2_SERVICE_GET_HIGH_STATE_MSG);
    case CMD_GET_LOW_STATE:
      node = rclcpp::Node::make_shared("LowState");
      lowState_client = node->create_client<a1_msgs::srv::LowState>(ROS2_SERVICE_GET_LOW_STATE_MSG);
    case CMD_GET_IMU:
      node = rclcpp::Node::make_shared("GetImuMsg");
      imu_client = node->create_client<a1_msgs::srv::Imu>(ROS2_SERVICE_GET_IMU_MSG);
    case CMD_GET_CARTESIAN:
      node = rclcpp::Node::make_shared("GetCartesianMsg");
      cartesian_client =
        node->create_client<a1_msgs::srv::Cartesian>(ROS2_SERVICE_GET_CARTESIAN_MSG);
    default:
      return;
  }
}

void TestNode::pub_pose(float yaw, float pitch, float roll, float bodyHeight)
{
  a1_msgs::msg::Pose msg;
  msg.yaw = yaw;
  msg.pitch = pitch;
  msg.roll = roll;
  msg.bodyheight = bodyHeight;
  RCLCPP_INFO(
    rclcpp::get_logger("rclcpp"), "yaw[%f], pitch[%f], roll[%f], bodyHeight[%f]",
    msg.yaw, msg.pitch, msg.roll, msg.bodyheight);
  pose_pub->publish(msg);
}
bool TestNode::pub_velocity(float forwardSpeed, float sideSpeed, float rotateSpeed)
{
  geometry_msgs::msg::Twist msg;
  msg.linear.x = forwardSpeed;
  msg.linear.y = sideSpeed;
  msg.angular.z = rotateSpeed;
  RCLCPP_INFO(
    rclcpp::get_logger("rclcpp"), "sending forwardSpeed[%f],sideSpeed[%f],rotateSpeed[%f]",
    msg.linear.x, msg.linear.y, msg.angular.z);
  walk_pub->publish(msg);
  return true;
}
bool TestNode::client_set_mode(uint8_t mode)
{
  size_t times = 0;

  auto request = std::make_shared<a1_msgs::srv::Mode::Request>();
  while (!mode_client->wait_for_service(1s)) {
    times++;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting times(%d)...", times);
    if (count_ == times) {
      return false;
    }
  }
  request->mode = mode;
  auto result = mode_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    auto cmd = result.get();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service return value : %d", cmd->value);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }

  return true;
}

bool TestNode::client_node_get_high_state()
{
  size_t times = 0;
  auto request = std::make_shared<a1_msgs::srv::HighState::Request>();

  while (!high_state_client->wait_for_service(1s)) {
    times++;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting times(%d)...", times);
    if (count_ == times) {
      return false;
    }
  }

  auto result = high_state_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    // auto HighState = result.get();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    return false;
  }
  return true;
}

bool TestNode::client_node_get_low_state()
{
  size_t times = 0;
  auto request = std::make_shared<a1_msgs::srv::LowState::Request>();
  while (!lowState_client->wait_for_service(1s)) {
    times++;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting times(%d)...", times);
    if (count_ == times) {
      return false;
    }
  }

  auto result = lowState_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    // auto LowState = result.get();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    return false;
  }
  return true;
}

bool TestNode::client_node_get_imu_msg()
{
  size_t times = 0;
  auto request = std::make_shared<a1_msgs::srv::Imu::Request>();
  while (!imu_client->wait_for_service(1s)) {
    times++;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting times(%d)...", times);
    if (count_ == times) {
      return false;
    }
  }

  auto result = imu_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    // auto IMU = result.get();

  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    return false;
  }
  return true;
}

bool TestNode::client_node_get_cartesian_msg()
{
  size_t times = 0;
  auto request = std::make_shared<a1_msgs::srv::Cartesian::Request>();
  while (!cartesian_client->wait_for_service(1s)) {
    times++;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting times(%d)...", times);
    if (count_ == times) {
      return false;
    }
  }

  auto result = cartesian_client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    auto Cartesian = result.get();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
    return false;
  }
  return true;
}
#endif  // A1_ROS_TEST_HPP_
