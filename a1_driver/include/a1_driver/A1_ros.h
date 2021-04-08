//
// Created by sun on 2021/3/1.
//

#ifndef UNITREE_DRIVER_A1_ROS_H
#define UNITREE_DRIVER_A1_ROS_H

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "a1_driver/A1_wrapper.h"
#include "a1_comm/A1_comm.h"
#include "geometry_msgs/msg/twist.hpp"
#include "a1_msgs/srv/mode.hpp"
#include "a1_msgs/srv/high_state.hpp"
#include "a1_msgs/srv/imu.hpp"
#include "a1_msgs/srv/cartesian.hpp"
#include "a1_msgs/msg/pose.hpp"

class A1ROS{
public:
    A1ROS(std::string node_name) : wrapper(A1Wrapper()) {
        this->node_name = node_name;
    };
    A1ROS(std::string node_name, uint8_t sport_mode) : wrapper(A1Wrapper(sport_mode)) {
        this->node_name = node_name;
    };
    int node_init(int argc, char *argv[]);
    void set_mode(const std::shared_ptr<a1_msgs::srv::Mode::Request> request,
                    std::shared_ptr<a1_msgs::srv::Mode::Response> response);
    void get_high_state_msg(const std::shared_ptr<a1_msgs::srv::HighState::Request> request,
                    std::shared_ptr<a1_msgs::srv::HighState::Response> response);
    void get_imu_msg(const std::shared_ptr<a1_msgs::srv::Imu::Request> request,
                    std::shared_ptr<a1_msgs::srv::Imu::Response> response);
    void get_cartesian_msg(const std::shared_ptr<a1_msgs::srv::Cartesian::Request> request,
                std::shared_ptr<a1_msgs::srv::Cartesian::Response> response);
    A1Wrapper wrapper;
private:
    std::string node_name;
};

#endif //UNITREE_DRIVER_A1_ROS_H