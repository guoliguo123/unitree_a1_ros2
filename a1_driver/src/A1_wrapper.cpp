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

#include "a1_driver/A1_wrapper.hpp"
#include <time.h>
bool A1Wrapper::set_velocity(
  float forwardSpeed, float sideSpeed,
  float rotateSpeed)
{
  if (forwardSpeed > POSITIVE_WALK_SPEED_MAX || forwardSpeed < NEGATIVE_WALK_SPEED_MAX)
      return false;
  if (sideSpeed > POSITIVE_WALK_SPEED_MAX || sideSpeed < NEGATIVE_WALK_SPEED_MAX)
      return false;
  if (rotateSpeed > POSITIVE_WALK_SPEED_MAX || rotateSpeed < NEGATIVE_WALK_SPEED_MAX)
      return false;
  memset(&highCmd, 0, sizeof(highCmd));
  if (forwardSpeed == 0 && sideSpeed == 0 && rotateSpeed == 0) {
      highCmd.mode = CMD_SET_MODE_FORCE_STAND;
  } else {
      highCmd.mode = CMD_SET_MODE_WALK;
      highCmd.forwardSpeed = forwardSpeed;
      highCmd.sideSpeed = sideSpeed;
      highCmd.rotateSpeed = rotateSpeed;
  }
  udp.SetSend(highCmd);
  return true;
}

bool A1Wrapper::set_mode(uint8_t mode)
{
  if (mode > CMD_SET_MODE_WALK)
    return false;
  memset(&highCmd, 0, sizeof(highCmd));
  highCmd.mode = mode;
  if (udp.SetSend(highCmd) != 0) {
    return false;
  }

  return true;
}

bool A1Wrapper::set_pose(float yaw, float pitch, float roll, float bodyHeight)
{
  if (yaw > POSITIVE_WALK_SPEED_MAX || yaw < NEGATIVE_WALK_SPEED_MAX)
    return false;
  if (pitch > POSITIVE_WALK_SPEED_MAX || pitch < NEGATIVE_WALK_SPEED_MAX)
    return false;
  if (roll > POSITIVE_WALK_SPEED_MAX || roll < NEGATIVE_WALK_SPEED_MAX)
    return false;
  if (bodyHeight > POSITIVE_WALK_SPEED_MAX || bodyHeight < NEGATIVE_WALK_SPEED_MAX)
    return false;
  memset(&highCmd, 0, sizeof(highCmd));
  highCmd.mode = CMD_SET_MODE_FORCE_STAND;
  highCmd.yaw = yaw;
  highCmd.pitch = pitch;
  highCmd.roll = roll;
  highCmd.bodyHeight = bodyHeight;
  udp.SetSend(highCmd);
  return true;
}

void A1Wrapper::recv_imu_msg() {udp.GetRecv(highState);}

void A1Wrapper::recv_cartesian_msg() {udp.GetRecv(highState);}

void A1Wrapper::recv_high_state() {udp.GetRecv(highState);}

void A1Wrapper::recv_low_state() {udp.GetRecv(lowState);}
