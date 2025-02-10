// BSD 3-Clause License
//
// Copyright 2025 Ekumen, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef TEENSY_DRIVER_H
#define TEENSY_DRIVER_H

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "math.h"
#include "time.h"

namespace ar4_hardware_interface {

class TeensyDriver {
 public:
  void init(std::string port, int baudrate, int num_joints);
  void setStepperSpeed(std::vector<double>& max_speed, std::vector<double>& max_accel);
  void update(std::vector<double>& pos_commands, std::vector<double>& joint_states);
  void getJointPositions(std::vector<double>& joint_positions);
  void calibrateJoints();

  TeensyDriver();

 private:
  bool initialised_;
  std::string version_;
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  int num_joints_;
  std::vector<double> joint_positions_deg_;
  std::vector<int> enc_calibrations_;
  rclcpp::Logger logger_ = rclcpp::get_logger("teensy_driver");
  rclcpp::Clock clock_ = rclcpp::Clock(RCL_ROS_TIME);

  // Comms with teensy
  void exchange(std::string outMsg);  // exchange joint commands/state
  bool transmit(std::string outMsg, std::string& err);
  void receive(std::string& inMsg);
  void sendCommand(std::string outMsg);  // send arbitrary commands

  void checkInit(std::string msg);
  void updateEncoderCalibrations(std::string msg);
  void updateJointPositions(std::string msg);
};

}  // namespace ar4_hardware_interface

#endif  // TEENSY_DRIVER
