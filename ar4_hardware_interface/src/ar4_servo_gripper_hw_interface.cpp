#include <algorithm>
#include <ar4_hardware_interface/ar4_servo_gripper_hw_interface.hpp>
#include <sstream>

namespace ar4_hardware_interface {

hardware_interface::CallbackReturn ARServoGripperHWInterface::on_init(
    const hardware_interface::HardwareComponentInterfaceParams& params) {
  RCLCPP_INFO(logger_, "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Finger travel (m, finger_joint_1) and the servo angles (deg) at its two ends; the mapping is linear.
  const auto& hw = info_.hardware_parameters;
  for (const char* name : {"closed_position", "open_position", "closed_servo_angle", "open_servo_angle"}) {
    if (hw.count(name) == 0) {
      RCLCPP_ERROR(logger_, "Required parameter '%s' not found", name);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  closed_position_ = std::stod(hw.at("closed_position"));
  open_position_ = std::stod(hw.at("open_position"));
  closed_servo_angle_ = std::stoi(hw.at("closed_servo_angle"));
  open_servo_angle_ = std::stoi(hw.at("open_servo_angle"));
  RCLCPP_INFO(
      logger_, "Gripper: %.4f m (closed) = %d deg, %.4f m (open) = %d deg", closed_position_, closed_servo_angle_,
      open_position_, open_servo_angle_);

  if (closed_position_ == open_position_) {
    RCLCPP_ERROR(logger_, "closed_position and open_position must differ");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Validate servo angle range
  if (closed_servo_angle_ >= open_servo_angle_) {
    RCLCPP_ERROR(
        logger_, "Invalid servo angle range: min (%d) must be less than max (%d)", closed_servo_angle_,
        open_servo_angle_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::string serial_port = info_.hardware_parameters.at("serial_port");
  int baud_rate = 115200;
  bool success = driver_.init(serial_port, baud_rate);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARServoGripperHWInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Activating hardware interface...");

  // initialize gripper position
  int pos_deg;
  bool success = driver_.getPosition(pos_deg);
  if (!success) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  position_ = servo_angle_to_linear_pos(pos_deg);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ARServoGripperHWInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Deactivating hardware interface...");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ARServoGripperHWInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(info_.joints[i].name, "position", &position_);
    state_interfaces.emplace_back(info_.joints[i].name, "velocity", &velocity_);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ARServoGripperHWInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(info_.joints[i].name, "position", &position_command_);
  }
  return command_interfaces;
}

hardware_interface::return_type ARServoGripperHWInterface::read(
    const rclcpp::Time& time,
    const rclcpp::Duration& /*period*/) {
  int pos_deg;
  bool success = driver_.getPosition(pos_deg);
  if (!success) {
    RCLCPP_ERROR(logger_, "Failed to read position from servo");
    return hardware_interface::return_type::ERROR;
  }
  position_ = servo_angle_to_linear_pos(pos_deg);
  std::string logInfo = "Gripper Pos: " + std::to_string(position_);
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ARServoGripperHWInterface::write(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  double position_command = position_command_;

  int pos_deg = linear_pos_to_servo_angle(position_command);
  std::string logInfo = "Gripper Cmd: " + std::to_string(pos_deg);
  RCLCPP_DEBUG_THROTTLE(logger_, clock_, 500, logInfo.c_str());
  bool success = driver_.writePosition(pos_deg);
  if (!success) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ar4_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ar4_hardware_interface::ARServoGripperHWInterface, hardware_interface::SystemInterface)
