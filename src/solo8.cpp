#include "solo8.hpp"

#include <odri_control_interface/common.hpp>

namespace solo {

using namespace odri_control_interface;

Solo8::Solo8() {
  /**
   * Joint properties
   */
  motor_inertias_.setZero();
  motor_torque_constants_.setZero();
  joint_gear_ratios_.setZero();
  motor_max_current_.setZero();
  max_joint_torques_.setZero();
  joint_zero_positions_.setZero();
  /**
   * Hardware status
   */
  for (unsigned i = 0; i < motor_enabled_.size(); ++i) {
    motor_enabled_[i] = false;
    motor_ready_[i] = false;
  }
  for (unsigned i = 0; i < motor_board_enabled_.size(); ++i) {
    motor_board_enabled_[0] = false;
    motor_board_errors_[0] = 0;
  }
  /**
   * Joint data
   */
  joint_positions_.setZero();
  joint_velocities_.setZero();
  joint_torques_.setZero();
  joint_target_torques_.setZero();
  joint_encoder_index_.setZero();
  /**
   * IMU data
   */
  imu_accelerometer_.setZero();
  imu_gyroscope_.setZero();
  imu_attitude_.setZero();
  imu_linear_acceleration_.setZero();
  imu_attitude_quaternion_.setZero();
  /**
   * Setup some known data
   */
  motor_max_current_.fill(400.0);  // TODO: set as paramters?
  motor_torque_constants_.fill(0.025);
  motor_inertias_.fill(0.045);
  joint_gear_ratios_.fill(9.0);
  /**
   * Drivers communication objects
   */
  active_estop_ = false;
  _is_calibrating = false;
  state_ = Solo8State::initial;
}

void Solo8::initialize(const std::string& network_id) {
  main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id);
}

}  // namespace solo
