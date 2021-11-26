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
  _is_calibrating = false;
  state_ = Solo8State::initial;
}

void Solo8::initialize(const std::string& network_id) {
  // joint limit parameters TODO: set as parameters
  double max_hip_angle = M_PI * 4;
  double max_knee_angle = M_PI * 4;
  double max_joint_velocities = 160.0;
  double safety_damping = 0.2;

  // define the master board
  main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id);

  // motor indices and polarities
  VectorXi motor_numbers(8);
  motor_numbers << 0, 1, 3, 2, 5, 4, 6, 7;
  VectorXb motor_reversed(8);
  motor_reversed << true, true, false, false, true, true, false, false;

  // joint angle limits
  Eigen::VectorXd joint_lower_limits(8);
  joint_lower_limits << -max_hip_angle, -max_knee_angle, -max_hip_angle,
      -max_knee_angle, -max_hip_angle, -max_knee_angle, -max_hip_angle,
      -max_knee_angle;
  Eigen::VectorXd joint_upper_limits(8);
  joint_upper_limits << max_hip_angle, max_knee_angle, max_hip_angle,
      max_knee_angle, max_hip_angle, max_knee_angle, max_hip_angle,
      max_knee_angle;

  // define the joint module
  joints_ = std::make_shared<odri_control_interface::JointModules>(
      main_board_ptr_, motor_numbers, motor_torque_constants_(0),
      joint_gear_ratios_(0), motor_max_current_(0), motor_reversed,
      joint_lower_limits, joint_upper_limits, max_joint_velocities,
      safety_damping);

  // define the IMU
  imu_ = std::make_shared<odri_control_interface::IMU>(main_board_ptr_);

  // define the joint calibrator
  Eigen::VectorXd position_offsets(8);
  position_offsets.fill(0.);
  std::vector<odri_control_interface::CalibrationMethod> directions{
      odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
      odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
      odri_control_interface::POSITIVE, odri_control_interface::POSITIVE,
      odri_control_interface::POSITIVE, odri_control_interface::POSITIVE};
  double calib_Kp = 5.0;
  double calib_Kd = 0.05;
  double calib_T = 1.0;
  double calib_dt = 0.001;
  calib_ctrl_ = std::make_shared<odri_control_interface::JointCalibrator>(
      joints_, directions, position_offsets, calib_Kp, calib_Kd, calib_T,
      calib_dt);

  // define the robot
  robot_ = std::make_shared<odri_control_interface::Robot>(
      main_board_ptr_, joints_, imu_, calib_ctrl_);

  // initialize the robot
  robot_->Init();
}

void Solo8::acquire_sensors() {
  robot_->ParseSensorData();
  auto joints = robot_->joints;
  auto imu = robot_->imu;

  // joint data
  joint_positions_ = joints->GetPositions();
  joint_velocities_ = joints->GetVelocities();
  joint_torques_ = joints->GetMeasuredTorques();
  joint_target_torques_ = joints->GetSentTorques();

  // imu data
  imu_linear_acceleration_ = imu->GetLinearAcceleration();
  imu_accelerometer_ = imu->GetAccelerometer();
  imu_gyroscope_ = imu->GetGyroscope();
  imu_attitude_ = imu->GetAttitudeEuler();
  imu_attitude_quaternion_ = imu->GetAttitudeQuaternion();

  // motor status
  ConstRefVectorXb motor_enabled = joints->GetEnabled();
  ConstRefVectorXb motor_ready = joints->GetReady();
  for (int i = 0; i < 8; i++) {
    motor_enabled_[i] = motor_enabled[i];
    motor_ready_[i] = motor_ready[i];
  }

  // motor board status
  ConstRefVectorXi motor_board_errors = joints->GetMotorDriverErrors();
  ConstRefVectorXb motor_driver_enabled = joints->GetMotorDriverEnabled();
  for (int i = 0; i < 4; i++) {
    motor_board_errors_[i] = motor_board_errors[i];
    motor_board_enabled_[i] = motor_driver_enabled[i];
  }
}

}  // namespace solo
