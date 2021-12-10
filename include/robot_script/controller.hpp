/**
 * Abstract class for a controller
 * Inputs: robot sensor data, other miscellaneous inputs (eg. time)
 * Outputs: PD+torque targets for the robot
 * All IO is done through private variables and getter/setter functions
 */

#pragma once

#include "solo8.hpp"

using namespace solo;

class Controller {
 public:
  /**
   * constructor to initialize internal variables
   */
  Controller(const std::shared_ptr<Solo8>& robot) {
    robot_ = robot;
    desired_positions_.setZero();
    desired_velocities_.setZero();
    desired_torques_.setZero();
  }

  /**
   * reads sensor data and updates internal PD+torque target variables
   * Should be implemented for each child class
   *
   * WARNING !!!!
   * The method robot_->acquire_sensors() has to be called prior to this
   * function call in order to compute the control with up to date sensor data
   */
  virtual void calc_control() = 0;

  /**
   * Getters for PD+torque targets computed by calc_control()
   */
  const Eigen::Ref<Vector8d> get_desired_positions() {
    return desired_positions_;
  }
  const Eigen::Ref<Vector8d> get_desired_velocities() {
    return desired_velocities_;
  }
  const Eigen::Ref<Vector8d> get_desired_torques() { return desired_torques_; }

 protected:
  /**
   * keep a pointer to the robot in order to read off sensor data
   * the intention is not for any commands to be sent to the robot from this
   * class--that should happen in the consumer code (eg main control loop)
   */
  std::shared_ptr<Solo8> robot_;

  /**
   * stores PD+Torque targets
   */
  Vector8d desired_positions_;
  Vector8d desired_velocities_;
  Vector8d desired_torques_;
};