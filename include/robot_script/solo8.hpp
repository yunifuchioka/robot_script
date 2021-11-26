/**
 * Class that represents the hardware Solo 8 robot
 * This is mostly copy+pasted from open dynamic robot initiative solo
 * https://raw.githubusercontent.com/open-dynamic-robot-initiative/solo/master/include/solo/solo8.hpp
 */

#pragma once

#include <odri_control_interface/calibration.hpp>
#include <odri_control_interface/robot.hpp>

#include "common.hpp"

namespace solo {

enum Solo8State { initial, ready, calibrate };

class Solo8 {
 public:
  /**
   * constructor
   */
  Solo8();

  /**
   * initialize the robot by setting aligning the motors and calibrate the
   * sensors to 0
   */
  void initialize(const std::string& network_id);

 private:
  /**
   * Joint properties
   */
  Vector8d motor_inertias_;         /**< motors inertia. */
  Vector8d motor_torque_constants_; /**< DCM motor torque constants. */
  Vector8d joint_gear_ratios_;      /**< joint gear ratios (9). */
  Vector8d motor_max_current_;      /**< Max appliable current before the robot
                                       shutdown. */
  Vector8d joint_zero_positions_;   /**< Offset to the theoretical "0" pose. */
  Eigen::Array<double, 8, 1> max_joint_torques_; /**< Max joint torques (Nm) */

  /**
   * @brief Main board drivers.
   *
   * PC <- Ethernet/Wifi -> main board <- SPI -> Motor Board
   */
  std::shared_ptr<MasterBoardInterface> main_board_ptr_;
};

}  // namespace solo