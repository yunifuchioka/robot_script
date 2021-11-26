#include "solo8.hpp"

#include <odri_control_interface/common.hpp>

namespace solo {

using namespace odri_control_interface;

Solo8::Solo8() {
  motor_inertias_.setZero();
  motor_torque_constants_.setZero();
  joint_gear_ratios_.setZero();
  motor_max_current_.setZero();
  max_joint_torques_.setZero();
  joint_zero_positions_.setZero();
}

void Solo8::initialize(const std::string& network_id) {
  main_board_ptr_ = std::make_shared<MasterBoardInterface>(network_id);
}

}  // namespace solo
