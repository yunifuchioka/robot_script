#include "network_controller.hpp"

using namespace solo;

void NetworkController::initialize_network(const std::string filename) {
  // load neural network model
  try {
    std::string model_dir = "../models/";
    network_ = torch::jit::load(model_dir + filename + "_script.pt");
  } catch (const c10::Error& e) {
    std::cerr << "error loading neural network model\n";
  }
}

void NetworkController::calc_control() {
  switch (controllerState_) {
    case ControllerState::homing:
      desired_positions_.setZero();
      desired_velocities_.setZero();
      desired_torques_.setZero();

      if (robot_->isReady()) {
        controllerState_ = ControllerState::stand;
        time_stamp_state_change_ = time_;
      }
      break;

    case ControllerState::stand:
      desired_positions_ << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;

      desired_positions_ =
          desired_positions_ *
          std::min(1.0, std::max(time_ - time_stamp_state_change_, 0.0));

      break;

    case ControllerState::motion:
      Vector8d desired_positions;
      VectorObservation observation;

      // set desired position to reference according to residual policy
      setReferenceMotionTraj();
      desired_positions = desired_positions_reference_;

      // get sensor data
      Vector8d joint_positions = robot_->get_joint_positions();
      Vector8d joint_velocities = robot_->get_joint_velocities();
      Eigen::Vector4d imu_attitude_quaternion =
          robot_->get_imu_attitude_quaternion();

      // calculate phase according to time
      double phase = 2.0 * M_PI / ref_traj_max_time_ * time_;

      observation.segment(0, 4) << imu_attitude_quaternion;
      observation.segment(4, 8) << joint_positions;
      // observation.segment(12, 8) << joint_velocities;
      observation.segment(12, 8) << filtered_velocity_;
      observation.segment(20, 2) << cos(phase), sin(phase);

      // convert Eigen double vector to torch double tensor. Note the matrix
      // transpose according to the conventions of Eigen and Torch
      torch::Tensor input_tensor =
          torch::from_blob(observation.data(), {1, NETWORK_INPUT_DIM},
                           at::kDouble)
              .clone();

      // convert torch double tensor to torchscript float IValue
      std::vector<torch::jit::IValue> input_ivalue;
      input_ivalue.push_back(input_tensor.to(torch::kFloat));

      // evaluate network
      torch::Tensor output_tensor =
          network_.forward(input_ivalue).toTuple()->elements()[0].toTensor();

      // convert network output to Eigen double vector. Note the matrix
      // transpose according to the conventions of Eigen and Torch
      VectorAction output(output_tensor.to(torch::kDouble).data_ptr<double>());

      // residual network
      desired_positions += output;

      // set Controller variable to send to motors
      desired_positions_ = desired_positions;
      desired_velocities_ = desired_velocities_reference_;
      desired_torques_ = desired_torques_reference_;
      break;
  }
}

void NetworkController::setReferenceMotionTraj() {
  Eigen::Vector3d base_pos;
  Eigen::Vector4d base_quat;
  Eigen::Matrix<double, 8, 1> desired_joint_position;
  Eigen::Matrix<double, 8, 1> desired_joint_velocity;
  Eigen::Matrix<double, 8, 1> desired_joint_torque;

  // find reference trajectory index corresponding to current time
  int traj_idx = (int)(time_ * ref_traj_max_idx_ / ref_traj_max_time_);
  traj_idx = traj_idx % ref_traj_max_idx_;

  Eigen::Matrix<double, 38, 1> traj_t;
  traj_t << ref_traj_.row(traj_idx).transpose();
  desired_joint_position << traj_t.segment(14, 8);
  desired_joint_velocity << traj_t.segment(22, 8);
  desired_joint_torque << traj_t.segment(30, 8);

  desired_positions_reference_ = desired_joint_position;
  desired_velocities_reference_ = desired_joint_velocity;
  desired_torques_reference_ = desired_joint_torque;
}