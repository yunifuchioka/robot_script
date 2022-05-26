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
  Vector8d desired_positions;
  VectorObservation observation;

  switch (motion_type_) {
    case MotionType::squat: {
      // set desired position to reference according to residual policy
      setReferenceMotionSquat();
      desired_positions = desired_positions_reference_;

      // set observation vector
      observation << cos(phase_), sin(phase_);

      break;
    }
    case MotionType::walk: {
      // set desired position to reference according to residual policy
      setReferenceMotionWalk();
      desired_positions = desired_positions_reference_;

      // construct observation vector
      observation << cos(phase_), sin(phase_);

      break;
    }

    case MotionType::walk_joint: {
      // set desired position to reference according to residual policy
      setReferenceMotionWalk();
      desired_positions = desired_positions_reference_;

      // get sensor data
      Vector8d joint_positions = robot_->get_joint_positions();
      Vector8d joint_velocities = robot_->get_joint_velocities();

      // construct observation vector
      observation.segment(0, 8) << joint_positions;
      observation.segment(8, 8) << joint_velocities;
      observation.segment(16, 2) << cos(phase_), sin(phase_);

      break;
    }
    case MotionType::walk_quat: {
      // set desired position to reference according to residual policy
      setReferenceMotionWalk();
      desired_positions = desired_positions_reference_;

      // get sensor data
      Vector8d joint_positions = robot_->get_joint_positions();
      Vector8d joint_velocities = robot_->get_joint_velocities();
      Eigen::Vector4d imu_attitude_quaternion =
          robot_->get_imu_attitude_quaternion();

      // observation.segment(0, 4) << imu_attitude_quaternion(3),
      //     -imu_attitude_quaternion(0), -imu_attitude_quaternion(1),
      //     -imu_attitude_quaternion(2);
      observation.segment(0, 4) << imu_attitude_quaternion(3),
          -imu_attitude_quaternion(0), -imu_attitude_quaternion(1),
          -imu_attitude_quaternion(2);
      observation.segment(4, 8) << joint_positions;
      observation.segment(12, 8) << joint_velocities;
      observation.segment(20, 2) << cos(phase_), sin(phase_);

      std::cout << observation.segment(0, 4).transpose() << std::endl
                << std::endl;

      break;
    }
  }

  // convert Eigen double vector to torch double tensor. Note the matrix
  // transpose according to the conventions of Eigen and Torch
  torch::Tensor input_tensor =
      torch::from_blob(observation.data(), {1, NETWORK_INPUT_DIM}, at::kDouble)
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
}

void NetworkController::setReferenceMotionSquat() {
  Eigen::Matrix<double, 8, 1> desired_joint_position;

  Eigen::Vector2d joint_front;
  Eigen::Vector2d joint_hind;

  double leg_length = 0.16;  // from URDF

  double amp = M_PI / 8.0;
  double theta = phase_;

  joint_front << M_PI / 4.0, -M_PI / 2.0;
  joint_hind << -M_PI / 4.0, M_PI / 2.0;

  // base_pos(2) += amp * leg_length * std::sin(theta);
  joint_front(0) += -amp * std::sin(theta);
  joint_front(1) += 2.0 * amp * std::sin(theta);
  joint_hind(0) += amp * std::sin(theta);
  joint_hind(1) += -2.0 * amp * std::sin(theta);

  desired_joint_position.segment(0, 2) << joint_front;
  desired_joint_position.segment(2, 2) << joint_front;
  desired_joint_position.segment(4, 2) << joint_hind;
  desired_joint_position.segment(6, 2) << joint_hind;

  desired_positions_reference_ = desired_joint_position;
}

void NetworkController::setReferenceMotionWalk() {
  Eigen::Matrix<double, 8, 1> desired_joint_position;

  desired_joint_position << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
      M_PI / 2, -M_PI / 4, M_PI / 2;

  double amp = M_PI / 12;
  double theta = phase_ * 24.0;  // 24 step cycles per entire clip

  desired_joint_position(0) += std::max(amp * sin(theta), 0.0);
  desired_joint_position(1) += std::min(-2.0 * amp * sin(theta), 0.0);
  desired_joint_position(2) += std::max(amp * sin(theta + M_PI), 0.0);
  desired_joint_position(3) += std::min(-2.0 * amp * sin(theta + M_PI), 0.0);
  desired_joint_position(4) += std::min(amp * sin(theta), 0.0);
  desired_joint_position(5) += std::max(-2.0 * amp * sin(theta), 0.0);
  desired_joint_position(6) += std::min(amp * sin(theta + M_PI), 0.0);
  desired_joint_position(7) += std::max(-2.0 * amp * sin(theta + M_PI), 0.0);

  desired_positions_reference_ = desired_joint_position;
}