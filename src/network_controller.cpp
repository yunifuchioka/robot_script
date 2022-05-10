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

  switch (motion_type_) {
    case MotionType::squat: {
      // set desired position to reference according to residual policy
      setReferenceMotionSquat();
      desired_positions = desired_positions_reference_;

      // construct observation vector
      VectorObservation observation;
      observation << cos(phase_), sin(phase_);
      // TODO: make phase definition consistent between [0,1] vs [0,2*PI]

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
      break;
    }
  }

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