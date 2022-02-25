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
    case MotionType::walk_sinusoid: {
      // set desired position to reference according to residual policy
      setReferenceMotionWalkSinusoid();
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
      desired_positions += output;
      break;
    }
  }

  desired_positions_ = desired_positions;
}

void NetworkController::setReferenceMotionWalkSinusoid() {
  Eigen::Matrix<double, 8, 1> desired_joint_position;

  desired_joint_position << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
      M_PI / 2, -M_PI / 4, M_PI / 2;

  double amp = M_PI / 12;
  double theta = phase_*8.0;

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