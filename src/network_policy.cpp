#include "network_policy.hpp"

NetworkPolicy::NetworkPolicy(const std::string filename) {
  // load neural network model
  try {
    std::string model_dir = "../models/";
    network = torch::jit::load(model_dir + filename + "_script.pt");
  } catch (const c10::Error& e) {
    std::cerr << "error loading neural network model\n";
  }
}

VectorAction NetworkPolicy::get_action(VectorObservation observation) {
  // convert Eigen double vector to torch double tensor
  // note the matrix transpose according to the conventions of Eigen and Torch
  torch::Tensor input_tensor =
      torch::from_blob(observation.data(), {1, NETWORK_INPUT_DIM}, at::kDouble)
          .clone();

  // convert torch double tensor to torchscript float IValue
  std::vector<torch::jit::IValue> input_ivalue;
  input_ivalue.push_back(input_tensor.to(torch::kFloat));

  // evaluate network
  torch::Tensor output_tensor =
      network.forward(input_ivalue).toTuple()->elements()[0].toTensor();

  // convert network output to Eigen double vector
  // note the matrix transpose according to the conventions of Eigen and Torch
  VectorAction output(output_tensor.to(torch::kDouble).data_ptr<double>());

  return output;
}