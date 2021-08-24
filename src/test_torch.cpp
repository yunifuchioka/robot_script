#include <Eigen/Dense>
#include <cassert>
#include <iostream>

#include "torch/script.h"

#define MODEL_INPUT_DIM 31
#define MODEL_OUTPUT_DIM 8

int main(int argc, char** argv) {
  // load neural network model
  torch::jit::script::Module module;
  try {
    std::string model_dir = "../models/";
    module = torch::jit::load(model_dir + argv[1] + "_script.pt");
  } catch (const c10::Error& e) {
    std::cerr << "error loading neural network model\n";
    return -1;
  }

  // prepare network input
  // initialize random Eigen double vector
  srand((unsigned int)time(0));  // initialize random seed
  Eigen::Matrix<double, MODEL_INPUT_DIM, 1> input;
  input << Eigen::MatrixXd::Random(MODEL_INPUT_DIM, 1);

  // convert Eigen double vector to torch double tensor
  // note the matrix transpose according to the conventions of Eigen and Torch
  torch::Tensor input_tensor =
      torch::from_blob(input.data(), {1, MODEL_INPUT_DIM}, at::kDouble).clone();

  // convert torch double tensor to torchscript float IValue
  std::vector<torch::jit::IValue> input_ivalue;
  input_ivalue.push_back(input_tensor.to(torch::kFloat));

  // evaluate network
  torch::Tensor output_tensor =
      module.forward(input_ivalue).toTuple()->elements()[0].toTensor();

  assert(output_tensor.size(1) == MODEL_OUTPUT_DIM);

  // convert network output to Eigen double vector
  // note the matrix transpose according to the conventions of Eigen and Torch
  Eigen::Matrix<double, 8, 1> output(
      output_tensor.to(torch::kDouble).data_ptr<double>());

  std::cout << input.transpose() << std::endl;
  std::cout << input_tensor << std::endl;
  std::cout << input_ivalue << std::endl;
  std::cout << std::endl;

  std::cout << output_tensor << std::endl;
  std::cout << output.transpose() << std::endl;

  return 0;
}