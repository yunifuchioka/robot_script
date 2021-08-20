#include <iostream>

#include "torch/script.h"

int main(int argc, char** argv) {
  torch::jit::script::Module module;
  try {
    module = torch::jit::load(argv[1]);
  } catch (const c10::Error& e) {
    std::cerr << "error loading the model\n";
    return -1;
  }

  std::vector<torch::jit::IValue> inputs;
  inputs.push_back(torch::rand({1, 31}));

  at::Tensor output = module.forward(inputs).toTuple()->elements()[0].toTensor();

  std::cout << output.cpu() << std::endl;

  return 0;
}