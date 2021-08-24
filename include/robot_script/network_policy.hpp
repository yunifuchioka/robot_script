#pragma once

#include <Eigen/Dense>

#include "torch/script.h"

#define NETWORK_INPUT_DIM 31
#define NETWORK_OUTPUT_DIM 8

typedef Eigen::Matrix<double, NETWORK_INPUT_DIM, 1> VectorObservation;
typedef Eigen::Matrix<double, NETWORK_OUTPUT_DIM, 1> VectorAction;

class NetworkPolicy {
 public:
  NetworkPolicy(const std::string filename);
  VectorAction get_action(VectorObservation observation);

 private:
  torch::jit::script::Module network;
};