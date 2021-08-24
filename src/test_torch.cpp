#include <iostream>

#include "network_policy.hpp"

int main(int argc, char** argv) {
  NetworkPolicy my_network_policy(argv[1]);

  VectorObservation observation;
  srand((unsigned int)time(0));  // initialize random seed
  observation << Eigen::MatrixXd::Random(NETWORK_INPUT_DIM, 1);

  VectorAction action = my_network_policy.get_action(observation);

  std::cout << observation.transpose() << std::endl;
  std::cout << std::endl;
  std::cout << action.transpose() << std::endl;

  return 0;
}