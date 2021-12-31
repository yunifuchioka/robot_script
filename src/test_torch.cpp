#include <iostream>

#include "solo8.hpp"
#include "network_controller.hpp"

int main(int argc, char** argv) {
std::shared_ptr<Solo8> robot = std::make_shared<Solo8>();
  NetworkController my_network_controller(robot);
  my_network_controller.initialize_network(argv[1]);

  NetworkController::VectorObservation observation;
  srand((unsigned int)time(0));  // initialize random seed
  observation << Eigen::MatrixXd::Random(NETWORK_INPUT_DIM, 1);
  my_network_controller.calc_control();

  NetworkController::VectorAction action = my_network_controller.get_desired_positions();

  std::cout << observation.transpose() << std::endl;
  std::cout << std::endl;
  std::cout << action.transpose() << std::endl;

  return 0;
}