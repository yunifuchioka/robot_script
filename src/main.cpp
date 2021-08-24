#include "main.hpp"
#include "network_policy.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* robot_if_void_ptr) {
  double dt = 0.001;

  NetworkPolicy my_network_policy("iter9900"); // TODO: don't hardcode this

  srand((unsigned int)time(0));  // initialize random seed
  VectorObservation observation;

  MasterBoardInterface* robot_if =
      (static_cast<MasterBoardInterface*>(robot_if_void_ptr));

  auto last = Clock::now();

  while (!robot_if->IsTimeout() && !robot_if->IsAckMsgReceived()) {
    if ((Clock::now() - last).count() > dt) {
      last = Clock::now();
      robot_if->SendInit();
    }
  }

  if (robot_if->IsTimeout()) {
    rt_printf("\nTimeout while waiting for ack.\n");
  }

  int counter = 0;
  while (!CTRL_C_DETECTED) {
    robot_if->ParseSensorData();

    observation << Eigen::MatrixXd::Random(NETWORK_INPUT_DIM, 1);
    VectorAction action = my_network_policy.get_action(observation);

    if (counter % 1000 == 0) {
      std::cout << "\x1B[2J\x1B[H";
      std::cout << "counter: " << counter << std::endl;
      std::cout << "timer: " << (double)(Clock::now() - last).count()/1000000000.0 << std:: endl << std::endl;
      std::cout << "action: " << action.transpose() << std::endl;
      last = Clock::now();
    }

    robot_if->SendCommand();

    real_time_tools::Timer::sleep_sec(dt);
    counter++;
  }

  return THREAD_FUNCTION_RETURN_VALUE;
}

int main(int argc, char** argv) {
  real_time_tools::RealTimeThread thread;
  enable_ctrl_c();

  if (argc != 2) {
    throw std::runtime_error(
        "Please provide the interface name (i.e. using 'ifconfig' on linux)");
  }

  MasterBoardInterface robot_if(argv[1]);
  robot_if.Init();

  thread.create_realtime_thread(&control_loop, &robot_if);

  rt_printf("control loop started \n");

  thread.join();

  return 0;
}
