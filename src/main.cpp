#include "main.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr) {
  double dt = 0.001;

  ThreadCalibrationData* thread_data_ptr =
      (static_cast<ThreadCalibrationData*>(thread_data_void_ptr));
  std::shared_ptr<MasterBoardInterface> robot_if = thread_data_ptr->robot_if;

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

  while (!CTRL_C_DETECTED) {
    robot_if->ParseSensorData();

    robot_if->SendCommand();

    real_time_tools::Timer::sleep_sec(dt);
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

  std::shared_ptr<MasterBoardInterface> robot_if =
      std::make_shared<MasterBoardInterface>(argv[1]);
  robot_if->Init();

  ThreadCalibrationData thread_data(robot_if);

  thread.create_realtime_thread(&control_loop, &thread_data);

  rt_printf("control loop started \n");

  thread.join();

  return 0;
}
