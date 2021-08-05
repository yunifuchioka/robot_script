#include "main.hpp"

using namespace solo;

THREAD_FUNCTION_RETURN_TYPE control_loop(void *) {
  auto t1 = Clock::now();
  int i = 0;

  while (!CTRL_C_DETECTED) {
    i++;

    if (i % 1000 == 0) {
      auto t2 = Clock::now();
      rt_printf("iteration %d, time=%lu\n", i, t2 - t1);
      t1 = Clock::now();
    }

    real_time_tools::Timer::sleep_microseconds(1000);
  }

  rt_printf("\nCtrl-c detected. Terminating control loop...\n");

  return THREAD_FUNCTION_RETURN_VALUE;
}

int main(int argc, char **argv) {
  real_time_tools::RealTimeThread thread;
  enable_ctrl_c();

  thread.block_memory();
  thread.create_realtime_thread(control_loop);

  rt_printf("control loop started \n");

  thread.join();

  return 0;
}
