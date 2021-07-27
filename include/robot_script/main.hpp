/**
 * This is mostly copy+pasted from open dynamic robot initiative solo
 * https://raw.githubusercontent.com/open-dynamic-robot-initiative/solo/master/include/solo/common_programs_header.hpp
 */

# pragma once

#include <iostream>
#include <chrono>
#include <signal.h>  // manage the ctrl+c signal
#include <atomic>    // thread safe flag for application shutdown management

#include <Eigen/Dense>

#include "real_time_tools/thread.hpp"
#include "real_time_tools/timer.hpp"

typedef std::chrono::high_resolution_clock Clock;

namespace solo {
/**
 * @brief This boolean is here to kill cleanly the application upon ctrl+c
 */
std::atomic_bool CTRL_C_DETECTED(false);

/**
 * @brief This function is the callback upon a ctrl+c call from the terminal.
 *
 * @param s is the id of the signal
 */
void my_handler(int) {
  CTRL_C_DETECTED = true;
}

/**
 * @brief Enable to kill the demos cleanly with a ctrl+c
 */
void enable_ctrl_c() {
  // make sure we catch the ctrl+c signal to kill the application properly.
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  CTRL_C_DETECTED = false;
}

/**
 * @brief Usefull tool for the demos and programs in order to print data in
 * real time.
 *
 * @param v_name  is a string defining the data to print.
 * @param v the vector to print.
 */
void print_vector(std::string v_name,
                  const Eigen::Ref<const Eigen::VectorXd> v) {
  v_name += ": [";
  rt_printf("%s", v_name.c_str());
  for (int i = 0; i < v.size(); ++i) {
    rt_printf("%0.3f, ", v(i));
  }
  rt_printf("]\n");
}

}  // namespace solo

