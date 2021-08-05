
#include <unistd.h>

#include <chrono>
#include <iostream>

#include "master_board_sdk/master_board_interface.h"

int main(int argc, char **argv) {
  if (argc != 2) {
    throw std::runtime_error(
        "Please provide the interface name "
        "(i.e. using 'ifconfig' on linux)");
  }

  double dt = 0.001;

  nice(-20);  // give the process a high priority

  MasterBoardInterface robot_if(argv[1]);
  robot_if.Init();

  std::chrono::time_point<std::chrono::system_clock> last =
      std::chrono::system_clock::now();
  while (!robot_if.IsTimeout() && !robot_if.IsAckMsgReceived()) {
    if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                         last))
            .count() > dt) {
      last = std::chrono::system_clock::now();
      robot_if.SendInit();
    }
  }

  if (robot_if.IsTimeout()) {
    printf("Timeout while waiting for ack.\n");
  }

  while (!robot_if.IsTimeout()) {
    if (((std::chrono::duration<double>)(std::chrono::system_clock::now() -
                                         last))
            .count() > dt) {
      // do stuff to check that the communication with the master board is
      // working eg. robot_if.PrintStats();

    } else {
      std::this_thread::yield();
    }
  }
  printf(
      "Masterboard timeout detected. Either the masterboard has been shut down "
      "or there has been a connection issue with the cable/wifi.\n");

  return 0;
}
