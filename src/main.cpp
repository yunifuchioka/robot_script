#include "main.hpp"

#include "common.hpp"
#include "solo8.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr) {
  ThreadCalibrationData* thread_data_ptr =
      (static_cast<ThreadCalibrationData*>(thread_data_void_ptr));
  std::shared_ptr<Solo8> robot = thread_data_ptr->robot;

  double dt_des = 0.001;
  double kp = 3.0;
  double kd = 0.05;
  double clamp_val = 5.0;

  Vector8d desired_joint_position;
  Vector8d desired_torque;

  robot->acquire_sensors();

  // Calibrates the robot.
  Vector8d joint_index_to_zero = thread_data_ptr->joint_index_to_zero;
  joint_index_to_zero << 0.126, 0.065, 0.384, 0.248, 0.994, -0.132, 0.339,
      0.622;
  robot->request_calibration(joint_index_to_zero);

  size_t count = 0;
  double t = 0.0;

  while (!CTRL_C_DETECTED) {
    robot->acquire_sensors();

    t += dt_des;

    desired_joint_position.setZero();

    desired_torque =
        kp * (desired_joint_position - robot->get_joint_positions()) -
        kd * robot->get_joint_velocities();

    clamp_val = 10.0 * std::min(1.0, t);
    for (int i = 0; i < 8; i++) {
      desired_torque(i) = std::min(desired_torque(i), clamp_val);
      desired_torque(i) = std::max(desired_torque(i), -clamp_val);
    }

    desired_torque.setZero();

    robot->send_target_joint_torque(desired_torque);

    real_time_tools::Timer::sleep_sec(dt_des);
    ++count;
  }  // endwhile
}

int main(int argc, char** argv) {
  real_time_tools::RealTimeThread thread;
  enable_ctrl_c();

  if (argc != 2) {
    throw std::runtime_error(
        "Please provide the interface name (i.e. using 'ifconfig' on linux)");
  }

  rt_printf("Please put the robot in zero position.\n");
  rt_printf("\n");
  rt_printf("Press enter to launch the calibration.\n");
  char str[256];
  std::cin.get(str, 256);  // get c-string

  std::shared_ptr<Solo8> robot = std::make_shared<Solo8>();
  robot->initialize(std::string(argv[1]));

  ThreadCalibrationData thread_data(robot);

  thread.create_realtime_thread(&control_loop, &thread_data);

  rt_printf("control loop started \n");

  while (!CTRL_C_DETECTED) {
    real_time_tools::Timer::sleep_sec(0.01);
  }

  thread.join();

  return 0;
}
