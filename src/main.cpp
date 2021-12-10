#include "main.hpp"

#include "common.hpp"
#include "phase_controller.hpp"
#include "solo8.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr) {
  ThreadCalibrationData* thread_data_ptr =
      (static_cast<ThreadCalibrationData*>(thread_data_void_ptr));
  std::shared_ptr<Solo8> robot = thread_data_ptr->robot;

  double dt_des = 0.001;
  double kp = 3.0;
  double kd = 0.05;
  double safety_delay = 2.0;
  double safety_torque_limit = 10.0;

  Vector8d joint_desired_positions;
  Vector8d joint_desired_velocities;
  Vector8d joint_desired_torques;

  robot->acquire_sensors();

  // Calibrates the robot.
  Vector8d joint_index_to_zero = thread_data_ptr->joint_index_to_zero;
  robot->request_calibration(joint_index_to_zero);

  robot->set_joint_position_gains(kp);
  robot->set_joint_velocity_gains(kd);

  size_t count = 0;
  double t = 0.0;

  PhaseController controller(robot);

  while (!CTRL_C_DETECTED) {
    robot->acquire_sensors();

    t += dt_des;

    // slowly goes from 0 to 1 after some delay
    double safety_interp = std::min(1.0, std::max(t - safety_delay, 0.0));

    // get desired PD+torque targets from controller
    controller.set_phase(18.0 * t);
    controller.calc_control();
    joint_desired_positions = controller.get_desired_positions();
    joint_desired_velocities = controller.get_desired_velocities();
    joint_desired_torques = controller.get_desired_torques();

    // warm start desired_joint_position for safety
    joint_desired_positions =
        joint_desired_positions * safety_interp +
        robot->get_joint_positions() * (1.0 - safety_interp);

    // clamp and warm start desired_torque for safety
    double clamp_val = safety_torque_limit * safety_interp;
    joint_desired_torques =
        joint_desired_torques.cwiseMin(clamp_val).cwiseMax(-clamp_val);

    // send desired PD+torque targets to robot
    robot->set_joint_desired_positions(joint_desired_positions);
    robot->set_joint_desired_velocities(joint_desired_velocities);
    robot->set_joint_desired_torques(joint_desired_torques);
    robot->send_joint_commands();

    if ((count % 100) == 0) {
      printf("\n");
      print_vector("des_joint_tau", joint_desired_torques);
      print_vector("    joint_pos", robot->get_joint_positions());
      print_vector("des_joint_pos", joint_desired_positions);
      print_vector("    joint_vel", robot->get_joint_velocities());
    }

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

  std::shared_ptr<Solo8> robot = std::make_shared<Solo8>();
  robot->initialize(std::string(argv[1]));

  ThreadCalibrationData thread_data(robot);

  rt_printf("Press enter to start the control loop \n");
  char str[256];
  std::cin.get(str, 256);  // get c-string

  thread.create_realtime_thread(&control_loop, &thread_data);

  rt_printf("control loop started \n");

  while (!CTRL_C_DETECTED) {
    real_time_tools::Timer::sleep_sec(0.01);
  }

  thread.join();

  return 0;
}
