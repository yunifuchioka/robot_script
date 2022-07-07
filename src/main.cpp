#include "main.hpp"

#include "common.hpp"
#include "imu_controller.hpp"
#include "network_controller.hpp"
#include "phase_controller.hpp"
#include "solo8.hpp"

using namespace solo;

static THREAD_FUNCTION_RETURN_TYPE control_loop(void* thread_data_void_ptr) {
  ThreadCalibrationData* thread_data_ptr =
      (static_cast<ThreadCalibrationData*>(thread_data_void_ptr));
  std::shared_ptr<Solo8> robot = thread_data_ptr->robot;

  double dt_des = 0.001;
  double kp = 3.0;
  double kd = 0.5;
  double safety_delay = 2.0;
  double safety_torque_limit = 100.0;

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

  NetworkController controller(robot);
  // controller.initialize_network("06-23-trot-angvel01");
  controller.initialize_network("07-06-stand-neg-force");
  // controller.initialize_network("07-05-front-hop-neg");
  // controller.initialize_network("07-05-trot-neg");
  controller.set_motion_type(NetworkController::MotionType::traj);

  Eigen::MatrixXd ref_traj;
  // ref_traj = openData("../traj/06-22-trot.csv");
  ref_traj = openData("../traj/07-05-stand-neg-force.csv");
  // ref_traj = openData("../traj/07-05-squat-neg-force.csv");
  // ref_traj = openData("../traj/07-05-front-hop-neg.csv");
  // ref_traj = openData("../traj/07-05-trot-neg.csv");
  controller.set_traj(ref_traj);

  double period = ref_traj(ref_traj.rows() - 1, 0);

  // buffer for storing joint velocity values for filtering
  // length of 20 corresponds to RL policy frequency
  Eigen::MatrixXd vel_buffer(8, 20);
  vel_buffer.setZero();
  unsigned int buffer_counter = 0;
  Vector8d filtered_velocity;
  filtered_velocity.setZero();

  // PhaseController controller(robot);
  // controller.set_motion_type(PhaseController::squat);
  // double period = 0.8;   // for squat
  // double period = 8.16;  // for walk

  auto tic = Clock::now();
  real_time_tools::Timer::sleep_sec(dt_des);
  auto toc = std::chrono::duration<double>(Clock::now() - tic).count();
  real_time_tools::Timer::sleep_sec(dt_des);

  Eigen::VectorXd log_vec(53);  // vector to print when logging to csv

  while (!CTRL_C_DETECTED) {
    robot->acquire_sensors();

    t += dt_des;

    // slowly goes from 0 to 1 after some delay
    double safety_interp = std::min(1.0, std::max(t - safety_delay, 0.0));

    if (count % 20 == 0) {  // control_dt = 0.02 in RL code

      // specify filtered velocity to controller to be input into network
      filtered_velocity = vel_buffer.rowwise().mean();
      controller.set_filtered_velocity(filtered_velocity);

      // update network policy
      controller.set_phase(2.0 * M_PI / period * t);
      if (t < period * 2.0) {
        // std::cout << t / (2.0 * period) << std::endl;
        controller.set_phase(0.0);
      }
      controller.calc_control();
      joint_desired_positions = controller.get_desired_positions();
      joint_desired_velocities = controller.get_desired_velocities();
      joint_desired_torques = controller.get_desired_torques();

      // reset velocity filter buffer
      vel_buffer.setZero();
      buffer_counter = 0;
    }

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

    // store velocity in buffer for filtering
    vel_buffer.col(buffer_counter) << robot->get_joint_velocities();
    buffer_counter++;

    if ((count % 1) == 0) {
      toc = std::chrono::duration<double>(Clock::now() - tic).count();

      log_vec(0) = toc;
      log_vec.segment(1, 4) = robot->get_imu_attitude_quaternion();
      log_vec.segment(5, 8) = robot->get_joint_positions();
      log_vec.segment(13, 8) = filtered_velocity;
      // log_vec.segment(13, 8) = robot->get_joint_velocities();
      log_vec.segment(21, 8) = robot->get_joint_torques();
      log_vec.segment(29, 8) = joint_desired_positions;
      log_vec.segment(37, 8) = joint_desired_velocities;
      log_vec.segment(45, 8) = joint_desired_torques;
      print_vector_csv(log_vec);

      // uncomment to get delta time rather than absolute time
      // tic = Clock::now();
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
