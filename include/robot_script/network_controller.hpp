/**
 * controller that uses a neural network trained by RL
 */

#pragma once

#include "controller.hpp"
#include "torch/script.h"

#define NETWORK_INPUT_DIM 22
#define NETWORK_OUTPUT_DIM 8

class NetworkController : public Controller {
 public:
  typedef Eigen::Matrix<double, NETWORK_INPUT_DIM, 1> VectorObservation;
  typedef Eigen::Matrix<double, NETWORK_OUTPUT_DIM, 1> VectorAction;

  /**
   * constructor calls the parent controller constructor, then initializes its
   * own internal variables
   */
  NetworkController(const std::shared_ptr<Solo8>& robot) : Controller{robot} {
    phase_ = 0;
    ref_traj_.setZero();
    desired_positions_reference_.setZero();
    desired_velocities_reference_.setZero();
    desired_torques_reference_.setZero();
    filtered_velocity_.setZero();
    // TODO: initialize network to something safe, prior to initialize_network
    // call
  }

  /**
   * Loads neural network model given by the specified filename
   */
  void initialize_network(const std::string filename);

  /**
   * Calculates control based phase variable and sensor data from the robot
   *
   * WARNING !!!!
   * The method robot_->acquire_sensors() has to be called prior to this
   * function call in order to compute the control with up to date sensor data
   */
  void calc_control();

  /**
   * setters for private variables
   */
  void set_phase(double phase) { phase_ = phase; };
  void set_traj(const Eigen::MatrixXd ref_traj) { ref_traj_ = ref_traj; };
  void set_filtered_velocity(const Vector8d filtered_velocity) {
    filtered_velocity_ = filtered_velocity;
  };

 private:
  double phase_;
  torch::jit::script::Module network_;
  Vector8d desired_positions_reference_;
  Vector8d desired_velocities_reference_;
  Vector8d desired_torques_reference_;
  Eigen::MatrixXd ref_traj_;
  Vector8d filtered_velocity_;

  void setReferenceMotionTraj();
};