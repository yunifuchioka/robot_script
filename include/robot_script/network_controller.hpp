/**
 * controller that uses a neural network trained by RL
 */

#pragma once

#include "controller.hpp"
#include "torch/script.h"

// #define NETWORK_INPUT_DIM 22
#define NETWORK_INPUT_DIM 18
#define NETWORK_OUTPUT_DIM 8

class NetworkController : public Controller {
 public:
  enum MotionType { squat, walk, walk_joint, walk_quat };
  typedef Eigen::Matrix<double, NETWORK_INPUT_DIM, 1> VectorObservation;
  typedef Eigen::Matrix<double, NETWORK_OUTPUT_DIM, 1> VectorAction;

  /**
   * constructor calls the parent controller constructor, then initializes its
   * own internal variables
   */
  NetworkController(const std::shared_ptr<Solo8>& robot) : Controller{robot} {
    phase_ = 0;
    motion_type_ = MotionType::squat;
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
  void set_motion_type(MotionType motion_type) { motion_type_ = motion_type; };

 private:
  double phase_;
  MotionType motion_type_;
  torch::jit::script::Module network_;
  Vector8d desired_positions_reference_;

  void setReferenceMotionSquat();
  void setReferenceMotionWalk();
};