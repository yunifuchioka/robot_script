#include "phase_controller.hpp"

using namespace solo;

void PhaseController::calc_control() {
  desired_positions_ << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
      M_PI / 2, -M_PI / 4, M_PI / 2;
}