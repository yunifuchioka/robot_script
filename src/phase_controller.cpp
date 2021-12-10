#include "phase_controller.hpp"

using namespace solo;

void PhaseController::calc_control() {
  Vector8d desired_positions;
  desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
      M_PI / 2, -M_PI / 4, M_PI / 2;
  double amp = M_PI / 12;

  desired_positions(0) += std::min(amp * sin(phase_), 0.0);
  desired_positions(1) += std::max(-2.0 * amp * sin(phase_), 0.0);
  desired_positions(2) += std::min(amp * sin(phase_ + M_PI), 0.0);
  desired_positions(3) += std::max(-2.0 * amp * sin(phase_ + M_PI), 0.0);
  desired_positions(4) += std::max(amp * sin(phase_), 0.0);
  desired_positions(5) += std::min(-2.0 * amp * sin(phase_), 0.0);
  desired_positions(6) += std::max(amp * sin(phase_ + M_PI), 0.0);
  desired_positions(7) += std::min(-2.0 * amp * sin(phase_ + M_PI), 0.0);

  desired_positions_ = desired_positions;
}