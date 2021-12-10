#include "phase_controller.hpp"

using namespace solo;

void PhaseController::calc_control() {
  Vector8d desired_positions;

  switch (motion_type_) {
    case MotionType::stand: {
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;
      break;
    }
    case MotionType::squat: {
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;
      double amp = M_PI / 8.0;
      desired_positions(0) += amp * sin(phase_);
      desired_positions(1) += -2.0 * amp * sin(phase_);
      desired_positions(2) += amp * sin(phase_);
      desired_positions(3) += -2.0 * amp * sin(phase_);
      desired_positions(4) -= amp * sin(phase_);
      desired_positions(5) -= -2.0 * amp * sin(phase_);
      desired_positions(6) -= amp * sin(phase_);
      desired_positions(7) -= -2.0 * amp * sin(phase_);
      break;
    }
    case MotionType::tilt_body: {
      desired_positions << M_PI / 4, -M_PI / 2, M_PI / 4, -M_PI / 2, -M_PI / 4,
          M_PI / 2, -M_PI / 4, M_PI / 2;
      double amp = M_PI / 20.0;
      desired_positions(0) += amp * sin(phase_);
      desired_positions(1) += -2.0 * amp * sin(phase_);
      desired_positions(2) += amp * sin(phase_);
      desired_positions(3) += -2.0 * amp * sin(phase_);
      desired_positions(4) += amp * sin(phase_);
      desired_positions(5) += -2.0 * amp * sin(phase_);
      desired_positions(6) += amp * sin(phase_);
      desired_positions(7) += -2.0 * amp * sin(phase_);
      break;
    }
    case MotionType::step_in_place: {
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
      break;
    }
  }

  desired_positions_ = desired_positions;
}