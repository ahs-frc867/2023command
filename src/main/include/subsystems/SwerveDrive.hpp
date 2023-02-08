#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandPtr.h>
#include <units/angle.h>

#include "subsystems/SwervePod.hpp"

namespace abval {
class SwerveDrive : public frc2::SubsystemBase {
  SwervePod Q1;

 public:
  SwerveDrive() : Q1(5, 2, 9, 8) {}
  // for now does not do rotation
  void SetVelocity(frc::Translation2d move) {
    Q1.SetPower(move.Norm().value() * 0.3);
    if (move.Norm() != 0.0_m) {
      Q1.SetTurn(move.Angle().Radians());
    }
  }
  frc2::CommandPtr zero() {
    return Subsystem::RunOnce([this]() { Q1.reset(); });
  }
};
}  // namespace abval