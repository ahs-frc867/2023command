#pragma once

#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <units/angle.h>

#include <array>

#include "subsystems/SwervePod.hpp"

namespace abval {
using Headings = std::array<units::radian_t, 4>;
class SwerveDrive : public frc2::SubsystemBase {
  SwervePod Q1;
  frc::SwerveDriveKinematics<4> kinematics;
  frc::HolonomicDriveController holonomic;
  struct target {
    frc::Trajectory trajectory;
    frc::Rotation2d heading;
    units::time::second_t time;
  };
  target current;

 public:
  // holonomic drive values are filler rn, adjust later
  SwerveDrive()
      : Q1(5, 2, 9, 8),
        kinematics(frc::Translation2d(0.2_m, 0.2_m),
                   frc::Translation2d(-0.2_m, 0.2_m),
                   frc::Translation2d(0.2_m, -0.2_m),
                   frc::Translation2d(-0.2_m, -0.2_m)),
        holonomic{frc2::PIDController{1, 0, 0}, frc2::PIDController{1, 0, 0},
                  frc::ProfiledPIDController<units::radian>{
                      1, 0, 0,
                      frc::TrapezoidProfile<units::radian>::Constraints{
                          6.28_rad_per_s, 3.14_rad_per_s / 1_s}}} {}
  // for now does not do rotation
  void SetVelocity(frc::Translation2d move) {
    Q1.SetPower(move.Norm().value() * 0.3);
    if (move.Norm() != 0_m) {
      Q1.SetTurn(move.Angle().Radians());
    }
  }

  void setTrajectory(frc::Trajectory t) {
    current.time = 0_s;
    current.trajectory = t;
  }

  void PIDupdate(frc::Pose2d robot) {
    auto states = kinematics.ToSwerveModuleStates(holonomic.Calculate(
        robot, current.trajectory.Sample(current.time), current.heading));
    // desaturate and apply states later
  }

  Headings getHeadings() const {
    return {Q1.getHeading(), 0_rad, 0_rad, 0_rad};
  }

  frc2::CommandPtr zero() {
    return Subsystem::RunOnce([this]() { Q1.reset(); });
  }
};
}  // namespace abval