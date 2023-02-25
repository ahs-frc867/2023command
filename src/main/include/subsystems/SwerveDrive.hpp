#pragma once

#include <AHRS.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc2/command/CommandPtr.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <array>
#include <chrono>

#include "subsystems/SwervePod.hpp"

namespace abval {
using Headings = std::array<units::radian_t, 4>;
class SwerveDrive : public frc2::SubsystemBase {
  std::array<SwervePod, 4> pods;
  frc::SwerveDriveKinematics<4> kinematics;

public:
  // holonomic drive values are filler rn, adjust later
  SwerveDrive()
      : pods{SwervePod(0, 1, 1, 0, "Q1"), SwervePod(2, 3, 3, 2, "Q2"),
             SwervePod(4, 5, 5, 4, "Q3"), SwervePod(6, 7, 7, 6, "Q4"),},
        kinematics(frc::Translation2d(12_in, -9.68_in),
                   frc::Translation2d(12_in, 9.68_in),
                   frc::Translation2d(-12_in, 9.68_in),
                   frc::Translation2d(-12_in, -9.68_in)) {}

  void setSpeed(frc::ChassisSpeeds c) {
    auto states = kinematics.ToSwerveModuleStates(c);
    kinematics.DesaturateWheelSpeeds(&states, 2_mps);
    for (int i = 0; i != 4; i++) {
      pods[i].setState(states[i]);
    }
  }

  void home() {
    for (auto &pod : pods) {
      pod.SetTurn(0_rad);
    }
  }

  Headings getHeadings() const {
    return {pods[0].getHeading(), pods[1].getHeading(), pods[2].getHeading(),
            pods[3].getHeading()};
  }

  void setPID(double p, double i, double d) {
    p += pods[0].turn_pid.GetP();
    i += pods[0].turn_pid.GetI();
    d += pods[0].turn_pid.GetD();
    for(auto& pod : pods){
      pod.turn_pid.SetP(p);
      pod.turn_pid.SetI(i);
      pod.turn_pid.SetD(d);
    }
    frc::SmartDashboard::PutNumber("P", pods[0].turn_pid.GetP());
    frc::SmartDashboard::PutNumber("I", pods[0].turn_pid.GetI());
    frc::SmartDashboard::PutNumber("D", pods[0].turn_pid.GetD());
  }

  ~SwerveDrive() {}
};
} // namespace abval