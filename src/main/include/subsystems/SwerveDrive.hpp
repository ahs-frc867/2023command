#pragma once

#include <AHRS.h>
#include <frc/controller/HolonomicDriveController.h>
#include <frc/controller/ProfiledPIDController.h>
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
  SwervePod Q1;
  SwervePod Q2;
  SwervePod Q3;
  SwervePod Q4;

  frc::SwerveDriveKinematics<4> kinematics;
  frc::HolonomicDriveController holonomic;
  AHRS &gyro;
  struct {
    frc::Trajectory trajectory;
    frc::Rotation2d heading;
    std::chrono::time_point<std::chrono::system_clock> begin;
  } target;
  bool onTrajectory = false;

  void Periodic() override {
    using units::meter_t, units::degree_t;
    if (onTrajectory) {
      units::second_t time = target.begin - std::chrono::system_clock::now();
      setSpeed(holonomic.Calculate(frc::Pose2d(meter_t(gyro.GetDisplacementX()),
                                               meter_t(gyro.GetDisplacementY()),
                                               degree_t(gyro.GetAngle())),
                                   target.trajectory.Sample(time),
                                   target.heading));
    }
  }

public:
  // holonomic drive values are filler rn, adjust later
  SwerveDrive(AHRS &g)
      : Q1(5, 2, 9, 8), Q2(0, 0, 0, 0), Q3(0, 0, 0, 0), Q4(0, 0, 0, 0),
        kinematics(frc::Translation2d(0.2_m, 0.2_m),
                   frc::Translation2d(-0.2_m, 0.2_m),
                   frc::Translation2d(0.2_m, -0.2_m),
                   frc::Translation2d(-0.2_m, -0.2_m)),
        holonomic{frc2::PIDController{1, 0, 0}, frc2::PIDController{1, 0, 0},
                  frc::ProfiledPIDController<units::radian>{
                      1, 0, 0,
                      frc::TrapezoidProfile<units::radian>::Constraints{
                          6.28_rad_per_s, 3.14_rad_per_s / 1_s}}},
        gyro(g) {}

  void setSpeed(frc::ChassisSpeeds c) {
    auto states = kinematics.ToSwerveModuleStates(c);
    kinematics.DesaturateWheelSpeeds(&states, 2_mps);
    auto [s1, s2, s3, s4] = states;
    Q1.setState(s1);
    Q2.setState(s2);
    Q3.setState(s3);
    Q4.setState(s4);
  }

  void setTrajectory(frc::Trajectory t) {
    target.begin = std::chrono::system_clock::now();
    target.trajectory = t;
  }

  Headings getHeadings() const {
    return {Q1.getHeading(), Q2.getHeading(), Q3.getHeading(), Q4.getHeading()};
  }
  ~SwerveDrive() {}
};
} // namespace abval