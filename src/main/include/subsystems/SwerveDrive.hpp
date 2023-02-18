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

#include "frc/smartdashboard/SmartDashboard.h"
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
  // AHRS &gyro;
  struct {
    frc::Trajectory trajectory;
    frc::Rotation2d heading;
    std::chrono::time_point<std::chrono::system_clock> begin;
  } target;
  bool onTrajectory = false;

  void Periodic() override {
    using units::meter_t, units::degree_t;
    // if (onTrajectory) {
    //   units::second_t time = target.begin - std::chrono::system_clock::now();
    //   setSpeed(holonomic.Calculate(frc::Pose2d(meter_t(gyro.GetDisplacementX()),
    //                                            meter_t(gyro.GetDisplacementY()),
    //                                            degree_t(gyro.GetAngle())),
    //                                target.trajectory.Sample(time),
    //                                target.heading));
    // }
  }

public:
  // holonomic drive values are filler rn, adjust later
  SwerveDrive()
      : Q1(0, 1, 3, 2, "Q1", 180_deg), Q2(2, 3, 1, 0, "Q2", 180_deg),
        Q3(6, 7, 7, 6, "Q4"), Q4(4, 5, 5, 4, "Q3"),
        kinematics(frc::Translation2d(12_in, 9.68_in),
                   frc::Translation2d(-12_in, 9.68_in),
                   frc::Translation2d(12_in, -9.68_in),
                   frc::Translation2d(-12_in, -9.68_in)),
        holonomic{frc2::PIDController{1, 0, 0}, frc2::PIDController{1, 0, 0},
                  frc::ProfiledPIDController<units::radian>{
                      1, 0, 0,
                      frc::TrapezoidProfile<units::radian>::Constraints{
                          6.28_rad_per_s, 3.14_rad_per_s / 1_s}}} //, gyro(g)
  {
    Q1.reverseTurn(false);
    Q2.reverseTurn(false);
    Q3.reverseTurn(false);
    Q4.reverseTurn(false);
  }

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

  void setPID(double p, double i, double d) {
    p += Q1.turn_pid.GetP();
    i += Q1.turn_pid.GetI();
    d += Q1.turn_pid.GetD();
    Q1.turn_pid.SetP(p);
    Q2.turn_pid.SetP(p);
    Q3.turn_pid.SetP(p);
    Q4.turn_pid.SetP(p);
    Q1.turn_pid.SetI(i);
    Q2.turn_pid.SetI(i);
    Q3.turn_pid.SetI(i);
    Q4.turn_pid.SetI(i);
    Q1.turn_pid.SetD(d);
    Q2.turn_pid.SetD(d);
    Q3.turn_pid.SetD(d);
    Q4.turn_pid.SetD(d);
    frc::SmartDashboard::PutNumber("P", Q1.turn_pid.GetP());
    frc::SmartDashboard::PutNumber("I", Q1.turn_pid.GetI());
    frc::SmartDashboard::PutNumber("D", Q1.turn_pid.GetD());
  }

  ~SwerveDrive() {}
};
} // namespace abval