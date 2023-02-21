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
#include "units/length.h"

namespace abval {
using Headings = std::array<units::radian_t, 4>;
class SwerveDrive : public frc2::SubsystemBase {
  frc::SwerveDriveKinematics<4> kinematics;
  frc::HolonomicDriveController holonomic;
  // AHRS &gyro;
  struct {
    frc::Trajectory trajectory;
    frc::Rotation2d heading;
    std::chrono::time_point<std::chrono::system_clock> begin;
  } target;

  bool on_trajectory = false;

  constexpr static units::length::meter_t pod_x = 12_in, pod_y = 9.6875_in;

  void Periodic() override {
    using units::meter_t, units::degree_t;
    // if (on_trajectory) {
    //   units::second_t time = target.begin - std::chrono::system_clock::now();
    //   setSpeed(holonomic.Calculate(frc::Pose2d(meter_t(gyro.GetDisplacementX()),
    //                                            meter_t(gyro.GetDisplacementY()),
    //                                            degree_t(gyro.GetAngle())),
    //                                target.trajectory.Sample(time),
    //                                target.heading));
    // }
  }

public:
  SwervePod FL;
  SwervePod FR;
  SwervePod BL;
  SwervePod BR;

  //-- Initialization

  SwerveDrive()
      : FL(2, 3, 3, 2, "FL"),
        FR(0, 1, 1, 0, "FR"), 
	      BL(6, 7, 7, 6, "BL"),
        BR(4, 5, 5, 4, "BR"),

        kinematics(frc::Translation2d(+pod_x, +pod_y),
                   frc::Translation2d(+pod_x, -pod_y),
                   frc::Translation2d(-pod_x, +pod_y),
                   frc::Translation2d(-pod_x, -pod_y)),

        // holonomic drive values are filler rn, adjust later
        holonomic{frc2::PIDController{1, 0, 0}, frc2::PIDController{1, 0, 0},
                  frc::ProfiledPIDController<units::radian>{
                      1, 0, 0,
                      frc::TrapezoidProfile<units::radian>::Constraints{
                          6.28_rad_per_s, 3.14_rad_per_s / 1_s}}} //, gyro(g)
  {
    FL.reverseTurn(false);
    FR.reverseTurn(false);
    BL.reverseTurn(false);
    BR.reverseTurn(false);
  }

  //-- General

  // Set swerve modules' target states from chassis's target state
  // Chassis state includes x, y, and angular velocities
  // Swerve module state includes drive and swerve velocities
  void setSpeed(frc::ChassisSpeeds c) {

    // Calculate target pod states from target state of chassis
    auto states = kinematics.ToSwerveModuleStates(c);

    // If one target speed exceeds limit, normalize all speeds by limit
    kinematics.DesaturateWheelSpeeds(&states, 2_mps);

    // Set pods to target states
    auto [sFL, sFR, sBL, sBR] = states;
    FL.setState(sFL);
    FR.setState(sFR);
    BL.setState(sBL);
    BR.setState(sBR);
  }

  //-- Pod drive

  void setPower(double power) {
    FL.setPower(power);
    FR.setPower(power);
    BL.setPower(power);
    BR.setPower(power);
  }

  //-- Pod turn

  Headings getHeadings() const {
    return {FL.getHeading(), FR.getHeading(), BL.getHeading(), BR.getHeading()};
  }

  // Reset zero point as pods' current rotations
  void zero() {
    FL.zero();
    FR.zero();
    BL.zero();
    BR.zero();
  }

  // Set pod rotations to 0
  void home() {
    FL.setTurn(0_rad);
    FR.setTurn(0_rad);
    BL.setTurn(0_rad);
    BR.setTurn(0_rad);
  }

  //-- Pod PID

  void enableTurnPID(bool b) {
    FL.enableTurnPID(b);
    FR.enableTurnPID(b);
    BL.enableTurnPID(b);
    BR.enableTurnPID(b);
  }

  void setTurnPID(double p, double i, double d) {
    FL.setTurnPID(p, i, d);
    FR.setTurnPID(p, i, d);
    BL.setTurnPID(p, i, d);
    BR.setTurnPID(p, i, d);

    frc::SmartDashboard::PutNumber("P", FR.getTurnP());
    frc::SmartDashboard::PutNumber("I", FR.getTurnI());
    frc::SmartDashboard::PutNumber("D", FR.getTurnD());
  }

  void incrementTurnPID(double dp, double di, double dd) {
    double p = FR.getTurnP() + dp;
    double i = FR.getTurnI() + di;
    double d = FR.getTurnD() + dd;

    setTurnPID(p, i, d);
  }

  //-- Chassis trajectory

  void setTrajectory(frc::Trajectory t) {
    target.begin = std::chrono::system_clock::now();
    target.trajectory = t;
  }

  ~SwerveDrive() {}
};
} // namespace abval
