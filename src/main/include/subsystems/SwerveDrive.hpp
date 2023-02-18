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
  
  //-- Initialization

  SwerveDrive()
      : Q2(2, 3, 1, 0, "Q2", 180_deg), Q1(0, 1, 3, 2, "Q1", 180_deg),
        Q4(4, 5, 5, 4, "Q4"), Q3(6, 7, 7, 6, "Q3"),
		
        kinematics(frc::Translation2d(12_in, -9.68_in),
                   frc::Translation2d(12_in, 9.68_in),
                   frc::Translation2d(-12_in, 9.68_in),
                   frc::Translation2d(-12_in, -9.68_in)),
		
		// holonomic drive values are filler rn, adjust later
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
    auto [s1, s2, s3, s4] = states;
    Q1.setState(s1);
    Q2.setState(s2);
    Q3.setState(s3);
    Q4.setState(s4);
  }

  //-- Pod drive

  void setPower(double power) {
    Q1.setPower(power);
    Q2.setPower(power);
    Q3.setPower(power);
    Q4.setPower(power);
  }

  //-- Pod turn

  Headings getHeadings() const {
    return {Q1.getHeading(), Q2.getHeading(), Q3.getHeading(), Q4.getHeading()};
  }

  // Reset zero point as pods' current rotations
  void zero() {
    Q1.zero();
    Q2.zero();
    Q3.zero();
    Q4.zero();
  }
  
  // Set pod rotations to 0
  void home() {
    Q1.SetTurn(0_rad);
    Q2.SetTurn(0_rad);
    Q3.SetTurn(0_rad);
    Q4.SetTurn(0_rad);
  }

  //-- Pod PID

  void enableTurnPID(bool b) {
    Q1.enableTurnPID(b);
    Q2.enableTurnPID(b);
    Q3.enableTurnPID(b);
    Q4.enableTurnPID(b);
  }

  void setTurnPID(double p, double i, double d) {
    Q1.setTurnPID(p,i,d);
	Q2.setTurnPID(p,i,d);
	Q3.setTurnPID(p,i,d);
	Q4.setTurnPID(p,i,d);

    frc::SmartDashboard::PutNumber("P", Q1.turn_pid.GetP());
    frc::SmartDashboard::PutNumber("I", Q1.turn_pid.GetI());
    frc::SmartDashboard::PutNumber("D", Q1.turn_pid.GetD());
  }

  void incrementTurnPID(double dp, double di, double dd) {
    double p = Q1.turn_pid.GetP() + dp;
    double i = Q1.turn_pid.GetI() + di;
    double d = Q1.turn_pid.GetD() + dd;
	
	setPID(p,i,d)
  }

  //-- Chassis trajectory

  void setTrajectory(frc::Trajectory t) {
    target.begin = std::chrono::system_clock::now();
    target.trajectory = t;
  }

  ~SwerveDrive() {}
};
} // namespace abval
