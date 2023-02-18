#pragma once

#include "frc/shuffleboard/SimpleWidget.h"
#include "networktables/GenericEntry.h"
#include "units/base.h"
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <fmt/format.h>
#include <frc/ADIS16470_IMU.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <math.h>
#include <units/angle.h>
#include <units/math.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

#include <cstdlib>
#include <numbers>
#include <string>
#include <string_view>

namespace abval {

// Turn motor PG71's gear ratio, swerve module's gear ratio, turn encoder's pulses per rotation
constexpr double turnGearRatio = ( 3179.0 / 226233.0 ) * ( 48.0 / 40.0 ) * ( 1.0 / 7.0 );

using std::numbers::pi;
using units::radian_t;

using ctre::phoenix::motorcontrol::can::TalonSRX;

class SwervePod : public frc2::SubsystemBase {

public:

  //-- Initialization

  SwervePod(int drive_id, int turn_id, 
            int encoder_channel_a, int encoder_channel_b, 
            std::string_view name, 
            radian_t rot = 0_rad)
      : drive_m(drive_id), turn_m(turn_id),
        turn_e(encoder_channel_a, encoder_channel_b), 
        turn_pid(1.0, 0.0, 0.0),
        name(name) {
    
    // Turn encoder
    turn_e.SetDistancePerPulse(2*pi * turnGearRatio);

    // Turn PID
    turn_pid.SetTolerance(0.0001);
    turn_pid.EnableContinuousInput(0, 2*pi);

    // Names for logging
    heading_name = this->name + " heading";
    setpoint_name = this->name + " setpoint";
    err_name = this->name + " error";
    err_name = this->name + " output";
  }

  //-- General state

  void setState(frc::SwerveModuleState s) {
    using namespace ctre::phoenix::motorcontrol;
    s = frc::SwerveModuleState::Optimize(s, getHeading());
    drive_m.Set(ControlMode::PercentOutput, s.speed.value());
    if (s.speed != 0_mps)
      SetTurn(s.angle.Radians());
  }

  //-- Drive

  void setPower(double percent) {
    using namespace ctre::phoenix::motorcontrol;
    drive_m.Set(ControlMode::PercentOutput, percent);
  }

  //-- Turn

  void SetTurn(radian_t r) {
    using namespace ctre::phoenix::motorcontrol;
    turn_pid.Reset();
    r = units::math::fmod(r, 360_deg);
    turn_pid.SetSetpoint(r.value());
  }

  void reverseTurn(bool b) { turn_e.SetReverseDirection(b); }

  void zero() { turn_e.Reset(); }
  void reset() { turn_pid.Reset(); }

  radian_t getHeading() const {
    return units::math::fmod(radian_t(turn_e.GetDistance()), 360_deg);
  }

  //-- PID

  void enableTurnPID(bool b) { turn_pid_enabled = b; }

  double getTurnP() { return turn_pid.GetP(); }
  double getTurnI() { return turn_pid.GetI(); }
  double getTurnD() { return turn_pid.getD(); }

  void setTurnPID(double p, double i, double d) {
    turn_pid.SetP(p);
    turn_pid.SetI(i);
    turn_pid.setD(d);
  }

  //-- Logging

  // Periodically do this
  void Periodic() override {
    using namespace ctre::phoenix::motorcontrol;

    // Set PID output format as percentage
    auto turn_pid_out = turn_pid.Calculate(turn_e.GetDistance());
    if (turn_pid_enabled)
      turn_m.Set(ControlMode::PercentOutput, turn_pid_out);
    
    // Log stuff
    frc::SmartDashboard::PutNumber(pid_name, turn_pid_out);
    frc::SmartDashboard::PutNumber(heading_name,
                                   getHeading().value() * (180/pi));
    frc::SmartDashboard::PutNumber(setpoint_name,
                                   turn_pid.GetSetpoint() * (180/pi));
    frc::SmartDashboard::PutNumber(err_name, 
                                   turn_pid.GetPositionError() * (180/pi));
  }

  // Pain
  void InitSendable(wpi::SendableBuilder &builder) override {
    builder.SetSmartDashboardType("Swerve pod");
    builder.AddDoubleProperty(
        "heading", [this]() { return units::degree_t(getHeading()).value(); },
        nullptr);
    builder.AddDoubleProperty(
        "setpoint",
        [this]() {
          return units::convert<units::angle::radian, units::angle::degree>(
              turn_pid.GetSetpoint());
        },
        nullptr);
  }

private:

  // Motors & encoders
  TalonSRX drive_m; // Drive motor
  TalonSRX turn_m; // Turn motor
  frc::Encoder turn_e; // Turn encoder
  frc2::PIDController turn_pid;

  // Names for logging
  std::string name; // Pod name
  std::string heading_name;
  std::string setpoint_name;
  std::string err_name;
  std::string out_name;

  bool turn_pid_enabled = true;
};

} // namespace abval
