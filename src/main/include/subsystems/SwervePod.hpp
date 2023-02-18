#pragma once

#include "frc/smartdashboard/SmartDashboard.h"
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <fmt/format.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <math.h>
#include <units/angle.h>
#include <units/math.h>

#include <numbers>
#include <string>
#include <string_view>

namespace abval {
// pg71 gear ratio, swerve gear ratio, pulses per rot
constexpr double swerveGearRatio = 3179.0 / 226233.0 * 48.0 / 40.0 / 7.0;
using ctre::phoenix::motorcontrol::can::TalonSRX;
using std::numbers::pi;
using units::radian_t;

class SwervePod : public frc2::SubsystemBase {
public:
  SwervePod(int drive_id, int swerve_id, int encoder_channel_a,
            int encoder_channel_b, std::string_view name, radian_t rot = 0_rad)
      : drive(drive_id), turn_m(swerve_id),
        turn_e(encoder_channel_a, encoder_channel_b), turn_pid(0.3, 0.0, 0.0),
        name(name) {
    turn_e.SetDistancePerPulse(pi * 2.0 * swerveGearRatio);
    heading_name = this->name + " heading";
    setpoint_name = this->name + " setpoint";
    err_name = this->name + " error";
    err_name = this->name + " output";
    Subsystem::SetDefaultCommand(Run([this]() {
      using namespace ctre::phoenix::motorcontrol;
      auto out = turn_pid.Calculate(turn_e.GetDistance());
      turn_m.Set(ControlMode::PercentOutput, out);
      frc::SmartDashboard::PutNumber(err_name, out);
    }));
  }

  void SetTurn(radian_t r) {
    using namespace ctre::phoenix::motorcontrol;
    r += rot;
    turn_pid.Reset();
    r = units::math::fmod(r, 360_deg);
    radian_t fwd_dist = calcOptimal(r);
    radian_t back_dist = calcOptimal(r + 180_deg);
    if (units::math::abs(fwd_dist) < units::math::abs(back_dist)) {
      turn_pid.SetSetpoint(fwd_dist.value() + turn_e.GetDistance());
      dir = 1.0;
    } else {
      
      turn_pid.SetSetpoint(back_dist.value() + turn_e.GetDistance());
      dir = -1.0;
    }
  }

  void reverseTurn(bool b) { turn_e.SetReverseDirection(b); }

  void setState(frc::SwerveModuleState s) {
    using namespace ctre::phoenix::motorcontrol;
    drive.Set(ControlMode::PercentOutput, dir * s.speed.value());
    if (s.speed != 0_mps)
      SetTurn(s.angle.Radians());
  }

  void Periodic() override {
    frc::SmartDashboard::PutNumber(heading_name, turn_e.GetDistance());
    frc::SmartDashboard::PutNumber(setpoint_name, turn_pid.GetSetpoint());
    frc::SmartDashboard::PutNumber(err_name, turn_pid.GetPositionError());
  }

  void reset() { turn_pid.Reset(); }

  radian_t getHeading() const {
    return units::math::fmod(radian_t(turn_e.GetDistance()), 360_deg);
  }

  frc2::PIDController turn_pid;

private:
  TalonSRX drive;
  TalonSRX turn_m;
  frc::Encoder turn_e;
  std::string name;
  std::string heading_name;
  std::string setpoint_name;
  std::string err_name;
  std::string out_name;
  radian_t rot = 0_rad;

  double dir = 1.0;

  // assumes normalized angles
  radian_t calcOptimal(radian_t target) {
    radian_t curr = getHeading();
    auto distance = target - curr;
    if (distance > 180_deg) {
      distance = distance - 360_deg;
    } else if (distance < -180_deg) {
      distance = distance + 360_deg;
    }
    return distance;
  }
};

} // namespace abval