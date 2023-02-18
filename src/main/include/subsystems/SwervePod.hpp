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
        turn_e(encoder_channel_a, encoder_channel_b), turn_pid(1.0, 0.0, 0.0),
        name(name) {
    turn_e.SetDistancePerPulse(pi * 2.0 * swerveGearRatio);
    turn_pid.SetTolerance(0.0001);
    heading_name = this->name + " heading";
    setpoint_name = this->name + " setpoint";
    err_name = this->name + " error";
    err_name = this->name + " output";
    turn_pid.EnableContinuousInput(0, 2 * pi);
  }

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

  void SetTurn(radian_t r) {
    using namespace ctre::phoenix::motorcontrol;
    turn_pid.Reset();
    r = units::math::fmod(r, 360_deg);
    turn_pid.SetSetpoint(r.value());
  }

  void reverseTurn(bool b) { turn_e.SetReverseDirection(b); }

  void setState(frc::SwerveModuleState s) {
    using namespace ctre::phoenix::motorcontrol;
    s = frc::SwerveModuleState::Optimize(s, getHeading());
    drive.Set(ControlMode::PercentOutput, s.speed.value());
    if (s.speed != 0_mps)
      SetTurn(s.angle.Radians());
  }

  void enablePID(bool b) { enabled = b; }

  void Periodic() override {
    using namespace ctre::phoenix::motorcontrol;
    auto out = turn_pid.Calculate(turn_e.GetDistance());
    if (enabled)
      turn_m.Set(ControlMode::PercentOutput, out);
    frc::SmartDashboard::PutNumber(err_name, out);
    frc::SmartDashboard::PutNumber(heading_name,
                                   getHeading().value() / (2 * pi) * 360);
    frc::SmartDashboard::PutNumber(setpoint_name,
                                   turn_pid.GetSetpoint() / (2 * pi) * 360);
    frc::SmartDashboard::PutNumber(err_name, turn_pid.GetPositionError() /
                                                 (2 * pi) * 360);
  }

  void setPower(double percent) {
    using namespace ctre::phoenix::motorcontrol;
    drive.Set(ControlMode::PercentOutput, percent);
  }

  void zero() { turn_e.Reset(); }
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
  bool enabled = true;

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