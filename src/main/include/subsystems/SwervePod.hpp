#pragma once

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

#include "frc/shuffleboard/SimpleWidget.h"
#include "networktables/GenericEntry.h"
#include "units/base.h"

namespace abval {
// pg71 gear ratio, swerve gear ratio, pulses per rot
constexpr double swerveGearRatio = 3179.0 / 226233.0 * 48.0 / 40.0 / 7.0;
using ctre::phoenix::motorcontrol::can::TalonSRX;
using std::numbers::pi;
using units::radian_t;
namespace umath = units::math;

class SwervePod : public frc2::SubsystemBase {
public:
  SwervePod(int drive_id, int swerve_id, int encoder_channel_a,
            int encoder_channel_b, std::string_view name, radian_t rot = 0_rad)
      : drive(drive_id), turn_m(swerve_id),
        turn_e(encoder_channel_a, encoder_channel_b), turn_pid(1.0, 0.0, 0.0) {
    turn_e.SetDistancePerPulse(pi * 2.0 * swerveGearRatio);
    turn_pid.SetTolerance(0.0001);
    auto prefix = std::string(name);
    heading_name = prefix + " heading";
    setpoint_name = prefix + " setpoint";
    err_name = prefix + " error";
    err_name = prefix + " output";
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
    turn_pid.SetSetpoint(r.value());
  }

  void reverseTurn(bool b) { turn_e.SetReverseDirection(b); }

  void setState(frc::SwerveModuleState s) {
    using namespace ctre::phoenix::motorcontrol;
    radian_t fwd = calcOptimal(s.angle.Radians());
    radian_t back = calcOptimal(s.angle.Radians() + 180_deg);
    if (umath::abs(fwd) < umath::abs(back)) {
      SetTurn(fwd + radian_t(turn_e.GetDistance()));
      drive.Set(ControlMode::PercentOutput, s.speed.value());
      // drive.current
    } else {
      SetTurn(back + radian_t(turn_e.GetDistance()));
      drive.Set(ControlMode::PercentOutput, -s.speed.value());
    }
  }

  void enablePID(bool b) { enabled = b; }

  void Periodic() override {
    using namespace units;
    using namespace ctre::phoenix::motorcontrol;
    auto out = turn_pid.Calculate(turn_e.GetDistance());
    if (enabled)
      turn_m.Set(ControlMode::PercentOutput, out);
    frc::SmartDashboard::PutNumber(err_name, out);
    frc::SmartDashboard::PutNumber(heading_name,
                                   getHeading().convert<degrees>().value());
    frc::SmartDashboard::PutNumber(
        setpoint_name, convert<radian, degrees>(turn_pid.GetSetpoint()));
    frc::SmartDashboard::PutNumber(
        err_name, convert<radian, degrees>(turn_pid.GetPositionError()));
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
  std::string heading_name;
  std::string setpoint_name;
  std::string err_name;
  std::string out_name;
  bool enabled = true;

  // assumes normalized angles
  radian_t calcOptimal(radian_t target) {
    radian_t curr = getHeading();
    auto distance = target - curr;
    while (distance >= 180_deg) {
      distance = distance - 360_deg;
    } 
    while (distance < -180_deg) {
      distance = distance + 360_deg;
    }
    return distance;
  }
};

} // namespace abval