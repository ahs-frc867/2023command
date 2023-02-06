#pragma once

#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <fmt/format.h>
#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <math.h>
#include <units/angle.h>
#include <units/math.h>

#include <numbers>

namespace abval {
// pg71 gear ratio, swerve gear ratio, pulses per rot
constexpr double swerveGearRatio = 3179.0 / 226233.0 * 48.0 / 40.0 / 7.0;
using ctre::phoenix::motorcontrol::can::TalonSRX;
using std::numbers::pi;
using units::radian_t;

inline double clamp(double a) {
  if (a > 1.0) return 1.0;
  if (a < -1.0) return -1.0;
  return a;
}

class SwervePod : public frc2::SubsystemBase {
 public:
  SwervePod(int drive_id, int swerve_id, int encoder_channel_a,
            int encoder_channel_b)
      : drive(drive_id),
        turn_m(swerve_id),
        turn_pid(-1.0, 0.0, 0.0),
        turn_e(encoder_channel_a, encoder_channel_b) {
    turn_e.SetDistancePerPulse(pi * 2.0 * swerveGearRatio);
  }

  frc2::CommandPtr SetPower(double percentage) {
    using namespace ctre::phoenix::motorcontrol;
    return frc2::Subsystem::RunOnce([=, this]() {
      drive.Set(ControlMode::PercentOutput, dir * percentage);
    });
  }

  frc2::CommandPtr SetTurnPower(double percentage) {
    using namespace ctre::phoenix::motorcontrol;
    return frc2::Subsystem::RunOnce(
        [=, this]() { turn_m.Set(ControlMode::PercentOutput, percentage); });
  }

  frc2::CommandPtr SetTurn(radian_t r) {
    using namespace ctre::phoenix::motorcontrol;
    turn_pid.Reset();
    turn_pid.SetSetpoint(r.value());
    return frc2::Subsystem::Run([=, this]() {
      turn_m.Set(ControlMode::PercentOutput,
                 clamp(turn_pid.Calculate(turn_e.GetDistance())));
      ;
    });
  }
  void Periodic() override {
    frc::SmartDashboard::PutNumber("rotation", turn_e.GetDistance());
    frc::SmartDashboard::PutNumber("set", turn_pid.GetSetpoint());
    frc::SmartDashboard::PutNumber("dist", turn_pid.GetPositionError());
  }

  void SimulationPeriodic() override { /*does nothing*/
  }

 private:
  TalonSRX drive;
  TalonSRX turn_m;
  frc::Encoder turn_e;

  frc2::PIDController turn_pid;
  double dir = 1.0;

  radian_t getAbs() const {
    return units::math::fmod(radian_t(turn_e.GetDistance()), 360_deg);
  }

  // assumes normalized angles
  static radian_t calcOptimal(radian_t curr, radian_t target) {
    auto distance = target - curr;
    if (distance > 180_deg) {
      distance = distance - 360_deg;
    } else if (distance < -180_deg) {
      distance = distance + 360_deg;
    }
    return distance;
  }
};

}  // namespace abval