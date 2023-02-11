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

class SwervePod : public frc2::SubsystemBase {
 public:
  SwervePod(int drive_id, int swerve_id, int encoder_channel_a,
            int encoder_channel_b)
      : drive(drive_id),
        turn_m(swerve_id),
        turn_e(encoder_channel_a, encoder_channel_b),
        turn_pid(1.0, 0.0, 0.0) {
    turn_e.SetDistancePerPulse(pi * 2.0 * swerveGearRatio);
    turn_e.SetReverseDirection(true);
  }

  void SetPower(double percentage) {
    using namespace ctre::phoenix::motorcontrol;
    drive.Set(ControlMode::PercentOutput, dir * percentage);
  }

  void SetTurn(radian_t r) {
    using namespace ctre::phoenix::motorcontrol;
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
  void Periodic() override {
    using namespace ctre::phoenix::motorcontrol;
    turn_m.Set(ControlMode::PercentOutput,
               turn_pid.Calculate(turn_e.GetDistance()));
    frc::SmartDashboard::PutNumber("rotation", turn_e.GetDistance());
    frc::SmartDashboard::PutNumber("set", turn_pid.GetSetpoint());
    frc::SmartDashboard::PutNumber("err", turn_pid.GetPositionError());
  }

  void SimulationPeriodic() override { /*does nothing*/
  }
  void reset() { turn_pid.Reset(); }

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
  radian_t calcOptimal(radian_t target) {
    radian_t curr = getAbs();
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