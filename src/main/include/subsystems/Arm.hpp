#pragma once

#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "frc/Encoder.h"
#include "frc/PneumaticsModuleType.h"
#include "frc/controller/PIDController.h"
#include <cmath>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DoubleSolenoid.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <numbers>
#include <units/angle.h>

namespace abval {
using ctre::phoenix::motorcontrol::can::TalonSRX;
using units::radian_t;
class Arm : public frc2::SubsystemBase {
  typedef units::unit_t<frc::ArmFeedforward::ka_unit> kA_t;
  typedef units::unit_t<frc::ArmFeedforward::kv_unit> kV_t;
  typedef units::unit_t<frc::ArmFeedforward::Velocity> v_t;
  TalonSRX motor;
  frc::Encoder encoder;
  frc2::PIDController pid;
  frc::DoubleSolenoid collector;
  frc::ArmFeedforward feedforward;
  double kS = 0, kV = 0, kG = 0;
  double prev_s;
  void Periodic() override {
    using ctre::phoenix::motorcontrol::ControlMode;
    auto distance = encoder.GetDistance();
    v_t v = (radian_t(distance) - radian_t(prev_s)) / 20_ms;
    prev_s = distance;
    motor.Set(
        ControlMode::PercentOutput,
        (pid.Calculate(distance) +
         feedforward.Calculate(radian_t(encoder.GetDistance()), v).value()) /
            (12_V).value());
  }

public:
  Arm(int motor_id, int chan_a, int chan_b)
      : motor(motor_id), encoder(chan_a, chan_b, false),
        collector(frc::PneumaticsModuleType::CTREPCM, 0, 1), pid(1, 0, 0),
        feedforward(0_V, 0_V, kV_t(0), kA_t(0)) {
    pid.EnableContinuousInput(0, std::numbers::pi * 2);
    encoder.SetDistancePerPulse(2 * std::numbers::pi / 600);
  }
  void setTarget(radian_t target) noexcept { pid.SetSetpoint(target.value()); }
};
} // namespace abval