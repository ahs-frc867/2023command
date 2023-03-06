#pragma once

#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "frc/Encoder.h"
#include "frc/PneumaticsModuleType.h"
#include "frc/controller/PIDController.h"
#include <cmath>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/DoubleSolenoid.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/SubsystemBase.h>
#include <numbers>
#include <units/angle.h>

namespace abval {
using ctre::phoenix::motorcontrol::can::TalonSRX;
using units::radian_t;
class Arm : public frc2::SubsystemBase {
  TalonSRX motor;
  frc::Encoder encoder;
  frc2::PIDController pid;
  frc::DoubleSolenoid collector;
  double kS = 0, kV = 0;
  void Periodic() override {
    using ctre::phoenix::motorcontrol::ControlMode;
    auto distance = encoder.GetDistance();
    auto err = pid.GetSetpoint() - distance;
    motor.Set(ControlMode::PercentOutput,
              pid.Calculate(distance) + kV * err + std::copysign(kS, err));
  }

public:
  Arm(int motor_id, int chan_a, int chan_b)
      : motor(motor_id), encoder(chan_a, chan_b, false),
        collector(frc::PneumaticsModuleType::CTREPCM, 0, 1), pid(1, 0, 0) {
    pid.EnableContinuousInput(0, std::numbers::pi * 2);
  }
  void setTarget(radian_t target) noexcept { pid.SetSetpoint(target.value()); }
};
} // namespace abval