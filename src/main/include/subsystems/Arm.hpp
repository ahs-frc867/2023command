#pragma once

#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "frc/Encoder.h"
#include "frc/PneumaticsModuleType.h"
#include "frc/controller/PIDController.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "units/voltage.h"
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
    frc::SmartDashboard::PutNumber("kG", feedforward.kG.value());
    frc::SmartDashboard::PutNumber("kS", feedforward.kS.value());
    frc::SmartDashboard::PutNumber("kV", feedforward.kV.value());
    // frc::SmartDashboard::PutNumber("kA", feedforward.kA.value());
  }

public:
  Arm()
      : motor(15), encoder(8, 9, false), pid(1, 0, 0),
        collector(frc::PneumaticsModuleType::CTREPCM, 0, 1),
        feedforward(0_V, 0_V, kV_t(0)) {
    pid.EnableContinuousInput(0, std::numbers::pi * 2);
    encoder.SetDistancePerPulse(2 * std::numbers::pi / 600);
  }
  void setTarget(radian_t target) noexcept { pid.SetSetpoint(target.value()); }
  void changeFeedConstants(double dG, double dS, double dV) {
    feedforward.kG += units::volt_t(dG);
    feedforward.kS += units::volt_t(dS);
    feedforward.kV += kV_t(dV);
    // feedforward.kA += kA_t(dA);
  }
  void open() { collector.Set(collector.kForward); }
  void close() { collector.Set(collector.kReverse); }
};
} // namespace abval