#pragma once

#include "frc/smartdashboard/SmartDashboard.h"
#include "rev/CANSparkMax.h"
#include "rev/CANSparkMaxLowLevel.h"
#include "rev/SparkMaxRelativeEncoder.h"
#include <frc2/command/SubsystemBase.h>
#include <numbers>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

namespace abval {
using units::meter_t;
using units::radian_t;
struct Winch : public frc2::SubsystemBase {
  Winch()
      : motor(8, rev::CANSparkMaxLowLevel::MotorType::kBrushed){
    // pid.SetP(1);
  }
  // void setPos(meter_t pos) {
  //   pid.SetReference((pos / spool_radius).convert<units::turns>().value(),
  //                    rev::CANSparkMax::ControlType::kPosition);
  // }
  void setPower(double p) { motor.Set(p); }

private:
  rev::CANSparkMax motor;
  // rev::SparkMaxRelativeEncoder encoder;
  // rev::SparkMaxPIDController pid;
  // radius (m) / ang (rad)
  static constexpr auto spool_radius = 0.965_in / 1_rad;
  void Periodic() override {
    // frc::SmartDashboard::PutNumber("neo position", encoder.GetPosition());
  }
};
} // namespace abval