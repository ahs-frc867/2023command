// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/RunCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "commands/Auto.hpp"

RobotContainer::RobotContainer() : Q1(5, 2, 9, 8) { ConfigureBindings(); }
namespace {
using namespace units;
radian_t joystickangle(frc::Joystick& j) {
  return math::atan2(meter_t(j.GetY()), meter_t(j.GetX()));
}
}  // namespace
void RobotContainer::ConfigureBindings() {
  using namespace units;
  auto loop = frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop();
  joystick.Trigger().OnTrue(Q1.SetPower(0.5)).OnFalse(Q1.SetPower(0.0));
  (joystick.AxisGreaterThan(0, 0.1, loop) ||
   joystick.AxisGreaterThan(1, 0.1, loop))
      .IfHigh([this]() { Q1.SetTurn(joystickangle(joystick)); });
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return autop::Auto(&Q1);
}
