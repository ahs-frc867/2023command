// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Auto.hpp"

RobotContainer::RobotContainer() : Q1(5, 2, 9, 8) { ConfigureBindings(); }

void RobotContainer::ConfigureBindings() {
  joystick.Trigger()
               .OnTrue(Q1.SetPower(0.5))
               .OnFalse(Q1.SetPower(0.0));
  joystick.Button(7).OnTrue(Q1.SetTurn(0_deg));
  joystick.Button(8).OnTrue(Q1.SetTurn(90_deg));
  joystick.Button(9).OnTrue(Q1.SetTurn(-90_deg));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return autop::Auto(&Q1);
}
