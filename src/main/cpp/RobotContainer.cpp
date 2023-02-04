// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/button/Trigger.h>

#include "commands/Autos.h"
#include "commands/ExampleCommand.h"

RobotContainer::RobotContainer() : Q1(5, 2, 9, 8) { ConfigureBindings(); }

void RobotContainer::ConfigureBindings() {
  frc2::Trigger([this] {
    return m_subsystem.ExampleCondition();
  }).OnTrue(ExampleCommand(&m_subsystem).ToPtr());

  m_driverController.Trigger()
               .OnTrue(Q1.SetPower(0.5))
               .OnFalse(Q1.SetPower(0.0));
  m_driverController.Button(7).OnTrue(Q1.SetTurn(0_deg));
  m_driverController.Button(8).OnTrue(Q1.SetTurn(90_deg));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return autos::ExampleAuto(&m_subsystem);
}
