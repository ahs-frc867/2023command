// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>

#include "Constants.hpp"
#include "subsystems/SwervePod.hpp"

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc2::CommandJoystick joystick{abval::Constants::kDriverPort};

  void ConfigureBindings();
  abval::SwervePod Q1;
};
