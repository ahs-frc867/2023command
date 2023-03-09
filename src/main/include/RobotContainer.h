// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <frc/event/EventLoop.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include "Constants.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "subsystems/Arm.hpp"

constexpr int operator""_port(unsigned long long i) noexcept { return i; }

class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

private:
  frc2::CommandJoystick joystick{0_port};

  void ConfigureBindings();
  abval::SwerveDrive swerve;
  AHRS gyro;
  pathplanner::SwerveAutoBuilder autoBuilder;
  frc::Pose2d basePose = {};
  frc::Pose2d getPose();
  abval::Arm arm;
};
