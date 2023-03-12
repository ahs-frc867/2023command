// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <AHRS.h>
#include <frc/PowerDistribution.h>
#include <frc/event/EventLoop.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/button/CommandXboxController.h>
#include <pathplanner/lib/auto/SwerveAutoBuilder.h>

#include "Constants.hpp"
#include "frc/kinematics/ChassisSpeeds.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "frc2/command/Subsystem.h"
#include "subsystems/Arm.hpp"
#include "subsystems/SwerveDrive.hpp"
#include "subsystems/Winch.hpp"

constexpr int operator""_port(unsigned long long i) noexcept { return i; }

class RobotContainer {
public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

private:
  frc2::CommandJoystick joystick{0_port};
  frc2::CommandXboxController controller{1_port};

  void ConfigureBindings();
  abval::SwerveDrive swerve;
  AHRS gyro;
  pathplanner::SwerveAutoBuilder autoBuilder;
  frc::Pose2d basePose = {};
  frc::Pose2d getPose();
  abval::Winch winch;
  abval::Arm arm;
  frc::PowerDistribution power;
};
