// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <AHRS.h>
#include <fmt/format.h>
#include <frc/SPI.h>
#include <frc/geometry/CoordinateSystem.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <wpi/mutex.h>
#include <frc2/command/InstantCommand.h>

#include "commands/Auto.hpp"
#include "field.hpp"
#include "frc2/command/RunCommand.h"
#include "subsystems/SwerveDrive.hpp"
#include "units/angular_velocity.h"
#include "units/velocity.h"

RobotContainer::RobotContainer()
    : //  gyro(frc::SPI::Port::kMXP),
      swerve() {
  ConfigureBindings();
}
void RobotContainer::ConfigureBindings() {
  using namespace units;
  swerve.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        auto speed =
            frc::ChassisSpeeds{.vx = meters_per_second_t(joystick.GetX()),
                               .vy = meters_per_second_t(joystick.GetY()),
                               .omega = radians_per_second_t(joystick.GetZ())};
        if (units::math::hypot(speed.vx, speed.vy) > .1_mps ||
            speed.omega > .1_rad_per_s)
          swerve.setSpeed(speed);
        else
          swerve.setSpeed(frc::ChassisSpeeds{});
        auto headings = swerve.getHeadings();
        frc::SmartDashboard::PutNumber("Q1h", headings[0].value());
        frc::SmartDashboard::PutNumber("Q2h", headings[1].value());
        frc::SmartDashboard::PutNumber("Q3h", headings[2].value());
        frc::SmartDashboard::PutNumber("Q4h", headings[3].value());
      },
      {&swerve}));
  joystick.Button(7).OnTrue(
      frc2::InstantCommand([this]() { swerve.setPID(0.1, 0, 0); }, {&swerve})
          .ToPtr());
  joystick.Button(9).OnTrue(
      frc2::InstantCommand([this]() { swerve.setPID(0, 0.1, 0); }, {&swerve})
          .ToPtr());
  joystick.Button(11).OnTrue(
      frc2::InstantCommand([this]() { swerve.setPID(0, 0, 0.1); }, {&swerve})
          .ToPtr());
  joystick.Button(8).OnTrue(
      frc2::InstantCommand([this]() { swerve.setPID(-0.1, 0, 0); }, {&swerve})
          .ToPtr());
  joystick.Button(10).OnTrue(
      frc2::InstantCommand([this]() { swerve.setPID(0, -0.1, 0); }, {&swerve})
          .ToPtr());
  joystick.Button(12).OnTrue(
      frc2::InstantCommand([this]() { swerve.setPID(0, 0, -0.1); }, {&swerve})
          .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::RunCommand([]() {}, {}).ToPtr();
}
