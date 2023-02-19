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
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <wpi/mutex.h>

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
          
        auto heading = 90_deg; // TODO: replace with IMU reading
          
        // Convert x, y, and angular velocities from field to chassis perspective.
        auto speed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            frc::ChassisSpeeds{
                .vx = -meters_per_second_t(joystick.GetX()),
                .vy = meters_per_second_t(joystick.GetY()),
                .omega = radians_per_second_t(joystick.GetZ())
            }, heading
        );
          
        // If speed is below threshold, don't take it
        auto min_linear_speed = 0.1_mps;
        auto min_angular_speed = 0.1_rad_per_s;
        if (units::math::hypot(speed.vx, speed.vy) < min_linear_speed &&
            units::math::fabs(speed.omega) < min_angular_speed)
        {
          swerve.setSpeed(frc::ChassisSpeeds{});
          return;
        }
        
        swerve.setSpeed(speed);
      },
      {&swerve}));

  static bool turn_pid_enabled = false;
    
  // Toggle pod turn PID
  joystick.Button(3).OnTrue(
      frc2::InstantCommand(
          [&, this]() {
            swerve.enableTurnPID(turn_pid_enabled);
            turn_pid_enabled = !turn_pid_enabled;
          },
          {&swerve})
          .ToPtr());
  
  // Zero pod encoders
  joystick.Button(4).OnTrue(
      frc2::InstantCommand([this]() { swerve.zero(); }, {&swerve}).ToPtr());
  
  // Return pods to 0 rotation
  joystick.Button(7).OnTrue(
      frc2::InstantCommand([this]() { swerve.home(); }, {&swerve}).ToPtr());
    
  // To drive or not to drive
  joystick.Button(11)
      .OnTrue(
          frc2::RunCommand([this]() { swerve.setPower(0.1); }, {&swerve})
              .ToPtr())
      .OnFalse(
          frc2::InstantCommand([this]() { swerve.setPower(0.0); }, {&swerve})
              .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::RunCommand([]() {}, {}).ToPtr();
}
