// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <fmt/format.h>
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/CoordinateSystem.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/button/Trigger.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>

#include "commands/Auto.hpp"
#include "field.hpp"
#include "subsystems/SwerveDrive.hpp"

RobotContainer::RobotContainer() { ConfigureBindings(); }
void RobotContainer::ConfigureBindings() {
  using namespace units;
  swerve.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        auto transform = frc::Translation2d(meter_t(joystick.GetX()),
                                            meter_t(joystick.GetY()));
        if (transform.Norm() > .1_m)
          swerve.SetVelocity(transform.RotateBy(frc::Rotation2d(90_deg)));
        else
          swerve.SetVelocity(frc::Translation2d(meter_t(0), meter_t(0)));
      },
      {&swerve}));
  joystick.Button(7).OnTrue(swerve.zero());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::RunCommand([]() {}).ToPtr();
}
