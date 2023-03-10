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
#include <pathplanner/lib/PathPlanner.h>
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

namespace pp = pathplanner;

const static std::unordered_map<std::string, std::shared_ptr<frc2::Command>>
    eventMap;

RobotContainer::RobotContainer()
    : gyro(frc::SPI::Port::kMXP),
      autoBuilder([this]() { return getPose(); },
                  [this](frc::Pose2d p) {
                    gyro.Reset();
                    basePose = p;
                  },
                  pp::PIDConstants(5.0, 0.0, 0.0),
                  pp::PIDConstants(0.5, 0.0, 0.0),
                  [this](frc::ChassisSpeeds s) { swerve.setSpeed(s); },
                  eventMap, {&swerve}) {
  ConfigureBindings();
}

frc::Pose2d RobotContainer::getPose() {
  using units::length::meter_t;
  return frc::Pose2d(frc::Translation2d(meter_t(gyro.GetDisplacementX()),
                                        meter_t(gyro.GetDisplacementY())),
                     gyro.GetRotation2d())
      .RelativeTo(basePose);
}

void RobotContainer::ConfigureBindings() {
  using namespace units;
  swerve.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        auto speed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            frc::ChassisSpeeds{.vx = meters_per_second_t(joystick.GetY()),
                               .vy = meters_per_second_t(joystick.GetX()),
                               .omega = radians_per_second_t(-joystick.GetZ())},
            0_deg);
        if (units::math::hypot(speed.vx, speed.vy) > .1_mps ||
            units::math::abs(speed.omega) > .1_rad_per_s)
          swerve.setSpeed(speed);
        else
          swerve.setSpeed(frc::ChassisSpeeds{});
      },
      {&swerve}));
  arm.SetDefaultCommand(frc2::RunCommand([this]() {
                          arm.setPower(controller.GetRawAxis(1));
                        }).ToPtr());
  controller.A()
      .OnTrue(frc2::InstantCommand([this]() { arm.open(); }, {&arm}).ToPtr())
      .OnFalse(
          frc2::InstantCommand([this]() { arm.neutral(); }, {&arm}).ToPtr());
  controller.B()
      .OnTrue(frc2::InstantCommand([this]() { arm.close(); }, {&arm}).ToPtr())
      .OnFalse(
          frc2::InstantCommand([this]() { arm.neutral(); }, {&arm}).ToPtr());
  controller.X()
      .OnTrue(frc2::InstantCommand([this]() { winch.setPower(1); }, {&winch})
                  .ToPtr())
      .OnFalse(frc2::InstantCommand([this]() { winch.setPower(0); }, {&winch})
                   .ToPtr());
  controller.Y()
      .OnTrue(frc2::InstantCommand([this]() { winch.setPower(-0.2); }, {&winch})
                  .ToPtr())
      .OnFalse(frc2::InstantCommand([this]() { winch.setPower(0); }, {&winch})
                   .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  pp::PathPlannerTrajectory bluepath = pp::PathPlanner::loadPath(
      "testpath", pp::PathConstraints(4_mps, 3_mps_sq));
  return autoBuilder.followPath(bluepath);
}
