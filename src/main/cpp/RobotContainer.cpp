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
  static int mode = 0;
  swerve.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        using namespace units::math;
        auto speed = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
            frc::ChassisSpeeds{.vx = -meters_per_second_t(joystick.GetX()),
                               .vy = meters_per_second_t(joystick.GetY()),
                               .omega = radians_per_second_t(joystick.GetZ())},
            90_deg);
        if (hypot(speed.vx, speed.vy) > .1_mps ||
            abs(speed.omega) > .1_rad_per_s)
          swerve.pods[mode].setState(
              frc::SwerveModuleState{.speed = hypot(speed.vx, speed.vy),
                                     .angle = atan2(speed.vy, speed.vx)});
        else
          swerve.setSpeed(frc::ChassisSpeeds{});
      },
      {&swerve}));
  joystick.Button(7).OnTrue(
      frc2::InstantCommand([this]() { swerve.home(); }, {&swerve}).ToPtr());
  joystick.Button(9).OnTrue(
      frc2::InstantCommand([this]() { mode = 0; }, {}).ToPtr());
  joystick.Button(10).OnTrue(
      frc2::InstantCommand([this]() { mode = 1; }, {}).ToPtr());
  joystick.Button(11).OnTrue(
      frc2::InstantCommand([this]() { mode = 2; }, {}).ToPtr());
  joystick.Button(12).OnTrue(
      frc2::InstantCommand([this]() { mode = 3; }, {}).ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  pp::PathPlannerTrajectory bluepath = pp::PathPlanner::loadPath(
      "bluepath", pp::PathConstraints(4_mps, 3_mps_sq));
  return autoBuilder.followPath(bluepath);
}
