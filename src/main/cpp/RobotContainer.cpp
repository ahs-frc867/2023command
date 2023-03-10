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
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  pp::PathPlannerTrajectory bluepath = pp::PathPlanner::loadPath(
      "testpath", pp::PathConstraints(4_mps, 3_mps_sq));
  return autoBuilder.followPath(bluepath);
}
