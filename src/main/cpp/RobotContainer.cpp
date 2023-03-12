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
#include <functional>
#include <pathplanner/lib/PathPlanner.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/math.h>
#include <wpi/mutex.h>

#include "commands/Auto.hpp"
#include "field.hpp"
#include "frc/PowerDistribution.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/RunCommand.h"
#include "subsystems/SwerveDrive.hpp"
#include "units/angular_velocity.h"
#include "units/velocity.h"

namespace pp = pathplanner;

struct BalanceCommand
    : public frc2::CommandHelper<frc2::Command, BalanceCommand> {
  typedef std::function<void(frc::ChassisSpeeds)> OutFun;
  AHRS &gyro;
  OutFun out;
  std::initializer_list<frc2::Subsystem *> req;

  BalanceCommand(AHRS &gyro, OutFun out,
                 std::initializer_list<frc2::Subsystem *> req)
      : gyro(gyro), out(out), req(req) {}
  virtual void Initialize() override {}
  virtual void Execute() override {
    if (gyro.GetYaw() > 15) {
      out(frc::ChassisSpeeds{.vx = 0.6_mps});
    } else if (gyro.GetYaw() < -15) {
      out(frc::ChassisSpeeds{.vx = -0.6_mps});
    }
  }

  virtual void End(bool interrupted) override {
    out(frc::ChassisSpeeds{.vx = 0_mps});
  }

  virtual wpi::SmallSet<frc2::Subsystem *, 4> GetRequirements() const override {
    return {req};
  }
};

RobotContainer::RobotContainer()
    : gyro(frc::SPI::Port::kMXP),
      autoBuilder(
          [this]() { return getPose(); },
          [this](frc::Pose2d p) {
            gyro.Reset();
            basePose = p;
          },
          pp::PIDConstants(5.0, 0.0, 0.0), pp::PIDConstants(0.5, 0.0, 0.0),
          [this](frc::ChassisSpeeds s) { swerve.setSpeed(s); },
          {{"balance",
            std::make_shared<BalanceCommand>(
                gyro, [this](frc::ChassisSpeeds s) { swerve.setSpeed(s); },
                std::initializer_list<frc2::Subsystem *>{&swerve})}},
          {&swerve}),
      power(0, frc::PowerDistribution::ModuleType::kCTRE) {
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
        if (units::math::hypot(speed.vx, speed.vy) > .2_mps ||
            units::math::abs(speed.omega) > .2_rad_per_s)
          swerve.setSpeed(speed);
        else
          swerve.setSpeed(frc::ChassisSpeeds{});
        frc::SmartDashboard::PutNumber("total pow:", power.GetTotalPower());
        frc::SmartDashboard::PutNumber("total current:",
                                       power.GetTotalCurrent());
      },
      {&swerve}));
  arm.SetDefaultCommand(frc2::RunCommand([this]() {
                          arm.setPower(controller.GetRawAxis(1));
                        }).ToPtr());
  joystick.Button(7)
      .OnTrue(frc2::InstantCommand([this]() { arm.open(); }, {&arm}).ToPtr())
      .OnFalse(
          frc2::InstantCommand([this]() { arm.neutral(); }, {&arm}).ToPtr());
  joystick.Button(8)
      .OnTrue(frc2::InstantCommand([this]() { arm.close(); }, {&arm}).ToPtr())
      .OnFalse(
          frc2::InstantCommand([this]() { arm.neutral(); }, {&arm}).ToPtr());
  joystick.Button(9)
      .OnTrue(frc2::InstantCommand([this]() { winch.setPower(1); }, {&winch})
                  .ToPtr())
      .OnFalse(frc2::InstantCommand([this]() { winch.setPower(0); }, {&winch})
                   .ToPtr());
  joystick.Button(10)
      .OnTrue(frc2::InstantCommand([this]() { winch.setPower(-0.2); }, {&winch})
                  .ToPtr())
      .OnFalse(frc2::InstantCommand([this]() { winch.setPower(0); }, {&winch})
                   .ToPtr());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return autoBuilder.followPathWithEvents(pp::PathPlanner::loadPath(
      "balance", pp::PathConstraints(1_mps, 1_mps_sq)));
}
