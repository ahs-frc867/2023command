#pragma once

#include "auto/ControlConstants.hpp"
#include "pathplanner/lib/PathPlannerTrajectory.h"
#include "pathplanner/lib/auto/PIDConstants.h"
#include "units/angular_velocity.h"
#include "units/velocity.h"
#include <frc/Timer.h>
#include <frc/controller/PIDController.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/CommandHelper.h>

namespace abval {
class SwerveFollowTraject
    : public frc2::CommandHelper<frc2::CommandBase, SwerveFollowTraject> {
  frc2::PIDController x_pid, y_pid, theta_pid;
  TranslationFeedforward xy_feed;
  TurnFeedForward theta_feed;
  std::function<abval::RobotState()> &in;
  std::function<void(frc::ChassisSpeeds)> &out;
  pp::PathPlannerTrajectory trajectory;
  std::initializer_list<frc2::Subsystem *> requirements;
  frc::Timer timer;
  SwerveFollowTraject(pp::PIDConstants xy_pid, pp::PIDConstants theta_pid,
                      TranslationFeedforward xy_feed,
                      TurnFeedForward theta_feed,
                      std::function<void(frc::ChassisSpeeds)> &out,
                      std::function<abval::RobotState()> &in,
                      pp::PathPlannerTrajectory trajectory,
                      std::initializer_list<frc2::Subsystem *> requirements)
      : x_pid(xy_pid.m_kP, xy_pid.m_kI, xy_pid.m_kD, xy_pid.m_period),
        y_pid(x_pid), theta_pid(theta_pid.m_kP, theta_pid.m_kI, theta_pid.m_kD,
                                theta_pid.m_period),
        xy_feed(xy_feed), theta_feed(theta_feed), in(in), out(out),
        trajectory(trajectory), requirements(requirements) {}

  void Initialize() override {
    timer.Reset();
    timer.Start();
  }

  void Execute() override {
    auto target = trajectory.sample(timer.Get());
    auto current = in();
    auto error = abval::calcError(target, current);
    auto x_out =
        x_pid.Calculate(current.x.value(), target.pose.X().value()) +
        xy_feed.calculate(error.x, error.vel_x(), error.accel_x()).value();
    auto y_out =
        y_pid.Calculate(current.y.value(), target.pose.Y().value()) +
        xy_feed.calculate(error.y, error.vel_y(), error.accel_y()).value();
    auto theta_out =
        theta_pid.Calculate(current.theta.value(),
                            target.pose.Rotation().Radians().value()) +
        theta_feed.calculate(error.theta).value();
    out(frc::ChassisSpeeds{.vx = units::meters_per_second_t(x_out),
                           .vy = units::meters_per_second_t(y_out),
                           .omega = units::radians_per_second_t(theta_out)});
  }

  void End(bool interrupted) override {
    timer.Stop();
    if (interrupted) {
      out(frc::ChassisSpeeds{});
    }
  }

  bool IsFinished() override {
    return timer.HasElapsed(trajectory.getTotalTime());
  }
};
} // namespace abval