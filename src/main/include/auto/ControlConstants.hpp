#pragma once

#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Translation2d.h"
#include "pathplanner/lib/PathPlannerTrajectory.h"
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/base.h>
#include <units/current.h>
#include <units/length.h>
#include <units/math.h>
#include <units/velocity.h>

#include <pathplanner/lib/PathPlanner.h>
#include <pathplanner/lib/PathPlannerTrajectory.h>

namespace abval {
namespace pp = pathplanner;
using namespace units::length;
using namespace units::velocity;
using namespace units::acceleration;
using namespace units::angle;
using namespace units::angular_velocity;
using namespace units::angular_acceleration;
using units::ampere;
using amp_per_accel_t =
    units::unit_t<units::compound_unit<ampere, units::squared<units::second>,
                                       units::inverse<units::meter>>>;
using amp_per_vel_t = units::unit_t<
    units::compound_unit<ampere, units::second, units::inverse<units::meter>>>;
using amp_per_angaccel_t =
    units::unit_t<units::compound_unit<ampere, units::squared<units::second>,
                                       units::inverse<units::radian>>>;
using amp_per_ang_vel_t = units::unit_t<
    units::compound_unit<ampere, units::second, units::inverse<units::radian>>>;

struct RobotState {
  meter_t x, y;
  radian_t theta;
  meters_per_second_t vel;
  meters_per_second_squared_t accel;
  operator frc::Pose2d() { return frc::Pose2d(x, y, theta); }
  constexpr meters_per_second_t vel_x() const noexcept {
    return vel * units::math::cos(theta);
  }
  constexpr meters_per_second_t vel_y() const noexcept {
    return vel * units::math::sin(theta);
  }
  constexpr meters_per_second_squared_t accel_x() const noexcept {
    return accel * units::math::cos(theta);
  }
  constexpr meters_per_second_squared_t accel_y() const noexcept {
    return accel * units::math::sin(theta);
  }
};

inline constexpr RobotState
calcError(pp::PathPlannerTrajectory::PathPlannerState target,
          RobotState current) {
  return RobotState{
      .x = target.pose.X() - current.x,
      .y = target.pose.Y() - current.y,
      .theta = target.pose.Rotation().Radians() - current.theta,
      .vel = target.velocity - current.vel,
      .accel = target.acceleration - current.accel,
  };
}

struct TranslationFeedforward {
  amp_per_accel_t kA;
  amp_per_vel_t kV;
  units::ampere_t kS;
  units::ampere_t calculate(meter_t s, meters_per_second_t v,
                            meters_per_second_squared_t a) {
    return units::math::copysign(kS, s) + v * kV + a * kA;
  }
};
struct TurnFeedForward {
  // amp_per_angaccel_t kA;
  // amp_per_ang_vel_t kW;
  // gyroscope is unable to calculate angular velocity and acceleration
  units::ampere_t kT;
  units::ampere_t calculate(radian_t s) { return units::math::copysign(kT, s); }
};
} // namespace abval