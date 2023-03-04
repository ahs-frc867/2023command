#pragma once

#include "auto/ControlConstants.hpp"
#include "frc/controller/PIDController.h"
#include "frc/kinematics/ChassisSpeeds.h"
#include "units/angular_acceleration.h"
#include "units/base.h"
#include "units/current.h"
#include "units/velocity.h"
#include <pathplanner/lib/auto/BaseAutoBuilder.h>

namespace abval {
class SwerveAutoController {
  SwerveAutoController(pp::PIDConstants translation, pp::PIDConstants yaw,
                       TranslationFeedforward xy, TurnFeedForward theta)
      : xy(translation), yaw(yaw), xyfeed(xy), yawfeed(theta) {}
  frc2::CommandPtr followPath(pp::PathPlannerTrajectory trajectory);

private:
  pp::PIDConstants xy, yaw;
  TranslationFeedforward xyfeed;
  TurnFeedForward yawfeed;

  std::function<void(frc::ChassisSpeeds)> outputSpeeds;
  std::initializer_list<frc2::Subsystem *> driveRequirements;
};
} // namespace abval