// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/RunCommand.h>

#include "subsystems/SwervePod.hpp"

namespace autop {
frc2::CommandPtr Auto(abval::SwervePod* subsystem) {
  return frc2::RunCommand([]() {}, {subsystem}).AndThen([]() {}, {subsystem});
}
}  // namespace autop
