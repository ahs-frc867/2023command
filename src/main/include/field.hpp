#pragma once
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Rotation3d.h>
#include <units/dimensionless.h>
#include <units/length.h>

namespace abval {
// Varaibles beginning with B are blue, R are Red
namespace Field {
frc::Rotation3d nullRot = {};
std::array<frc::Pose3d, 4> BstagedPieces = {
    frc::Pose3d{-47.36_in, 22.39_in, 0_in, nullRot},
    frc::Pose3d{-47.36_in, -25.61_in, 0_in, nullRot},
    frc::Pose3d{-47.36_in, -73.61_in, 0_in, nullRot},
    frc::Pose3d{-47.36_in, -121.61_in, 0_in, nullRot}};
frc::Pose3d chargeStationFarLeft = {-47.36_in, 22.39_in, 0_in, nullRot};
std::array<frc::Pose3d, 4> RstagedPieces;
}  // namespace Field
}  // namespace abval