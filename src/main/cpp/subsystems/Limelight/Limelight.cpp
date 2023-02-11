#include "../include/subsystems/Limelight/Limelight.h"

#include <fmt/format.h>
#include <wpinet/PortForwarder.h>
namespace abval {
// private functions

Limelight::Limelight() {}

Limelight::~Limelight() {}

Limelight* Limelight::GetInstance() {
  static Limelight limelight = {};
  return &limelight;
}

std::shared_ptr<nt::NetworkTable> Limelight::getTable() {
  return nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

// public functions

void Limelight::setupPortForwarding() {
  for (int i = 5800; i <= 5805; i++) {
    wpi::PortForwarder::GetInstance().Add(i, "limelight.local", i);
  }
}

// Basic Targeting Data

bool Limelight::getValidTargetDetected() {  // tv
  return getTable()->GetBoolean("tv", false);
}

double Limelight::getTargetHorizontalOffset() {  // tx
  return getTable()->GetNumber("tx", 0.0);
}

double Limelight::getTargetVerticalOffset() {  // ty
  return getTable()->GetNumber("ty", 0.0);
}

double Limelight::getTargetArea() {  // ta
  return getTable()->GetNumber("ta", 0.0);
}

double Limelight::getPipelineLatency() {  // tl
  return getTable()->GetNumber("tl", 0.0);
}

double Limelight::getBoundingBoxShort() {  // tshort
  return getTable()->GetNumber("tshort", 0.0);
}

double Limelight::getBoundingBoxLong() {  // tlong
  return getTable()->GetNumber("tlong", 0.0);
}

double Limelight::getRoughBoundingHorizontal() {  // thor
  return getTable()->GetNumber("thor", 0.0);
}

double Limelight::getRoughBoundingVertical() {  // tvert
  return getTable()->GetNumber("tvert", 0.0);
}

double Limelight::getActivePipelineIndex() {  // getpipe
  return getTable()->GetNumber("getpipe", 0.0);
}

double Limelight::getTargetClassID() {  // tclass
  return getTable()->GetNumber("tclass", 0.0);
}

// April Tag and 3D data

std::vector<double> Limelight::getBotPose() {
  return getTable()->GetNumberArray("botpose", std::vector<double>(6));
}

std::vector<double> Limelight::getBotPose_wpiblue() {
  return getTable()->GetNumberArray("botpose_wpiblue", std::vector<double>(6));
}

std::vector<double> Limelight::getBotPose_wpired() {
  return getTable()->GetNumberArray("botpose_wpired", std::vector<double>(6));
}

std::vector<double> Limelight::getCameraPose_targetspace() {
  return getTable()->GetNumberArray("camerapose_targetspace",
                                    std::vector<double>(6));
}

std::vector<double> Limelight::getTargetPose_cameraspace() {
  return getTable()->GetNumberArray("targetpose_cameraspace",
                                    std::vector<double>(6));
}

std::vector<double> Limelight::getTargetPose_robotspace() {
  return getTable()->GetNumberArray("targetpose_robotspace",
                                    std::vector<double>(6));
}

std::vector<double> Limelight::getBotPose_targetspace() {
  return getTable()->GetNumberArray("botpose_targetspace",
                                    std::vector<double>(6));
}

double Limelight::getPrimaryAprilTagID() {
  return getTable()->GetNumber("tid", 0.0);
}

// Camera controls, reference network tables api for options

void Limelight::setLEDMode(int mode) { getTable()->PutNumber("ledMode", mode); }

void Limelight::setCameraMode(int mode) {
  getTable()->PutNumber("camMode", mode);
}

void Limelight::setPipeline(int index) {
  getTable()->PutNumber("pipeline", index);
}

void Limelight::setStreamMode(int mode) {
  getTable()->PutNumber("stream", mode);
}
}  // namespace abval