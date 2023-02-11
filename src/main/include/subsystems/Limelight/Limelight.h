#pragma once

#include <vector>

#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableInstance.h"

namespace abval {
class Limelight {
 private:

  Limelight();
  ~Limelight();

  std::shared_ptr<nt::NetworkTable> getTable();

 public:
  static Limelight* GetInstance();

  void setupPortForwarding();

  // Basic Targeting Data

  bool getValidTargetDetected();       // tv
  double getTargetHorizontalOffset();  // tx
  double getTargetVerticalOffset();    // ty
  double getTargetArea();              // ta

  double getPipelineLatency();  // tl

  double getBoundingBoxShort();  // tshort
  double getBoundingBoxLong();   // tlong

  double getRoughBoundingHorizontal();  // thor
  double getRoughBoundingVertical();    // tvert

  double getActivePipelineIndex();  // getpipe

  double getTargetClassID();  // tclass

  // April Tag and 3D data

  std::vector<double> getBotPose();
  std::vector<double> getBotPose_wpiblue();
  std::vector<double> getBotPose_wpired();

  std::vector<double> getCameraPose_targetspace();
  std::vector<double> getTargetPose_cameraspace();

  std::vector<double> getTargetPose_robotspace();
  std::vector<double> getBotPose_targetspace();

  double getPrimaryAprilTagID();

  void setLEDMode(int mode);
  void setCameraMode(int mode);
  void setPipeline(int index);
  void setStreamMode(int mode);
};
}  // namespace abval