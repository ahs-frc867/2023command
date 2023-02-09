#include "../include/subsystems/Limelight/Limelight.h"
#include <wpinet/PortForwarder.h>
#include <fmt/format.h>
#include <frc/smartdashboard/SmartDashboard.h>

abval::Limelight* abval::Limelight::instance = nullptr;

// private funcs

abval::Limelight::Limelight(){}

abval::Limelight::~Limelight(){}

abval::Limelight* abval::Limelight::GetInstance(){
    if (Limelight::instance == nullptr){
        Limelight::instance = new Limelight();
    }
    return Limelight::instance;
}

//

std::shared_ptr<nt::NetworkTable> abval::Limelight::getTable(){
  return nt::NetworkTableInstance::GetDefault().GetTable("limelight");
}

// public funcs

void abval::Limelight::setupPortForwarding(){
  fmt::print("test");

  for (int i = 5800; i <= 5805; i++){
    wpi::PortForwarder::GetInstance().Add(i, "limelight.local", i);
  }

  fmt::print("test");
}

// Basic Targeting Data

bool abval::Limelight::getValidTargetDetected(){ // tv
  return getTable()->GetBoolean("tv", false);
}

double abval::Limelight::getTargetHorizontalOffset(){ // tx
  return getTable()->GetNumber("tx", 0.0);
}

double abval::Limelight::getTargetVerticalOffset(){ // ty
  return getTable()->GetNumber("ty", 0.0);
}

double abval::Limelight::getTargetArea(){ // ta
  return getTable()->GetNumber("ta", 0.0);
}

double abval::Limelight::getPipelineLatency(){ // tl
  return getTable()->GetNumber("tl", 0.0);
}

double abval::Limelight::getBoundingBoxShort(){ // tshort 
  return getTable()->GetNumber("tshort", 0.0);
}

double abval::Limelight::getBoundingBoxLong(){ // tlong
  return getTable()->GetNumber("tlong", 0.0);
}

double abval::Limelight::getRoughBoundingHorizontal(){ // thor
  return getTable()->GetNumber("thor", 0.0);
}

double abval::Limelight::getRoughBoundingVertical(){ // tvert
  return getTable()->GetNumber("tvert", 0.0);
}

double abval::Limelight::getActivePipelineIndex(){ // getpipe
  return getTable()->GetNumber("getpipe", 0.0);
}

double abval::Limelight::getTargetClassID(){ // tclass
  return getTable()->GetNumber("tclass", 0.0);
}

// April Tag and 3D data

std::vector<double> abval::Limelight::getBotPose(){
  return getTable()->GetNumberArray("botpose", std::vector<double>(6));
}

std::vector<double> abval::Limelight::getBotPose_wpiblue(){
  return getTable()->GetNumberArray("botpose_wpiblue", std::vector<double>(6));
}

std::vector<double> abval::Limelight::getBotPose_wpired(){
  return getTable()->GetNumberArray("botpose_wpired", std::vector<double>(6));
}

std::vector<double> abval::Limelight::getCameraPose_targetspace(){
  return getTable()->GetNumberArray("camerapose_targetspace", std::vector<double>(6));
}

std::vector<double> abval::Limelight::getTargetPose_cameraspace(){
  return getTable()->GetNumberArray("targetpose_cameraspace", std::vector<double>(6));
}

std::vector<double> abval::Limelight::getTargetPose_robotspace(){
  return getTable()->GetNumberArray("targetpose_robotspace", std::vector<double>(6));
}

std::vector<double> abval::Limelight::getBotPose_targetspace(){
  return getTable()->GetNumberArray("botpose_targetspace", std::vector<double>(6));
}

double abval::Limelight::getPrimaryAprilTagID(){
  return getTable()->GetNumber("tid", 0.0);
}

// Camera controls, reference network tables api for options

void abval::Limelight::setLEDMode(int mode){ 
  getTable()->PutNumber("ledMode", mode);
}

void abval::Limelight::setCameraMode(int mode){
  getTable()->PutNumber("camMode", mode);
}

void abval::Limelight::setPipeline(int index){
  getTable()->PutNumber("pipeline", index);
}

void abval::Limelight::setStreamMode(int mode){
  getTable()->PutNumber("stream", mode);
}