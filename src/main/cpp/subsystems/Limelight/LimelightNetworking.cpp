#include "../include/subsystems/Limelight/LimelightNetworking.h"
#include <wpinet/PortForwarder.h>

abval::limelightNetworking* abval::limelightNetworking::instance = nullptr;

//

abval::limelightNetworking::limelightNetworking(){}

abval::limelightNetworking::~limelightNetworking(){}

abval::limelightNetworking* abval::limelightNetworking::GetInstance(){
    if (limelightNetworking::instance == nullptr){
        limelightNetworking::instance = new limelightNetworking();
    }
    return limelightNetworking::instance;
}

//

void abval::limelightNetworking::setupPortForwarding(){
  for (int i = 5800; i <= 5805; i++){
    wpi::PortForwarder::GetInstance().Add(i, "limelight.local", i);
  }
}


