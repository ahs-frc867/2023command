#include "../include/subsystems/Limelight/Limelight.h"
#include <wpinet/PortForwarder.h>
#include <fmt/format.h>
#include <frc/smartdashboard/SmartDashboard.h>

abval::Limelight* abval::Limelight::instance = nullptr;

//

abval::Limelight::Limelight(){}

abval::Limelight::~Limelight(){}

abval::Limelight* abval::Limelight::GetInstance(){
    if (Limelight::instance == nullptr){
        Limelight::instance = new Limelight();
    }
    return Limelight::instance;
}

//

void abval::Limelight::setupPortForwarding(){
  fmt::print("test");

  for (int i = 5800; i <= 5805; i++){
    wpi::PortForwarder::GetInstance().Add(i, "limelight.local", i);
  }

  fmt::print("test");
}


