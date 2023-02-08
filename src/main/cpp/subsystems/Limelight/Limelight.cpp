#include "../include/subsystems/Limelight/Limelight.h"
#include <wpinet/PortForwarder.h>

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
  for (int i = 5800; i <= 5805; i++){
    wpi::PortForwarder::GetInstance().Add(i, "limelight.local", i);
  }
}


