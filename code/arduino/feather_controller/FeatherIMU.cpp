#include "FeatherIMU.hpp"


FeatherIMU IMU;


void FeatherIMU::init() {

  

  // Initialise angles
  roll = 0.0f;
  pitch = 0.0f;
  heading = 0.0f;
}


void FeatherIMU::calibrate() {
  
}


void FeatherIMU::update() {
  // READ SENSORS

  // CALCULATE ANGLES
}

