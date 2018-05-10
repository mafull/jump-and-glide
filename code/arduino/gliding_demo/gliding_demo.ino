#include "imu.h"

#include <Servo.h>


#define BAUD_RATE     115200

#define PIN_SERVO_LW  9
#define PIN_SERVO_RW  10

#define ANGLE_LW_MIN  80
#define ANGLE_LW_MAX  100
#define ANGLE_RW_MIN  80
#define ANGLE_RW_MAX  100


String rxString;
bool rxMsgComplete = false;

Servo lwServo, rwServo;

bool running = false;


void setup() {
  // Initialise serial comms
  Serial.begin(BAUD_RATE);
  rxString.reserve(256);
  Serial.println("\rSerial initialised");


  // Initialise IMU
  if (imu.init()) {
    Serial.println("IMU initialised");
    Serial.print("Calibrating gyros... ");
    imu.calibrateGyros();
    imu.setInitialOrientation();
    Serial.println("Calibrated");
  } else {
    Serial.println("Failed to initialise IMU!");
    while (1) {}
  }

  // Initialise Servos
  lwServo.attach(PIN_SERVO_LW);
  rwServo.attach(PIN_SERVO_RW);
  lwServo.write(90);
  rwServo.write(90);
}


void loop() {
  if (rxMsgComplete) {
    if (rxString == "start") {
      running = true;

      Serial.println("START command received");
    } else if (rxString == "stop") {
      running = false;
      
      Serial.println("STOP command received");
    } else if (rxString == "reset") {
      running = false;
      
      Serial.println("RESET command received");
      Serial.print("Resetting IMU... ");
      imu.calibrateGyros();
      imu.setInitialOrientation();
      Serial.println("Done");
    }
    rxString = "";
    rxMsgComplete = false;
  }

  
  // Get IMU data
  float roll, pitch, heading;
  imu.update();
  imu.getRPH(&roll, &pitch, &heading);

  if (running) {
    // Calculate servo positions
    float lwAngle, rwAngle;
    lwAngle = map(roll, -90, 90, ANGLE_LW_MIN, ANGLE_LW_MAX);
    lwAngle = constrain(lwAngle, ANGLE_LW_MIN, ANGLE_LW_MAX);
    rwAngle = map(roll, -90, 90, ANGLE_RW_MIN, ANGLE_RW_MAX);
    rwAngle = constrain(rwAngle, ANGLE_RW_MIN, ANGLE_RW_MAX);
  
    // Update servos
    lwServo.write(lwAngle);
    rwServo.write(rwAngle);
  
    // Print data
    Serial.print(roll);
    Serial.print(" ");
    Serial.print(pitch);
    Serial.print(" ");
    Serial.print(lwAngle);
    Serial.print(" ");
    Serial.println(rwAngle);
  } else {
    lwServo.write
  }
  
  //delay(10);
}


void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    
    rxString += c;
    
    if (c == '\n') {
      rxMsgComplete = true;
    }    
  } 
}
