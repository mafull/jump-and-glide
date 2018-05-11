#include "imu.h"

#include <Servo.h>


#define BAUD_RATE             115200
#define SERVO_UPDATE_RATE_HZ  10

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
bool rollMode = false;


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
    } else if (rxString == "roll") {
      rollMode = true;

      Serial.println("ROLL command received");
    } else if (rxString == "both") {
      rollMode = false;

      Serial.println("BOTH command received");
    }
    
    rxString = "";
    rxMsgComplete = false;
  }

  
  // Get IMU data
  float roll, pitch, heading;
  imu.update();
  imu.getRPH(&roll, &pitch, &heading);

  // Calculate servo positions
  int8_t lwAngle, rwAngle;
  if (running) {
    lwAngle = map(roll, -90, 90, ANGLE_LW_MIN, ANGLE_LW_MAX);
    lwAngle = constrain(lwAngle, ANGLE_LW_MIN, ANGLE_LW_MAX);
    rwAngle = map(roll, -90, 90, ANGLE_RW_MIN, ANGLE_RW_MAX);
    rwAngle = constrain(rwAngle, ANGLE_RW_MIN, ANGLE_RW_MAX);

    if (!rollMode) {
      int8_t tmpL, tmpR;
      
      tmpL = map(pitch, -90, 90, ANGLE_LW_MIN, ANGLE_LW_MAX);
      tmpL = constrain(tmpL, ANGLE_LW_MIN, ANGLE_LW_MAX);
      tmpR = map(-pitch, -90, 90, ANGLE_RW_MIN, ANGLE_RW_MAX);
      tmpR = constrain(tmpR, ANGLE_RW_MIN, ANGLE_RW_MAX);

      tmpL -= 90;
      tmpR -= 90;

//      Serial.print(tmpL);
//      Serial.print(" ");
//      Serial.println(tmpR);
      
      lwAngle += tmpL;
      rwAngle += tmpR;
    }
  } else {
    lwAngle = 90;
    rwAngle = 90;
  }

  // Update servos
  static uint32_t prevMillis = 0;
  static const uint32_t MIN_DIFF = 1000 / SERVO_UPDATE_RATE_HZ;
  if ((millis() - prevMillis) > MIN_DIFF) {
    lwServo.write(lwAngle);
    rwServo.write(rwAngle);

    prevMillis = millis();
  }
  

  // Print data
  Serial.print(roll);
  Serial.print(" ");
  Serial.print(pitch);
  Serial.print(" ");
  Serial.print(lwAngle);
  Serial.print(" ");
  Serial.println(rwAngle);
  
  //delay(10);
}


void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    
    if (c == '\n')
      rxMsgComplete = true;
    else 
      rxString += c;   
  } 
}
