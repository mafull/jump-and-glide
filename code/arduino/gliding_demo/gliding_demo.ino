#include "imu.h"

#include <Servo.h>


#define BAUD_RATE     115200
#define PIN_SERVO_LW  9
#define PIN_SERVO_RW  10

String rxString;

Servo lwServo, rwServo;

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
  // Get IMU data
  float roll, pitch, heading;
  imu.update();
  imu.getRPH(&roll, &pitch, &heading);

  // Set servo positions
  


  Serial.print(roll);
  Serial.print(" ");
  Serial.println(pitch);

  

  //delay(10);
}
