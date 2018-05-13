#include <math.h>
#include "FeatherIMU.hpp"
#include "FeatherServos.hpp"
#include "PID.hpp"
#include "Utils.hpp"


#define CONTROL_LAUNCH_ANGLE_DEG  70.0f
#define GLIDE_THRESHOLD_MPS       1.0f
#define LANDED_THRESHOLD_MPSS     2*GRAVITY

#define LOOP_FREQUENCY_HZ         100

#define PIN_BUTTON                9
#define PIN_LED_STATUS            13
#define PIN_MOTOR                 10

#define SERVO_NUM_CLUTCH		      2     // Pin 11
#define SERVO_NUM_LEFT_WING		    0     // Pin 5
#define SERVO_NUM_RIGHT_WING	    1     // Pin 6

#define PID_HEADING_KP            0.1
#define PID_HEADING_KI            0.001
#define PID_HEADING_KD            0.01
#define PID_PITCH_KP              0.1
#define PID_PITCH_KI              0.001
#define PID_PITCH_KD              0.01

typedef enum State_t {IDLE = 0,
                      WINDING,
                      PREJUMP,
                      JUMPING,
                      GLIDING,
                      NUM_STATES} State;


State state = IDLE;
float desiredHeading = 0.0f;
float vertical_vel = 0;
PID *headingPID, *pitchPID;

void advanceState()
{
  state = (State)(((int)state + 1)%NUM_STATES);
}


void setup()
{
  // Initialise serial
  Serial.begin(115200);
  Serial.println("Serial initialised");

  // Initialise button
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(9), advanceState, RISING);

  // Initialise motor
  pinMode(PIN_MOTOR, OUTPUT);

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

  // Initialise PID controllers
  headingPID = new PID(PID_HEADING_KP, PID_HEADING_KI, PID_HEADING_KD);
  pitchPID = new PID(PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD);

	// Initialise timers for servos
	Servos.init();

	// Set initial servo positions
	Servos.setAngle(SERVO_NUM_CLUTCH, 0.0f);
	Servos.setAngle(SERVO_NUM_LEFT_WING, 0.0f);
	Servos.setAngle(SERVO_NUM_RIGHT_WING, 0.0f);

	// Enable all servos
	Servos.enable();

}


void controlLoop()
{
  // Default states
  bool motorOn = false;         // Winding motor on/off
  float angleC = 0.0f;          // Clutch servo angle
  float angleLW = 0.0f;         // Left wing servo angle
  float angleRW = 0.0f;         // Right wing servo angle


  // ---- INPUTS ----
  // Get IMU data
  float roll, pitch, heading;
  imu.update();
  imu.getRPH(&roll, &pitch, &heading);


  // ---- STATE MACHINE ----
  switch(state)
  {
    case IDLE:
    {
      delay(1000);
      state = WINDING;

      break;
    }
    case WINDING:
    {
      // Turn winding motor on
      motorOn = true;

      // Once pitched enough, jump
      if(pitch >= CONTROL_LAUNCH_ANGLE_DEG*M_PI/180)
      {
        motorOn = false;
        state = PREJUMP;
      }

      break;
    }
    case PREJUMP:
    {
      // While stationary, measure the direction of gravity
      vertical_vel = 0;

      // Disengage leg clutch to jump
      angleC = -45.0f;
      state = JUMPING;

      break;
    }
    case JUMPING:
    {
      // Integrate IMU.vertical_acc to get vertical velocity
      vertical_vel += imu.getVertAcc()/LOOP_FREQUENCY_HZ;

      // Once at peak of jump, disengage wing clutch  to glide
      if((imu.getVertAcc() < 0) && (vertical_vel <= GLIDE_THRESHOLD_MPS))
      {
        // Return clutch servo to the centre position
        angleC = 45.0f;

        // Reset PID controllers
        headingPID->reset();
        pitchPID->reset();

        state = GLIDING;
      }

      break;
    }
    case GLIDING:
    {
      // Once landed, reset
      if(imu.getVertAcc() > LANDED_THRESHOLD_MPSS)
      {
        angleC = 0.0f;
        state = IDLE;
      } else {
        // Update PID controllers and combine their outputs
        // Servos are flipped, hence - sign
        angleLW = 90 + pitchPID->update(-15.0f, pitch) + headingPID->update(0.0f, heading);
        angleRW = -(90 + pitchPID->update(-15.0f, pitch) - headingPID->update(0.0f, heading));
      }
      break;
    }
    default:
    {
      // WHY ARE YOU HERE
      break;
    }
  }


  // ---- OUTPUTS ----
  // Motor
  digitalWrite(PIN_MOTOR, motorOn ? HIGH : LOW);

  // Servos
	Servos.setAngle(SERVO_NUM_CLUTCH, angleC);
  Servos.setAngle(SERVO_NUM_LEFT_WING, angleLW);
  Servos.setAngle(SERVO_NUM_RIGHT_WING, angleRW);


  // ---- LOGGING ----
  char str[128] = "";
  char pitchStr[5] = "";
  char rollStr[5] = "";
  char headingStr[5] = "";
  dtostrf(pitch, 4, 2, pitchStr);
  dtostrf(roll, 4, 2, rollStr);
  dtostrf(heading, 4, 2, headingStr);
  sprintf(str, "%01d|-> rph: %s %s %s | servos: %4d %4d %4d | motor: %1d",
    state,
    rollStr, pitchStr, headingStr,
    (int)angleC, (int)angleLW, (int)angleRW,
    motorOn);
  Serial.println(str);
}


void loop()
{
	controlLoop();
  delay(1000/LOOP_FREQUENCY_HZ);
}
