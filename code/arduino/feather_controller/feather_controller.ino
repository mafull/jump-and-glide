#include "FeatherIMU.hpp"
#include "FeatherServos.hpp"
#include "PID.hpp"
#include "Utils.hpp"


#define CONTROL_LAUNCH_ANGLE_DEG  70.0f

#define LOOP_FREQUENCY_HZ         100

#define PIN_BUTTON                9
#define PIN_LED_STATUS            13
#define PIN_MOTOR                 10

#define SERVO_NUM_CLUTCH		      2     // Pin 11
#define SERVO_NUM_LEFT_WING		    0     // Pin 5
#define SERVO_NUM_RIGHT_WING	    1     // Pin 6

typedef enum State_t {IDLE = 0,
                      WINDING,
                      PREJUMP,
                      JUMPING,
                      GLIDING,
                      NUM_STATES} State;


State state = IDLE;
float desiredHeading = 0.0f;

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
  IMU.init();
  
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
  IMU.updateData();
  
  int ain = analogRead(0);
  ain = constrain(ain, 0, 1024);


  // ---- STATE MACHINE ----
  switch(state) 
  {
    case IDLE:

      break;

    case WINDING:
      // Turn winding motor on
      motorOn = true;

      // Once pitched enough, jump
      if(IMU.pitch >= CONTROL_LAUNCH_ANGLE_DEG) 
      {
        motorOn = false;
        state = PREJUMP;
      }
      
      break;

    case PREJUMP:
      // While stationary, measure the direction of gravity
      IMU.calibrateGravity();
      state = JUMPING;
      
      break;

    case JUMPING:
      // Disengage leg clutch
      angleC = -45.0f;

      // Integrate IMU.vertical_acc to get vertical velocity

      // Once at peak of jump, deploy wings
      if(0 /* vertical velocity <= threshold */) state = GLIDING;
      
      break;

    case GLIDING:
      // Disengage wing clutch
      angleC = 45.0f;

      // CALCULATE DESIRED PITCH/ROLL

      // Use desired pitch/roll to generate servo outputs
      {
      float a = map(ain, 0, 1024, -90, 90);
      angleLW = a;
      angleRW = -a;
      }

      // Once landed, reset
      if(0 /* DO CLEVER STUFF HERE */) state = IDLE;
      
      break;

    default:
      // WHY ARE YOU HERE
      break;
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
  dtostrf(IMU.pitch, 4, 2, pitchStr);
  dtostrf(IMU.roll, 4, 2, rollStr);
  dtostrf(IMU.heading, 4, 2, headingStr);
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
