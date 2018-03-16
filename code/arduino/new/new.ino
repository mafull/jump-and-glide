#include "FeatherServos.hpp"
#include "Interrupts.hpp"


#define SERVO_NUM_CLUTCH		2
#define SERVO_NUM_LEFT_WING		0
#define SERVO_NUM_RIGHT_WING	1


void setup() {
	// Initialise timers for servos
	Servos.init();

	// Set initial servo positions
	Servos.setAngle(SERVO_NUM_CLUTCH, 180.0f);
	Servos.setAngle(SERVO_NUM_LEFT_WING, 180.0f);
	Servos.setAngle(SERVO_NUM_RIGHT_WING, 180.0f);

	// Enable all servos
	Servos.enable();
}


void controlLoop() {
	Servos.setAngle();


}


void loop() {
	controlLoop();
}