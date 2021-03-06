#ifndef __FEATHER_SERVOS_HPP
#define __FEATHER_SERVOS_HPP

#include <stdint.h>


#define  SERVO_ANGLE_LIMIT_DEG     90.0f


class FeatherServos {
	public:
		void init();

		void enable(int8_t servoNum = -1);
		void disable(int8_t servoNum = -1);

		void setAngle(uint8_t servoNum, float angleDeg);


	private:
    void initGCLK4();
		void initTCC0();
		void initTCC2();
};


// Preinstantiate
extern FeatherServos Servos;


#endif	// __FEATHER__SERVOS_HPP

