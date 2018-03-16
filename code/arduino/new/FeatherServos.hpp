#ifndef __FEATHER_SERVOS_HPP
#define __FEATHER_SERVOS_HPP


class FeatherServos {
	public:
		void init();

		void enable(int8_t servoNum = -1);
		void disable(int8_t servoNum = -1);

		void setAngle(uint8_t servoNum, float angleDeg);


	private:
		void initTCC0();
		void initTCC2();

		const uint8_t _servoCount = 3;
		const uint8_t _servoPins[_servoCount] = {5, 6, 11};
};


// Preinstantiate
extern FeatherServos Servos;


#endif	// __FEATHER__SERVOS_HPP