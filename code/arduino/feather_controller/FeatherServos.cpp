#include "FeatherServos.hpp"

#include <Arduino.h>


// Preinstantiate
FeatherServos Servos;


const uint8_t _servoCount = 3;
const uint8_t _servoPins[_servoCount] = {5, 6, 11};


// ---- PUBLIC FUNCTIONS ----
// Initialise the timers
void FeatherServos::init() {
  initGCLK4();
	initTCC0();
	initTCC2();
}


// Enable PWM output
//    servoNum    Number of the servo to enable (-1 enables all)
void FeatherServos::enable(int8_t servoNum)
{
  if(servoNum > _servoCount) return;
  else if(servoNum >= 0) {
    uint8_t mux;
    
    switch(servoNum) {
      case 0:
        mux = PORT_PMUX_PMUXO_F;
        break;
      case 1:
        mux = PORT_PMUX_PMUXE_F;
        break;
      case 2:
        mux = PORT_PMUX_PMUXE_E;
        break;
      default:
        return;
        break;      
    }
    
    uint8_t pin = _servoPins[servoNum];
    PORT->Group[g_APinDescription[pin].ulPort].PMUX[g_APinDescription[pin].ulPin >> 1].reg = mux;
  } else {
    // Enable all servos
    for(int n = 0; n < _servoCount; n++) enable(n);
  }
}


// Disable PWM output
//    servoNum    Number of the servo to disable (-1 disables all)
void FeatherServos::disable(int8_t servoNum)
{
  if(servoNum > _servoCount) return;
  else if(servoNum >= 0) {
    uint8_t mux;
    
    switch(servoNum) {
      case 0:
        mux = PORT_PMUX_PMUXO_F;
        break;
      case 1:
        mux = PORT_PMUX_PMUXE_F;
        break;
      case 2:
        mux = PORT_PMUX_PMUXE_E;
        break;
      default:
        return;
        break;      
    }
    
    uint8_t pin = _servoPins[servoNum];
    PORT->Group[g_APinDescription[pin].ulPort].PMUX[g_APinDescription[pin].ulPin >> 1].reg = ~mux;
  } else {
    // Disable all servos
    for(int n = 0; n < _servoCount; n++) disable(n);
  }
}


// Set the servo angle
//    servoNum    Number of the servo to update
//    angleDeg    Angle in degrees (limited to +-ANGLE_LIMIT_DEG)
void FeatherServos::setAngle(uint8_t servoNum, float angleDeg)
{
  if(servoNum > _servoCount) return;

  // Constrain angle between limits
  angleDeg = angleDeg < -SERVO_ANGLE_LIMIT_DEG ? -SERVO_ANGLE_LIMIT_DEG : (angleDeg > SERVO_ANGLE_LIMIT_DEG ? SERVO_ANGLE_LIMIT_DEG : angleDeg);

  // Map angle to pulse width
  uint16_t pulseWidth = 500 + ((angleDeg + SERVO_ANGLE_LIMIT_DEG) / (SERVO_ANGLE_LIMIT_DEG * 2)) * 2000;

  // Update PWM duty cycle
  switch(servoNum) {
    case 0:
      // TCCO - CC1
      REG_TCC0_CC1 = pulseWidth;
      while (TCC0->SYNCBUSY.bit.CC1);

      break;
    case 1:
      // TCC0 - CC2
      REG_TCC0_CC2 = pulseWidth;
      while (TCC0->SYNCBUSY.bit.CC2);
      
      break;
    case 2:
      // TCC2 - CC0
      REG_TCC2_CC0 = pulseWidth;
      while (TCC2->SYNCBUSY.bit.CC0);
      break;
    default:
      // Shouldn't get here

      break;
  }
}


// ---- PRIVATE FUNCTIONS ----
void FeatherServos::initGCLK4() {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |          // Divide the 48MHz clock source by 3: 48MHz/3=16MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization
  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization 
}


void FeatherServos::initTCC0() {
  // Enable the port multiplexer for digital pins D5, D6
  PORT->Group[g_APinDescription[5].ulPort].PINCFG[g_APinDescription[5].ulPin].bit.PMUXEN = 1;
  PORT->Group[g_APinDescription[6].ulPort].PINCFG[g_APinDescription[6].ulPin].bit.PMUXEN = 1;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |            // Reverse the output polarity on all TCC0 outputs
                   TCC_WAVE_WAVEGEN_DSBOTH;       // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 
  REG_TCC0_PER = 20000;                           // Set the frequency of the PWM on TCC0 to 50Hz
  while (TCC0->SYNCBUSY.bit.PER);                 // Wait for synchronization
  
  // Set the PWM signal to output 50% duty cycle
  REG_TCC0_CC1 = 1500;                            // TCC0 CC1 - on D5
  while (TCC0->SYNCBUSY.bit.CC1);                 // Wait for synchronization
  REG_TCC0_CC2 = 1500;                            // TCC0 CC2 - on D6
  while (TCC0->SYNCBUSY.bit.CC2);                 // Wait for synchronization
  
  // Divide the 16MHz signal by 8 giving 2MHz TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}


void FeatherServos::initTCC2() {
  // Enable the port multiplexer for digital pin D11
  PORT->Group[g_APinDescription[11].ulPort].PINCFG[g_APinDescription[11].ulPin].bit.PMUXEN = 1;

  // Feed GCLK4 to TCC2 and TC3
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC2 and TC3
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC2_TC3;    // Feed GCLK5 to TCC2 and TC3
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC2_WAVE |= TCC_WAVE_POL(0xF) |            // Reverse the output polarity on all TCC2 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH;      // Setup dual slope PWM on TCC2
  while (TCC2->SYNCBUSY.bit.WAVE);                // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation:
  REG_TCC2_PER = 20000;                           // Set the frequency of the PWM on TCC2 to 50Hz
  while (TCC2->SYNCBUSY.bit.PER);                 // Wait for synchronization
  
  // Set the PWM signal to output 50% duty cycle
  REG_TCC2_CC0 = 1500;                            // TCC2 CC0 - on D11
  while (TCC2->SYNCBUSY.bit.CC0);                 // Wait for synchronization
  
  // Divide the 16MHz signal by 8 giving 2MHz TCC2 timer tick and enable the outputs
  REG_TCC2_CTRLA |= TCC_CTRLA_PRESCALER_DIV8 |    // Divide GCLK4 by 8
                    TCC_CTRLA_ENABLE;             // Enable the TCC2 output
  while (TCC2->SYNCBUSY.bit.ENABLE);              // Wait for synchronization  
}

