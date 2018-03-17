#include "PID.hpp"


PID::PID(const float& kp, const float& ki, const float& kd, const float& iLimit) :
  _kp(kp), _ki(ki), _kd(kd),
  _iLimit(iLimit),
  _limitITerm(iLimit > 0.0f ? true : false)
{
  
}


float PID::update(const float& desired, const float& actual, const float& dt) {
  // Update new state
  float e = desired - actual;
  _ie += (e * dt);
  float de = (e - _ePrev) / dt;

  // Store error for next iteration
  _ePrev = e;

  // Limit I-term if necessary
  float iTerm = _ki * _ie;
  if(_limitITerm) iTerm = iTerm < -_iLimit ? -_iLimit : (iTerm > _iLimit ? _iLimit : iTerm);

  // Generate output
  return (_kp * e) + iTerm + (_kd * de);
}

