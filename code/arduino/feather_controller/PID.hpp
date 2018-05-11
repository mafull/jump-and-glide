#ifndef __PID_HPP
#define __PID_HPP

class PID {
  public:
    PID(const float& kp, const float& ki, const float& kd, const float& iLimit = -1.0f);

    float update(const float& desired, const float& actual, const float& dt);

  private:
    // Settings
    float _kp, _ki, _kd;
    float _iLimit;
    bool _limitITerm;

    // Persistent terms
    float _ePrev;
    float _ie;
};

#endif  // __PID_HPP

