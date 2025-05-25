// Motor_Control.h
#ifndef SINGLE_MOTOR_H
#define SINGLE_MOTOR_H

#include <Arduino.h>

class Motor_Control {
public:
  Motor_Control(int pwm1, int pwm2, int encA, int encB, bool reversed = false);

  void init();
  void update();
  void setTargetSpeed(float rpm);
  float getCurrentRPM() const;
  void handleEncoderTick();

private:
  int _pwm1, _pwm2, _encA, _encB;
  volatile long _encoderTicks = 0;
  float _targetRPM = 0;
  float _currentRPM = 0;
  bool _reversed;

  float _kp = 1.0, _ki = 0.0, _kd = 0.0;
  float _errorSum = 0, _lastError = 0;
  unsigned long _lastUpdateTime = 0;
  long _lastTicks = 0;

  void applyPWM(float pwmValue);
};

#endif
