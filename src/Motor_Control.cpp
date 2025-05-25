// Motor_Control.cpp
#include "Motor_Control.h"

Motor_Control::Motor_Control(int pwm1, int pwm2, int encA, int encB, bool reversed)
  : _pwm1(pwm1), _pwm2(pwm2), _encA(encA), _encB(encB), _reversed(reversed) {}

void Motor_Control::init() {
  pinMode(_pwm1, OUTPUT);
  pinMode(_pwm2, OUTPUT);
  pinMode(_encA, INPUT_PULLUP);
  pinMode(_encB, INPUT_PULLUP);

  _lastUpdateTime = millis();
}

void Motor_Control::handleEncoderTick() {
  _encoderTicks++;
}

void Motor_Control::setTargetSpeed(float rpm) {
  _targetRPM = rpm;
}

float Motor_Control::getCurrentRPM() const {
  return _currentRPM;
}

void Motor_Control::update() {
  unsigned long now = millis();
  unsigned long dt = now - _lastUpdateTime;
  if (dt < 20) return;

  long currentTicks = _encoderTicks;
  long deltaTicks = currentTicks - _lastTicks;
  _lastTicks = currentTicks;

  // Encoder resolution: assume 20 ticks/rev for example
  const float ticksPerRev = 20.0;
  _currentRPM = (deltaTicks / ticksPerRev) * (60000.0 / dt);

  float error = _targetRPM - _currentRPM;
  _errorSum += error * dt;
  float dError = (error - _lastError) / dt;
  _lastError = error;

  float output = _kp * error + _ki * _errorSum + _kd * dError;
  applyPWM(output);

  _lastUpdateTime = now;
}

void Motor_Control::applyPWM(float pwmValue) {
  pwmValue = constrain(pwmValue, -255, 255);
  if (_reversed) pwmValue = -pwmValue;

  if (pwmValue > 0) {
    analogWrite(_pwm1, pwmValue);
    analogWrite(_pwm2, 0);
  } else {
    analogWrite(_pwm1, 0);
    analogWrite(_pwm2, -pwmValue);
  }
}
