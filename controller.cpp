#include "controller.h"

Controller::Controller(float kp, float ki, float kd, float ts)
    : _kp(kp), _ki(ki), _kd(kd), _dt(ts), _integral(0.0f), _prevError(0.0f),
      _integralMin(-1000.0f), _integralMax(1000.0f) {}

void Controller::setTunings(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void Controller::reset() {
    _integral = 0.0f;
    _prevError = 0.0f;
}

void Controller::setIntegralLimits(float min, float max) {
    _integralMin = min;
    _integralMax = max;
}

float Controller::compute(float setpoint, float measurement) {
    float error = setpoint - measurement;

    _integral += error * _dt;

    // Clamping für Anti-Windup
    if (_integral > _integralMax) _integral = _integralMax;
    if (_integral < _integralMin) _integral = _integralMin;

    float derivative = (error - _prevError) / _dt;

    _prevError = error;

    return _kp * error + _ki * _integral + _kd * derivative;
}
