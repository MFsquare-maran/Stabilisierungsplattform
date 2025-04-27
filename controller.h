#ifndef CONTROLLER_H
#define CONTROLLER_H

class Controller {
public:
    Controller(float kp, float ki, float kd, float ts);

    void setTunings(float kp, float ki, float kd);
    void reset();

    float compute(float setpoint, float measurement);

    // Neu für Anti-Windup
    void setIntegralLimits(float min, float max);

private:
    float _kp, _ki, _kd;
    float _dt;

    float _integral;
    float _prevError;

    // Neu für Anti-Windup
    float _integralMin = -1000.0f;
    float _integralMax = 1000.0f;
};

#endif // CONTROLLER_H
