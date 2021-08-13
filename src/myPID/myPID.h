#ifndef myPID_h
#define myPID_h
#include "Arduino.h"

class MyPID {
private:
    uint8_t outputMin, outputMax;
    float lastError;
public:
    float kp, ki, kd;
    float input; uint8_t setpoint, output;
    unsigned long previousTime;
    MyPID(float kp, float ki, float kd);
    ~MyPID();
    void computePID();
    void setMinMax(uint8_t min, uint8_t max);
};

#endif