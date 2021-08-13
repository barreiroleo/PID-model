#include "myPID.h"

MyPID::MyPID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

MyPID::~MyPID() {}

void MyPID::setMinMax(uint8_t min, uint8_t max) {
    this->outputMin = min;
    this->outputMax = max;
}

void MyPID::computePID() {
    unsigned long elapsedTime, currentTime;
    float error, cumError, rateError, now_output;
    currentTime = millis();
    elapsedTime = (currentTime - previousTime);
    error = setpoint - input;

    // Integral del error
    cumError += error * elapsedTime;
    // Derivada del error
    rateError = (error - lastError) / elapsedTime;
    // Sumador. Salida de PID.
    now_output = kp * error + ki * cumError + kd * rateError;
    // Corrección de valores extremos.
    if (now_output < outputMin) { output = outputMin; }
    if (now_output > outputMax) { output = outputMax; }
    else { output = now_output; }

    lastError = error;
    previousTime = currentTime;
}