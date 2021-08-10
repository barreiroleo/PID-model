#ifndef myPID_h
#define myPID_h
#include "Arduino.h"

const uint8_t buttom_up   = 3;
const uint8_t buttom_down = 4;
const uint8_t PIN_INPUT   = A0;
const uint8_t PIN_OUTPUT  = 11;
// variables externas del controlador
double Input; uint8_t Output, Setpoint;
// variables internas del controlador
unsigned long currentTime, previousTime, elapsedTime;
double error, lastError, cumError, rateError;
float kp=5.0, ki=0.0003, kd=0.0;

void PID_begin();
void updateSetpoint();
void updateTemp();
void updatePID();
double computePID(double);


void PID_begin() {
    Input = analogRead(PIN_INPUT);
    Setpoint = 50;  // Setpoint en °C
    pinMode(buttom_up, INPUT);
    pinMode(buttom_down, INPUT);
}

void updateSetpoint() {
    if (digitalRead(buttom_down)) Setpoint--;
    if (digitalRead(buttom_up)) Setpoint++;
}
void updateTemp() {
    Input = analogRead(PIN_INPUT);
    Input = (Input / 1023.0) * 5.0 * 20;
}

void updatePID() {
    Output = computePID(Input);
    analogWrite(PIN_OUTPUT, Output);
}

double computePID(double inp) {
    currentTime = millis();
    elapsedTime = (currentTime - previousTime);

    error = Setpoint - Input;
    // Integral del error
    cumError += error * elapsedTime;
    // Derivada del error
    rateError = (error - lastError) / elapsedTime;
    // Sumador. Salida de PID.
    float output = kp * error + ki * cumError + kd * rateError;
    // Corrección de valores extremos.
    if (output < 0) { output = 0; }
    if (output > 255) { output = 255; }

    lastError = error;
    previousTime = currentTime;
    return output;
}


#endif