// Parece indiferente al tunneo. No levanta la salida.
// |
// |Setpoint---------------------------------------
// |
// |
// |
// |
// |Output__________________________________________
// |________________________________________________

#define __proteus
#ifdef __proteus
#pragma GCC optimize ("-O0")
#pragma GCC push_options
#endif

#include "src/LIVE_SERIAL.h"
#include "src/ArduinoPIDController/src/PIDController.hpp"

// Asignaciones pins
#define buttom_up 3
#define buttom_down 4
const int PIN_INPUT = A0;
const int PIN_OUTPUT = 11;

// float kp=5.0, ki=0.0003, kd=0.0;
PID::PIDParameters < double > parameters(1.0, 0.001, 0);
PID::PIDController < double > pidController(parameters);

void setup() {
    Serial.begin(9600);
    pinMode(buttom_up, INPUT);
    pinMode(buttom_down, INPUT);
    pinMode(PIN_OUTPUT, OUTPUT);
    pidController.Input = getTemp();
    pidController.Setpoint = 50;
    pidController.SetOutputLimits(0, 255);
    pidController.TurnOn();
}

void loop() {
    pidController.Input = getTemp();
    pidController.Update();
    analogWrite(PIN_OUTPUT, pidController.Output);

    delay(100);
    // if (digitalRead(buttom_down)) pidController.Setpoint--;
    // if (digitalRead(buttom_up)) pidController.Setpoint++;

    uint32_t currentTime = millis();
    LIVESERIAL_MILLIS("Setpoint", String(pidController.Setpoint), currentTime);
    LIVESERIAL_MILLIS("Temp", String(pidController.Input), currentTime);
    LIVESERIAL_MILLIS("PWM", String(pidController.Output), currentTime);
}

float getTemp(){return (analogRead(PIN_INPUT) / 1023.0) * 5.0 * 20;}

#ifdef __proteus
#pragma GCC pop_options
#endif