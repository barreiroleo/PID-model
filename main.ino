// Disable optimization for Proteus.
#pragma GCC optimize ("-O0")
#pragma GCC push_options

const uint8_t PIN_INPUT   = A0;
const uint8_t PIN_OUTPUT  = 11;

#include "src/LIVE_SERIAL.h"
#include "src/myPID/myPID.h"

MyPID myPID(5.0, 0.0003, 0.0);

void setup() {
    Serial.begin(9600);
    myPID.setpoint = 50;
    myPID.setMinMax(0, 255);
}

void loop() {
    myPID.input = updateTemp();
    myPID.computePID();
    delay(100);
    
    analogWrite(PIN_OUTPUT, myPID.output);
    LIVESERIAL_MILLIS("Setpoint", String(myPID.setpoint), myPID.previousTime);
    LIVESERIAL_MILLIS("Temp", String(myPID.input), myPID.previousTime);
    LIVESERIAL_MILLIS("PWM", String(myPID.output), myPID.previousTime);
}

float updateTemp() {
    return ((analogRead(PIN_INPUT) / 1023.0) * 5.0 * 20);
}

#pragma GCC pop_options