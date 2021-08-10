// Disable optimization for Proteus.
#pragma GCC optimize ("-O0")
#pragma GCC push_options

#include "src/LIVE_SERIAL.h"
#include "src/myPID/myPID.h"

void setup() {
    Serial.begin(9600);
    PID_begin();    
}

void loop() {
    updatePID();
    delay(100);
        
    LIVESERIAL_MILLIS("Setpoint", String(Setpoint), currentTime);
    LIVESERIAL_MILLIS("Temp", String(Input), currentTime);
    LIVESERIAL_MILLIS("PWM", String(Output), currentTime);
}

#pragma GCC pop_options