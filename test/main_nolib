#define __proteus
#ifdef __proteus
#pragma GCC optimize ("-O0")
#pragma GCC push_options
#endif

#include "src/LIVE_SERIAL.h"

// Asignaciones pins
#define buttom_up 3
#define buttom_down 4
const int PIN_INPUT = A0;
const int PIN_OUTPUT = 11;

// Constantes del controlador
float kp=5.0, ki=0.0003, kd=0.0;

// variables externas del controlador
double Input;
uint8_t Output, Setpoint;

// variables internas del controlador
unsigned long currentTime, previousTime, elapsedTime;
double error, lastError, cumError, rateError;

void setup() {
    Serial.begin(9600);
    Input = analogRead(PIN_INPUT);
    pinMode(buttom_up, INPUT);
    pinMode(buttom_down, INPUT);
    Setpoint = 50;  // Setpoint en °C
}

void loop() {
    Input = analogRead(PIN_INPUT);
    Input = (Input / 1023.0) * 5.0 * 20;
    Output = computePID(Input);
    delay(100);
    analogWrite(PIN_OUTPUT, Output);
    if (digitalRead(buttom_down)) Setpoint--;
    if (digitalRead(buttom_up)) Setpoint++;
    LIVESERIAL_MILLIS("Setpoint", String(Setpoint), currentTime);
    LIVESERIAL_MILLIS("Temp", String(Input), currentTime);
    LIVESERIAL_MILLIS("PWM", String(Output), currentTime);
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

#ifdef __proteus
#pragma GCC pop_options
#endif