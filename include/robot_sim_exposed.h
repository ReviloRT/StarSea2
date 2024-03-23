
#ifndef ROBOT_SIM_EXPOSED_H
#define ROBOT_SIM_EXPOSED_H

#include "robot_sim.h"
#include "helper.h"

// General maths functions
double map(double value, double low_in, double high_in, double low_out, double high_out);
double min(double in1, double in2);
double max(double in1, double in2);

// Time functions
uint64_t micros();
uint64_t millis();
void delay(int millis);
void delayMicroseconds(int micros);


// Pin functions
void pinMode(int pin, bool direction);
void digitalWrite(int pin, bool value);
void pwmWrite(int pin, int micros);
uint16_t analogRead(int pin);
bool digitalRead(int pin);

#endif // ROBOT_SIM_EXPOSED_H