#ifndef ROBOT_SIM_H
#define ROBOT_SIM_H

#include "ss_parameters.h"

#define INPUT 1
#define OUTPUT 0
#define HIGH 1
#define LOW 0
#define DEC 10

#define A0 0
#define A1 1
#define A2 2
#define A3 3
#define A4 4
#define A5 5
#define A6 6
#define A7 7
#define A8 8
#define A9 9
#define A10 10

#define LED_BUILTIN 11


#include <string>
#include <iostream>

static int serial_output_count = 0;

extern void pwmWrite(int pin, int micros);

class String : public std::string {
public:
    String(double);
    String(const char[]);
    String(std::basic_string<char>);
    String operator+(const char []);
    String operator+(std::basic_string<char>);
};

struct SoftwareSerial {
private:
    bool active = false;
public:
    SoftwareSerial();
    SoftwareSerial(int pin_tx, int pin_rx);
    void begin(int baud_rate);
    void print(std::string val);
    void print(int val);
    void print(double val);
    void print(double val, int ignore);

};

struct Servo {
private:
    int pin = -1;
public:
    Servo();
    void attach(int pin);
    void write(int angle);
    void writeMicroseconds(int micros);
};

struct RobotCodeState {
    bool led_pindir = OUTPUT;
    bool gyro_pindir = OUTPUT;
    bool irs_0_pindir = OUTPUT;
    bool irs_1_pindir = OUTPUT;
    bool irl_0_pindir = OUTPUT;
    bool irl_1_pindir = OUTPUT;
    bool ul_T_pindir = OUTPUT;
    bool ul_E_pindir = OUTPUT;
    bool bat_pindir = OUTPUT;
    bool m_FL_pindir = OUTPUT;
    bool m_FR_pindir = OUTPUT;
    bool m_RL_pindir = OUTPUT;
    bool m_RR_pindir = OUTPUT;

    bool led_value = false;
    double battery_percent = 100;
    double m_FL_power = 0;
    double m_FR_power = 0;
    double m_RL_power = 0;
    double m_RR_power = 0;
    double ul_response_1 = 0;
    double ul_response_2 = 0;

};

#endif // ROBOT_SIM_H