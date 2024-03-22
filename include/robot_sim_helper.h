#ifndef ROBOT_SIM_HELPERS
#define ROBOT_SIM_HELPERS

#include "ss_parameters.h"

#include <string>
#include <iostream>

#define INPUT 1
#define OUTPUT 0
#define HIGH 1
#define LOW 0

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

#define ROBOT_RENDER_SCALE 0.0006

// #define String std::string
class String : public std::string {
public:
    String(double);
    String(const char[]);
    String(std::basic_string<char>);
    String operator+(const char []);
    String operator+(std::basic_string<char>);
    
};

struct Wall {
    double x1;
    double y1;
    double x2;
    double y2;
};

struct PhysicalModel {
    double width = 100;
    double length = 100;
    double wheelf = 8;
    double wheels = 8;

    double ulx = 0;
    double uly = 0;
    double ulrot = 0;
    double irS0x = 0;
    double irS0y = 0;
    double irS0rot = 0;
    double irS1x = 0;
    double irS1y = 0;
    double irS1rot = 0;
    double irL0x = 0;
    double irL0y = 0;
    double irL0rot = 0;
    double irL1x = 0;
    double irL1y = 0;
    double irL1rot = 0;
};

struct CodeState {
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

};

struct SoftwareSerial {
private:
public:
    SoftwareSerial();
    SoftwareSerial(int pin_tx, int pin_rx);
    void begin(int baud_rate);
    void print(std::string val);
    void print(int val);
    void print(double val);
};

struct Servo {
private:
    int pin = -1;
public:
    Servo();
    void attach(int pin);
    void writeMicroseconds(int micros);
};

#include "state_robot.h"

#endif // ROBOT_SIM_HELPERS