
#include "robot_sim_helper.h"

String::String(double a) {std::to_string(a);}
String::String(const char a[]) : std::string(a) {}
String::String(std::basic_string<char> a) : std::string(a) {}
String String::operator+(const char a[]) {return *this + std::string(a);}
String String::operator+(std::basic_string<char> a) {return a + *this;}

SoftwareSerial::SoftwareSerial() {
}
SoftwareSerial::SoftwareSerial(int pin_tx, int pin_rx) {
}
void SoftwareSerial::begin(int baud_rate) {
}
void SoftwareSerial::print(std::string val) {
    std::cout << val;
}
void SoftwareSerial::print(int val) {
    std::cout << val;
}
void SoftwareSerial::print(double val) {
    std::cout << val;
}

Servo::Servo() {}
void Servo::attach(int new_pin) {
    this->pin = new_pin;
}
void Servo::writeMicroseconds(int micros) {
    pwmWrite(pin, micros);
}


