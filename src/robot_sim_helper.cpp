
#include "robot_sim_helper.h"

String::String(double a) {std::to_string(a);}
String::String(const char a[]) : std::string(a) {}
String::String(std::basic_string<char> a) : std::string(a) {}
String String::operator+(const char a[]) {return *this + std::string(a);}
String String::operator+(std::basic_string<char> a) {return a + *this;}

SerialSim::SerialSim() {
}
SerialSim::SerialSim(int pin_tx, int pin_rx) {
}
void SerialSim::begin(int baud_rate) {
}
void SerialSim::print(std::string val) {
    std::cout << val;
}
void SerialSim::print(int val) {
    std::cout << val;
}
void SerialSim::print(double val) {
    std::cout << val;
}

ServoSim::ServoSim() {}
void ServoSim::attach(int new_pin) {
    this->pin = new_pin;
}
void ServoSim::writeMicroseconds(int micros) {
    robot->pwmWrite(pin, micros);
}


