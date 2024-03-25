#include "robot_sim_exposed.h"

// General math functions
double map(double value, double low_in, double high_in, double low_out, double high_out) {
    return map_range(value, low_in, high_in, low_out, high_out);
}
double min(double in1, double in2) {
    return std::min(in1,in2);
}
double max(double in1, double in2) {
    return std::max(in1,in2);
}

// Time functions
uint64_t micros() {
    sim_robot.wait_to_sync();
    sim_robot.lock();
    double time = sim_robot.get_time();
    sim_robot.unlock();
    return time*1000.0*1000.0;
}
uint64_t millis() {
    sim_robot.wait_to_sync();
    sim_robot.lock();
    double time = sim_robot.get_time();
    sim_robot.unlock();
    return time*1000.0;
}
void delay(int millis) {
    sim_robot.wait_to_sync();
    sim_robot.lock();
    sim_robot.add_time(((double)millis)/1000.0);
    sim_robot.unlock();
    _sleep(millis/ROBOT_SIM_RATE);
}
void delayMicroseconds(int micros) {
    sim_robot.wait_to_sync();
    sim_robot.lock();
    sim_robot.add_time(((double)micros)/1000.0/1000.0);
    sim_robot.unlock();
}

// Pin functions
void pinMode(int pin, bool direction) {
    sim_robot.wait_to_sync();
    sim_robot.lock();
    switch (pin){
    case LED_BUILTIN:
        sim_robot.code.led_pindir = direction;
        break;
    case GYROSCOPE_PIN:
        sim_robot.code.gyro_pindir = direction;
        break;
    case IR_SHORT_PIN_0:
        sim_robot.code.irs_0_pindir = direction;
        break;
    case IR_SHORT_PIN_1:
        sim_robot.code.irs_1_pindir = direction;
        break;
    case IR_LONG_PIN_0:
        sim_robot.code.irl_0_pindir = direction;
        break;
    case IR_LONG_PIN_1:
        sim_robot.code.irl_1_pindir = direction;
        break;
    case US_PIN_TRIGGER:
        sim_robot.code.ul_T_pindir = direction;
        break;
    case US_PIN_ECHO:
        sim_robot.code.ul_E_pindir = direction;
        break;
    case T0_PIN:
        break;
    case T1_PIN:
        break;
    case T2_PIN:
        break;
    case T3_PIN:
        break;
    case FL_MOTOR_PIN:
        sim_robot.code.m_FL_pindir = direction;
        break;
    case FR_MOTOR_PIN:
        sim_robot.code.m_FR_pindir = direction;
        break;
    case BL_MOTOR_PIN:
        sim_robot.code.m_RL_pindir = direction;
        break;
    case BR_MOTOR_PIN:
        sim_robot.code.m_RR_pindir = direction;
        break;
    case A0:
        sim_robot.code.bat_pindir = direction;
        break;
    default:
        break;
    }
    sim_robot.unlock();
}
void digitalWrite(int pin, bool value) {
    sim_robot.wait_to_sync();

    sim_robot.lock();
    
    
    switch (pin) {
    case LED_BUILTIN:
        if (sim_robot.code.led_pindir == INPUT) return;
        sim_robot.code.led_value = value;
        break;
    case US_PIN_TRIGGER:
        if (value == false) break;
        sim_robot.ultrasonic_trigger();
        break;
    }
    sim_robot.unlock();
}
void pwmWrite(int pin, int micros) {
    sim_robot.wait_to_sync();
    sim_robot.lock();
    double motor_percent = map_range(micros,1000,2000,-100,100);
    switch (pin) {
    case FL_MOTOR_PIN:
        if (sim_robot.code.m_FL_pindir == OUTPUT)
        sim_robot.code.m_FL_power = motor_percent;
        break;
    case FR_MOTOR_PIN:
        if (sim_robot.code.m_FR_pindir == OUTPUT)
        sim_robot.code.m_FR_power = motor_percent;
        break;
    case BL_MOTOR_PIN:
        if (sim_robot.code.m_RL_pindir == OUTPUT)
        sim_robot.code.m_RL_power = motor_percent;
        break;
    case BR_MOTOR_PIN:
        if (sim_robot.code.m_RR_pindir == OUTPUT)
        sim_robot.code.m_RR_power = motor_percent;
        break;
    default:
        break;
    }
    sim_robot.unlock();
    sim_robot.add_time(1/1000.0/1000.0);

}
uint16_t analogRead(int pin) {
    sim_robot.wait_to_sync();
    sim_robot.lock();
    uint16_t ret = 0;
    switch (pin){
    case GYROSCOPE_PIN:
        ret = sim_robot.get_gyro();
        break;
    case IR_SHORT_PIN_0:
        ret = sim_robot.get_infrared(pin);
        break;
    case IR_SHORT_PIN_1:
        ret = sim_robot.get_infrared(pin);
        break;
    case IR_LONG_PIN_0:
        ret = sim_robot.get_infrared(pin);
        break;
    case IR_LONG_PIN_1:
        ret = sim_robot.get_infrared(pin);
        break;
    case A0:
        ret = sim_robot.get_battery();
        break;
    default:
        break;
    }
    sim_robot.unlock();
    sim_robot.add_time(5/1000.0/1000.0);
    return ret;
}
bool digitalRead(int pin) {
    sim_robot.wait_to_sync();
    sim_robot.lock();
    bool ret = 0;
    double cur_time = sim_robot.get_time();
    switch (pin){
    case US_PIN_ECHO:
        if (sim_robot.code.ul_E_pindir == OUTPUT) ret = false;
        ret = sim_robot.ultrasonic_response();
        break;
    default:
        ret = 0;
        break;
    }
    
    sim_robot.add_time(10.0/1000.0/1000.0);

    sim_robot.unlock();
    return ret;
}
