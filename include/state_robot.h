#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "ss_parameters.h"
#include "state.h"
#include "robot_sim_helper.h"
#include "helper.h"

#include "parameters.h"

#include <array>
#include <chrono>
#include <thread>

class Arena;
class RobotState;
class ControllerState;


class Arena {
public:
    std::vector<Wall> walls;
public:
    Arena& operator=(const Arena& other);
    void add_wall(Wall wall);
    void add_wall(double x1, double y1, double x2, double y2);
    double get_distance(double x, double y, double rot) const;
    void render(SDL_Renderer* sdlr) const;
};

class ControllerState {
private:
    CodeState set;
    PhysicalModel model;
    Arena arena;
    double time = 0;
    double target_time = 0;
    std::mutex thread_lock;
    RobotState *robot_pos;
    std::thread r_code_thread;

public:

    void init();
    
    unsigned long ultrasonic_pulse();
    double get_infrared(int pin);
    double get_gyro();
    double get_battery();

    void set_target_time(double target);
    double get_target_time();
    double get_time();
    double add_time(double time);

    void change_robot_state(RobotState *ptr);
    CodeState& get_code();
    Arena* get_arena();
    PhysicalModel* get_model();
    void render(SDL_Renderer* sdlr) const;
    void lock();
    void unlock();
    

};


class RobotState : public State {
private:

public:
    // Initialisation functions
    RobotState();
    RobotState(std::vector<double> new_data);
    RobotState(std::vector<double> new_data_0, std::vector<double> new_data_1);
    RobotState(const RobotState &other);
    RobotState& operator=(const RobotState &other);
    
    // Main controlled functions
    void solve_deltas(RobotState &output, double time);
    virtual void render(SDL_Renderer* sdlr) const override;
    void motors_to_delta(RobotState &other);

    double get_data(int array, int idx) const;
    
};

// double pow(double raw, double exponent);
double map(double value, double low_in, double high_in, double low_out, double high_out);
double min(double in1, double in2);
double max(double in1, double in2);

uint64_t micros();
uint64_t millis();
void delay(int millis);
void delayMicroseconds(int micros);
void infiniteWhile();

void pinMode(int pin, bool direction);
void digitalWrite(int pin, bool value);
void pwmWrite(int pin, int micros);
uint16_t analogRead(int pin);


extern ControllerState sim_robot;
extern SoftwareSerial Serial;

extern void setup();
extern void loop();

void run_robot_code();


#endif // ROBOT_STATE_H