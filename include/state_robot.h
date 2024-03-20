#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "ss_parameters.h"
#include "state.h"
#include "robot_sim_helper.h"
#include "helper.h"

#include "parameters.h"

#include <array>
#include <chrono>


class Arena {
    PhysicalRobot model;
    std::vector<Wall> walls;
public:
    void add_wall(Wall wall);
    void add_wall(double x1, double y1, double x2, double y2);
    double get_distance(double x, double y, double rot) const;
    void render(SDL_Renderer* sdlr) const;
};


class RobotState : public State {
private:
    Arena *arena = NULL;
    PhysicalRobot *model = NULL;

    CodeState set;

public:

    RobotState();
    RobotState(std::vector<double> new_data);
    RobotState(std::vector<double> new_data_0, std::vector<double> new_data_1);
    RobotState(const RobotState &other);
    RobotState& operator=(const RobotState &other);

    void set_arena(Arena *a_ptr);
    void set_model(PhysicalRobot *m_ptr);
    
    unsigned long ultrasonic_pulse();
    double get_infrared(int pin);
    double get_gyro();
    double get_battery();
    void motors_to_delta(RobotState &other) const;

    double get_state_time();
    void add_code_time(double time);
    void stuck(bool stuck);
    bool is_stuck();
    CodeState& get_robot_set();
    
    void solve_deltas(RobotState &output, double time);
    virtual void render(SDL_Renderer* sdlr) const override;
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


static RobotState *robot = NULL;
static SoftwareSerial Serial;

extern void setup();
extern void loop();


#endif // ROBOT_STATE_H