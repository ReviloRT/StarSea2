
#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

class RobotSim;

#include "ss_parameters.h"
#include "robot_sim_helper.h"
#include "robot_sim_physics.h"
#include "robot_sim_arena.h"
#include "robot_sim_kinematics.h"

#include "parameters.h"

#include <thread>

class RobotSim {
private:
    
    double code_time = 0;
    double target_time = 0;
    std::mutex thread_lock;
    std::thread robot_code_thread;
    
public:

    Arena arena;
    RobotModel model;
    RobotCodeState code;
    RobotKinematics *robot_pos;
    std::vector<Line> render_measurements;

public:

    void init();
    
    double ultrasonic_pulse();
    void ultrasonic_trigger();
    bool ultrasonic_response();
    double get_infrared(int pin);
    double get_gyro();
    double get_battery();

    void set_target_time(double target);
    double get_target_time();
    double get_time();
    double add_time(double time);
    void wait_to_sync();

    void change_robot_state(RobotKinematics *ptr);
    void render(SDL_Renderer* sdlr);
    void lock();
    void unlock();
};

extern RobotSim sim_robot;
extern SoftwareSerial Serial;
extern void setup();
extern void loop();

void run_robot_code();

#endif //ROBOT_CONTROLLER_H