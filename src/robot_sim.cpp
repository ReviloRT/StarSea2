
#include "robot_sim.h"

RobotSim sim_robot;
SoftwareSerial Serial;


void RobotSim::init() {
    robot_code_thread = std::thread(run_robot_code);
}

double RobotSim::ultrasonic_pulse() {
    if (code.ul_T_pindir == INPUT) return 0;
    if (code.ul_E_pindir == OUTPUT) return 0;
    
    double pos[3];
    robot_pos->get_position(pos);
    
    double ul_x = pos[0] + cos(pos[2]) * model.ulx - sin(pos[2]) * model.uly;  
    double ul_y = pos[1] - sin(pos[2]) * model.uly + cos(pos[2]) * model.ulx;  
    double ul_rot = pos[2] + model.ulrot;
    double dist = arena.get_distance(ul_x,ul_y,ul_rot);
    // std::cout << " Ultrasonic Pulse: " << dist << std::endl;
    
    double pulse_time = dist*5.80/1000.0/1000.0;

    return pulse_time;
}
void RobotSim::ultrasonic_trigger() {
    if (code.ul_T_pindir == INPUT) return;
    double pulse_time = ultrasonic_pulse();
    code.ul_response_1 = code_time + 50.0/1000.0/1000.0;
    code.ul_response_2 = code.ul_response_1 + pulse_time;
    // std::cout << "Ultrasonic Trigger " << time << ", " << pulse_time << ", " << code.ul_response_1 << ", " << code.ul_response_2 << std::endl;
}
bool RobotSim::ultrasonic_response() {
    if (code.ul_response_2 <= code_time) {
        code.ul_response_1 = 0;
        code.ul_response_2 = 0;
        return false;
    } else if (code.ul_response_1 <= code_time) {
        return true;
    } else {
        return false;
    }
}
double RobotSim::get_infrared(int pin) {
    
    double pos[3];
    robot_pos->get_position(pos);

    double model_ir_x, model_ir_y, model_ir_rot, scale, exponent;
    switch (pin) {
    case IR_SHORT_PIN_0:
        if (code.irs_0_pindir == OUTPUT) return 0;
        model_ir_x = model.irS0x;
        model_ir_y = model.irS0y;
        model_ir_rot = model.irS0rot;
        scale = IR_SHORT_SCALE_DATASHEET;
        exponent = IR_SHORT_EXPONENT_DATASHEET;
        break;
    case IR_SHORT_PIN_1:
        if (code.irs_1_pindir == OUTPUT) return 0;
        model_ir_x = model.irS1x;
        model_ir_y = model.irS1y;
        model_ir_rot = model.irS1rot;
        scale = IR_SHORT_SCALE_DATASHEET;
        exponent = IR_SHORT_EXPONENT_DATASHEET;
        break;
    case IR_LONG_PIN_0:
        if (code.irl_0_pindir == OUTPUT) return 0;
        model_ir_x = model.irL0x;
        model_ir_y = model.irL0y;
        model_ir_rot = model.irL0rot;
        scale = IR_LONG_SCALE_DATASHEET;
        exponent = IR_LONG_EXPONENT_DATASHEET;
        break;
    case IR_LONG_PIN_1:
        if (code.irl_1_pindir == OUTPUT) return 0;
        model_ir_x = model.irL1x;
        model_ir_y = model.irL1y;
        model_ir_rot = model.irL1rot;
        scale = IR_LONG_SCALE_DATASHEET;
        exponent = IR_LONG_EXPONENT_DATASHEET;
        break;
    default:
        model_ir_x = 0;
        model_ir_y = 0;
        model_ir_rot = 0;
        scale = 1;
        exponent = 1;
        break;
    }
    
    double ir_x = pos[0] + cos(pos[2]) * model_ir_x - sin(pos[2]) * model_ir_y;  
    double ir_y = pos[1] - sin(pos[2]) * model_ir_y + cos(pos[2]) * model_ir_x;  
    double ir_rot = pos[2] + model_ir_rot;
    double dist = arena.get_distance(ir_x,ir_y,ir_rot);
    double measured = pow(dist/scale,1/exponent);
    // std::cout << "Infrared " << dist << ", " << measured << std::endl;

    return measured;
}
double RobotSim::get_gyro() {
    double pos[3];
    robot_pos->get_position(pos);
    if (code.gyro_pindir == OUTPUT) return 0;
    return map_range(pos[2],-250,250,1023*0.5/5.0,1023.0*4.5/5.0);
}
double RobotSim::get_battery() {
    if (code.bat_pindir == OUTPUT) return 0;
    double bat = code.battery_percent;
    return map_range(bat,0,100,720,860);
}

void RobotSim::set_target_time(double target){
    if (target > target_time)
    target_time = target;
}
double RobotSim::get_target_time() {
    double target = target_time;
    return target;
}
double RobotSim::get_time() {
    double curr_time = code_time;
    return curr_time;
}
double RobotSim::add_time(double time_inc) {
    code_time += time_inc;
    double curr_time = code_time;
    return curr_time;
}
void RobotSim::wait_to_sync() {   
    lock(); 
    double t1 = get_target_time();
    double t2 = get_time();
    unlock();
    while (t1 < t2) {
        lock();
        t1 = get_target_time();
        t2 = get_time();
        unlock();
        _sleep(10);
    }
}

void RobotSim::change_robot_state(RobotKinematics *ptr) {
    robot_pos = ptr;
}

void RobotSim::render(SDL_Renderer* sdlr) const {
    arena.render(sdlr);
}
void RobotSim::lock(){
    thread_lock.lock();
}
void RobotSim::unlock(){
    thread_lock.unlock();
}



void run_robot_code() {
    setup();
    
    sim_robot.lock();
    sim_robot.add_time(10.0/1000.0);
    sim_robot.unlock();
    int last_int_time = 0;
        
    while(true) {
        sim_robot.lock();
        double sim_time = sim_robot.get_time();
        double tar_time = sim_robot.get_target_time();
        sim_robot.unlock();

        if (sim_time > last_int_time) {
            std::cout << "### Robot Thread Time: " << sim_time <<" ###" << std::endl;
            last_int_time = (int) sim_time + 1;
        }

        if (tar_time > sim_time) {
            loop();
            sim_robot.lock();
            sim_robot.add_time(10.0/1000.0);
            sim_robot.unlock();
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    }
    
}