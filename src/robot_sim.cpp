
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
    
    double ul_pos[3];
    ul_pos[0] = pos[0] + cos(pos[2]) * model.ulx - sin(pos[2]) * model.uly;  
    ul_pos[1] = pos[1] + sin(pos[2]) * model.ulx + cos(pos[2]) * model.uly;  
    ul_pos[2] = pos[2] + model.ulrot;
    double dist = RobotPhysics::pos_to_ultra_dist(ul_pos);
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
    bool is_long = false;
    switch (pin) {
    case IR_SHORT_PIN_0:
        if (code.irs_0_pindir == OUTPUT) return 0;
        model_ir_x = model.irS0x;
        model_ir_y = model.irS0y;
        model_ir_rot = model.irS0rot;
        #ifdef IR_SHORT_SCALE_DATASHEET
        scale = IR_SHORT_SCALE_DATASHEET;
        exponent = IR_SHORT_EXPONENT_DATASHEET;
        #else
        scale = IR_SHORT_SCALE_DATASHEET0;
        exponent = IR_SHORT_EXPONENT_DATASHEET0;
        #endif
        break;
    case IR_SHORT_PIN_1:
        if (code.irs_1_pindir == OUTPUT) return 0;
        model_ir_x = model.irS1x;
        model_ir_y = model.irS1y;
        model_ir_rot = model.irS1rot;
        #ifdef IR_SHORT_SCALE_DATASHEET
        scale = IR_SHORT_SCALE_DATASHEET;
        exponent = IR_SHORT_EXPONENT_DATASHEET;
        #else
        scale = IR_SHORT_SCALE_DATASHEET1;
        exponent = IR_SHORT_EXPONENT_DATASHEET1;
        #endif
        break;
    case IR_LONG_PIN_0:
        if (code.irl_0_pindir == OUTPUT) return 0;
        model_ir_x = model.irL0x;
        model_ir_y = model.irL0y;
        model_ir_rot = model.irL0rot;
        #ifdef IR_LONG_SCALE_DATASHEET
        scale = IR_LONG_SCALE_DATASHEET;
        exponent = IR_LONG_EXPONENT_DATASHEET;
        #else
        scale = IR_LONG_SCALE_DATASHEET0;
        exponent = IR_LONG_EXPONENT_DATASHEET0;
        #endif
        is_long = true;
        break;
    case IR_LONG_PIN_1:
        if (code.irl_1_pindir == OUTPUT) return 0;
        model_ir_x = model.irL1x;
        model_ir_y = model.irL1y;
        model_ir_rot = model.irL1rot;
        #ifdef IR_LONG_SCALE_DATASHEET
        scale = IR_LONG_SCALE_DATASHEET;
        exponent = IR_LONG_EXPONENT_DATASHEET;
        #else
        scale = IR_LONG_SCALE_DATASHEET1;
        exponent = IR_LONG_EXPONENT_DATASHEET1;
        #endif
        is_long = true;
        break;
    default:
        model_ir_x = 0;
        model_ir_y = 0;
        model_ir_rot = 0;
        scale = 1;
        exponent = 1;
        break;
    }
    
    double ir_pos[3];
    ir_pos[0] = pos[0] + cos(pos[2]) * model_ir_x - sin(pos[2]) * model_ir_y;  
    ir_pos[1] = pos[1] + sin(pos[2]) * model_ir_x + cos(pos[2]) * model_ir_y;  
    ir_pos[2] = pos[2] + model_ir_rot;
    
    double dist;
    if (is_long) dist = RobotPhysics::pos_to_long_ir_dist(ir_pos);
    else dist = RobotPhysics::pos_to_short_ir_dist(ir_pos);

    double measured = pow(dist/scale,1/exponent);
    // std::cout << "Infrared " << dist << ", " << measured << std::endl;

    return measured;
}
double RobotSim::get_gyro() {
    double vel[3];
    robot_pos->get_velocity(vel);
    if (code.gyro_pindir == OUTPUT) return 0;
    return map_range(vel[2],-250.0/180.0*PI,250.0/180.0*PI,1023*0.8/5.0,1023.0*4.2/5.0);
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
        _sleep(1);
    }
}

void RobotSim::change_robot_state(RobotKinematics *ptr) {
    robot_pos = ptr;
}

void RobotSim::render(SDL_Renderer* sdlr, std::vector<double> const &pos) {
    arena.render(sdlr);
    
    drawRect(sdlr, pos[0],pos[1],sim_robot.model.width, sim_robot.model.length, pos[2], RENDER_SCALE);
    double new_x = rotate_point_x(model.irL0x,model.irL0y,pos[2]);
    double new_y = rotate_point_y(model.irL0x,model.irL0y,pos[2]);
    drawRect(sdlr, pos[0]+new_x,pos[1]+new_y,model.width/5, model.length/5, pos[2]+model.irL0rot, RENDER_SCALE);
    new_x = rotate_point_x(model.irL1x,model.irL1y,pos[2]);
    new_y = rotate_point_y(model.irL1x,model.irL1y,pos[2]);
    drawRect(sdlr, pos[0]+new_x,pos[1]+new_y,model.width/5, model.length/5, pos[2]+model.irL1rot, RENDER_SCALE);
    new_x = rotate_point_x(model.irS0x,model.irS0y,pos[2]);
    new_y = rotate_point_y(model.irS0x,model.irS0y,pos[2]);
    drawRect(sdlr, pos[0]+new_x,pos[1]+new_y,model.width/5, model.length/5, pos[2]+model.irS0rot, RENDER_SCALE);
    new_x = rotate_point_x(model.irS1x,model.irS1y,pos[2]);
    new_y = rotate_point_y(model.irS1x,model.irS1y,pos[2]);
    drawRect(sdlr, pos[0]+new_x,pos[1]+new_y,model.width/5, model.length/5, pos[2]+model.irS1rot, RENDER_SCALE);
    new_x = rotate_point_x(model.ulx,model.uly,pos[2]);
    new_y = rotate_point_y(model.ulx,model.uly,pos[2]);
    drawRect(sdlr, pos[0]+new_x,pos[1]+new_y,model.width/5, model.length/5, pos[2]+model.ulrot, RENDER_SCALE);

    // std::cout << "Render robot " << render_measurements.size() << std::endl;
    for (Line line : render_measurements) {
        int px1 = coord_to_px(line.x1*RENDER_SCALE);
        int py1 = coord_to_py(line.y1*RENDER_SCALE);
        int px2 = coord_to_px(line.x2*RENDER_SCALE);
        int py2 = coord_to_py(line.y2*RENDER_SCALE);
        SDL_RenderDrawLine(sdlr,px1,py1,px2,py2);
    } 
    SDL_RenderDrawLinesF(sdlr,path.data(),path.size());
    render_measurements.clear();
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
            sim_robot.add_time(40.0/1000.0);
            sim_robot.unlock();
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(10));
    }
    
}