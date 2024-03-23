#include "state_robot.h"


ControllerState sim_robot;
SoftwareSerial Serial;

void Arena::add_wall(Wall wall) {
    this->walls.push_back(wall);
}
void Arena::add_wall(double x1, double y1, double x2, double y2) {
    Wall wall = {x1,y1,x2,y2};
    this->walls.push_back(wall);
}
double Arena::get_distance(double x, double y, double rot) const {


    double min_distance = 1000000000000000000;
    for (auto w : walls){
        // Parametric Equation
        // s*(x2-x1) - t*(x4-x3) = (x3-x1);
        // s*(y2-y1) - t*(y4-y3) = (y3-y1);

        // Simplification
        // (x4 - x3) = cos(rot)     (b1)
        // (y4 - y4) = sin(rot)     (b2)

        // Crammers Rule
        // for a1*x + b1*y = c1, a2*x + b2*y = c2
        // x = (c1*b2 - c2*b1) / (a1*b2 - a2*b1)
        // y = (a1*c2 - a2*c1) / (a1*b2 - a2*b1)

        // Applied Crammers
        // s = ((x3-x1)*sin(rot) - (y3-y1)*cos(rot)) / ((x2-x1)*sin(rot) - (y2-y1)*cos(rot))
        // t = ((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / ((x2-x1)*sin(rot) - (y2-y1)*cos(rot))

        // Parallel check
        // ((x2-x1)*sin(rot) - (y2-y1)*cos(rot)) == 0

        // Equivalences
        // x1 = w.x1, y1 = w.y1
        // x2 = w.x2, y2 = w.y2
        // x3 = x, y3 = y

        double denominator = (w.x2-w.x1)*sin(rot) - (w.y2-w.y1)*cos(rot);
        if (denominator == 0) continue;
        double s = ((x-w.x1)*sin(rot) - (y-w.y1)*cos(rot)) / denominator;
        double t = ((w.x2-w.x1)*(y-w.y1) - (w.y2-w.y1)*(x-w.x1)) / denominator;
                
        if ((s >= 0) && (s <= 1) && (t <= 0)) {
            double dist = -t;
            if (dist < min_distance) min_distance = dist;
        }
    }

    return min_distance;    
    
}
void Arena::render(SDL_Renderer* sdlr) const {
    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);
    for (size_t i = 0; i < this->walls.size(); i++) {
        const Wall &w = walls[i];
        int px1 = coord_to_px(w.x1*ROBOT_RENDER_SCALE);
        int py1 = coord_to_py(w.y1*ROBOT_RENDER_SCALE);
        int px2 = coord_to_px(w.x2*ROBOT_RENDER_SCALE);
        int py2 = coord_to_py(w.y2*ROBOT_RENDER_SCALE);
        SDL_RenderDrawLine(sdlr,px1,py1,px2,py2);
        // std::cout << "Arena: double (" << w.x1 << " , " << w.y1 << "), (" << w.x2 << " , " << w.y2 << ")" << "                 " <<std::endl;
        // std::cout << "Arena: px     (" << px1 << " , " << py1 << "), (" << px2 << " , " << py2 << ")" << "                 " <<std::endl;
    }
    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
}





// Init functions
RobotState::RobotState() : State() {
    data_0.assign(3,0);
    data_1.assign(3,0);
}
RobotState::RobotState(std::vector<double> new_data) : State(new_data) {
    data_0 = new_data;
    data_1.assign(3,0);
}
RobotState::RobotState(std::vector<double> new_data_0, std::vector<double> new_data_1) : State(new_data_0, new_data_1) {
    data_0 = new_data_0;
    data_1 = new_data_1;
}
RobotState::RobotState(const RobotState &other) : State(other){

}
RobotState& RobotState::operator=(const RobotState &other) {
    State::operator=(other);

    return *this;
}


// Main functions
void RobotState::solve_deltas(RobotState &output, double target_time) {
    sim_robot.change_robot_state(this);
    output = *this;
    motors_to_delta(output);
    sim_robot.set_target_time(target_time);

}

void RobotState::render(SDL_Renderer* sdlr) const {
    sim_robot.render(sdlr);
    PhysicalModel &model = *sim_robot.get_model();
    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);
    drawRect(sdlr, data_0[0],data_0[1],model.width, model.length, data_0[2], ROBOT_RENDER_SCALE);
    // drawRect(sdlr, data_0[0]+model.irL0x,data_0[1]+model.irL0y,model.width/10, model.length/10, data_0[2]+model.irL0rot, ROBOT_RENDER_SCALE);


    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
    // std::cout << data_0[0] << ", " << data_0[1] << ", " << data_0[2] << std::endl;
}



void RobotState::motors_to_delta(RobotState &other) {
    sim_robot.lock();
    CodeState &set = sim_robot.get_code();
    PhysicalModel *model = sim_robot.get_model();
    other.data_0[0] = ( - set.m_FL_power - set.m_FR_power + set.m_RL_power + set.m_RR_power);
    other.data_0[1] = ( - set.m_FL_power + set.m_FR_power - set.m_RL_power + set.m_RR_power);
    other.data_0[2] = 1.0 / (model->wheelf + model->wheels) * (- set.m_FL_power + set.m_FR_power + set.m_RL_power - set.m_RR_power);
    std::cout << "Motor to delta A: " << other.data_0[0] << ", " << other.data_0[1] << ", " << other.data_0[2] << std::endl;
    std::cout << "Motor to delta B: " << set.m_FL_power << ", " << set.m_FR_power << ", " << set.m_RL_power << ", " << set.m_RR_power << std::endl;    
    sim_robot.unlock();
}

double RobotState::get_data(int array, int idx) const {
    if (array == 0) return data_0[idx];
    if (array == 1) return data_1[idx];
    return 0;
}


void ControllerState::init() {
    r_code_thread = std::thread(run_robot_code);
}

double ControllerState::ultrasonic_pulse() {
    if (set.ul_T_pindir == INPUT) return 0;
    if (set.ul_E_pindir == OUTPUT) return 0;
    double rob_x = robot_pos->get_data(0,0);
    double rob_y = robot_pos->get_data(0,1);
    double rob_w = robot_pos->get_data(0,2);
    
    double ul_x = rob_x + cos(rob_w) * model.ulx - sin(rob_w) * model.uly;  
    double ul_y = rob_y - sin(rob_w) * model.uly + cos(rob_w) * model.ulx;  
    double ul_rot = rob_w + model.ulrot;
    double dist = get_arena()->get_distance(ul_x,ul_y,ul_rot);
    // std::cout << " Ultrasonic Pulse: " << dist << std::endl;
    
    double pulse_time = dist*5.80/1000.0/1000.0;

    return pulse_time;
}
void ControllerState::ultrasonic_trigger() {
    if (set.ul_T_pindir == INPUT) return;
    double pulse_time = ultrasonic_pulse();
    set.ul_response_1 = time + 50.0/1000.0/1000.0;
    set.ul_response_2 = set.ul_response_1 + pulse_time;
    // std::cout << "Ultrasonic Trigger " << time << ", " << pulse_time << ", " << set.ul_response_1 << ", " << set.ul_response_2 << std::endl;
}
bool ControllerState::ultrasonic_response() {
    if (set.ul_response_2 <= time) {
        set.ul_response_1 = 0;
        set.ul_response_2 = 0;
        return false;
    } else if (set.ul_response_1 <= time) {
        return true;
    } else {
        return false;
    }
}

double ControllerState::get_infrared(int pin) {
    lock();
    double rob_x = robot_pos->get_data(0,0);
    double rob_y = robot_pos->get_data(0,1);
    double rob_w = robot_pos->get_data(0,2);
    unlock();

    double model_ir_x, model_ir_y, model_ir_rot, scale, exponent;
    switch (pin) {
    case IR_SHORT_PIN_0:
        if (set.irs_0_pindir == OUTPUT) return 0;
        model_ir_x = model.irS0x;
        model_ir_y = model.irS0y;
        model_ir_rot = model.irS0rot;
        scale = IR_SHORT_SCALE_DATASHEET;
        exponent = IR_SHORT_EXPONENT_DATASHEET;
        break;
    case IR_SHORT_PIN_1:
        if (set.irs_1_pindir == OUTPUT) return 0;
        model_ir_x = model.irS1x;
        model_ir_y = model.irS1y;
        model_ir_rot = model.irS1rot;
        scale = IR_SHORT_SCALE_DATASHEET;
        exponent = IR_SHORT_EXPONENT_DATASHEET;
        break;
    case IR_LONG_PIN_0:
        if (set.irl_0_pindir == OUTPUT) return 0;
        model_ir_x = model.irL0x;
        model_ir_y = model.irL0y;
        model_ir_rot = model.irL0rot;
        scale = IR_LONG_SCALE_DATASHEET;
        exponent = IR_LONG_EXPONENT_DATASHEET;
        break;
    case IR_LONG_PIN_1:
        if (set.irl_1_pindir == OUTPUT) return 0;
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
    
    double ir_x = rob_x + cos(rob_w) * model_ir_x - sin(rob_w) * model_ir_y;  
    double ir_y = rob_y - sin(rob_w) * model_ir_y + cos(rob_w) * model_ir_x;  
    double ir_rot = rob_w + model_ir_rot;
    double dist = get_arena()->get_distance(ir_x,ir_y,ir_rot);
    double measured = pow(dist/scale,1/exponent);
    // std::cout << "Infrared " << dist << ", " << measured << std::endl;

    return measured;
}
double ControllerState::get_gyro() {
    lock();
    double rob_omega = robot_pos->get_data(1,2);
    unlock();
    if (set.gyro_pindir == OUTPUT) return 0;
    return map(rob_omega,-250,250,1023*0.5/5.0,1023.0*4.5/5.0);
}
double ControllerState::get_battery() {
    lock();
    if (set.bat_pindir == OUTPUT) return 0;
    double bat = set.battery_percent;
    unlock();
    return map(bat,0,100,720,860);
}

void ControllerState::set_target_time(double target){
    lock();
    if (target > target_time)
    target_time = target;
    unlock();
}
double ControllerState::get_target_time() {
    lock();
    double target = target_time;
    unlock();
    return target;
}
double ControllerState::get_time() {
    lock();
    double curr_time = time;
    unlock();
    return curr_time;
}
double ControllerState::add_time(double time_inc) {
    lock();
    time += time_inc;
    double curr_time = time;
    unlock();
    return curr_time;
}

void ControllerState::change_robot_state(RobotState *ptr) {
    lock();
    robot_pos = ptr;
    unlock();
}

CodeState& ControllerState::get_code() {
    return set;
}
Arena* ControllerState::get_arena() {
    return &arena;
}
PhysicalModel* ControllerState::get_model() {
    return &model;
}
void ControllerState::render(SDL_Renderer* sdlr) const {
    arena.render(sdlr);
}
void ControllerState::lock() {
    thread_lock.lock();
}
void ControllerState::unlock() {
    thread_lock.unlock();
}



// General math functions
double map(double value, double low_in, double high_in, double low_out, double high_out) {
    return (value - low_in) * (high_out - low_out)/(high_in - low_in) + low_out;
}
double min(double in1, double in2) {
    return std::min(in1,in2);
}
double max(double in1, double in2) {
    return std::max(in1,in2);
}

// Time functions
uint64_t micros() {
    return sim_robot.get_time()*1000.0*1000.0;
}
uint64_t millis() {
    return sim_robot.get_time()*1000.0;
}
void delay(int millis) {
    sim_robot.add_time(((double)millis)/1000.0);
    _sleep(millis);
}
void delayMicroseconds(int micros) {
    sim_robot.add_time(((double)micros)/1000.0/1000.0);
}
void infiniteWhile() {
    delay(100);
}

// Pin functions
void pinMode(int pin, bool direction) {
    sim_robot.lock();
    CodeState &set = sim_robot.get_code();
    switch (pin){
    case LED_BUILTIN:
        set.led_pindir = direction;
        break;
    case GYROSCOPE_PIN:
        set.gyro_pindir = direction;
        break;
    case IR_SHORT_PIN_0:
        set.irs_0_pindir = direction;
        break;
    case IR_SHORT_PIN_1:
        set.irs_1_pindir = direction;
        break;
    case IR_LONG_PIN_0:
        set.irl_0_pindir = direction;
        break;
    case IR_LONG_PIN_1:
        set.irl_1_pindir = direction;
        break;
    case US_PIN_TRIGGER:
        set.ul_T_pindir = direction;
        break;
    case US_PIN_ECHO:
        set.ul_E_pindir = direction;
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
        set.m_FL_pindir = direction;
        break;
    case FR_MOTOR_PIN:
        set.m_FR_pindir = direction;
        break;
    case BL_MOTOR_PIN:
        set.m_RL_pindir = direction;
        break;
    case BR_MOTOR_PIN:
        set.m_RR_pindir = direction;
        break;
    case A0:
        set.bat_pindir = direction;
        break;
    default:
        break;
    }
    sim_robot.unlock();
}
void digitalWrite(int pin, bool value) {
    sim_robot.lock();
    CodeState &set = sim_robot.get_code();
    
    switch (pin) {
    case LED_BUILTIN:
        if (set.led_pindir == INPUT) return;
        set.led_value = value;
        break;
    case US_PIN_TRIGGER:
        if (value == false) break;
        sim_robot.unlock();
        sim_robot.ultrasonic_trigger();
        sim_robot.lock();
        break;
    }
    sim_robot.unlock();
}
void pwmWrite(int pin, int micros) {
    sim_robot.lock();
    CodeState &set = sim_robot.get_code();
    double motor_percent = map(micros,1000,2000,-100,100);
    switch (pin) {
    case FL_MOTOR_PIN:
        if (set.m_FL_pindir == OUTPUT)
        set.m_FL_power = motor_percent;
        break;
    case FR_MOTOR_PIN:
        if (set.m_FR_pindir == OUTPUT)
        set.m_FR_power = motor_percent;
        break;
    case BL_MOTOR_PIN:
        if (set.m_RL_pindir == OUTPUT)
        set.m_RL_power = motor_percent;
        break;
    case BR_MOTOR_PIN:
        if (set.m_RR_pindir == OUTPUT)
        set.m_RR_power = motor_percent;
        break;
    default:
        break;
    }
    sim_robot.unlock();
}
uint16_t analogRead(int pin) {
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
    return ret;
}
bool digitalRead(int pin) {
    bool ret = 0;
    double cur_time = sim_robot.get_time();
    sim_robot.lock();
    CodeState &set = sim_robot.get_code();
    switch (pin){
    case US_PIN_ECHO:
        if (set.ul_E_pindir == OUTPUT) ret = false;
        ret = sim_robot.ultrasonic_response();
        break;
    default:
        ret = 0;
        break;
    }
    
    sim_robot.unlock();
    sim_robot.add_time(10.0/1000.0/1000.0);
    return ret;
}

void run_robot_code() {
    setup();
    
    sim_robot.add_time(10.0/1000.0);
    int last_int_time = 0;
        
    while(true) {
        double sim_time = sim_robot.get_time();
        double tar_time = sim_robot.get_target_time();

        if ( sim_time > last_int_time) {
            std::cout << "### Robot Thread Time: " << sim_time <<" ###" << std::endl;
            last_int_time = (int) sim_time + 1;
        }
        if (tar_time > sim_time) {
            loop();
            sim_robot.add_time(50.0/1000.0);
            // std::cout << "Robot Update" << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));
    }
    
}