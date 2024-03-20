#include "state_robot.h"


void Arena::add_wall(Wall wall) {
    walls.push_back(wall);
}
void Arena::add_wall(double x1, double y1, double x2, double y2) {
    Wall wall = {x1,y1,x2,y2};
    walls.push_back(wall);
}
double Arena::get_distance(double x, double y, double rot) const {
    
    double min_distance = 1000000000000000000;
    for (auto w : walls){
        // s*(w.x2-w.x1) - t*cos(rot) = x-w.x1;
        // s*(w.y2-w.y1) - t*sin(rot) = y-w.y1;

        if (cos(rot)*(w.y2-w.y1)-sin(rot)*(w.x2-w.x1) == 0) continue;
        
        double t,s;

        t = ((w.x2-w.x1)*(y-w.y1)-(x-w.x1)*(w.y2-w.y1))/(cos(rot)*(w.y2-w.y1)-sin(rot)*(w.x2-w.x1));
        
        if ((w.x2-w.x1) != 0) {
            s = (x- w.x1 + t*cos(rot))/(w.x2-w.x1);
        } else if ((w.y2-w.y1) != 0) {
            s = (y - w.y1 + t*sin(rot))/(w.y2-w.y1);
        } else continue;

        if ((s >= 0) && (t <= 1)) {
            double dist = sqrt(s*s + t*t);
            if (dist < min_distance) min_distance = dist;
        }
    }

    return min_distance;    
    
}
void Arena::render(SDL_Renderer* sdlr) const {
    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);
    for (auto w : walls) {
        int px1 = coord_to_px(w.x1*ROBOT_RENDER_SCALE);
        int py1 = coord_to_py(w.y1*ROBOT_RENDER_SCALE);
        int px2 = coord_to_px(w.x2*ROBOT_RENDER_SCALE);
        int py2 = coord_to_py(w.y2*ROBOT_RENDER_SCALE);
        SDL_RenderDrawLine(sdlr,px1,py1,px2,py2);
        // std::cout << "double (" << w.x1 << " , " << w.y1 << "), (" << w.x2 << " , " << w.y2 << ")" << "                 " <<std::endl;
        // std::cout << "px     (" << px1 << " , " << py1 << "), (" << px2 << " , " << py2 << ")" << "                 " <<std::endl;
    }
    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
}



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
    arena = other.arena;
    model = other.model;
    set = other.set;
}
RobotState& RobotState::operator=(const RobotState &other) {
    State::operator=(other);
    arena = other.arena;
    model = other.model;
    set = other.set;
    return *this;
}

void RobotState::set_arena(Arena *a_ptr) {
    arena = a_ptr;
}
void RobotState::set_model(PhysicalRobot *m_ptr) {
    model = m_ptr;
}

unsigned long RobotState::ultrasonic_pulse() {
    if (set.ul_T_pindir == INPUT) return 0;
    if (set.ul_E_pindir == OUTPUT) return 0;
    
    double ul_x = data_0[0] + cos(data_0[2]) * model->ulx - sin(data_0[2]) * model->uly;  
    double ul_y = data_0[1] - sin(data_0[2]) * model->uly + cos(data_0[2]) * model->ulx;  
    double ul_rot = data_0[2] + model->ulrot;
    double dist = arena->get_distance(ul_x,ul_y,ul_rot);
    double measured = dist*58.0;

    return measured;
}
double RobotState::get_infrared(int pin) {
    if (set.gyro_pindir == OUTPUT) return 0;
    double model_ir_x, model_ir_y, model_ir_rot, scale, exponent;
    switch (pin) {
    case IR_SHORT_PIN_0:
        model_ir_x = model->irS0x;
        model_ir_y = model->irS0y;
        model_ir_rot = model->irS0rot;
        scale = IR_SHORT_SCALE_DATASHEET;
        exponent = IR_SHORT_EXPONENT_DATASHEET;
        break;
    case IR_SHORT_PIN_1:
        model_ir_x = model->irS1x;
        model_ir_y = model->irS1y;
        model_ir_rot = model->irS1rot;
        scale = IR_SHORT_SCALE_DATASHEET;
        exponent = IR_SHORT_EXPONENT_DATASHEET;
        break;
    case IR_LONG_PIN_0:
        model_ir_x = model->irL0x;
        model_ir_y = model->irL0y;
        model_ir_rot = model->irL0rot;
        scale = IR_LONG_SCALE_DATASHEET;
        exponent = IR_LONG_EXPONENT_DATASHEET;
        break;
    case IR_LONG_PIN_1:
        model_ir_x = model->irL1x;
        model_ir_y = model->irL1y;
        model_ir_rot = model->irL1rot;
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
    
    double ir_x = data_0[0] + cos(data_0[2]) * model_ir_x - sin(data_0[2]) * model_ir_y;  
    double ir_y = data_0[1] - sin(data_0[2]) * model_ir_y + cos(data_0[2]) * model_ir_x;  
    double ir_rot = data_0[2] + model_ir_rot;
    double dist = arena->get_distance(ir_x,ir_y,ir_rot);
    double measured = pow(dist/scale,- exponent);
    
    return measured;
}
double RobotState::get_gyro() {
    if (set.gyro_pindir == OUTPUT) return 0;
    return map(data_1[2],-250,250,1023*0.5/5.0,1023.0*4.5/5.0);
}
double RobotState::get_battery() {
    if (set.bat_pindir == OUTPUT) return 0;
    return map(set.battery_percent,0,100,720,860);
}
void RobotState::motors_to_delta(RobotState &other) const {
    other.data_0[0] = 0.25 * 0.01 *  ( set.m_FL_power + set.m_FR_power + set.m_RL_power + set.m_RR_power);
    other.data_0[1] = 0.25 * 0.01 *  ( set.m_FL_power - set.m_FR_power - set.m_RL_power + set.m_RR_power);
    other.data_0[2] = 0.25 * 0.01 * 1/ (model->wheelf + model->wheels) * (-set.m_FL_power + set.m_FR_power - set.m_RL_power + set.m_RR_power);
}

void RobotState::solve_deltas(RobotState &output, double target_time) {
    robot = this;
    output = *this;

    // std::cout << " Solve Deltas A: " << this->get_state_time() << ", " << robot->get_state_time() << ", " << output.get_state_time() << std::endl;

    if (robot->is_stuck()) robot->add_code_time(max(target_time - robot->get_state_time(),0));

    while (robot->get_state_time() < target_time) {

        std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();

        if (robot->get_robot_set().setup_complete == false) {
            setup();
            robot->get_robot_set().setup_complete = true;
        } else {
            loop();
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        
        uint64_t nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();
        // uint64_t time_on_mega = nanos*(4000.0/20.0);
        uint64_t time_on_mega = nanos*(4000.0/20.0);
        robot->state_time += (double)time_on_mega/1000.0/1000.0/1000.0;
    }
    robot->motors_to_delta(output);
    // std::cout << " Solve Deltas B: " << this->get_state_time() << ", " << robot->get_state_time() << ", " << output.get_state_time() << std::endl;

}

void RobotState::render(SDL_Renderer* sdlr) const {
    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);
    SDL_Rect rect;
    rect.x = coord_to_px((data_0[0] - model->width/2.0)*ROBOT_RENDER_SCALE);
    rect.y = coord_to_py((data_0[1] - model->length/2.0)*ROBOT_RENDER_SCALE);
    rect.w = length_to_px(model->width * ROBOT_RENDER_SCALE);
    rect.h = length_to_py(model->length * ROBOT_RENDER_SCALE);
    SDL_RenderDrawRect(sdlr,&rect);
    arena->render(sdlr);
    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
}

double RobotState::get_state_time() {
    return state_time;
}
void RobotState::add_code_time(double time) {
    state_time + time;
}
void RobotState::stuck(bool stuck) {
    set.stuck = stuck;
}
bool RobotState::is_stuck() {
    return set.stuck;
}
CodeState& RobotState::get_robot_set() {
    return set;
}

// double pow(double raw, double exponent) {
//     return std::pow(raw,exponent);
// }
double map(double value, double low_in, double high_in, double low_out, double high_out) {
    return (value - low_in) * (high_out - low_out)/(high_in - low_in) + low_out;
}
double min(double in1, double in2) {
    return std::min(in1,in2);
}
double max(double in1, double in2) {
    return std::max(in1,in2);
}

uint64_t micros() {
    return robot->get_state_time()*1000.0*1000.0;
}
uint64_t millis() {
    return robot->get_state_time()*1000.0;
}
void delay(int millis) {
    robot->add_code_time(((double)millis)/1000.0);
}
void delayMicroseconds(int micros) {
    robot->add_code_time(((double)micros)/1000.0/1000.0);
}
void infiniteWhile() {
    robot->stuck(true);
}


void pinMode(int pin, bool direction) {
    CodeState &set = robot->get_robot_set();
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
}
void digitalWrite(int pin, bool value) {
    CodeState &set = robot->get_robot_set();
    if (pin == LED_BUILTIN) {
        if (set.led_pindir == INPUT) return;
        set.led_value = value;
    }
}
void pwmWrite(int pin, int micros) {
    CodeState &set = robot->get_robot_set();
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
}
uint16_t analogRead(int pin) {
    CodeState &set = robot->get_robot_set();
    switch (pin){
    case GYROSCOPE_PIN:
        return robot->get_gyro();
    case IR_SHORT_PIN_0:
        return robot->get_infrared(pin);
    case IR_SHORT_PIN_1:
        return robot->get_infrared(pin);
    case IR_LONG_PIN_0:
        return robot->get_infrared(pin);
    case IR_LONG_PIN_1:
        return robot->get_infrared(pin);
    case T0_PIN:
        break;
    case T1_PIN:
        break;
    case T2_PIN:
        break;
    case T3_PIN:
        break;
    case A0:
        return robot->get_battery();
    default:
        break;
    }
    return 0;
}



