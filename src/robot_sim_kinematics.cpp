#include "robot_sim_kinematics.h"

#include "robot_sim.h"

RobotKinematics::RobotKinematics() : State() {
    data_0.assign(3,0);
    data_1.assign(3,0);
}
RobotKinematics::RobotKinematics(std::vector<double> new_data) : State(new_data) {
    data_0 = new_data;
    data_1.assign(3,0);
}
RobotKinematics::RobotKinematics(std::vector<double> new_data_0, std::vector<double> new_data_1) : State(new_data_0, new_data_1) {
    data_0 = new_data_0;
    data_1 = new_data_1;
}
RobotKinematics::RobotKinematics(const RobotKinematics &other) : State(other){

}
RobotKinematics& RobotKinematics::operator=(const RobotKinematics &other) {
    State::operator=(other);

    return *this;
}


void RobotKinematics::solve_deltas(RobotKinematics &output, double target_time) {
    output = *this;
    sim_robot.lock();
    sim_robot.change_robot_state(this);
    RobotPhysics::motors_to_velocities(output.data_0);
    // output.data_0 = output.data_1;
    // RobotPhysics::motors_to_accelerations(output.data_1);
    sim_robot.set_target_time(target_time);
    sim_robot.unlock();

}
void RobotKinematics::render(SDL_Renderer* sdlr) const {
    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);
    sim_robot.lock();

    sim_robot.arena.render(sdlr);
    drawRect(sdlr, data_0[0],data_0[1],sim_robot.model.width, sim_robot.model.length, data_0[2], ROBOT_RENDER_SCALE);
    // drawRect(sdlr, data_0[0]+model.irL0x,data_0[1]+model.irL0y,model.width/10, model.length/10, data_0[2]+model.irL0rot, ROBOT_RENDER_SCALE);

    sim_robot.unlock();
    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
    // std::cout << data_0[0] << ", " << data_0[1] << ", " << data_0[2] << std::endl;
}
void RobotKinematics::get_position(double pos[3]) const {
    pos[0] = data_0[0];
    pos[1] = data_0[1];
    pos[2] = data_0[2];
}
void RobotKinematics::get_velocity(double vel[3]) const {
    vel[0] = data_1[0];
    vel[1] = data_1[1];
    vel[2] = data_1[2];
}