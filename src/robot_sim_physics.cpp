
#include "robot_sim_physics.h"

#include "robot_sim.h"

void RobotPhysics::motors_to_angvels(double mPowers[4], double angVels[4]) {
    angVels[0] = mPowers[0] * sim_robot.model.motor_scale_FL / 10000.0 * sim_robot.model.max_rpm * 2.0 * PI * sim_robot.model.wheel_rad;
    angVels[1] = mPowers[1] * sim_robot.model.motor_scale_FR / 10000.0 * sim_robot.model.max_rpm * 2.0 * PI * sim_robot.model.wheel_rad;
    angVels[2] = mPowers[2] * sim_robot.model.motor_scale_RL / 10000.0 * sim_robot.model.max_rpm * 2.0 * PI * sim_robot.model.wheel_rad;
    angVels[3] = mPowers[3] * sim_robot.model.motor_scale_RR / 10000.0 * sim_robot.model.max_rpm * 2.0 * PI * sim_robot.model.wheel_rad;
}

void RobotPhysics::angvels_to_cartesian(double angVels[4], double vels[3]) {
    vels[0] = sim_robot.model.wheel_rad * (   angVels[0] + angVels[1] + angVels[2] + angVels[3]) / 4.0;
    vels[1] = sim_robot.model.wheel_rad * (   angVels[0] - angVels[1] - angVels[2] + angVels[3]) / 4.0;
    vels[2] = sim_robot.model.wheel_rad * ( - angVels[0] + angVels[1] - angVels[2] + angVels[3]) / 4.0 /  (sim_robot.model.wheel_length + sim_robot.model.wheel_width);
}

void RobotPhysics::motors_to_velocities(double vels[3]) {
    double motor_powers[4] = {sim_robot.code.m_FL_power,  sim_robot.code.m_FR_power, sim_robot.code.m_RL_power, sim_robot.code.m_RR_power};
    double angVels[4];
    motors_to_angvels(motor_powers,angVels);
    angvels_to_cartesian(angVels,vels);
}   

void RobotPhysics::motors_to_velocities(std::vector<double> &vels_vect) {
    double vels[3];
    motors_to_velocities(vels);
    vels_vect.assign(vels,vels+3); 
}

