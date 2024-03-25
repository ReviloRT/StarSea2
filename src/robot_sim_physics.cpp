
#include "robot_sim_physics.h"

#include "robot_sim.h"

std::random_device RobotPhysics::random_device;
std::mt19937 RobotPhysics::random_generator{random_device()};
std::normal_distribution<double> RobotPhysics::normal_dist{0.0,1.0};


// Motor powers to motor speeds
void RobotPhysics::motors_to_ang_vels(double const m_powers[4], double ang_vels[4]) {
    double k =  sim_robot.model.motor_max_rpm * 2.0 * PI / 100.0;
    ang_vels[0] = m_powers[0] * sim_robot.model.motor_scale_FL * k;
    ang_vels[1] = m_powers[1] * sim_robot.model.motor_scale_FR * k;
    ang_vels[2] = m_powers[2] * sim_robot.model.motor_scale_RL * k;
    ang_vels[3] = m_powers[3] * sim_robot.model.motor_scale_RR * k;
}

void RobotPhysics::motors_to_ang_accels(double const m_powers[4], double const ang_vels[4], double ang_accels[4]) {
    double k = sim_robot.model.motor_max_rpm * 2.0 * PI / 100.0;
    double inv_tau = 1.0 / sim_robot.model.motor_time_constant;
    ang_accels[0] = ( m_powers[0] * sim_robot.model.motor_scale_FL * k - ang_vels[0]) * inv_tau;
    ang_accels[1] = ( m_powers[1] * sim_robot.model.motor_scale_FR * k - ang_vels[1]) * inv_tau;
    ang_accels[2] = ( m_powers[2] * sim_robot.model.motor_scale_RL * k - ang_vels[2]) * inv_tau;
    ang_accels[3] = ( m_powers[3] * sim_robot.model.motor_scale_RR * k - ang_vels[3]) * inv_tau;
}


// Mechanum transforms
void RobotPhysics::angular_to_robot(double const angular[4], double robot[3]) {
    double turn_const = 0.25 / (sim_robot.model.wheel_length + sim_robot.model.wheel_width);
    robot[0] = sim_robot.model.wheel_rad * (   angular[0] + angular[1] + angular[2] + angular[3]) * 0.25;
    robot[1] = sim_robot.model.wheel_rad * (   angular[0] - angular[1] - angular[2] + angular[3]) * 0.25;
    robot[2] = sim_robot.model.wheel_rad * ( - angular[0] + angular[1] - angular[2] + angular[3]) * turn_const;
}

void RobotPhysics::robot_to_angular(double angular[4], double const robot[3]) {
    double turn_const = (sim_robot.model.wheel_length + sim_robot.model.wheel_width);
    angular[0] = (robot[0] + robot[1] - robot[2] * turn_const) / sim_robot.model.wheel_rad;
    angular[1] = (robot[0] - robot[1] + robot[2] * turn_const) / sim_robot.model.wheel_rad;
    angular[2] = (robot[0] - robot[1] - robot[2] * turn_const) / sim_robot.model.wheel_rad;
    angular[3] = (robot[0] + robot[1] + robot[2] * turn_const) / sim_robot.model.wheel_rad;
}

// Robot_frame to global transforms (actually identical)
void RobotPhysics::robot_to_global(double const robot[3], double global[3], double const robot_in_global[3]) {
    global[0] = robot[0] * cos(robot_in_global[2]) - robot[1] * sin(robot_in_global[2]);
    global[1] = robot[0] * sin(robot_in_global[2]) + robot[1] * cos(robot_in_global[2]);
    global[2] = robot[2];
}

void RobotPhysics::global_to_robot(double robot[3], double const global[3], double const robot_in_global[3]) {
    robot[0] =   global[0] * cos(robot_in_global[2]) + global[1] * sin(robot_in_global[2]);
    robot[1] = - global[0] * sin(robot_in_global[2]) + global[1] * cos(robot_in_global[2]);
    robot[2] = global[2];
}   

void RobotPhysics::motors_to_global_velocities(std::vector<double> &global_vels_vect, std::vector<double> const &global_pos_vect) {
    double motor_powers[4] = {sim_robot.code.m_FL_power,  sim_robot.code.m_FR_power, sim_robot.code.m_RL_power, sim_robot.code.m_RR_power};
    double ang_vels[4], robot_vels[3], global_vels[3];
    motors_to_ang_vels(motor_powers,ang_vels);
    angular_to_robot(ang_vels,robot_vels);
    robot_to_global(robot_vels,global_vels,global_pos_vect.data());
    global_vels_vect.assign(global_vels,global_vels+3); 
}


void RobotPhysics::motors_to_global_accelerations(std::vector<double> &global_accels_vect, std::vector<double> const &global_vels_vect, std::vector<double> const &global_pos_vect) {
    double motor_powers[4] = {sim_robot.code.m_FL_power,  sim_robot.code.m_FR_power, sim_robot.code.m_RL_power, sim_robot.code.m_RR_power};
    double ang_accels[4], ang_vels[4], robot_accels[3], robot_vels[3], global_accels[3];
    global_to_robot(robot_vels, global_vels_vect.data(), global_pos_vect.data());
    robot_to_angular(ang_vels, robot_vels);
    motors_to_ang_accels(motor_powers,ang_vels,ang_accels);
    angular_to_robot(ang_accels,robot_accels);
    robot_to_global(robot_accels,global_accels,global_pos_vect.data());
    global_accels_vect.assign(global_accels,global_accels+3); 
} 


double RobotPhysics::rand_gaussian(double std_dev) {
    return RobotPhysics::normal_dist(RobotPhysics::random_generator)*std_dev;
}

double RobotPhysics::line_length(Line line) {
    double dx = line.x2 - line.x1;
    double dy = line.y2 - line.y1;
    return sqrt(dx*dx + dy*dy);
}

double RobotPhysics::pos_to_rand_dist(double pos[3], double rand_theta, double rand_xy, double min_val, double max_val) {
    double intersect[2] = {0,0};
    double pos_rand_theta[3] = {pos[0], pos[1], pos[2]+rand_gaussian(1)*rand_theta};
    double dist = sim_robot.arena.get_intersect(pos_rand_theta,intersect);
    double clamp_scaling =  (dist > max_val ? max_val/dist : 1) * (dist < min_val ? min_val/dist : 1);
    intersect[0] = pos[0] + (intersect[0] - pos[0]) * clamp_scaling;
    intersect[1] = pos[1] + (intersect[1] - pos[1]) * clamp_scaling;
    intersect[0] += rand_gaussian(rand_xy) * dist / max_val;
    intersect[1] += rand_gaussian(rand_xy) * dist / max_val;
    Line line = {pos[0],pos[1], intersect[0], intersect[1]};
    sim_robot.render_measurements.push_back(line);
    return line_length(line);
}


double RobotPhysics::pos_to_short_ir_dist(double pos[3]) {
    return pos_to_rand_dist(pos, sim_robot.model.short_ir_rand_theta, sim_robot.model.short_ir_rand_xy, sim_robot.model.short_ir_min, sim_robot.model.short_ir_max);
}

double RobotPhysics::pos_to_long_ir_dist(double pos[3]) {
    return pos_to_rand_dist(pos, sim_robot.model.long_ir_rand_theta, sim_robot.model.long_ir_rand_xy, sim_robot.model.long_ir_min, sim_robot.model.long_ir_max);
}

double RobotPhysics::pos_to_ultra_dist(double pos[3]) {
    return pos_to_rand_dist(pos, sim_robot.model.ultra_rand_theta, sim_robot.model.ultra_rand_xy, sim_robot.model.ultra_min, sim_robot.model.ultra_max);
}
