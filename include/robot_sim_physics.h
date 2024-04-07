#ifndef ROBOT_PHYSICS_H
#define ROBOT_PHYSICS_H

#include "ss_parameters.h"
#include "robot_sim_arena.h"
#include <vector>
#include <random>

// *POI*
// Define robot physical model (all in mm and rad)
struct RobotModel {
    double width = 190;
    double length = 160;
    double wheel_length = 177;
    double wheel_width = 155;
    double wheel_rad = 25;
    double motor_max_rpm = 2;
    double motor_time_constant = 0.4;

    double ulx = 60;
    double uly = 0;
    double ulrot = 0;
    double irS0x = 80;
    double irS0y = 0;
    double irS0rot = 0;
    double irS1x = -80;
    double irS1y = 0;
    double irS1rot = PI;
    double irL0x = 60;
    double irL0y = 60;
    double irL0rot = PI/2;
    double irL1x = -60;
    double irL1y = 45;
    double irL1rot = PI/2;

    double motor_scale_FL = -1.15;
    double motor_scale_FR = 1.08;
    double motor_scale_RL = -0.97;
    double motor_scale_RR = 0.93;
    double dead_zone_percent = 0.3;

    double short_ir_rand_theta = PI/45.0;
    double short_ir_rand_xy = 25;
    double short_ir_min = 40;
    double short_ir_max = 300;

    double long_ir_rand_theta = PI/45.0;
    double long_ir_rand_xy = 30;
    double long_ir_min = 100;
    double long_ir_max = 800;

    double ultra_rand_theta = PI/35.0;
    double ultra_rand_xy = 40;
    double ultra_min = 20;
    double ultra_max = 3000;
};


namespace RobotPhysics {

double dead_zone(double power);
double battery_charge(double power);
double non_linearity(double power);
double motor_adjustments(double power);

void motors_to_ang_vels(double const m_powers[4], double ang_vels[4]);
void motors_to_ang_accels(double const m_powers[4], double const ang_vels[4], double ang_accels[4]);

void angular_to_robot(double const angular[4], double robot[3]);
void robot_to_angular(double angular[4], double const robot[3]);

void robot_to_global(double const robot[3], double global[3], double const robot_in_global[3]);
void global_to_robot(double robot[3], double const global[3], double const robot_in_global[3]);

void motors_to_global_velocities(std::vector<double> &global_vels_vect, std::vector<double> const &global_pos_vect);
void motors_to_global_accelerations(std::vector<double> &global_accels_vect, std::vector<double> const &global_vels_vect, std::vector<double> const &global_pos_vect);

double rand_gaussian(double std_dev);
double line_length(Line line);
double pos_to_rand_dist(double pos[3], double rand_theta, double rand_xy, double min_val, double max_val);

double pos_to_short_ir_dist(double pos[3]);
double pos_to_long_ir_dist(double pos[3]);
double pos_to_ultra_dist(double pos[3]);

extern std::random_device random_device;
extern std::mt19937 random_generator;
extern std::normal_distribution<double> normal_dist;

};





#endif // ROBOT_PHYSICS_H