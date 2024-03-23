#ifndef ROBOT_PHYSICS_H
#define ROBOT_PHYSICS_H

#include "ss_parameters.h"
#include <vector>


struct RobotModel {
    double width = 190;
    double length = 160;
    double wheel_length = 177;
    double wheel_width = 155;
    double wheel_rad = 25;
    double max_rpm = 2;

    double ulx = 0;
    double uly = 0;
    double ulrot = 0;
    double irS0x = 0;
    double irS0y = 0;
    double irS0rot = 0;
    double irS1x = 0;
    double irS1y = 0;
    double irS1rot = PI;
    double irL0x = 10;
    double irL0y = 10;
    double irL0rot = PI/2;
    double irL1x = -10;
    double irL1y = 10;
    double irL1rot = PI/2;
    double motor_scale_FL = 1;
    double motor_scale_FR = 1;
    double motor_scale_RL = 1;
    double motor_scale_RR = 1;
};


namespace RobotPhysics {

void motors_to_angvels(double mPowers[4], double angVels[4]);
void angvels_to_cartesian(double angVels[4], double vels[3]);
void motors_to_velocities(double vels[3]);
void motors_to_velocities(std::vector<double> &vels_vect);

};





#endif // ROBOT_PHYSICS_H