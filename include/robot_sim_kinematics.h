#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "ss_parameters.h"
#include "state.h"
#include "helper.h"



class RobotKinematics : public State {
private:

public:
    // Initialisation functions
    RobotKinematics();
    RobotKinematics(std::vector<double> new_data);
    RobotKinematics(std::vector<double> new_data_0, std::vector<double> new_data_1);
    RobotKinematics(const RobotKinematics &other);
    RobotKinematics& operator=(const RobotKinematics &other);
    
    // Main controlled functions
    void solve_deltas(RobotKinematics &output, double time);
    virtual void render(SDL_Renderer* sdlr) const override;

    void get_position(double pos[3]) const ;
    void get_velocity(double vel[3]) const ;
    
};



#endif // ROBOT_STATE_H