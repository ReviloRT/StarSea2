
#include "integrator.h"

template<class T_state>
double IntegratorBase<T_state>::solve(T_state &stateIn, T_state &stateOut, double timeIn){
    stateOut = stateIn;
    return 1;
}

template<class T_state>
void IntegratorBase<T_state>::set_dt(double new_dt) {
    std::cout << "Integrator dt set to: " << new_dt << std::endl;
    dt = new_dt;
}

template<class T_state>
double NoIntegration<T_state>::solve(T_state &stateIn, T_state &stateOut, double timeIn){
    stateIn.solve_deltas(stateOut, timeIn);
    return 1;
}

template<class T_state>
double Euler<T_state>::solve(T_state &stateIn, T_state &stateOut, double timeIn){
    // std::cout << "Euler Solve A: " <<  stateIn.get_state_time() << ", " <<  stateOut.get_state_time() << std::endl;

    stateIn.solve_deltas(this->k1, timeIn);
    // std::cout << "Euler Solve B: " <<  stateIn.get_state_time() << ", " <<  stateOut.get_state_time() << std::endl;

    stateOut = stateIn;
    // std::cout << "Euler Solve C: " <<  stateIn.get_state_time() << ", " <<  stateOut.get_state_time() << std::endl;

    // std::cout << "Euler, accel[0] = " << stateOut.data_1[0] <<", " << stateOut.data_1[1]<<", " << stateOut.data_1[2]  << std::endl;
    stateOut.pm(this->k1,this->dt);
    // std::cout << "Euler Solve D: " <<  this->dt << std::endl;

    return this->dt;
}

template<class T_state>
double Euler2ndOrder<T_state>::solve(T_state &stateIn, T_state &stateOut, double timeIn){
    stateIn.solve_deltas(this->k1, timeIn);
    stateOut = stateIn;
    stateOut.pm(this->k1,this->dt);
    stateOut.downpm(this->k1,this->dt*this->dt);
    return this->dt;
}

template<class T_state>
double RK4<T_state>::solve(T_state &stateIn, T_state &stateOut, double timeIn) {
    stateIn.solve_deltas(this->k1, timeIn);
    this->y1 = stateIn;
    this->y1.pm(this->k1,0.5*this->dt);
    this->y1.solve_deltas(this->k2, timeIn+0.5*this->dt);

    this->y2 = stateIn;
    this->y2.pm(this->k2,0.5*this->dt);
    this->y2.solve_deltas(this->k3, timeIn+0.5*this->dt);

    this->y3 = stateIn;
    this->y3.pm(this->k3,1*this->dt);
    this->y3.solve_deltas(this->k4, timeIn+this->dt);
    
    stateOut = stateIn;
    stateOut.pm(this->k1, 1.0/6.0*this->dt);
    stateOut.pm(this->k2, 1.0/3.0*this->dt);
    stateOut.pm(this->k3, 1.0/3.0*this->dt);
    stateOut.pm(this->k4, 1.0/6.0*this->dt);
    return this->dt;
}

template<class T_state>
double RKN4<T_state>::solve(T_state &stateIn, T_state &stateOut, double timeIn) {
    stateIn.solve_deltas(this->k1, timeIn);
    this->y1 = stateIn;
    this->y1.pm(this->k1,0.5*this->dt);
    this->y1.downpm(this->k1,0.125*this->dt*this->dt);
    this->y1.solve_deltas(this->k2, timeIn+0.5*this->dt);

    this->y2 = stateIn;
    this->y2.pm(this->k2,0.5*this->dt);
    this->y2.downpm(this->k2,0.125*this->dt*this->dt);
    this->y2.solve_deltas(this->k3, timeIn+0.5*this->dt);

    this->y3 = stateIn;
    this->y3.pm(this->k3,this->dt);
    this->y3.downpm(this->k3,this->dt*this->dt);
    this->y3.solve_deltas(this->k4, timeIn+this->dt);

    stateOut = stateIn;
    stateOut.pm(this->k1, 1.0/6.0*this->dt);
    stateOut.pm(this->k2, 1.0/3.0*this->dt);
    stateOut.pm(this->k3, 1.0/3.0*this->dt);
    stateOut.pm(this->k4, 1.0/6.0*this->dt);

    return this->dt;
}

template class IntegratorBase<State>;
template class NoIntegration<State>;
template class Euler<State>;
template class Euler2ndOrder<State>;
template class RK4<State>;
template class RKN4<State>;

template class IntegratorBase<GravityPoints>;
template class NoIntegration<GravityPoints>;
template class Euler<GravityPoints>;
template class Euler2ndOrder<GravityPoints>;
template class RK4<GravityPoints>;
template class RKN4<GravityPoints>;

template class IntegratorBase<Orbit>;
template class NoIntegration<Orbit>;
template class Euler<Orbit>;
template class Euler2ndOrder<Orbit>;
template class RK4<Orbit>;
template class RKN4<Orbit>;

template class IntegratorBase<RobotState>;
template class NoIntegration<RobotState>;
template class Euler<RobotState>;
template class Euler2ndOrder<RobotState>;
template class RK4<RobotState>;
template class RKN4<RobotState>;