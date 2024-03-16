
#include "integrator.h"

template<class T_state>
double IntegratorBase<T_state>::solve(T_state &stateIn, T_state &stateOut){
    stateOut = stateIn;
    return 1;
}

template<class T_state>
double NoIntegration<T_state>::solve(T_state &stateIn, T_state &stateOut){
    stateIn.solve_deltas(stateOut);
    return 1;
}

template<class T_state>
void Euler<T_state>::set_dt(double new_dt) {
    dt = new_dt;
}

template<class T_state>
double Euler<T_state>::solve(T_state &stateIn, T_state &stateOut){
    stateIn.solve_deltas(this->k1);
    stateOut = stateIn;
    // std::cout << "Euler, accel[0] = " << stateOut.data_1[0] <<", " << stateOut.data_1[1]<<", " << stateOut.data_1[2]  << std::endl;
    stateOut.pm(this->k1,this->dt);
    return this->dt;
}

template<class T_state>
double Euler2ndOrder<T_state>::solve(T_state &stateIn, T_state &stateOut){
    stateIn.solve_deltas(this->k1);
    stateOut = stateIn;
    stateOut.pm(this->k1,this->dt);
    stateOut.downpm(this->k1,this->dt*this->dt);
    return this->dt;
}

template<class T_state>
double RK4<T_state>::solve(T_state &stateIn, T_state &stateOut) {
    stateIn.solve_deltas(this->k1);
    this->y1 = stateIn;
    this->y1.pm(this->k1,0.5*this->dt);
    this->y1.solve_deltas(this->k2);

    this->y2 = stateIn;
    this->y2.pm(this->k2,0.5*this->dt);
    this->y2.solve_deltas(this->k3);

    this->y3 = stateIn;
    this->y3.pm(this->k3,1*this->dt);
    this->y3.solve_deltas(this->k4);
    
    stateOut = stateIn;
    stateOut.pm(this->k1, 1.0/6.0*this->dt);
    stateOut.pm(this->k2, 1.0/3.0*this->dt);
    stateOut.pm(this->k3, 1.0/3.0*this->dt);
    stateOut.pm(this->k4, 1.0/6.0*this->dt);
    return this->dt;
}

template<class T_state>
double RKN4<T_state>::solve(T_state &stateIn, T_state &stateOut) {
    stateIn.solve_deltas(this->k1);
    this->y1 = stateIn;
    this->y1.pm(this->k1,0.5*this->dt);
    this->y1.downpm(this->k1,0.125*this->dt*this->dt);
    this->y1.solve_deltas(this->k2);

    this->y2 = stateIn;
    this->y2.pm(this->k2,0.5*this->dt);
    this->y2.downpm(this->k2,0.125*this->dt*this->dt);
    this->y2.solve_deltas(this->k3);

    this->y3 = stateIn;
    this->y3.pm(this->k3,this->dt);
    this->y3.downpm(this->k3,this->dt*this->dt);
    this->y3.solve_deltas(this->k4);

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