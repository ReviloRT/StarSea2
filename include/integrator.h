#ifndef INTEGRATORBASE_H
#define INTEGRATORBASE_H

#include "parameters.h"
#include "state.h"
#include "state_gravN.h"
#include "state_orbit.h"

template<class T_state>
class IntegratorBase {
protected:
public:
    virtual double solve(T_state &stateIn, T_state &stateOut);
};

template<class T_state>
class NoIntegration : public IntegratorBase<T_state> {
protected:
public:
    virtual double solve(T_state &stateIn, T_state &stateOut);
};


template<class T_state>
class Euler : public IntegratorBase<T_state> {
protected:
    double dt = 1;
    T_state k1;
public:
    void set_dt(double dt);
    virtual double solve(T_state &stateIn, T_state &stateOut);
};

template<class T_state>
class Euler2ndOrder : public Euler<T_state> {
protected:
public:
    virtual double solve(T_state &stateIn, T_state &stateOut);
};

template<class T_state>
class RK4 : public Euler<T_state> {
protected:
    T_state k2;
    T_state k3;
    T_state k4;
    T_state y1;
    T_state y2;
    T_state y3;
public:
    virtual double solve(T_state &stateIn, T_state &stateOut);
};

template<class T_state>
class RKN4 : public RK4<T_state> {
protected:
public:
    virtual double solve(T_state &stateIn, T_state &stateOut);
};

#endif // INTEGRATORBASE_H