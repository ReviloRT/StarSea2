#ifndef SOLVER_H
#define SOLVER_H

#include "ss_parameters.h"
#include "state.h"
#include "state_gravN.h"
#include "state_orbit.h"
#include "state_robot.h"
#include "integrator.h"
#include <algorithm>
#include <array>

template<class T_state>
class Solver {
protected:

    double _time = 0;
    double _step_dt = 1;
    double _end = 0;
    int _substeps = 1;

public:
    virtual void set_step_dt(double step_dt);
    virtual void set_end(double end);
    virtual void set_t0(double t0);
    virtual void set_substeps(int substeps);
    virtual double get_time();
    

    virtual void solve_step();
    virtual void render(SDL_Renderer* sdlr);

};

template<class T_state>
class StepSolver : public Solver<T_state> {
protected:
    int counter = 0;
    std::vector<T_state> states;

public:
    void set_step_dt(double step_dt);
    void set_end(double end);
    void set_substeps(int substeps);
    void set_state(T_state state);

    void solve_step();
    void render(SDL_Renderer* sdlr);

};

template<class T_integrator,class T_state>
class IntegratorSolver : public Solver<T_state> {
protected:

    T_integrator integrator;

    std::array<T_state,5> calc_states;
    std::array<int,5> curr_idxs = {0,1,2,3,4};
    std::array<int,5> new_idxs = {0,1,2,3,4};
    std::mutex idx_lock;

public:

    void set_integrator(T_integrator integrator);
    void set_state(T_state state);

    void solve_step();

    void solve_dynamics();
    void solve_interactions();
    void render(SDL_Renderer* sdlr);
    void complete_step();

};

#endif // SOLVER_H