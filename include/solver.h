#ifndef SOLVER_H
#define SOLVER_H

#include "parameters.h"
#include "state.h"
#include "state_gravN.h"
#include "state_orbit.h"
#include "integrator.h"
#include <algorithm>
#include <array>

template<class T_integrator,class T_state>
class Solver {
private:

    double _time = 0;
    double _step_dt = 0;
    double _end = 0;

    T_integrator integrator;

    std::array<T_state,5> calc_states;
    std::array<int,5> curr_idxs = {0,1,2,3,4};
    std::array<int,5> new_idxs = {0,1,2,3,4};
    std::mutex idx_lock;

public:

    void set_integrator(T_integrator integrator);
    void set_state(T_state state);
    void set_step_dt(double step_dt);
    void set_end(double end);
    double get_time();

    void solve_dynamics();
    void solve_interactions();
    void render(SDL_Renderer* sdlr);
    void complete_step();

   
};

#endif // SOLVER_H