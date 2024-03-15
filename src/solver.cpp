
#include "solver.h"

template<class T_integrator,class T_state> 
void Solver<T_integrator,T_state>::
solve_dynamics() {
    T_state *input = &calc_states[curr_idxs[0]];
    T_state *output = &calc_states[curr_idxs[1]];
    
    double dt_remaining = _step_dt;
    if ((_end > 0) && (_time >= _end)) return;

    while (dt_remaining >= DBL_MIN) {
        std::swap(curr_idxs[1],curr_idxs[2]);

        input->read_lock();
        output->write_lock();

        double dt = integrator.solve(*input,*output);

        output->write_unlock();
        input->read_unlock();

        _time += dt;
        dt_remaining -= dt;

        input = &calc_states[curr_idxs[2]];
        output = &calc_states[curr_idxs[1]];

    }

    idx_lock.lock();
    new_idxs[1] = curr_idxs[2];
    new_idxs[3] = curr_idxs[1];
    new_idxs[2] = curr_idxs[3];
    idx_lock.unlock();
}

template<class T_integrator,class T_state> 
void Solver<T_integrator,T_state>::
solve_interactions() {
    T_state &input = calc_states[curr_idxs[3]];
    T_state &output = calc_states[curr_idxs[4]];

    input.read_lock();
    output.write_lock();

    input.solve_interactions(output);

    output.write_unlock();
    input.read_unlock();

    idx_lock.lock();
    new_idxs[0] = curr_idxs[4];
    new_idxs[4] = curr_idxs[0];
    idx_lock.unlock();
}

template<class T_integrator,class T_state> 
void Solver<T_integrator,T_state>::
render(SDL_Renderer* sdlr) {
    T_state &input = calc_states[curr_idxs[0]];

    input.read_lock();

    input.render(sdlr);

    input.read_unlock();
}

template<class T_integrator,class T_state> 
void Solver<T_integrator,T_state>::
complete_step() {
    idx_lock.lock();
    curr_idxs = new_idxs;
    idx_lock.unlock();
}

template<class T_integrator,class T_state> 
void Solver<T_integrator,T_state>::
set_integrator(T_integrator new_integrator) {
    integrator = new_integrator;
}
template<class T_integrator,class T_state> 
void Solver<T_integrator,T_state>::
set_state(T_state state) {
    calc_states[0] = state;
    // calc_states.fill(state);
}
template<class T_integrator,class T_state> 
void Solver<T_integrator,T_state>::
set_step_dt(double step_dt) {
    this->_step_dt = step_dt;
}
template<class T_integrator,class T_state> 
void Solver<T_integrator,T_state>::
set_end(double end) {
    this->_end = end;
}

template<class T_integrator,class T_state> 
double Solver<T_integrator,T_state>::
get_time() {
    return _time;
}

template class Solver<IntegratorBase<State>,State>;
template class Solver<Euler<State>,State>;
template class Solver<Euler2ndOrder<State>,State>;
template class Solver<RK4<State>,State>;
template class Solver<RKN4<State>,State>;

template class Solver<IntegratorBase<GravityPoints>,GravityPoints>;
template class Solver<Euler<GravityPoints>,GravityPoints>;
template class Solver<Euler2ndOrder<GravityPoints>,GravityPoints>;
template class Solver<RK4<GravityPoints>,GravityPoints>;
template class Solver<RKN4<GravityPoints>,GravityPoints>;

template class Solver<IntegratorBase<Orbit>,Orbit>;
template class Solver<Euler<Orbit>,Orbit>;
template class Solver<Euler2ndOrder<Orbit>,Orbit>;
template class Solver<RK4<Orbit>,Orbit>;
template class Solver<RKN4<Orbit>,Orbit>;
