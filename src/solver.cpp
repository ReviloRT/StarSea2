
#include "solver.h"


template<class T_state> 
void Solver<T_state>::
set_step_dt(double step_dt) {
    this->_step_dt = step_dt;
}

template<class T_state> 
void Solver<T_state>::
set_end(double end) {
    this->_end = end;
}

template<class T_state> 
void Solver<T_state>::
set_t0(double t0) {
    this->_time = t0;
}

template<class T_state> 
void Solver<T_state>::
set_substeps(int substeps) {
    this->_substeps = substeps;
}

template<class T_state> 
double Solver<T_state>::
get_time() {
    return _time;
}

template<class T_state> 
void Solver<T_state>::
solve_step() {}

template<class T_state> 
void Solver<T_state>::
render(SDL_Renderer* sdlr) {}




template<class T_state> 
void StepSolver<T_state>::
set_step_dt(double step_dt) {
    Solver<T_state>::set_step_dt(step_dt);
    counter = 0;
    this->states.assign(this->_end/this->_step_dt*this->_substeps+2,T_state());
}
template<class T_state> 
void StepSolver<T_state>::
set_end(double end) {
    Solver<T_state>::set_end(end);
    counter = 0;
    this->states.assign(this->_end/this->_step_dt*this->_substeps+2,T_state());
}
template<class T_state> 
void StepSolver<T_state>::
set_substeps(int substeps) {
    Solver<T_state>::set_substeps(substeps);
    counter = 0;
    this->states.assign(this->_end/this->_step_dt*this->_substeps+2,T_state());
}
template<class T_state> 
void StepSolver<T_state>::
solve_step() {
    for (int i = 0; i < this->_substeps; i++) {
        if ((this->_end > 0) && (this->_time >= this->_end)) return;

        double subdt = this->_step_dt/this->_substeps;

        T_state &input = this->states[this->counter];
        T_state &output = this->states[this->counter+1];

        input.read_lock();
        output.write_lock();

        input.solve_next_state(output,subdt);
        this->_time += subdt;
        this->counter ++;

        output.write_unlock();
        input.read_unlock();
    }

}
template<class T_state> 
void StepSolver<T_state>::
render(SDL_Renderer* sdlr) {

    for (int i = 1; i < counter; i++) {
        this->states[i].read_lock();
        this->states[i].render(sdlr,states[i-1]);
        this->states[i].read_unlock();
    }

    // this->states[this->counter].read_lock();
    // this->states[this->counter].render(sdlr);
    // this->states[this->counter].read_unlock();

}
template<class T_state> 
void StepSolver<T_state>::
set_state(T_state state) {
    this->states[0] = state;
}






template<class T_integrator,class T_state> 
void IntegratorSolver<T_integrator,T_state>::
solve_dynamics() {
    T_state *input = &calc_states[curr_idxs[0]];
    T_state *output = &calc_states[curr_idxs[1]];
    
    double dt_remaining = this->_step_dt;
    if ((this->_end > 0) && (this->_time >= this->_end)) return;

    while (dt_remaining >= DBL_MIN) {
        int temp = curr_idxs[1];
        curr_idxs[1] = curr_idxs[2];
        curr_idxs[2] = temp;

        input->read_lock();
        output->write_lock();

        double dt = integrator.solve(*input,*output);

        output->write_unlock();
        input->read_unlock();

        this->_time += dt;
        dt_remaining -= dt;

        input = &calc_states[curr_idxs[2]];
        output = &calc_states[curr_idxs[1]];

    }

    idx_lock.lock();
    new_idxs[2] = curr_idxs[1];
    new_idxs[3] = curr_idxs[2];
    new_idxs[1] = curr_idxs[3];
    idx_lock.unlock();
}

template<class T_integrator,class T_state> 
void IntegratorSolver<T_integrator,T_state>::
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
void IntegratorSolver<T_integrator,T_state>::
render(SDL_Renderer* sdlr) {
    T_state &input = calc_states[curr_idxs[0]];

    input.read_lock();

    input.render(sdlr);

    input.read_unlock();
}

template<class T_integrator,class T_state> 
void IntegratorSolver<T_integrator,T_state>::
complete_step() {
    idx_lock.lock();
    curr_idxs = new_idxs;
    idx_lock.unlock();
    
}

template<class T_integrator,class T_state> 
void IntegratorSolver<T_integrator,T_state>::
set_integrator(T_integrator new_integrator) {
    integrator = new_integrator;
}

template<class T_integrator,class T_state> 
void IntegratorSolver<T_integrator,T_state>::
set_state(T_state state) {
    calc_states[0] = state;
    // calc_states.fill(state);
}

template<class T_integrator,class T_state> 
void IntegratorSolver<T_integrator,T_state>::
solve_step() {
    solve_dynamics();
    complete_step();
    solve_interactions();
    complete_step();
}




template class Solver<Orbit>;
template class StepSolver<Orbit>;

template class IntegratorSolver<IntegratorBase<State>,State>;
template class IntegratorSolver<Euler<State>,State>;
template class IntegratorSolver<Euler2ndOrder<State>,State>;
template class IntegratorSolver<RK4<State>,State>;
template class IntegratorSolver<RKN4<State>,State>;

template class IntegratorSolver<IntegratorBase<GravityPoints>,GravityPoints>;
template class IntegratorSolver<Euler<GravityPoints>,GravityPoints>;
template class IntegratorSolver<Euler2ndOrder<GravityPoints>,GravityPoints>;
template class IntegratorSolver<RK4<GravityPoints>,GravityPoints>;
template class IntegratorSolver<RKN4<GravityPoints>,GravityPoints>;
