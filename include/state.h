#ifndef STATE_H
#define STATE_H

#include "ss_parameters.h"
#include "helper.h"

#include <vector>
#include <mutex>
#include <semaphore>
#include <iostream>
#include "SDL.h"
#include "SDL_image.h"

class State {
private:
    std::mutex change_lock;
    std::mutex counter_lock;
    int read_counter = 0;

public:
    std::vector<double> data_0;
    std::vector<double> data_1;
    int order = 0;

    double state_time = 0;
    
public:
    State();
    State(std::vector<double> new_data);
    State(std::vector<double> new_data_0, std::vector<double> new_data_1);
    State(const State &other);
    
    State& operator=(const State &other);
    State& operator<<(const State &other);
    State& zero();

    State& operator+=(const State &other);
    State& operator-=(const State &other);
    State& operator*=(const State &other);
    State& operator/=(const State &other);

    State& operator+=(double val);
    State& operator-=(double val);
    State& operator*=(double val);
    State& operator/=(double val);

    State& pm(const State &other, double val);
    State& mm(const State &other, double val);
    State& downpm(const State &other, double val);
    State& downmm(const State &other, double val);

    void read_lock();
    void read_unlock();
    void write_lock();
    void write_unlock();

    double get_state_time() const;

    virtual double solve_next_state(State &output, double param) const;
    virtual void solve_deltas(State &output, double time) const;
    virtual void solve_interactions(State &output) const;
    virtual void render(SDL_Renderer* sdlr) const;
};


#endif // STATE_H