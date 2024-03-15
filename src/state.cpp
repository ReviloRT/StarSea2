
#include "state.h"


State::State() {

}
State::State(std::vector<double> new_data) {
    data_0 = new_data;
    order = 1;
}
State::State(std::vector<double> new_data_0, std::vector<double> new_data_1) {
    data_0 = new_data_0;
    data_1 = new_data_1;
    order = 2;
}
State::State(const State &other) {
    data_0 = other.data_0;
    data_1 = other.data_1;
    order = other.order;

}

State& State::operator=(const State &other) {
    data_0 = other.data_0;
    data_1 = other.data_1;
    order = other.order;
    return *this;
}
State& State::operator<<(const State &other) {
    if (this->data_0.size() != other.data_0.size()) {
        this->data_0.assign(other.data_0.size(),0);
    };
    if (this->data_1.size() != other.data_1.size()) {
        this->data_1.assign(other.data_1.size(),0);
    };
    order = other.order;
}
State& State::zero() {
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] = 0;
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] = 0;
}

State& State::operator+=(const State &other) {
    if (this->order != other.order) return *this;
    if (this->data_0.size() != other.data_0.size()) return *this;
    if (this->data_1.size() != other.data_1.size()) return *this;

    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] += other.data_0[i];
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] += other.data_1[i];
    
    return *this;
    
}
State& State::operator-=(const State &other) {
    if (this->order != other.order) return *this;
    if (this->data_0.size() != other.data_0.size()) return *this;
    if (this->data_1.size() != other.data_1.size()) return *this;

    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] -= other.data_0[i];
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] -= other.data_1[i];
    
    return *this;
}
State& State::operator*=(const State &other) {
    if (this->order != other.order) return *this;
    if (this->data_0.size() != other.data_0.size()) return *this;
    if (this->data_1.size() != other.data_1.size()) return *this;

    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] *= other.data_0[i];
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] *= other.data_1[i];
    
    return *this;
}
State& State::operator/=(const State &other) {
    if (this->order != other.order) return *this;
    if (this->data_0.size() != other.data_0.size()) return *this;
    if (this->data_1.size() != other.data_1.size()) return *this;

    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] /= other.data_0[i];
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] /= other.data_1[i];
    
    return *this;
}

State& State::operator+=(double val) {
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] += val;
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] += val;

    return *this;
}
State& State::operator-=(double val) {
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] -= val;
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] -= val;

    return *this;
}
State& State::operator*=(double val) {
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] *= val;
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] *= val;

    return *this;
}
State& State::operator/=(double val) {
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] /= val;
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] /= val;

    return *this;
}

State& State::pm(const State &other, double val) {
    if (this->data_0.size() != other.data_0.size()) {
        std::cout << "Error pm data_0" << std::endl;
        return *this;
    }
    if (this->data_1.size() != other.data_1.size()) {
        std::cout << "Error pm data_1" << std::endl;
        return *this;
    }
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] += other.data_0[i]*val;
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] += other.data_1[i]*val;
    return *this;
}
State& State::mm(const State &other, double val) {
    if (this->data_0.size() != other.data_0.size()) {
        std::cout << "Error pm data_0" << std::endl;
        return *this;
    }
    if (this->data_1.size() != other.data_1.size()) {
        std::cout << "Error pm data_1" << std::endl;
        return *this;
    }
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] -= other.data_0[i]*val;
    for (int i = 0; i < this->data_1.size(); i++) this->data_1[i] -= other.data_1[i]*val;
    return *this;
}
State& State::downpm(const State &other, double val) {
    if (this->data_0.size() != other.data_1.size()) return *this;
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] += other.data_1[i]*val;
    return *this;
}
State& State::downmm(const State &other, double val) {
    if (this->data_0.size() != other.data_1.size()) return *this;
    for (int i = 0; i < this->data_0.size(); i++) this->data_0[i] -= other.data_1[i]*val;
    return *this;
}

void State::read_lock() {
    change_lock.lock(); // Wait for a write to finish
    counter_lock.lock();  // Ensure we can increment read counter
    read_counter ++; 
    counter_lock.unlock();
    change_lock.unlock();
}
void State::read_unlock() {
    counter_lock.lock(); // Ensure we can deincrement read counter
    read_counter --;
    counter_lock.unlock();
}
void State::write_lock() {
    change_lock.lock();
    while (true) {
        counter_lock.lock();
        if (read_counter == 0) break;
        counter_lock.unlock();
    }
    counter_lock.unlock();
}
void State::write_unlock() {
    change_lock.unlock();
}

void State::solve_dynamics(State &output) const {
    output = *this;

}
void State::solve_interactions(State &output) const {
    output = *this;

}
void State::render(SDL_Renderer* sldr) const {
    std::cout << "Rendered DataArray!" << std::endl;
}


