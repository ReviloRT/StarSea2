#ifndef STATE_ORBIT_H
#define STATE_ORBIT_H

#include "state.h"
#include <math.h> 

struct ClassicalOrbitalElements {
    double semimajor_axis = 0;
    double eccentricity = 0;
    double inclination = 0;
    double longitude_accend = 0;
    double argument_periap = 0;
    double mean_anomaly = 0;
};

class Orbit : public State {
protected:
    std::vector<ClassicalOrbitalElements> objects;
    int path_samples = 1000;
    int solved = false;

    double kepler(double e, double M, double Tol);
public:
    Orbit();
    Orbit(ClassicalOrbitalElements new_object);
    Orbit(std::vector<ClassicalOrbitalElements> new_objects);
    Orbit(const Orbit &other);
    Orbit& operator=(const Orbit &other);

    void set_path_samples(int n);

    virtual void solve_dynamics(Orbit &output) const;
    // virtual void solve_interactions(Orbit &output) const;
    // virtual void render(SDL_Renderer* sdlr) const override;

};

#endif // STATE_ORBIT_H