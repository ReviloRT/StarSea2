#ifndef STATE_ORBIT_H
#define STATE_ORBIT_H

#include "state.h"
#include <math.h> 

# define PI 3.141592653589793238462643383279502884L

struct OrbitalElements {
    double semimajor_axis = 0;
    double eccentricity = 0;
    double inclination = 0;
    double longitude_accend = 0;
    double argument_periap = 0;
    double true_anomaly = 0;

    double radius = 0;
    double mean_anomaly = 0;

    double x = 0;
    double y = 0;
    double z = 0;
    double dx = 0;
    double dy = 0;
    double dz = 0;

    bool solved = false;

    void convertDegToRad() {
        inclination = inclination / 180.0 * PI;
        longitude_accend = longitude_accend / 180.0 * PI;
        argument_periap = argument_periap / 180.0 * PI;
        true_anomaly = true_anomaly / 180.0 * PI;
        mean_anomaly = mean_anomaly / 180.0 * PI;
    }
};

class Orbit : public State {
public:
    double render_scale = 1;
    double time = 0;
    double tolerance = 1;
    double gravitational_param = 1;
    std::vector<OrbitalElements> objects;

    double kepler(double e, double M, double Tol) const;

public:
    Orbit();
    Orbit(OrbitalElements new_object);
    Orbit(std::vector<OrbitalElements> new_objects);
    Orbit(const Orbit &other);
    Orbit& operator=(const Orbit &other);

    void set_tolerance(double tol);
    void set_render_scale(double scale);
    void set_gravitational_param(double mu);

    double solve_next_state(Orbit &output, double dt) const;
    virtual void render(SDL_Renderer* sdlr) const override;
    void render(SDL_Renderer* sdlr, Orbit &last) const;

};

#endif // STATE_ORBIT_H