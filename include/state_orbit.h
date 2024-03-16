#ifndef STATE_ORBIT_H
#define STATE_ORBIT_H

#include "state.h"
#include <math.h> 

# define M_PI 3.141592653589793238462643383279502884L

struct ClassicalOrbitalElements {
    double time_0 = 0;
    double semimajor_axis = 0;
    double eccentricity = 0;
    double inclination = 0;
    double longitude_accend = 0;
    double argument_periap = 0;
    double mean_anomaly = 0;

    void convertDegToRad() {
        inclination = inclination / 180.0 * M_PI;
        longitude_accend = longitude_accend / 180.0 * M_PI;
        argument_periap = argument_periap / 180.0 * M_PI;
        mean_anomaly = mean_anomaly / 180.0 * M_PI;
    }
};

class Orbit : public State {
private:
    std::vector<ClassicalOrbitalElements> objects;
    double end_time = 1;
    double dt = 1;
    double tollerance = 1;
    double render_scale = 1;

    double kepler(double e, double M, double Tol) const;
    int id_to_index(int obji, int id, int dim) const;
    int index_to_obj(int index) const;
    int index_to_id(int index) const;
    int index_to_dim(int index) const;

    void polarToCartesian(double rad, double theta, double &x, double &y) const;

public:
    Orbit();
    Orbit(ClassicalOrbitalElements new_object);
    Orbit(std::vector<ClassicalOrbitalElements> new_objects);
    Orbit(const Orbit &other);
    Orbit& operator=(const Orbit &other);

    void set_end_time(double end);
    void set_dt(double new_dt);
    void set_tollerance(double tol);
    void set_render_scale(double scale);

    virtual void solve_dynamics(Orbit &output) const;
    virtual void solve_interactions(Orbit &output) const;
    virtual void render(SDL_Renderer* sdlr) const override;

};

#endif // STATE_ORBIT_H