
#include "state_orbit.h"


double Orbit::kepler(double eccentricity, double mean_anomaly, double tollerance) {
    double eccentric_anomaly = mean_anomaly;
    double error = 10000000000000000;
    double error_dash;

    while(error > tollerance) {
        error = eccentric_anomaly - eccentricity*sin(eccentric_anomaly) - mean_anomaly;
        error_dash = 1 - eccentricity*cos(eccentric_anomaly);
        eccentric_anomaly -= error / error_dash;
    }

    double theta = 2.0 * atan(sqrt((1 + eccentricity)/(1-eccentricity)) * tan(eccentric_anomaly/2.0));
    return theta;
}

Orbit::Orbit() : State() {}

Orbit::Orbit(ClassicalOrbitalElements new_object) : State() {
    objects.push_back(new_object);
}
Orbit::Orbit(std::vector<ClassicalOrbitalElements> new_objects) : State() {
    objects = new_objects;
}
Orbit::Orbit(const Orbit &other) : State(other) {
    objects = other.objects;
    path_samples = other.path_samples;
    solved = other.solved;

}
Orbit& Orbit::operator=(const Orbit &other) {
    State::operator=(other);
    objects = other.objects;
    path_samples = other.path_samples;
    solved = other.solved;
    return *this;
}

void Orbit::set_path_samples(int n) {
    path_samples = n;
}

void Orbit::solve_dynamics(Orbit &output) const {
}

