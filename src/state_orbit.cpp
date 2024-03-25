
#include "state_orbit.h"


double Orbit::kepler(double eccentricity, double mean_anomaly, double tollerance) const {
    double eccentric_anomaly = mean_anomaly;
    double error = 10000000000000000;
    double error_dash;

    while(abs(error) > tollerance) {
        error = eccentric_anomaly - eccentricity*sin(eccentric_anomaly) - mean_anomaly;
        error_dash = 1 - eccentricity*cos(eccentric_anomaly);
        eccentric_anomaly -= error / error_dash;

        // std::cout << "Error: " << error << ", " << eccentric_anomaly << ", " << error_dash << std::endl;
    }

    double theta = 2.0 * atan(sqrt((1 + eccentricity)/(1-eccentricity)) * tan(eccentric_anomaly/2.0));
    // std::cout << "Kepler Theta: " << theta <<  ", " << tan(eccentric_anomaly/2.0) << ", " << sqrt((1 + eccentricity)/(1-eccentricity)) << ", " << sqrt((1 + eccentricity)/(1-eccentricity)) * tan(eccentric_anomaly/2.0) << std::endl; 
    return theta;
}


Orbit::Orbit() : State() {}
Orbit::Orbit(OrbitalElements new_object) : State() {
    objects.push_back(new_object);
}
Orbit::Orbit(std::vector<OrbitalElements> new_objects) : State() {
    objects = new_objects;
}
Orbit::Orbit(const Orbit &other) : State(other) {
    *this = other;

}
Orbit& Orbit::operator=(const Orbit &other) {
    State::operator=(other);
    objects = other.objects;
    tolerance = other.tolerance;
    render_scale = other.render_scale;
    gravitational_param = other.gravitational_param;
    time = other.time;
    return *this;
}

void Orbit::set_tolerance(double tol) {
    tolerance = tol;
}
void Orbit::set_render_scale(double scale) {
    render_scale = scale;
}
void Orbit::set_gravitational_param(double mu) {
    gravitational_param = mu;
}

double Orbit::solve_next_state(Orbit &output, double dt) const {
    output = *this;
    for (size_t i = 0; i < objects.size(); i++) {
        OrbitalElements const &o = objects[i];
        double period = 2 * PI * sqrt(o.semimajor_axis*o.semimajor_axis*o.semimajor_axis / gravitational_param) / (24*60*60);
        double mean_anomaly = o.mean_anomaly + 2 * PI * (dt / period);
        double theta = kepler(o.eccentricity, mean_anomaly, tolerance);
        double radius = o.semimajor_axis * (1 - o.eccentricity * o.eccentricity) / (1 + o.eccentricity * cos(theta));
        
        // std::cout << "Solve Next State 1: " << period << ", " << mean_anomaly << ", " << radius << ", " << theta << ", " << std::endl;
        // std::cout << "Solve Next State 2: " << o.eccentricity << ", " << tolerance << ", " << o.semimajor_axis << ", " << dt << ", " << std::endl;

        output.objects[i].true_anomaly = theta;
        output.objects[i].mean_anomaly = mean_anomaly;
        output.objects[i].radius = radius;
        output.objects[i].x = radius * cos(theta);
        output.objects[i].y = radius * sin(theta);
        output.objects[i].solved = true;
    }
    output.time += dt;
    return dt;
}
void Orbit::render(SDL_Renderer* sdlr) const {

    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);

    for (size_t obji = 0; obji < objects.size(); obji++) {
        OrbitalElements const &o = objects[obji];

        if (o.solved == false) break;

        int px = coord_to_px(o.x*render_scale);
        int py = coord_to_py(o.y*render_scale);

        // std::cout << "pix points " << px << ", " << py << ", " << o.x << ", " << o.y << std::endl;
        
        SDL_Rect rect;
        rect.x = px-5;
        rect.y = py-5;
        rect.w = 10;
        rect.h = 10;
        SDL_RenderDrawRect(sdlr,&rect);
        
    }
    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
}
void Orbit::render(SDL_Renderer* sdlr, Orbit &last) const {

    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);

    for (size_t obji = 0; obji < objects.size(); obji++) {
        OrbitalElements const &o = objects[obji];
        OrbitalElements const &o_last = last.objects[obji];

        if (o_last.solved == false) continue;

        int px_last = coord_to_px(o_last.x*render_scale);
        int py_last = coord_to_py(o_last.y*render_scale);

        int px = coord_to_px(o.x*render_scale);
        int py = coord_to_py(o.y*render_scale);
        SDL_RenderDrawLine(sdlr, px, py, px_last, py_last);

        // std::cout << "pix points " << px << ", " << py << ", " << o.x*render_scale << ", " << o.y*render_scale << std::endl;


        
        
    }
    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
}


  