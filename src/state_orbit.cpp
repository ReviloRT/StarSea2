
#include "state_orbit.h"


double Orbit::kepler(double eccentricity, double mean_anomaly, double tollerance) const {
    double eccentric_anomaly = mean_anomaly;
    double error = 10000000000000000;
    double error_dash;

    while(abs(error) > tollerance) {
        error = eccentric_anomaly - eccentricity*sin(eccentric_anomaly) - mean_anomaly;
        error_dash = 1 - eccentricity*cos(eccentric_anomaly);
        eccentric_anomaly -= error / error_dash;

        // std::cout << "Error: " << error << std::endl;
    }

    double theta = 2.0 * atan(sqrt((1 + eccentricity)/(1-eccentricity)) * tan(eccentric_anomaly/2.0));
    return theta;
}

Orbit::Orbit() : State() {}

Orbit::Orbit(ClassicalOrbitalElements new_object) : State() {
    objects.push_back(new_object);
    set_path_samples(path_samples);
}
Orbit::Orbit(std::vector<ClassicalOrbitalElements> new_objects) : State() {
    objects = new_objects;
    set_path_samples(path_samples);
}
Orbit::Orbit(const Orbit &other) : State(other) {
    *this = other;

}
Orbit& Orbit::operator=(const Orbit &other) {
    State::operator=(other);
    objects = other.objects;
    path_samples = other.path_samples;
    solved = other.solved;
    tollerance = other.tollerance;
    render_scale = other.render_scale;
    return *this;
}

void Orbit::set_end_time(double end) {
    end_time = end;
    data_0.assign(objects.size()*end_time/dt*3,0);
    data_1.assign(objects.size()*end_time/dt*3,0);
}
void Orbit::set_dt(double new_dt) {
    dt = new_dt;
    data_0.assign(objects.size()*end_time/dt*3,0);
    data_1.assign(objects.size()*end_time/dt*3,0);
}
void Orbit::set_tollerance(double tol) {
    tollerance = tol;
}
void Orbit::set_render_scale(double scale) {
    render_scale = scale;
}

int Orbit::id_to_index(int obji, int id, int dim) const {
    return obji*path_samples*3 + id*3+dim;
}
int Orbit::index_to_obj(int index) const {
    return index/(path_samples*3);
}
int Orbit::index_to_id(int index) const {
    return (index - index_to_obj(index) * path_samples*3)/3;
}
int Orbit::index_to_dim(int index) const {
    return (index  - index_to_obj(index) * path_samples*3 - index_to_id(index)*3);
}
void Orbit::polarToCartesian(double rad, double theta, double &x, double &y) const {
    x = rad * sin(theta);
    y = rad * cos(theta);
}

void Orbit::solve_dynamics(Orbit &output) const {
    if (solved == path_samples) return;
    
    // std::cout << solved << ", " << path_samples << ", " << objects.size() << std::endl;

    output = *this;
    for (int i = 0; i < objects.size(); i++) {
        double mean_anomaly = objects[i].mean_anomaly + 2 * M_PI * solved / (path_samples-2);
        double eccentricty = objects[i].eccentricity;
        double theta = kepler(eccentricty, mean_anomaly, tollerance);
        double radius = objects[i].semimajor_axis * (1- eccentricty)  * (1- eccentricty) / (1 + eccentricty * cos(theta));
        output.data_0[id_to_index(i,solved,0)] = radius;
        output.data_0[id_to_index(i,solved,1)] = theta;
    }
    
    output.solved ++;
}
void Orbit::solve_interactions(Orbit &output) const {
    output = *this;

}
void Orbit::render(SDL_Renderer* sdlr) const {

    if (solved == 0) {return;}

    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);

    for (int obji = 0; obji < objects.size(); obji++) {

        double cart_x, cart_y; 
        double rad = data_0[id_to_index(obji,0,0)] * render_scale;
        double theta = data_0[id_to_index(obji,0,1)];
        polarToCartesian(rad, theta, cart_x, cart_y);

        int last_px = coord_to_px(cart_x);
        int last_py = coord_to_py(cart_y);
        int px, py;

        // std::cout << "First point at: " << rad << ", " << theta << ", " << px << ", " << py <<  std::endl;

        for (int i = 1; i < solved; i++) {
            rad = data_0[id_to_index(obji,i,0)]  * render_scale;
            theta = data_0[id_to_index(obji,i,1)];
            polarToCartesian(rad, theta, cart_x, cart_y);
            px = coord_to_px(cart_x);
            py = coord_to_py(cart_y);

            // std::cout << "Point at: " << rad << ", " << theta << ", " << px << ", " << py <<  std::endl;
            
            SDL_RenderDrawLine(sdlr, last_px, last_py, px, py);

            last_px = px;
            last_py = py;

        }
        
    }
    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
}


  