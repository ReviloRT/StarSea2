#include "state_gravN.h"

GravityPoints::GravityPoints() : State() {}
GravityPoints::GravityPoints(std::vector<double> new_data) : State(new_data) {}
GravityPoints::GravityPoints(std::vector<double> new_data_0, std::vector<double> new_data_1) : State(new_data_0,new_data_1) {}
GravityPoints::GravityPoints(const GravityPoints &other) : State(other) {}
GravityPoints& GravityPoints::operator=(const GravityPoints &other) {
    State::operator=(other);
    return *this;
}

void GravityPoints::solve_deltas(GravityPoints &output) const {
    output<<*this;
    output.zero();
    for (int i = 0; i < numPoints(); i++) {
        output.data_0[id_to_index(i,0)] = this->data_1[id_to_index(i,0)];
        output.data_0[id_to_index(i,1)] = this->data_1[id_to_index(i,1)];
        output.data_0[id_to_index(i,2)] = this->data_1[id_to_index(i,2)];
        for (int j = 0; j < numPoints(); j++) {
            if (i == j) continue;
            double dx = this->data_0[id_to_index(i,0)] - this->data_0[id_to_index(j,0)];
            double dy = this->data_0[id_to_index(i,1)] - this->data_0[id_to_index(j,1)];
            double dz = this->data_0[id_to_index(i,2)] - this->data_0[id_to_index(j,2)];
            double dist = sqrt(dx*dx + dy*dy + dz*dz);
            double accel = grav_profile(dist)/dist;
            output.data_1[id_to_index(i,0)] += dx*accel;
            output.data_1[id_to_index(i,1)] += dy*accel;
            output.data_1[id_to_index(i,2)] += dz*accel;
        }
    }
    // std::cout << "GravP, solve dynamics: " << numPoints() << " x[0] = " << output.data_0[0] << std::endl;
}
void GravityPoints::solve_interactions(GravityPoints &output) const {
    output<<*this;
    output = *this;

}
void GravityPoints::render(SDL_Renderer* sdlr) const {

    // Set our color for the draw functions
    SDL_SetRenderDrawColor(sdlr, 255, 255, 255, 255);
    for (int i = 0; i < numPoints(); i++) {
        // Now we can draw our point
        int px = coord_to_px(data_0[id_to_index(i,0)]);
        int py = coord_to_py(data_0[id_to_index(i,1)]);
        
        SDL_Rect rect;
        rect.x = px-5;
        rect.y = py-5;
        rect.w = 10;
        rect.h = 10;
        SDL_RenderDrawRect(sdlr,&rect);

        // std::cout << "GravP, Render, (x,y) = " <<data_0[id_to_index(i,0)] << " " << data_0[id_to_index(i,1)] << std::endl;
    }
    // Set the color to what was before
    SDL_SetRenderDrawColor(sdlr, 0, 0, 0, 255);
    // .. you could do some other drawing here
    // And now we present everything we draw after the clear.
    
}

int GravityPoints::id_to_index(int id, int dim) {
    return id*3+dim;
}
int GravityPoints::index_to_id(int index) {
    return index/3;
}
double GravityPoints::grav_profile(double dist) {
    if (dist == 0) return 0;
    return -1.0/(dist*dist);
}

int GravityPoints::numPoints() const {
    return index_to_id(data_0.size());
}

