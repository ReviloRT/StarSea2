
#include "robot_sim_arena.h"


void Arena::add_line(Line wall) {
    this->walls.push_back(wall);
}
void Arena::add_line(double x1, double y1, double x2, double y2) {
    Line wall = {x1,y1,x2,y2};
    this->walls.push_back(wall);
}
double Arena::get_distance(double pos[3]) const {

    double min_distance = 1000000000000000000;
    for (auto w : walls){
        // Parametric Equation
        // s*(x2-x1) - t*(x4-x3) = (x3-x1);
        // s*(y2-y1) - t*(y4-y3) = (y3-y1);

        // Simplification
        // (x4 - x3) = cos(rot)     (b1)
        // (y4 - y4) = sin(rot)     (b2)

        // Crammers Rule
        // for a1*x + b1*y = c1, a2*x + b2*y = c2
        // x = (c1*b2 - c2*b1) / (a1*b2 - a2*b1)
        // y = (a1*c2 - a2*c1) / (a1*b2 - a2*b1)

        // Applied Crammers
        // s = ((x3-x1)*sin(rot) - (y3-y1)*cos(rot)) / ((x2-x1)*sin(rot) - (y2-y1)*cos(rot))
        // t = ((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1)) / ((x2-x1)*sin(rot) - (y2-y1)*cos(rot))

        // Parallel check
        // ((x2-x1)*sin(rot) - (y2-y1)*cos(rot)) == 0

        // Equivalences
        // x1 = w.x1, y1 = w.y1
        // x2 = w.x2, y2 = w.y2
        // x3 = x, y3 = y

        double denominator = (w.x2-w.x1)*sin(pos[2]) - (w.y2-w.y1)*cos(pos[2]);
        if (denominator == 0) continue;
        double s = ((pos[0]-w.x1)*sin(pos[2]) - (pos[1]-w.y1)*cos(pos[2])) / denominator;
        double t = ((w.x2-w.x1)*(pos[1]-w.y1) - (w.y2-w.y1)*(pos[0]-w.x1)) / denominator;
                
        if ((s >= 0) && (s <= 1) && (t <= 0)) {
            double dist = -t;
            if (dist < min_distance) min_distance = dist;
        }
    }

    return min_distance;    
    
}
double Arena::get_intersect(double pos[3], double intersect[2]) const {
    double dist = get_distance(pos);
    intersect[0] = pos[0] + dist*cos(pos[2]);
    intersect[1] = pos[1] + dist*sin(pos[2]);
    return dist;

}
void Arena::render(SDL_Renderer* sdlr) const {
    for (size_t i = 0; i < this->walls.size(); i++) {
        const Line &w = walls[i];
        int px1 = coord_to_px(w.x1*RENDER_SCALE);
        int py1 = coord_to_py(w.y1*RENDER_SCALE);
        int px2 = coord_to_px(w.x2*RENDER_SCALE);
        int py2 = coord_to_py(w.y2*RENDER_SCALE);
        SDL_RenderDrawLine(sdlr,px1,py1,px2,py2);
        // std::cout << "Arena: double (" << w.x1 << " , " << w.y1 << "), (" << w.x2 << " , " << w.y2 << ")" << "                 " <<std::endl;
        // std::cout << "Arena: px     (" << px1 << " , " << py1 << "), (" << px2 << " , " << py2 << ")" << "                 " <<std::endl;
    }
}



