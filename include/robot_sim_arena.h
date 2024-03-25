#ifndef ROBOT_ARENA_H
#define ROBOT_ARENA_H

#include "ss_parameters.h"
#include "helper.h"

#include <vector>
#include <SDL.h>

struct Line {
    double x1;
    double y1;
    double x2;
    double y2;
};


class Arena {
public:
    std::vector<Line> walls;
public:
    void add_line(Line wall);
    void add_line(double x1, double y1, double x2, double y2);
    double get_distance(double pos[3]) const;
    double get_intersect(double pos[3], double intersect[2]) const;
    void render(SDL_Renderer* sdlr) const;
};


#endif // ROBOT_ARENA_H