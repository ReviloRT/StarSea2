#ifndef ROBOT_ARENA_H
#define ROBOT_ARENA_H

#include "ss_parameters.h"
#include "helper.h"

#include <vector>
#include <SDL.h>

struct Wall {
    double x1;
    double y1;
    double x2;
    double y2;
};


class Arena {
public:
    std::vector<Wall> walls;
public:
    void add_wall(Wall wall);
    void add_wall(double x1, double y1, double x2, double y2);
    double get_distance(double x, double y, double rot) const;
    void render(SDL_Renderer* sdlr) const;
};


#endif // ROBOT_ARENA_H