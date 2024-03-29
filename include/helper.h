#ifndef STARSEA_UTILS_H
#define STARSEA_UTILS_H

#include "ss_parameters.h"
#include <iostream>
#include <string>
#include <SDL.h>

#define RETURN_CARRAIGE "                                                                                  \n"

double coord_to_px(double coordx);
double coord_to_py(double coordy);
double length_to_px(double coordx);
double length_to_py(double coordy);
double rotate_point_x(double x, double y, double theta) ;
double rotate_point_y(double x, double y, double theta) ;
void drawRect(SDL_Renderer* sdlr, double cx, double cy, double wid, double len, double theta, double scale);
void print(std::string str);
double map_range(double value, double low_in, double high_in, double low_out, double high_out);

#endif // STARSEA_UTILS_H