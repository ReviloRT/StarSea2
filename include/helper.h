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
void drawRect(SDL_Renderer* sdlr, double cx, double cy, double wid, double len, double theta, double scale);
void print(std::string str);
uint64_t get_cycles();

#endif // STARSEA_UTILS_H