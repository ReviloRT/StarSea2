#ifndef STARSEA_UTILS_H
#define STARSEA_UTILS_H

#include "ss_parameters.h"
#include <iostream>
#include <string>

#define RETURN_CARRAIGE "                                                                                  \r"

int coord_to_px(double coordx);
int coord_to_py(double coordy);
void print(std::string str);
uint64_t get_cycles();

#endif // STARSEA_UTILS_H