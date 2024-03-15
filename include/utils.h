#ifndef UTILS_H
#define UTILS_H

#include "parameters.h"
#include <iostream>
#include <string>

#define RETURN_CARRAIGE "                                                                                  \r"

int coord_to_px(double coordx);
int coord_to_py(double coordy);
void print(std::string str);


#endif // UTILS_H