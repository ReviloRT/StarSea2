
#include "utils.h"

int coord_to_px(double coordx) {
    return coordx*SCREEN_WIDTH/2 + SCREEN_WIDTH/2;
}
int coord_to_py(double coordy) {
    return coordy*SCREEN_HEIGHT/2 + SCREEN_HEIGHT/2;
}

void print(std::string str) {
    std::cout << str << std::endl;
}