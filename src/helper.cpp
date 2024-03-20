
#include "helper.h"

int coord_to_px(double coordx) {
    int px =  length_to_px(coordx) + SCREEN_WIDTH/2;
    return __min(__max(0,px),SCREEN_WIDTH);
}
int coord_to_py(double coordy) {
    int py = length_to_py(coordy) + SCREEN_HEIGHT/2;
    return __min(__max(0,py),SCREEN_HEIGHT);
}

int length_to_px(double coordx) {
    return (coordx*SCREEN_HEIGHT/SCREEN_WIDTH)*SCREEN_WIDTH/2;
}
int length_to_py(double coordy) {
    return coordy*SCREEN_HEIGHT/2;
}

void print(std::string str) {
    std::cout << str << std::endl;
}