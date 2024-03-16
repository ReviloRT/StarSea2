
#include "utils.h"

int coord_to_px(double coordx) {
    int px =  (coordx*SCREEN_HEIGHT/SCREEN_WIDTH)*SCREEN_WIDTH/2 + SCREEN_WIDTH/2;
    return __min(__max(0,px),SCREEN_WIDTH);
}
int coord_to_py(double coordy) {
    int py = coordy*SCREEN_HEIGHT/2 + SCREEN_HEIGHT/2;
    return __min(__max(0,py),SCREEN_HEIGHT);
}

void print(std::string str) {
    std::cout << str << std::endl;
}