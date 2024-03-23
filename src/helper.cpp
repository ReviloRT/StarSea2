
#include "helper.h"

double coord_to_px(double coordx) {
    double px =  length_to_px(coordx) + SCREEN_WIDTH/2.0;
    return __min(__max(0,px),SCREEN_WIDTH);
}
double coord_to_py(double coordy) {
    double py = length_to_py(coordy) + SCREEN_HEIGHT/2.0;
    return __min(__max(0,py),SCREEN_HEIGHT);
}

double length_to_px(double coordx) {
    return (coordx*SCREEN_HEIGHT/SCREEN_WIDTH)*SCREEN_WIDTH/2.0;
}
double length_to_py(double coordy) {
    return coordy*SCREEN_HEIGHT/2.0;
}

void print(std::string str) {
    std::cout << str << std::endl;
}

void drawRect(SDL_Renderer* sdlr, double cx, double cy, double wid, double len, double theta, double scale) {
    double l1x = wid/2.0*cos(theta) - len/2.0*sin(theta);
    double l1y = wid/2.0*sin(theta) + len/2.0*cos(theta);
    double l2x = wid/2.0*cos(theta) + len/2.0*sin(theta);
    double l2y = wid/2.0*sin(theta) - len/2.0*cos(theta);

    SDL_FPoint corners[5];
    corners[0] = {coord_to_px((cx + l1x)*scale), coord_to_py((cy + l1y)*scale)};
    corners[1] = {coord_to_px((cx + l2x)*scale), coord_to_py((cy + l2y)*scale)};
    corners[3] = {coord_to_px((cx - l2x)*scale), coord_to_py((cy - l2y)*scale)};
    corners[2] = {coord_to_px((cx - l1x)*scale), coord_to_py((cy - l1y)*scale)};
    corners[4] = {coord_to_px((cx + l1x)*scale), coord_to_py((cy + l1y)*scale)};
    SDL_RenderDrawLinesF(sdlr,corners,5);
}