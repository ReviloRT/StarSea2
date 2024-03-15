#include <iostream>
#include "SDL.h"
#include "SDL_image.h"

#include "parameters.h"
#include "solver.h"

#define PROJECT_NAME "starsea2"
#define WINDOW_NAME "StarSea2"



uint64_t next_time = SDL_GetTicks();

uint64_t time_left(void)
{
    uint64_t now;

    now = SDL_GetTicks64();
    if(next_time <= now)
        return 0;
    else
        return next_time - now;
}


int main( int argc, char* args[] )
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
    SDL_Window* window = SDL_CreateWindow( WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
    SDL_Renderer* sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // std::vector<double> initp = {0.1,0,0,-0.1,0,0,0.2,0,0,0,0.3,0};
    // std::vector<double> initv = {0,1.5,0,0,-1.5,0,0.5,0,0,-0.5,0,0};
    std::vector<double> initp = {0.1,0,0,-0.1,0,0};
    std::vector<double> initv = {0,1.5,0,0,-1.5,0};

    Solver<RKN4<GravityPoints>,GravityPoints> solver;
    solver.set_step_dt(0.005);
    solver.set_end(0);

    GravityPoints inital_state(initp, initv);
    solver.set_state(inital_state);

    RKN4<GravityPoints> integrator;
    integrator.set_dt(0.001);
    solver.set_integrator(integrator);


    while (true) {
        SDL_Event e;
        if (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                break;
            }
        }

        SDL_RenderClear(sdl_renderer);

        solver.solve_dynamics();
        solver.complete_step();
        solver.solve_interactions();
        solver.complete_step();
        solver.render(sdl_renderer);
        solver.complete_step();
                
        std::cout << " Time = " << solver.get_time()  << RETURN_CARRAIGE;

        SDL_RenderPresent(sdl_renderer);
        SDL_Delay(time_left());
        next_time += 1000.0/FRAME_RATE;
        
    }

    SDL_DestroyRenderer(sdl_renderer);
    SDL_DestroyWindow(window);
    IMG_Quit();
    SDL_Quit();

    return 0;
}