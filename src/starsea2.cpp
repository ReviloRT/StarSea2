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

    Solver<NoIntegration<Orbit>,Orbit> solver;
    solver.set_end(0);

    // time_0, semimajor_axis, eccentricity, inclination, longitude_accend, argument_periap, mean_anomaly
    ClassicalOrbitalElements test = {2460363.5 , 1.5E+08, 0, 0, 0, 0, 0};


    ClassicalOrbitalElements earth = {2460363.5 , 1.494717817616397E+08, 1.609324185477713E-02, 4.199654324999817E-03, 1.710497684212477E+02, 2.900465900940238E+02, 5.097503158142240E+01};
    earth.convertDegToRad();

    ClassicalOrbitalElements apophis = {2460363.5 , 1.380341225671715E+08, 1.914001793094363E-01, 3.339176196586902E+00, 2.039515880458148E+02, 1.266242353951886E+02, 3.240927661569113E+02};
    apophis.convertDegToRad();

    std::vector<ClassicalOrbitalElements> objects = {earth,apophis};
    Orbit inital_state(objects);
    inital_state.set_tollerance(1E-12);
    inital_state.set_path_samples(100);
    inital_state.set_render_scale(0.000000005);
    solver.set_state(inital_state);

    NoIntegration<Orbit> integrator;
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

        // _sleep(1000);
                
        std::cout << " Step = " << solver.get_time()  << RETURN_CARRAIGE;

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