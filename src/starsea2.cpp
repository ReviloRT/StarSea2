#include <iostream>
#include "SDL.h"
#include "SDL_image.h"

#include "parameters.h"
#include "solver.h"


uint64_t last_time = SDL_GetTicks();

#include <chrono>




double manage_framerate() {
    uint64_t now = SDL_GetTicks64();
    double frame_time = 1000.0/FRAME_RATE;
    
    if ((now - last_time) < frame_time) {
        uint64_t delay = frame_time - (now - last_time);
        SDL_Delay(delay);
    }

    double fps = 1000.0/ ((double) (now- last_time));
    last_time = now;
    return fps;
}


int main( int argc, char* args[] )
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
    SDL_Window* window = SDL_CreateWindow( WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
    SDL_Renderer* sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    StepSolver<Orbit> solver;
    solver.set_end(2460363.5 + 400);
    solver.set_step_dt(1);
    solver.set_substeps(5);
    solver.set_t0(2460363.5);

    // semimajor_axis, eccentricity, inclination, longitude_accend, argument_periap, true_anomaly, radius, mean_anomaly, x, y ,z ,dx ,dy ,dz
    OrbitalElements test0 = {1.5E+08, 0};
    OrbitalElements test1 = {1.5E+08, 0.1};
    OrbitalElements test2 = {1.5E+08, 0.2};
    OrbitalElements test3 = {1.5E+08, 0.3};
    OrbitalElements test4 = {1.5E+08, 0.4};
    OrbitalElements test5 = {1.5E+08, 0.5};
    OrbitalElements test6 = {1.5E+08, 0.6};
    OrbitalElements test7 = {1.5E+08, 0.7};
    OrbitalElements test8 = {1.5E+08, 0.8};
    OrbitalElements test9 = {1.5E+08, 0.9};
    OrbitalElements test10 = {1.5E+08, 0.99};

    OrbitalElements earth = {1.494717817616397E+08, 1.609324185477713E-02, 4.199654324999817E-03, 1.710497684212477E+02, 2.900465900940238E+02, 0, 0, 5.097503158142240E+01};
    earth.convertDegToRad();
    OrbitalElements apophis = {1.380341225671715E+08, 1.914001793094363E-01, 3.339176196586902E+00, 2.039515880458148E+02, 1.266242353951886E+02, 0, 0, 3.240927661569113E+02};
    apophis.convertDegToRad();

    std::vector<OrbitalElements> objects = {test0, test1, test2, test3, test4, test5, test6, test7, test8, test9, test10};
    Orbit inital_state(objects);
    inital_state.set_tolerance(1E-12);
    inital_state.set_gravitational_param(1.3271244E+11);
    inital_state.set_render_scale(0.000000005);

    solver.set_state(inital_state);

    while (true) {
        SDL_Event e;
        if (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                break;
            }
        }

        SDL_RenderClear(sdl_renderer);

        // std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        solver.solve_step();
        // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        solver.render(sdl_renderer);
        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        // std::cout << "Time difference 1 = " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "us" << std::endl;
        // std::cout << "Time difference 2 = " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;

            
        std::cout << " Time: " << solver.get_time() - 2460363.5  << " FPS: " << manage_framerate() << RETURN_CARRAIGE;

        SDL_RenderPresent(sdl_renderer);
        
        
        
    }

    SDL_DestroyRenderer(sdl_renderer);
    SDL_DestroyWindow(window);
    IMG_Quit();
    SDL_Quit();

    return 0;
}