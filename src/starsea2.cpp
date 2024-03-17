#include <iostream>
#include "SDL.h"
#include "SDL_image.h"
#include <chrono>

#include "ss_parameters.h"
// #include "solver.h"


uint64_t last_time = SDL_GetTicks();




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
    std::cout << "Hello?" << std::endl;
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
    SDL_Window* window = SDL_CreateWindow( WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
    SDL_Renderer* sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // StepSolver<RobotState> solver;
    // solver.set_end(100);
    // solver.set_step_dt(0.1);
    // solver.set_substeps(5);
    // solver.set_t0(0);

    // RobotState inital_state;

    // Arena arena;
    // arena.add_wall(1000,1000,-1000,1000);
    // arena.add_wall(1000,-1000,-1000,-1000);
    // arena.add_wall(1000,1000,1000,-1000);
    // arena.add_wall(-1000,1000,-1000,-1000);
    // inital_state.set_arena(&arena);

    // PhysicalRobot model;
    // inital_state.set_model(&model);

    // solver.set_state(inital_state);

    while (true) {
        SDL_Event e;
        if (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                break;
            }
        }

        SDL_RenderClear(sdl_renderer);

        // std::chrono::steady_clock::time_point t0 = std::chrono::steady_clock::now();
        // solver.solve_step();
        // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        // solver.render(sdl_renderer);
        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        // std::cout << "Time difference 1 = " << std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count() << "us" << std::endl;
        // std::cout << "Time difference 2 = " << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;

            
        // std::cout << " Time: " << solver.get_time()  << " FPS: " << manage_framerate() << RETURN_CARRAIGE;

        SDL_RenderPresent(sdl_renderer);
        
        
        
    }

    SDL_DestroyRenderer(sdl_renderer);
    SDL_DestroyWindow(window);
    IMG_Quit();
    SDL_Quit();

    return 0;
}