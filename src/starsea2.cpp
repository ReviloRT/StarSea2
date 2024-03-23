#include <iostream>
#include "SDL.h"
#include "SDL_image.h"
#include <chrono>

#include "ss_parameters.h"
#include "solver.h"


uint64_t last_time = SDL_GetTicks64();
int last_report = -1;
double manage_framerate();


int main( int argc, char* args[] )
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
    SDL_Window* window = SDL_CreateWindow( WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
    SDL_Renderer* sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    IntegratorSolver<Euler<RobotState>,RobotState> solver;
    solver.set_interactions(false);
    solver.set_end(100);
    solver.set_dt(1.0/FRAME_RATE);
    solver.set_substeps(1);
    solver.set_t0(0);

    RobotState inital_state;
    solver.set_state(inital_state);
    
    Arena &arena = *sim_robot.get_arena();
    arena.add_wall(1000,1000,1000,-1000);
    arena.add_wall(1000,1000,-1000,1000);
    arena.add_wall(-1000,-1000,1000,-1000);
    arena.add_wall(-1000,-1000,-1000,1000);

    sim_robot.init();


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

        double fps = manage_framerate();

        if (solver.get_time() >= last_report + 1 ){
            std::cout << "*** Time: " << solver.get_time()  << " FPS: " << fps << " ***" <<RETURN_CARRAIGE;
            last_report = (int)solver.get_time();
        }

        SDL_RenderPresent(sdl_renderer);
    }

    SDL_DestroyRenderer(sdl_renderer);
    SDL_DestroyWindow(window);
    IMG_Quit();
    SDL_Quit();

    return 0;
}

double manage_framerate() {
    uint64_t now = SDL_GetTicks64();
    double frame_time = 1000.0/FRAME_RATE;

    if (now - last_time < frame_time) {
        uint64_t delay = frame_time - (now - last_time);
        _sleep(delay);
    }

    now = SDL_GetTicks64();
    double fps = 1000.0/ ((double) (now - last_time));
    last_time = now;
    return fps;
}