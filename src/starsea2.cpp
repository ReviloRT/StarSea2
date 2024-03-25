#include <iostream>
#include "SDL.h"
#include "SDL_image.h"
#include <chrono>
#include <time.h>

#include "ss_parameters.h"
#include "solver.h"
#include "robot_sim.h"


uint64_t last_time = SDL_GetTicks64();
int last_report = -1;
double manage_framerate();

int main( int argc, char* args[] )
{
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
    SDL_Window* window = SDL_CreateWindow( WINDOW_NAME, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
    SDL_Renderer* sdl_renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    // Set up integrator
    IntegratorSolver<Euler<RobotKinematics>,RobotKinematics> solver;
    solver.set_interactions(false);
    solver.set_end(100);
    solver.set_dt(1.0/FRAME_RATE*ROBOT_SIM_RATE);
    solver.set_substeps(1);
    solver.set_t0(0);
    
    // Random start position (uncomment for random start)
    // srand(time(0));
    // srand(0);
    // double x_init = rand() % 1500 - 750;
    // double y_init = rand() % 750 - 375;
    // double r_init = ((double)(rand() % 1000))/1000.0 * 2*PI;
    // std::cout << x_init << ", " << y_init << ", " << r_init << std::endl;
    // _sleep(1000);
    // std::vector<double> pos = { x_init, y_init, r_init}; 

    // *POI*
    std::vector<double> pos = { 0, 0, 0}; // Hardcoded Start position 
    RobotKinematics inital_state(pos);
    solver.set_state(inital_state);
    
    // Generate Arena walls
    sim_robot.arena.add_line(1000,500,1000,-500);
    sim_robot.arena.add_line(1000,500,-1000,500);
    sim_robot.arena.add_line(-1000,-500,1000,-500);
    sim_robot.arena.add_line(-1000,-500,-1000,500);

    // Start thread
    sim_robot.init();


    while (true) {
        SDL_Event e;
        if (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                break;
            }
        }
        SDL_RenderClear(sdl_renderer);

        // Solve
        solver.solve_step_inplace();

        // Render
        solver.render(sdl_renderer);

        // Handle framerate
        double fps = manage_framerate();
        if (solver.get_time() >= last_report + 1 ){
            std::cout << "*** Time: " << solver.get_time()  << " FPS: " << fps << " ***" << RETURN_CARRAIGE;
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