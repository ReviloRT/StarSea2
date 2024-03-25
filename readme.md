# STARSEA - ROBOT EDITION

This has taken way too much of my time (although it's actually been pretty quick in the making)
I need to do actual work. And sleep... That is severly lacking.
But it's cool! 

## Features:
STUFF:
- we have real time simulation
- we have sensors with noise and range, 
- we have motors with 2nd order dynmaics, 
- we have arduino code running with no changes,
- we have **GRAPHICS**,
- we have super speed *if your computer can handle it*
- we have second order integrators
- we have some orbit stuff lying around
- we have **MULTITHREADING**

Yep, Over Powered. Like my initals.

## Setup:
1. Ensure you have meson installed: 

`pip3 install meson`

2. Clone the repository from github (git clone **repository html**)
4. Ensure you are in the root directory (ie ../starsea2/) `cd StarSea2`
5. Run the following commands:

`git submodule update --init`

`mkdir subprojects`

`meson wrap install sdl2`

`meson setup build`

(Note this may take a moment as it downloads sdl2)

## Compile and Run:

### First time
There are weird things happening with having the sdl2 lib available for execution
You may need to manually copy:

`../StarSea2/build/subprojects/SDL2-2.28.5/libsdl2.dll`
to 
`../StarSea2/build/`

I tried to set this up to happen automatically in the meson.build file but no luck.

### Normally
1. Ensure you are in the build folder (ie ../StarSea2/build ) `cd build`
2. Compile any changes (must be in build folder)

`meson compile`

3. Execute (must also be in build folder)

`starsea2`


## Notes:
Any new .cpp files in the robot source code must be included in the meson.build file under the robot_sources list

If you are looking to change any parameters about the robot, search the repository for "*POI*" (point of interest)
This will direct you to:
- The starting position (can be hard coded or randomised) in src/starsea.cpp
- The simulation speed (relative to real time) in src/ss_parameters.h
- The robot sensor and physical parameters in src/robot_sim_physics.h

### The Robot Code:

To change robot code, go the ../StarSea2/submodules/706Project1 folder to change robot code. 
No changes are necessary from real robot code except in cases of while loops with no delays, pinchecks, etc (ie, time only progresses when the code calls something in the simulator)

Changes made to the robot code here can be treated like the normal git repository if your terminal is inside the ../StarSea2/submodules/706Project1 (it will ignore the surrounding git things, love submodules) 

**WHEN IN THE 706Project1 FOLDER IT IS EXACTLY LIKE YOUR NORMAL GIT**

**IF YOU ARE NOT IN THE 706Project1 FOLDER** -  *(ie you have been compiling and running stuff)*

**YOUR GIT CALLS WONT DO WHAT YOU WANT** -  *(they will get the entire starsea project instead)*