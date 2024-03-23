project('starsea2', 'c', 'cpp',
  version : '0.1',
  default_options : ['warning_level=0'])

sdl2_dep = dependency('sdl2')
sdl2_main_dep = dependency('sdl2main')
sdl2_image_dep = dependency('sdl2_image')

inc = include_directories(['include',
                               'submodules/706Project1/include'
                              ])

sources = ['src/starsea2.cpp',
           'src/integrator.cpp',
           'src/solver.cpp',
           'src/state.cpp',
           'src/state_gravN.cpp',
           'src/state_orbit.cpp',
           'src/state_robot.cpp',
           'src/robot_sim_helper.cpp',
           'src/helper.cpp'
           ]

robot_sources = [
           'submodules/706Project1/src/main.cpp',
           'submodules/706Project1/src/basedrive.cpp',
           'submodules/706Project1/src/blink.cpp',
           'submodules/706Project1/src/gyro.cpp',
           'submodules/706Project1/src/InfraRed.cpp',
           'submodules/706Project1/src/logger.cpp',
           'submodules/706Project1/src/motor.cpp',
           'submodules/706Project1/src/phototrans.cpp',
           'submodules/706Project1/src/PID.cpp',
           'submodules/706Project1/src/sensors.cpp',
           'submodules/706Project1/src/ultrasonic.cpp',
           'submodules/706Project1/src/utils.cpp',
           'submodules/706Project1/src/karman.cpp'
           ]


executable('starsea2',
           sources, 
           robot_sources,
           include_directories: inc,
           install : true,
          #  win_subsystem: 'windows',
           dependencies: [sdl2_dep, sdl2_main_dep,sdl2_image_dep],
          )


run_command(
  'cp',
  'build/subprojects/SDL2-2.28.5/libsdl2.dll',
  'build/libsdl2.dll'
)

run_command(
  'cp',
  'build/subprojects/SDL2_image-2.6.3/libsdl2image.dll',
  'build/libsdl2image.dll'
)