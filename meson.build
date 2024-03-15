project('starsea2', 'c', 'cpp',
  version : '0.1',
  default_options : ['warning_level=0'])

sdl2_dep = dependency('sdl2')
sdl2_main_dep = dependency('sdl2main')
sdl2_image_dep = dependency('sdl2_image')

inc = include_directories('include')

sources = ['src/starsea2.cpp',
           'src/integrator.cpp',
           'src/solver.cpp',
           'src/state.cpp',
           'src/state_gravN.cpp',
           'src/state_orbit.cpp',
           'src/utils.cpp',
           ]


executable('starsea2',
           sources,
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