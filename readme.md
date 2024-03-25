
## Setup:
1. Ensure you have meson installed: pip3 install meson
2. Ensure you are in the root directory (ie ../Starsea2/)
3. Run the following commands
    meson setup build && cd build           (note: this will not complete)

meson init -C build --build

## Compile:
cd build
meson compile

## Run:
starsea2


## Notes:
Any new .cpp files in the robot source code must be included in the meson.build file under the robot_sources list