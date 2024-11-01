# A1_M_Gr_C RI 22/23 - Assignment 1

## Group members

- Ana Barros - <up201806593@fe.up.pt>
- Henrique Melo Ribeiro - <up201806529@up.pt>
- João Costa - <up201806560@fe.up.pt>

## Directory structure

```txt
.
├── c_world               # world map and robot definition
├── experiments           # data colected furing experiments and python script to process them
├── paper                 # source of the paper (LaTeX)
├── paper.pdf             # paper
├── readme.txt            # this file
├── runsim.sh             # run the simulator with the correct world and robot
└── src                   # source code
    ├── CMakeLists.txt
    ├── c_turtle          # our source code
    └── flatland          # flatland simulator
```

## Requirements

- Ros Noetic 
- Flatland - Included in the [src directory](./src)

## Compile and run

### Compile

Run `catkin_make` in the project's root directory and then `source devel/setup.sh`. Note that _sourcing_
the [devel/setup.sh file] should be done for each terminal running parts of the project.

### Start simulator

Starting the simulator can be done by calling `roslaunch flatland_server server.launch world_path:="$(pwd)/c_world/world.yaml"`,
or, more conviniently, by executing the [runsim.sh script](./runsim.sh).

### Start robot controller

Starting the controller can be done by calling `roslaunch c_turtle c_turtle.launch`. As you will probably do this in another
shell instance, don't forget to `source devel/setup.sh`. The controller uses the config stored on
[its config file](./src/c_turtle/param/params.yaml). Feel free to play with it.

Note: if you want to change the sampling rate of the LiDAR, you have to update it both in the
[robot config file](./c_world/cturtle.model.yaml) and on the
[controller config file](./src/c_turtle/param/params.yaml).
