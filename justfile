init:
    bash /opt/ros/jazzy/setup.bash

build:
    colcon build
    bash install/local_setup.bash