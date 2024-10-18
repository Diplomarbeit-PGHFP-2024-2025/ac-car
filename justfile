set shell := ["bash", "-c"]

init:
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source install/local_setup.bash" >> ~/.bashrc

build:
    colcon build