set shell := ["bash", "-c"]

init:
    python3 -m venv ./venv --system-site-packages --symlinks
    touch ./venv/COLCON_IGNORE
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ./install/local_setup.bash" >> ~/.bashrc

install:
    python3 -m pip install uagents==0.17.0
    python3 -m pip install git+https://github.com/Diplomarbeit-PGHFP-2024-2025/aca-protocols.git@bb4b4cd176b42fdec82ac08593957161af0c6e2f

build:
    colcon build

run-agent:
    ros2 run fetch_agent agent