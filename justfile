set shell := ["bash", "-c"]
mod rust
mod agent

init:
    python3 -m venv ./venv --system-site-packages --symlinks
    touch ./venv/COLCON_IGNORE
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ./install/local_setup.bash" >> ~/.bashrc

install:
    python3 -m pip install uagents==0.17.1
    python3 -m pip install git+https://github.com/Diplomarbeit-PGHFP-2024-2025/aca-protocols.git@ffbf199cde07a21d41ff08e14450fae9594082cd
    python3 -m pip install ruff

build:
    just agent build

    colcon build --packages-select py_pubsub

lint:
    just rust lint
    just agent lint

fix:
    just rust fix
    just agent fix