set shell := ["bash", "-c"]

mod agent
mod route

init:
    python3 -m venv ./venv --system-site-packages --symlinks
    touch ./venv/COLCON_IGNORE
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ./install/local_setup.bash" >> ~/.bashrc

install:
    python3 -m pip install uagents==0.17.1
    python3 -m pip install git+https://github.com/Diplomarbeit-PGHFP-2024-2025/aca-protocols.git@1cff4119948d933274e99c51f898e8da1fe88ac7
    python3 -m pip install ruff
    python3 -m pip install python-dotenv

build:
    colcon build

lint:
    ruff check
    ruff format --check

fix:
    ruff check --fix
    ruff format