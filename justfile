set shell := ["bash", "-c"]

mod agent
mod route
mod drive

init:
    python3 -m venv ./venv --system-site-packages --symlinks
    touch ./venv/COLCON_IGNORE
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    echo "source ./install/local_setup.bash" >> ~/.bashrc

install:
    python3 -m pip install uagents==0.17.1
    python3 -m pip install git+https://github.com/Diplomarbeit-PGHFP-2024-2025/aca-protocols.git@038a64cb06671faa5eb0593d035eb94fa249396a
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

send_drive_to:
    ros2 action send_goal /drive_to_station custom_action_interfaces/action/DriveToStation "{green_energy: 1.57, cost_per_kwh: 3.5, charging_wattage: 0.1, max_km: 200, target_soc: 80}"