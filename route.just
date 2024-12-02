lint:
    ruff check src/route_manager/**/*.py
    ruff format --check src/route_manager/**/*.py

fix:
    ruff check --fix src/route_manager/**/*.py
    ruff format src/route_manager/**/*.py

build:
    colcon build --packages-select route_manager

run:
    ros2 run route_manager route_manager