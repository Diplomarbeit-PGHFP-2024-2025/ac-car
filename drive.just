lint:
    ruff check src/driving_engine/**/*.py
    ruff format --check src/driving_engine/**/*.py

fix:
    ruff check --fix src/driving_engine/**/*.py
    ruff format src/driving_engine/**/*.py

build:
    colcon build --packages-select driving_engine

run:
    ros2 run driving_engine driving_engine