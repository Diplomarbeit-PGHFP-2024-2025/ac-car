lint:
    ruff check src/fetch_agent/**/*.py
    ruff format --check src/fetch_agent/**/*.py

fix:
    ruff check --fix src/fetch_agent/**/*.py
    ruff format src/fetch_agent/**/*.py

build:
    colcon build --packages-select fetch_agent

run:
    WAITING_TIME=25 ros2 run fetch_agent agent