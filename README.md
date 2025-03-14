# AC-Car

[Ros2 init](https://github.com/Diplomarbeit-PGHFP-2024-2025/.github/blob/main/profile/Ros2.md)

## running

build all modules

```bash
just build
```

### FetchAgent

The FetchAgent is the core of the project manage the communication to the station via fetchAI and calling the
DriveEngine and RouteManager

```bash
just agent run
```

### DriveEngine

The DriveEngine is the module that cals MotionControl(Bosch Drive Api) based on the path it gets from the RouteManger.

```bash
just drive run
```

### RouteManger

The RouteManger is the module that stores the map in a map.json and calculations the path to the station via a modified
A*.

```bash
just route run
```

---

you need to run all the modules for the car to work!

via you can send a test drive_to command

```bash
just send_drive_to
```

### creating a new ros2 module

ros2 pkg create --build-type ament_python --license Apache-2.0 [module-name]

### Todo

* drive_engine crashing "only length-1 arrays can be converted to Python scalars"
* fetch_agent crashing while waiting for "drive_to_station" cause fetchAI takes over running python - "await wasn't used with future"