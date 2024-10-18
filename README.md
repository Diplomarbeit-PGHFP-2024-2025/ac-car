# Ros2

# first start

once the dev container is open follow https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

before https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html#id5 there might be an issue you need to go
into /etc/apt/sources.list.d and remove the ros2.list

---

## just

[just](https://crates.io/crates/just) is used to simplify running of commands

```bash
sudo apt install just
```

---

# use

for every new terminal run

```bash
just init
```

to run some packages

```bash
just build
```

run example:

```bash
ros2 run py_pubsub listener
```


