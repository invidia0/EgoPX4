# EgoPX4

**EgoPX4** is an integration of the [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm#) planner with the PX4 simulation stack, supporting both SITL and real-world deployment on Uvify IFO-S drones.

This repository builds on the original implementations from:

* [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm#)
* [Fast Drone 250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)

## 🚀 Setup Instructions

## 0. Setup with Docker
**Organize your project folder like this:**

```perl
my-project/
├── Dockerfile
├── src/
│ └── EgoPX4
│ └── ... (your ROS packages and code)
└── ... (other files)
```

**Build your Docker image (run this from the `my-project` folder):**

```bash
docker build -t egopx4-image .
```

**Run the Docker container with your workspace mounted and necessary environment and device access:**

```bash
docker run -it --rm \
  --name egopx4-container \
  --privileged \
  --net=host \
  --pid=host \
  --ipc=host \
  --gpus=all \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -v "$(pwd):/home/ws" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev/dri:/dev/dri \
  egopx4-image \
  /bin/bash
```

This command:

- Mounts your current folder into /home/ws inside the container

- Forwards X11 and GPU devices so GUI apps work

- Runs an interactive bash shell

**To start the container again without rebuilding or rerunning:**
```bash
docker start egopx4-container
```
**To get an interactive shell inside the running container:**
```bash
docker exec -it egopx4-container /bin/bash
```

### 💡 Tip: Simplify Docker commands with shell functions

To avoid typing long Docker commands every time, add these helper functions to your shell (`~/.bashrc` or `~/.zshrc`):

```bash
# Run a new container interactively
function drun() {
    docker run -it --rm \
        --name "$1" \
        --privileged \
        --net=host --pid=host --ipc=host --gpus=all \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_VISIBLE_DEVICES=all \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        -v "$(pwd):/home/ws" \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v /dev/dri:/dev/dri \
        "$2" \
        /bin/bash
}

# Start an existing container
function dst() {
    docker start "$1"
}

# Exec into a running container
function dex() {
    docker exec -it "$1" /bin/bash
}
```

### Usage examples:

* Run a new container (removes it when you exit):

  ```bash
  drun egopx4-container egopx4-image
  ```

* Start a stopped container:

  ```bash
  dst egopx4-container
  ```

* Attach to a running container shell:

  ```bash
  dex egopx4-container
  ```

## 1. Clone with Submodules

Clone the repository and initialize submodules (EGO-Swarm and the PX4 simulator):

```bash
git clone --recurse-submodules https://github.com/youruser/EgoPX4.git
cd EgoPX4
```

> 💡 If you already cloned without submodules, you can run:
>
> ```bash
> git submodule update --init --recursive
> ```

## 2. Build the Workspace

Build the catkin workspace:

```bash
catkin_make
```

## 3. Start the PX4 Uvify IFO-S SITL

```bash
roslaunch uvify_sitl sim_bringup.launch
```

> ⚠️ **Important:** Make sure you’ve checked out the correct branch of the PX4 simulator:
>
> ```
> git checkout ego-swarm-exp
> ```

## 4. Launch EGO-Swarm

```bash
roslaunch offboard run_in_px4.launch
```

## 5. Start the OFFBOARD Controller and Translator

This node bridges EGO-Swarm and MAVROS by translating planned trajectories:

```bash
rosrun offboard ego_to_mavros
```

---

## 🎮 You're Ready!
