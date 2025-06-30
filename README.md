# EgoPX4

**EgoPX4** is an integration of the [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm#) planner with the PX4 simulation stack, supporting both SITL and real-world deployment on Uvify IFO-S drones.

This repository builds on the original implementations from:

* [EGO-Swarm](https://github.com/ZJU-FAST-Lab/ego-planner-swarm#)
* [Fast Drone 250](https://github.com/ZJU-FAST-Lab/Fast-Drone-250)

---

## 🚀 Setup Instructions

### 1. Clone with Submodules

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

### 2. Build the Workspace

Build the catkin workspace:

```bash
catkin_make
```

### 3. Start the PX4 Uvify IFO-S SITL

```bash
roslaunch uvify_sitl sim_bringup.launch
```

> ⚠️ **Important:** Make sure you’ve checked out the correct branch of the PX4 simulator:
>
> ```
> git checkout ego-swarm-exp
> ```

### 4. Launch EGO-Swarm

```bash
roslaunch offboard run_in_px4.launch
```

### 5. Start the OFFBOARD Controller and Translator

This node bridges EGO-Swarm and MAVROS by translating planned trajectories:

```bash
rosrun offboard ego_to_mavros
```

---

## 🎮 You're Ready!
