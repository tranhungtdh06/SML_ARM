# SML-Arm — Open-Source 6-DOF Robotic Arm

**SML-Arm** is a complete open-source 6-DOF articulated robotic arm project built around stepper-driven harmonic drives, a parallel 2-finger gripper, and a full **ROS 2 Jazzy + Gazebo Harmonic + `ros2_control` + MoveIt 2** simulation and control stack optimized to run smoothly even on modest PCs.

> **Project Status**: The **ROS 2 / Gazebo Harmonic / MoveIt 2 / PyQt5 Control Studio / Pick-and-Place** software stack (`src/`) is complete and verified. Structured templates, BOMs, wiring checklists, and shared C/C++ protocol headers are scaffolded in [`hardware/`](hardware/README.md), [`electrical/`](electrical/README.md), and [`firmware/`](firmware/README.md) ready for CAD, KiCad, and MCU firmware drops.

---

## Repository Structure

```text
sml_arm/
├── .github/                  # GitHub Actions ROS 2 Jazzy CI + Issue/PR templates
├── docs/                     # System architecture, mechanical assembly & Control Studio manual
├── electrical/               # Schematics, PCB, wiring tables & electrical BOM template
├── firmware/                 # MCU firmware structure & shared binary protocol (sml_arm_protocol.h)
├── hardware/                 # STEP/STL CAD folders, 2D drawings & mechanical BOM template
└── src/                      # Colcon-ready ROS 2 Jazzy packages
    ├── sml_arm_bringup/      # Unified launch files (Gazebo sim, mock hardware, MoveIt 2 + GUI)
    ├── sml_arm_control/      # ros2_control config, FK/IK solver, Pick-and-Place demo & Control Studio GUI
    ├── sml_arm_description/  # URDF/Xacro model, parallel gripper, visual (~17 MB) & collision (~120 KB) STLs
    ├── sml_arm_gazebo/       # Gazebo Harmonic SDF world (pick/place workcell) & ros_gz_bridge config
    └── sml_arm_moveit_config/# MoveIt 2 SRDF, KDL/TRAC-IK kinematics, OMPL & Pilz PTP planners
```

### Subsystem Documentation

| Subsystem | Scope | Documentation |
| :--- | :--- | :--- |
| **System Overview** | Full software/hardware data flow & ROS 2 interfaces | [`docs/architecture.md`](docs/architecture.md) |
| **Mechanical Hardware** | 6-DOF + gripper kinematics table, CAD import guide & BOM | [`hardware/README.md`](hardware/README.md) |
| **Assembly & Calibration** | Mechanical build sequence & `[0..0]` home zero alignment | [`docs/assembly_guide.md`](docs/assembly_guide.md) |
| **Electrical** | Power bus, stepper driver wiring pinouts, E-Stop & BOM | [`electrical/README.md`](electrical/README.md) |
| **Firmware** | Real-time MCU control loop & binary protocol header | [`firmware/README.md`](firmware/README.md) |
| **Control Studio GUI** | Full UI operator manual (`arm_control_gui.py`) | [`docs/control_studio_manual.md`](docs/control_studio_manual.md) |
| **Contributing** | Branch workflow, hardware/ROS 2 verification checks | [`CONTRIBUTING.md`](CONTRIBUTING.md) |

---

## Requirements (ROS 2 Jazzy on Ubuntu 24.04)

- **ROS 2 Jazzy Jalisco** (`ros-jazzy-desktop`)
- **Gazebo Harmonic + `ros2_control`**: `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control`, `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-joint-state-broadcaster`, `ros-jazzy-joint-trajectory-controller`, `ros-jazzy-position-controllers`
- **MoveIt 2**: `ros-jazzy-moveit`, `ros-jazzy-pilz-industrial-motion-planner`, `ros-jazzy-moveit-configs-utils`
- **GUI & Tools**: `python3-pyqt5`, `python3-numpy`, `liburdfdom-tools`

---

## Clone & Build Workspace

Clone directly as a ROS 2 colcon workspace (or into `~/sml_arm_ws`):

```bash
git clone git@github.com:tranhungtdh06/sml_arm.git ~/sml_arm_ws
cd ~/sml_arm_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start (ROS 2 Simulation & MoveIt 2)

### 1. Full System: Gazebo Harmonic + MoveIt 2 + Control Studio GUI (Recommended)

- **Full CAD Visual Model (`fast_visual:=false`, default)**:
  ```bash
  ros2 launch sml_arm_bringup sml_arm_moveit.launch.py
  ```
- **Low-Spec PC Mode (`fast_visual:=true`)** — renders the lightweight `collision/` STL meshes (`~120 KB` total) in Gazebo instead of the high-poly `visual/` CAD meshes (`~17 MB` total):
  ```bash
  ros2 launch sml_arm_bringup sml_arm_moveit.launch.py fast_visual:=true
  ```
- Add `use_rviz:=true` if you also want RViz2 open alongside Gazebo and Control Studio (`use_rviz` defaults to `false` to save GPU/RAM).

### 2. Ultra-Lightweight Mock Hardware Mode (No Gazebo)

```bash
ros2 launch sml_arm_bringup sml_arm_moveit.launch.py use_sim:=false
# Or standalone MoveIt RViz demo:
ros2 launch sml_arm_moveit_config demo.launch.py
```

### 3. Gazebo Simulation Only / Standalone Control Studio GUI

```bash
# Terminal 1: Start Gazebo Harmonic + ros2_control
ros2 launch sml_arm_bringup sml_arm_sim.launch.py fast_visual:=true use_rviz:=false

# Terminal 2: Launch Control Studio GUI
ros2 run sml_arm_control arm_control_gui.py
```

### 4. URDF Inspection Only

```bash
ros2 launch sml_arm_description display.launch.py
```

---

## Pick-and-Place Demo & CLI Utilities

Gazebo spawns `pick_table`, `place_table`, and a dynamic `pick_cube` ready for grasping:

```bash
# Run automated 10-step Pick-and-Place demo (once or looping at custom speed)
ros2 run sml_arm_control pick_and_place_demo.py
ros2 run sml_arm_control pick_and_place_demo.py --loop --speed-scale 1.5

# Forward Kinematics (local chain + MoveIt /compute_fk)
ros2 run sml_arm_control arm_kinematics.py --fk 0.0 -0.45 0.90 0.0 0.65 0.0

# Inverse Kinematics + immediate execution
ros2 run sml_arm_control arm_kinematics.py --ik 0.0 -0.48601 0.71907 0.7 0.0 0.0 --execute
ros2 run sml_arm_control arm_kinematics.py --ik 0.0 -0.35 0.70 --position-only

# CLI preset / joint target / gripper / topic daemon
ros2 run sml_arm_control arm_commander.py --pose ready --gripper open
ros2 run sml_arm_control arm_commander.py --joints 0.2 -0.4 0.8 0.0 0.6 0.0 --duration 3.0
ros2 run sml_arm_control arm_commander.py --daemon
```

Or inside **Control Studio GUI**, click **`Run Pick & Place Demo`** (with optional **`Loop`**) in Panel 6. See [`docs/control_studio_manual.md`](docs/control_studio_manual.md) for every button, jog slider, IK planner setting, and waypoint sequencer control.

---

## Low-Spec PC Optimization Highlights

1. **250 Hz Gazebo Physics (`4 ms` step)** in [`src/sml_arm_gazebo/worlds/empty_arm_world.sdf`](src/sml_arm_gazebo/worlds/empty_arm_world.sdf): Cuts physics CPU load compared to `1000 Hz` while supporting `100 Hz` `ros2_control`, with shadows and unused sensor/contact plugins disabled.
2. **Low-Poly Visual Option (`fast_visual:=true`)**: Replaces `~17 MB` high-poly CAD visual meshes with `~120 KB` simplified STL meshes while keeping identical kinematics.
3. **Single 3D Viewer by Default**: `sml_arm_moveit.launch.py` defaults to `use_rviz:=false` when `use_gui:=true`.
4. **Throttled Streaming & Rendering**: Gazebo runs with `-v 1`, `robot_state_publisher` & `joint_trajectory_controller` publish at `30 Hz`, real-time jog commands coalesce at `<= 30 Hz`, and the GUI 3D canvas redraws at `20 Hz`.

---

## License

Released under the [Apache License 2.0](LICENSE) — Copyright 2026 Tran Duc Hung and SML-Arm Contributors.
