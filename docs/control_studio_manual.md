# Control Studio UI User Manual (`arm_control_gui.py`)

`arm_control_gui.py` is the PyQt5 Industrial Teach Pendant & Control Studio GUI for **SML-Arm**.

## Launching the GUI

- **Full System (Gazebo Harmonic + MoveIt 2 + Control Studio GUI)**:
  ```bash
  ros2 launch sml_arm_bringup sml_arm_moveit.launch.py
  # Lightweight visual mode for modest GPUs/CPUs:
  ros2 launch sml_arm_bringup sml_arm_moveit.launch.py fast_visual:=true
  ```
- **Standalone GUI (connecting to an already-running simulation or mock bringup)**:
  ```bash
  ros2 run sml_arm_control arm_control_gui.py
  ```
- **Headless Offscreen Self-Test**:
  ```bash
  QT_QPA_PLATFORM=offscreen ros2 run sml_arm_control arm_control_gui.py --self-test
  ```

---

## Interface Panels

### Top Status & Safety Bar
- **Status Badges**:
  - `JointStates: <Hz>` — turns green when `/joint_states` is actively streaming.
  - `ros2_control: READY` — turns green when `/arm_trajectory_controller/follow_joint_trajectory` and `/arm_trajectory_controller/joint_trajectory` are available.
  - `MoveIt 2: READY` — turns green when `move_group` (`/compute_ik`, `/move_action`) is active.
- **`Duration (s)`**: Motion duration used for explicit trajectory commands (`Execute Target`, preset buttons, `Solve & Execute IK`, and taught waypoints).
- **`Vel %`**: Velocity scaling (`5%`–`100%`) applied to MoveIt 2 planning and real-time jog lookahead timing.
- **`Real-Time Jog (Instant)`** *(checked by default)*:
  - **Checked**: Dragging joint sliders, changing joint spinboxes, or clicking/holding Joint (`-`/`+`) or Cartesian (`-X/+X` ... `-Yaw/+Yaw`) jog buttons streams live commands directly over `/arm_trajectory_controller/joint_trajectory` (`<= 30 Hz`) without action handshake delay.
  - **Unchecked**: Controls only update the **green target ghost preview** in the 3D viewport until an Execute button is clicked.
- **`EMERGENCY STOP`**: Cancels active `ros2_control` and `MoveIt 2` goals immediately and commands the arm to hold its current actual joint angles.

### Panel 1: Joint Teach Pendant (`6-DOF`) & Gripper Bar
- **`Units`**: Switch display/input units between **Degrees (`°`)** and **Radians (`rad`)** across `dof_joint1`–`dof_joint6`.
- **`Jog Step`**: Select `0.5°`, `1°`, `5°`, or `15°` per `-`/`+` click; press and hold `-`/`+` for continuous auto-repeat jogging (`25 Hz`).
- **Execution Buttons**:
  - **`Execute Target (ros2_control)`**: Executes a smooth trajectory to target joint angles over `Duration (s)`.
  - **`Plan & Execute (MoveIt 2)`**: Plans a collision-checked joint-space path via `move_group` (`OMPL` or `Pilz`) and executes it.
  - **`Sync Target <- Actual`**: Snaps the target preview back to current live joint angles.
  - **`Zero Target (Home)`**: Resets the target preview to `[0, 0, 0, 0, 0, 0]` without immediately moving the robot.
- **End-Effector Gripper Bar**:
  - Live finger stroke slider (`0.0–40.0 mm` per finger) plus **`Open (38mm)`**, **`Grasp (15.5mm)`**, and **`Close (0mm)`** quick buttons publishing directly to `/gripper_controller/commands`.

### Panel 2: Forward Kinematics (Live TCP Pose)
- Displays live end-effector (`ee_link`) position `XYZ (m)` and orientation `RPY (°)` in `world` and `base_link` frames.
- **`Verify Actual Pose via MoveIt /compute_fk`**: Cross-checks live FK against MoveIt 2 `/compute_fk`.

### Panel 3: Cartesian TCP Jog Pad (World Frame)
- **`Linear Step`**: `1 mm`, `5 mm`, `10 mm`, `25 mm`, `50 mm` per tick (`X`, `Y`, `Z`).
- **`Angular Step`**: `1°`, `5°`, `15°` per tick (`Roll`, `Pitch`, `Yaw`).
- **Axis Buttons**: Click or press-and-hold `+X/-X`, `+Y/-Y`, `+Z/-Z`, `+Roll/-Roll`, `+Pitch/-Pitch`, `+Yaw/-Yaw`. Uses warm-started Damped-Least-Squares IK (`< 0.5 ms` solve) and blocks unreachable workspace targets safely.

### Panel 4: Inverse Kinematics (IK) & MoveIt Pose Planner
- Enter target `X, Y, Z (m)` and `Roll, Pitch, Yaw (°)` or click **`Copy Current EE`**.
- **`Position-Only (3D XYZ)`**: Ignores orientation constraints when solving IK or planning.
- **`IK Solver`**: `auto` (MoveIt `/compute_ik` with local DLS fallback), `moveit`, or `local`.
- **`Pipeline`**: Choose `ompl` or `pilz_industrial_motion_planner`.
- **Actions**: **`Solve IK (Preview)`**, **`Solve & Execute IK`**, and **`MoveIt Plan to Pose`**.

### Panel 5: 3D Arm Preview & Presets
- **Camera Controls**: Left-drag to orbit, scroll wheel to zoom, **`Reset View`**, **`Clear Trail`**.
- **Color Legend**: Solid **Cyan/Orange** chain = live robot + gripper jaws; Dashed **Green** chain = target/IK preview; **Slate/Teal boxes** = `pick_table` & `place_table`; **Red cube** = `pick_cube`; **RGB triads** = world origin & EE frame.
- **Quick Presets**: **`Home`**, **`Ready`**, **`Inspect`**, **`Pre Pick`**, **`Pick`**, **`Pre Place`**, **`Place`**.

### Panel 6: Teach & Playback Sequencer & Pick-and-Place Demo
- Click **`Run Pick & Place Demo`** (optionally check **`Loop`** first) to load and execute the 10-step Pick-and-Place cycle moving `pick_cube` from `pick_table` to `place_table` with coordinated gripper commands.
- Record custom programs using **`Teach Actual`** / **`Teach Target`** (stores joint angles + `Grip(mm)` opening), **`Move Selected`**, **`Delete`**, **`Run Sequence`**, and **`Save JSON`** / **`Load JSON`**.
