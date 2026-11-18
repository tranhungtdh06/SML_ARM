# Contributing to SML-Arm

Thank you for contributing to **SML-Arm**! All contributions across hardware CAD, electrical schematics, embedded firmware, ROS 2 software, and documentation are welcome under the [Apache-2.0 License](LICENSE).

---

## Subsystem Guidelines

### 1. Hardware (`hardware/`)
- Export editable 3D assemblies in STEP format under `hardware/cad/step/` and 3D-printable STLs under `hardware/cad/stl_print/` (millimeter units, print orientation specified in commit/PR description).
- Keep gripper mechanisms in `hardware/cad/gripper/` and engineering drawings in `hardware/drawings/`.
- When geometry or link transforms change, synchronize the URDF/Xacro visual meshes (`src/sml_arm_description/meshes/visual/`), lightweight low-poly collision STLs (`src/sml_arm_description/meshes/collision/`), inertial properties, and joint origins in [`src/sml_arm_description/urdf/sml_arm_macro.xacro`](src/sml_arm_description/urdf/sml_arm_macro.xacro).
- Update [`hardware/bom/mechanical_bom.csv`](hardware/bom/mechanical_bom.csv) whenever fasteners, bearings, motors, or drive ratios change.

### 2. Electrical (`electrical/`)
- Commit native ECAD source files (e.g., KiCad schematics and PCB layouts) alongside PDF exports under `electrical/schematics/` and `electrical/pcb/`.
- Keep harness diagrams and connector pinouts updated in `electrical/wiring/` and [`electrical/README.md`](electrical/README.md).
- Update [`electrical/bom/electrical_bom.csv`](electrical/bom/electrical_bom.csv) with manufacturer part numbers, voltage/current ratings, and quantities.
- Clearly document any high-current or E-Stop safety interlock changes before opening a pull request.

### 3. Firmware (`firmware/`)
- Keep host-to-MCU wire protocol changes backward-compatible or bump `SML_PROTOCOL_VERSION` in [`firmware/protocol/sml_arm_protocol.h`](firmware/protocol/sml_arm_protocol.h).
- Document MCU target, clock/timer configuration, and build instructions in `firmware/README.md` or the controller subfolder.
- Ensure watchdog timeouts and hardware/software joint limits align with [`src/sml_arm_description/urdf/sml_arm_macro.xacro`](src/sml_arm_description/urdf/sml_arm_macro.xacro).

### 4. ROS 2, Simulation & Control Studio (`src/`)
Before opening a pull request touching `src/`, verify the workspace locally on ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# Validate Xacro -> URDF -> SDF
xacro src/sml_arm_description/urdf/sml_arm.urdf.xacro use_sim:=true > /tmp/sml_arm.urdf
check_urdf /tmp/sml_arm.urdf
gz sdf -p /tmp/sml_arm.urdf > /tmp/sml_arm.sdf
gz sdf -k /tmp/sml_arm.sdf
gz sdf -k src/sml_arm_gazebo/worlds/empty_arm_world.sdf

# Run headless Control Studio self-test
QT_QPA_PLATFORM=offscreen ros2 run sml_arm_control arm_control_gui.py --self-test
```

---

## Pull Request Workflow

1. Fork the repository and create a topic branch from `main` (e.g., `feat/hardware-step-v1`, `fix/moveit-limits`).
2. Keep commits focused by subsystem (`hardware`, `electrical`, `firmware`, `ros2`, `docs`).
3. Fill out the [Pull Request Template](.github/pull_request_template.md) with test evidence or photos for hardware changes.
4. Ensure GitHub Actions ROS 2 CI passes on your branch.
