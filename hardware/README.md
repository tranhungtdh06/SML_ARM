# SML-Arm Mechanical Hardware (`hardware/`)

This directory holds the mechanical CAD assemblies, 3D-printable STL files, engineering drawings, and Bill of Materials (BOM) for the **SML-Arm** 6-DOF robotic manipulator (`arm_assembly_2`) and parallel 2-finger gripper end-effector.

---

## Directory Layout

```text
hardware/
├── bom/
│   └── mechanical_bom.csv   # Fasteners, bearings, harmonic drives, motors & structural links
├── cad/
│   ├── gripper/             # Source CAD & printable STLs for the parallel gripper
│   ├── step/                # Full master assembly and sub-assembly STEP files (.step/.stp)
│   └── stl_print/           # Print-oriented STL parts ready for slicing
├── drawings/                # 2D engineering drawings, tolerances & hole callouts (PDF/DXF)
└── README.md
```

---

## Mechanical & Kinematic Architecture

SML-Arm is a 6-axis articulated manipulator driven by **stepper motors paired with harmonic drives (`J1`–`J6`)**, terminating in an ISO/custom wrist flange (`ee_link`) fitted with a **parallel 2-finger gripper**.

### Joint & Actuator Specification (Synchronized with URDF)

All joint names, axes, travel limits, and velocity/effort bounds below match [`src/sml_arm_description/urdf/sml_arm_macro.xacro`](../src/sml_arm_description/urdf/sml_arm_macro.xacro):

| Joint | Type | Parent Link -> Child Link | Local Axis `(x, y, z)` | Position Limits | Effort Limit | Velocity Limit | Drive Mechanism |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `dof_joint1` | Revolute | `j1_harmonic_drive` -> `link_1` | `(0, 0, -1)` | `±3.14159 rad` (`±180°`) | `100.0 N·m` | `2.5 rad/s` | `J1` Stepper + Harmonic Drive |
| `dof_joint2` | Revolute | `j2_harmonic_drive` -> `link_2__1_` | `(-1, 0, 0)` | `±1.5708 rad` (`±90°`) | `100.0 N·m` | `2.5 rad/s` | `J2` Stepper + Harmonic Drive |
| `dof_joint3` | Revolute | `j3_harmonic_drive` -> `link_3__1_` | `(1, 0, 0)` | `±2.35619 rad` (`±135°`) | `100.0 N·m` | `2.5 rad/s` | `J3` Stepper + Harmonic Drive |
| `dof_joint4` | Revolute | `j4_harmonic_drive` -> `link_4__1_` | `(0, 0, -1)` | `±3.14159 rad` (`±180°`) | `50.0 N·m` | `2.5 rad/s` | `J4` Stepper + Harmonic Drive |
| `dof_joint5` | Revolute | `j5_harmonic_drive` -> `link_5__1_` | `(-1, 0, 0)` | `±2.0944 rad` (`±120°`) | `50.0 N·m` | `2.5 rad/s` | `J5` Stepper + Harmonic Drive |
| `dof_joint6` | Revolute | `j6_harmonic_drive` -> `ee_link` | `(0, 0, -1)` | `±3.14159 rad` (`±180°`) | `50.0 N·m` | `3.0 rad/s` | `J6` Stepper + Harmonic Drive |
| `gripper_left_finger_joint` | Prismatic | `gripper_base_link` -> `gripper_left_finger` | `(0, 1, 0)` | `0.000 – 0.040 m` (`0–40 mm`) | `50.0 N` | `0.5 m/s` | Parallel Rack / Lead Screw |
| `gripper_right_finger_joint` | Prismatic | `gripper_base_link` -> `gripper_right_finger` | `(0, -1, 0)` | `0.000 – 0.040 m` (`0–40 mm`) | `50.0 N` | `0.5 m/s` | Parallel Rack / Lead Screw |

> **Gripper Geometry Note**: With both prismatic fingers at `0.040 m`, the parallel jaw stroke spans `0–80 mm` total opening. `tcp_link` sits `+0.065 m` along local `+Z` from `gripper_base_link` (`ee_link` origin).

---

## Importing CAD & Syncing URDF Meshes

1. **Master CAD**: Place editable multi-body CAD files (`.step` / `.stp`) into `hardware/cad/step/` and print-ready STLs into `hardware/cad/stl_print/`.
2. **URDF Visual vs. Collision Meshes**:
   - High-detail CAD exports live in [`src/sml_arm_description/meshes/visual/`](../src/sml_arm_description/meshes/visual/) (`~17 MB` total).
   - Simplified low-poly convex/decimated STLs live in [`src/sml_arm_description/meshes/collision/`](../src/sml_arm_description/meshes/collision/) (`~120 KB` total) and are also used when launching simulation in lightweight mode (`fast_visual:=true`).
   - Keep origin frames identical between source CAD exports and STL meshes to avoid joint offsets in Gazebo and MoveIt 2.
3. **BOM Updates**: Record exact part numbers, materials, print infill/layer settings, and fastener lengths in [`bom/mechanical_bom.csv`](bom/mechanical_bom.csv).
