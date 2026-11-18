# SML-Arm Mechanical Assembly & Zero-Calibration Guide

This guide outlines the physical build order, electrical harness routing practices, and mechanical zero-alignment workflow so the physical **SML-Arm** matches the URDF coordinate frames used by Gazebo Harmonic and MoveIt 2.

---

## 1. Kinematic Chain & Assembly Order

Assemble from the ground upward, checking backlash and fastener torque after each joint stage:

1. **Pedestal & Axis 1 (`base_link` -> `j1_harmonic_drive` -> `link_1`)**
   - Anchor `base_link` rigidly to the workbench or optical breadboard before installing upper links.
   - Mate `J1` stepper motor (`j1_stepper_motor`) and harmonic reducer (`j1_harmonic_drive`) inside the base cavity; mount `link_1` rotating around `-Z` (`dof_joint1`, `±180°`).
2. **Shoulder Axis (`j2_harmonic_drive` -> `link_2__1_`)**
   - Install `J2` drive (`j2_stepper_motor` + `j2_harmonic_drive`) on the shoulder turret (`link_1`) and mount `link_2__1_` rotating around `-X` (`dof_joint2`, `±90°`).
3. **Elbow Axis (`j3_harmonic_drive` -> `link_3__1_`)**
   - Mount `J3` drive at the distal end of `link_2__1_` and attach `link_3__1_` rotating around `+X` (`dof_joint3`, `±135°`).
4. **Forearm Roll & Wrist Pitch (`j4_harmonic_drive` -> `link_4__1_` -> `j5_harmonic_drive` -> `link_5__1_`)**
   - Install `J4` drive (`dof_joint4`, axis `-Z`, `±180°`) carrying `link_4__1_`.
   - Install `J5` drive (`dof_joint5`, axis `-X`, `±120°`) carrying the wrist pitch clevis `link_5__1_`.
5. **Tool Flange & Parallel Gripper (`j6_harmonic_drive` -> `ee_link` -> `gripper_base_link`)**
   - Install `J6` drive (`dof_joint6`, axis `-Z`, `±180°`) and attach `gripper_base_link` flush against `ee_link` (`gripper_base_joint`).
   - Fit left and right parallel jaws (`gripper_left_finger`, `gripper_right_finger`, `0.0–0.040 m` travel per finger) with high-friction grip pads for slip-free grasping.

---

## 2. Mechanical Zero Calibration (`[0, 0, 0, 0, 0, 0]`)

To ensure hardware motion matches RViz2, Gazebo, and Control Studio previews:

1. Command or visually preview **`Home` (`[0, 0, 0, 0, 0, 0]`)** in software via:
   ```bash
   ros2 launch sml_arm_description display.launch.py
   ```
2. Mechanically position all six axes to match the upright neutral `home` pose shown in RViz2.
3. Align physical homing limit/Hall switches (`J1`–`J6`) and record calibrated homing offsets in the MCU firmware (`firmware/`) so powering up and running `SML_MSG_HOME` returns each joint cleanly to `0.000 rad`.
4. Verify sign conventions axis by axis at low speed (`Vel % = 10%`) against the right-hand rule axes listed in [`hardware/README.md`](../hardware/README.md).

---

## 3. Cable Harness & Drag Routing Notes

- Leave generous service loops (at least 15–20% slack outside full `±180°` yaw/roll travel on `dof_joint1`, `dof_joint4`, and `dof_joint6`) to prevent conductor fatigue.
- Separate high-current stepper phase leads from logic homing sensor / encoder lines where possible using shielded twisted pairs.
- Secure cables cleanly to `link_1`–`link_5__1_` so harnesses cannot pinch inside shoulder/elbow clevises or obstruct the parallel gripper jaws.
