# SML-Arm Embedded Firmware (`firmware/`)

This directory contains the microcontroller (MCU) firmware, gripper controller firmware, and the shared binary host-to-MCU communication protocol bridging real hardware to ROS 2 (`ros2_control`).

---

## Directory Layout

```text
firmware/
├── gripper_controller/    # Standalone or daisy-chained gripper actuator firmware
├── mcu_controller/        # Main 6-axis motion controller firmware (stepper/encoder/safety loop)
├── protocol/
│   └── sml_arm_protocol.h # Shared packed C/C++ serial/CAN frame & CRC-16 definitions
└── README.md
```

---

## Real-Time Control Architecture

The MCU sits immediately below the ROS 2 `ros2_control` hardware interface (`update_rate: 100 Hz` configured in [`src/sml_arm_control/config/sml_arm_controllers.yaml`](../src/sml_arm_control/config/sml_arm_controllers.yaml)):

1. **Deterministic Motion Loop (Target >= 1–10 kHz step/current ISR, 100–250 Hz host telemetry)**:
   - Receives `SmlArmCommandFrame` target joint angles (`dof_joint1`..`dof_joint6` in rad) and finger stroke (`0.0–0.040 m`).
   - Enforces software joint limits identical to [`src/sml_arm_description/urdf/sml_arm_macro.xacro`](../src/sml_arm_description/urdf/sml_arm_macro.xacro).
   - Converts target joint angles -> motor steps / encoder counts via each axis harmonic drive gear ratio.
   - Generates smooth pulse trains or fieldbus setpoints and reads actual joint positions/velocities back into `SmlArmStateFrame`.
2. **Homing & Calibration**:
   - Executes sequential or grouped axis homing (`SML_MSG_HOME`) onto limit/Hall switches and applies calibrated zero offsets so real hardware zero aligns with URDF `[0, 0, 0, 0, 0, 0]`.
3. **Safety Interlocks & Watchdog**:
   - Monitors host heartbeat timeout; if host frames stop arriving within the watchdog window, transitions outputs into controlled hold/disable.
   - Monitors hardware E-Stop (`SML_STATUS_ESTOP`) and per-axis driver fault signals (`SML_STATUS_DRIVER_FAULT`), immediately halting step generation and reporting status back to ROS 2.

---

## Host-to-MCU Binary Protocol (`protocol/sml_arm_protocol.h`)

- **Magic Header**: `0x53 0x4D` (`'S'`, `'M'`)
- **Protocol Version**: `1`
- **Endianness**: Little-endian
- **Integrity**: CRC-16-CCITT (`0x1021`, initial value `0xFFFF`) over payload bytes up to `crc16`.
- **Supported Message IDs**:
  - `SML_MSG_CMD_JOINTS` (`0x01`): Target positions (`float[6]`, rad), velocities (`float[6]`, rad/s), and left/right gripper openings (`float[2]`, m).
  - `SML_MSG_STATE_JOINTS` (`0x02`): Measured joint positions/velocities, gripper openings, and axis status bitmask.
  - `SML_MSG_HOME` (`0x03`): Request axis homing bitmask (`bit 0..5` arm axes, `bit 6` gripper).
  - `SML_MSG_ESTOP` (`0x04`): Software emergency halt / clear-fault request.

Compile/include `firmware/protocol/sml_arm_protocol.h` on both the MCU firmware target and a future C++ `ros2_control` `hardware_interface::SystemInterface` plugin to guarantee binary layout compatibility.
