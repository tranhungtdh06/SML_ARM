## Summary
Briefly describe the changes made in this pull request.

## Affected Subsystems
- [ ] Hardware CAD / BOM (`hardware/`)
- [ ] Electrical / Schematics / BOM (`electrical/`)
- [ ] Firmware / Protocol (`firmware/`)
- [ ] ROS 2 Packages (`src/`)
- [ ] Documentation (`docs/`, `README.md`)

## Verification Checklist
- [ ] `colcon build --symlink-install` succeeds cleanly
- [ ] `check_urdf` / `gz sdf -k` passes if URDF, meshes, or SDF files were modified
- [ ] `QT_QPA_PLATFORM=offscreen ros2 run sml_arm_control arm_control_gui.py --self-test` passes if control/GUI scripts were modified
- [ ] Physical hardware photos / simulation screenshots / logs attached where helpful
