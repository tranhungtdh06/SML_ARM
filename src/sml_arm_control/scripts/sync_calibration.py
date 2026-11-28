#!/usr/bin/env python3
"""Sync firmware calibration_profile.json speeds/accels to MoveIt 2 and URDF configs."""

import argparse
import json
import math
import os
import re
import sys
from typing import Dict, Tuple

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from arm_kinematics import find_calibration_profile  # noqa: E402


def _find_repo_root(cal_path: str) -> str:
    cur = os.path.dirname(os.path.realpath(cal_path))
    for _ in range(8):
        if os.path.isdir(os.path.join(cur, 'src', 'sml_arm_moveit_config')):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    raise FileNotFoundError('Could not locate workspace root containing src/sml_arm_moveit_config')


def _load_joint_specs(cal_path: str) -> Dict[int, Tuple[float, float, float, float]]:
    with open(cal_path, 'r', encoding='utf-8') as f:
        joints = json.load(f)['joints']
    specs: Dict[int, Tuple[float, float, float, float]] = {}
    for j in range(1, 7):
        entry = joints[str(j)]
        spd_deg = float(entry['SPEED'])
        acc_deg = float(entry['ACCEL'])
        specs[j] = (spd_deg, acc_deg, math.radians(spd_deg), math.radians(acc_deg))
    return specs


def _update_joint_limits_yaml(yaml_path: str, specs: Dict[int, Tuple[float, float, float, float]]) -> None:
    with open(yaml_path, 'r', encoding='utf-8') as f:
        text = f.read()

    for j in range(1, 7):
        spd_deg, acc_deg, spd_rad, acc_rad = specs[j]
        pattern = re.compile(
            rf'(  dof_joint{j}:\n'
            rf'    has_velocity_limits: true\n'
            rf'    max_velocity:\s*)[^\n]+(\n'
            rf'    has_acceleration_limits: true\n'
            rf'    max_acceleration:\s*)[^\n]+'
        )
        replacement = (
            rf'\g<1>{spd_rad:.4f}     # {spd_deg:.2f} deg/s'
            rf'\g<2>{acc_rad:.4f} # {acc_deg:.2f} deg/s^2'
        )
        text, count = pattern.subn(replacement, text, count=1)
        if count != 1:
            raise RuntimeError(f'Failed to update dof_joint{j} in {yaml_path}')

    with open(yaml_path, 'w', encoding='utf-8') as f:
        f.write(text)


def _update_urdf_macro(xacro_path: str, specs: Dict[int, Tuple[float, float, float, float]]) -> None:
    with open(xacro_path, 'r', encoding='utf-8') as f:
        text = f.read()

    for j in range(1, 7):
        _, _, spd_rad, _ = specs[j]
        pattern = re.compile(
            rf'(<joint name="\$\{{prefix\}}dof_joint{j}" type="revolute">.*?<limit [^>]*velocity=")[^"]+("/>)',
            re.DOTALL,
        )
        text, count = pattern.subn(rf'\g<1>{spd_rad:.4f}\g<2>', text, count=1)
        if count != 1:
            raise RuntimeError(f'Failed to update dof_joint{j} velocity limit in {xacro_path}')

    with open(xacro_path, 'w', encoding='utf-8') as f:
        f.write(text)


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Sync calibration_profile.json speed/accel to joint_limits.yaml and sml_arm_macro.xacro.'
    )
    parser.add_argument(
        '--profile',
        default=None,
        help='Path to calibration_profile.json (auto-detected if omitted).',
    )
    args = parser.parse_args()

    cal_path = args.profile or find_calibration_profile()
    if not cal_path or not os.path.isfile(cal_path):
        print('ERROR: calibration_profile.json not found.', file=sys.stderr)
        return 1

    root = _find_repo_root(cal_path)
    yaml_path = os.path.join(root, 'src', 'sml_arm_moveit_config', 'config', 'joint_limits.yaml')
    xacro_path = os.path.join(root, 'src', 'sml_arm_description', 'urdf', 'sml_arm_macro.xacro')

    specs = _load_joint_specs(cal_path)
    _update_joint_limits_yaml(yaml_path, specs)
    _update_urdf_macro(xacro_path, specs)

    print(f'Synced calibration from: {cal_path}')
    for j in range(1, 7):
        spd_deg, acc_deg, spd_rad, acc_rad = specs[j]
        print(
            f'  dof_joint{j}: {spd_deg:6.2f} deg/s ({spd_rad:.4f} rad/s), '
            f'{acc_deg:6.2f} deg/s^2 ({acc_rad:.4f} rad/s^2)'
        )
    print(f'Updated:\n  - {yaml_path}\n  - {xacro_path}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
