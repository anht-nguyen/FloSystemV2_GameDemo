#!/usr/bin/env python3

"""
Standalone pose testing script (no ROS/MoveIt). Sends Dynamixel goal positions directly
via Dynamixel SDK.

What it does:
- Read named poses (group_state) and joint angles (radians) from SRDF
- Read joint ID mapping and mechanical zero offsets (degrees) from YAML
- Select pose/arm/repeat from CLI and command Dynamixels

Notes:
- Assumes X-series, Protocol 2.0, Position Control Mode (Operating Mode = 3)
- Commands 4 joints per arm (l1..l4 / r1..r4)
- Dependencies:
    pip3 install dynamixel-sdk pyyaml

Examples:
  Right arm waving 3 cycles (SRDF sequence R_wave_start/R_wave_end):
    python3 arm_poses_test.py \
      --ports /dev/ttyUSB0 \
      --arm right \
      --pose wave \
      --repeat 3

  Use an SRDF pose name directly (single move):
    python3 arm_poses_test.py --ports /dev/ttyUSB0 --pose R_raise --arm right

  Dual ports, left/right on USB0/USB1:
    python3 arm_poses_test.py \
      --ports /dev/ttyUSB0 /dev/ttyUSB1 \
      --arm left --pose L_reach_side
"""
"""
CMD EXAMPLE:
python3 /home/rrl/FloSystemV2_GameDemo/flo_humanoid/scripts/test/arm_poses_test.py \
  --ports /dev/ttyUSB0 \
  --arm left \
  --pose swing_lateral \
  --repeat 5 \
  --init --eeprom-wait-ms 50 \
  --profile-accel 500 --profile-vel 200 \
  --p-gain 80 --d-gain 24 \
  --final-torque-off
"""

import argparse
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List, Tuple

import yaml

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except Exception as exc:
    raise SystemExit("请先安装 dynamixel-sdk: pip3 install dynamixel-sdk\n错误: {}".format(exc))


# Control Table Addresses (X-series / Protocol 2.0)
ADDR_TORQUE_ENABLE = 64
ADDR_DRIVE_MODE = 10
ADDR_OPER_MODE = 11
ADDR_GOAL_POSITION = 116
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84


def degrees_to_dxl_ticks(angle_deg: float) -> int:
    angle = angle_deg % 360.0
    ticks = int(round((angle / 360.0) * 4096.0))
    return max(0, min(4095, ticks))


def open_ports(devs: List[str], baudrate: int) -> Dict[str, PortHandler]:
    handlers: Dict[str, PortHandler] = {}
    for dev in devs:
        ph = PortHandler(dev)
        if not ph.openPort():
            print(f"[ERR] 打开端口失败: {dev}")
            continue
        if not ph.setBaudRate(baudrate):
            print(f"[ERR] 设置波特率失败: {dev} @ {baudrate}")
            ph.closePort()
            continue
        handlers[dev] = ph
        print(f"[OK ] 打开端口: {dev} @ {baudrate}")
    return handlers


def load_yaml(path: Path) -> dict:
    with open(path, 'r') as f:
        return yaml.safe_load(f)


def parse_srdf_named_states(srdf_path: Path) -> Dict[str, Dict[str, float]]:
    """Returns: {state_name: {joint_name: value_in_radians}}"""
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    ns: Dict[str, Dict[str, float]] = {}
    for gs in root.findall('group_state'):
        name = gs.get('name')
        if not name:
            continue
        joints: Dict[str, float] = {}
        for j in gs.findall('joint'):
            jn = j.get('name')
            val = float(j.get('value'))
            joints[jn] = val
        ns[name] = joints
    return ns


def comm_ok(packet: PacketHandler, comm_result: int, dxl_error: int, note: str = "") -> bool:
    if comm_result != 0 or dxl_error != 0:
        print("[ERR] {} comm={}({}), err={}({})".format(
            note, comm_result, packet.getTxRxResult(comm_result), dxl_error, packet.getRxPacketError(dxl_error)))
        return False
    return True


def ensure_operational(packet: PacketHandler, ph: PortHandler, dxl_ids: List[int], *,
                       set_mode_3: bool, eeprom_wait_ms: int,
                       profile_accel: int, profile_vel: int,
                       p_gain: int = None, i_gain: int = None, d_gain: int = None) -> None:
    for dxl_id in dxl_ids:
        # torque off
        cr, er = packet.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 0)
        comm_ok(packet, cr, er, f"{dxl_id}: torque off")
        # mode = 3
        if set_mode_3:
            cr, er = packet.write1ByteTxRx(ph, dxl_id, ADDR_OPER_MODE, 3)
            comm_ok(packet, cr, er, f"{dxl_id}: set opmode=3")
            time.sleep(eeprom_wait_ms / 1000.0)
        # profile
        if profile_accel is not None:
            cr, er = packet.write4ByteTxRx(ph, dxl_id, ADDR_PROFILE_ACCELERATION, int(profile_accel))
            comm_ok(packet, cr, er, f"{dxl_id}: profile accel")
        if profile_vel is not None:
            cr, er = packet.write4ByteTxRx(ph, dxl_id, ADDR_PROFILE_VELOCITY, int(profile_vel))
            comm_ok(packet, cr, er, f"{dxl_id}: profile vel")
        # PID gains（2 字节）
        if p_gain is not None:
            cr, er = packet.write2ByteTxRx(ph, dxl_id, ADDR_POSITION_P_GAIN, int(p_gain))
            comm_ok(packet, cr, er, f"{dxl_id}: P gain")
        if i_gain is not None:
            cr, er = packet.write2ByteTxRx(ph, dxl_id, ADDR_POSITION_I_GAIN, int(i_gain))
            comm_ok(packet, cr, er, f"{dxl_id}: I gain")
        if d_gain is not None:
            cr, er = packet.write2ByteTxRx(ph, dxl_id, ADDR_POSITION_D_GAIN, int(d_gain))
            comm_ok(packet, cr, er, f"{dxl_id}: D gain")
        # torque on
        cr, er = packet.write1ByteTxRx(ph, dxl_id, ADDR_TORQUE_ENABLE, 1)
        comm_ok(packet, cr, er, f"{dxl_id}: torque on")


def plan_pose_sequence(arm: str, pose: str, repeat: int) -> List[str]:
    arm = arm.lower()
    # Highest-level semantic to SRDF pose name sequence minimum mapping
    if pose.lower() == 'wave':
        if arm.startswith('r'):
            return ["R_wave_start", "R_wave_end"] * max(1, repeat)
        else:
            return ["L_wave_start", "L_wave_end"] * max(1, repeat)
    if pose.lower() == 'raise':
        return ["R_raise"] * repeat if arm.startswith('r') else ["L_raise"] * repeat
    if pose.lower() in ('reach_side', 'reach'):
        return ["R_reach_side"] * repeat if arm.startswith('r') else ["L_reach_side"] * repeat
    if pose.lower() in ('swing_lateral', 'waveb'):
        return ["R_waveb", "R_d_bell"] * max(1, repeat) if arm.startswith('r') else ["L_waveb", "L_d_bell"] * max(1, repeat)
    # Directly use pose as SRDF pose name
    return [pose]


def apply_pose(packet: PacketHandler,
               left_port: PortHandler,
               right_port: PortHandler,
               joint_id_map: Dict[str, int],
               offsets_deg: Dict[str, float],
               srdf_states: Dict[str, Dict[str, float]],
               pose_name: str,
               *,
               dry_run: bool = False) -> None:
    if pose_name not in srdf_states:
        raise ValueError(f"SRDF did not find pose: {pose_name}")

    joints_rad = srdf_states[pose_name]

    # Target angle (degrees), by bridging logic: most joints take deg + offset; r4 takes -deg + offset
    target_deg: Dict[str, float] = {}
    for jn, val_rad in joints_rad.items():
        deg = math.degrees(val_rad)
        if jn in ('l1', 'r4'):
            deg = -deg
        target_deg[jn] = deg

    # Apply offsets
    for jn, deg in list(target_deg.items()):
        off = offsets_deg.get(jn, 0.0)
        target_deg[jn] = deg + off

    # Left and right ports need to find IDs separately
    # Construct (port, id, ticks) list
    cmds: List[Tuple[PortHandler, int, int]] = []
    for jn, deg in target_deg.items():
        dxl_id = joint_id_map.get(jn)
        if dxl_id is None:
            continue
        ticks = degrees_to_dxl_ticks(deg)
        ph = left_port if jn.startswith('l') else right_port
        if ph is None:
            raise RuntimeError(f"未提供 {'左' if jn.startswith('l') else '右'}臂串口，但姿态需要 {jn}")
        cmds.append((ph, dxl_id, ticks))

    # 发送
    for ph, dxl_id, ticks in cmds:
        if dry_run:
            print(f"[DRY] ID {dxl_id} → {ticks}")
            continue
        cr, er = packet.write4ByteTxRx(ph, dxl_id, ADDR_GOAL_POSITION, ticks)
        comm_ok(packet, cr, er, f"goal pos id={dxl_id}")


def main():
    # Project root directory: .../FloSystemV2_GameDemo
    repo_root = Path(__file__).resolve().parents[3]
    # Configuration files are in the package: flo_humanoid/config and flo_core/config
    default_ids = repo_root / 'flo_humanoid' / 'config' / 'dynamixel_ids.yaml'
    default_offsets = repo_root / 'flo_humanoid' / 'config' / 'dynamixel_offsets.yaml'
    default_srdf = repo_root / 'flo_core' / 'config' / 'flov2_robot_description.srdf'

    ap = argparse.ArgumentParser(description="Standalone pose testing (Dynamixel SDK)")
    ap.add_argument('--ports', nargs='+', required=True, help='Serial device (1 or 2): /dev/ttyUSB0 [/dev/ttyUSB1]')
    ap.add_argument('--baudrate', type=int, default=1000000)
    ap.add_argument('--pose', required=True, help='Pose name or high-level action alias: wave/raise/reach_side/... or SRDF group_state name')
    ap.add_argument('--arm', choices=['left','right','dual'], default='right', help='Select action arm (for high-level aliases)')
    ap.add_argument('--repeat', type=int, default=1, help='High-level alias action cycles')
    ap.add_argument('--ids-yaml', type=str, default=str(default_ids))
    ap.add_argument('--offsets-yaml', type=str, default=str(default_offsets))
    ap.add_argument('--srdf', type=str, default=str(default_srdf))
    ap.add_argument('--init', action='store_true', help='Initialize before execution (torque off/set mode 3/set Profile/torque on)')
    ap.add_argument('--eeprom-wait-ms', type=int, default=50)
    ap.add_argument('--profile-accel', type=int, default=50, help='Profile Accel (4bytes)')
    ap.add_argument('--profile-vel', type=int, default=50, help='Profile Vel (4bytes)')
    ap.add_argument('--p-gain', type=int, help='Position P Gain (2bytes)')
    ap.add_argument('--i-gain', type=int, help='Position I Gain (2bytes)')
    ap.add_argument('--d-gain', type=int, help='Position D Gain (2bytes)')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--final-torque-off', action='store_true', help='Turn off torque for the IDs involved in this run')
    args = ap.parse_args()

    ids_cfg = load_yaml(Path(args.ids_yaml))
    offsets_cfg = load_yaml(Path(args.offsets_yaml))
    srdf_states = parse_srdf_named_states(Path(args.srdf))

    joint_id_map: Dict[str, int] = ids_cfg['joint_id_map']
    offsets_deg: Dict[str, float] = offsets_cfg['offsets']

    # Open ports
    ports = open_ports(args.ports, args.baudrate)
    if not ports:
        raise SystemExit('No available ports, exit.')
    # Select left/right ports (1 port when left/right same port; 2 ports when left/right in order)
    devs = list(ports.keys())
    left_port = ports[devs[0]]
    right_port = ports[devs[0]] if len(devs) == 1 else ports[devs[1]]

    packet = PacketHandler(2.0)
    
    # ID set to control is determined by pose
    seq = plan_pose_sequence(args.arm, args.pose, args.repeat)
    
    joint_names: List[str] = []
    for pose_name in seq:
        if pose_name not in srdf_states:
            print(f"[WARN] SRDF did not find pose: {pose_name}, will skip")
            continue
        for jn in srdf_states[pose_name].keys():
            if jn not in joint_names:
                joint_names.append(jn)
    dxl_ids = [joint_id_map[jn] for jn in joint_names if jn in joint_id_map]
    # Left/right ports separately involved IDs (calculate regardless of initialization, used for --final-torque-off)
    left_ids = [joint_id_map[jn] for jn in joint_names if jn.startswith('l') and jn in joint_id_map]
    right_ids = [joint_id_map[jn] for jn in joint_names if jn.startswith('r') and jn in joint_id_map]

    if args.init and dxl_ids:
        # Left/right ports separately initialize their IDs (simply by joint name归属)
        if left_ids:
            ensure_operational(packet, left_port, left_ids,
                               set_mode_3=True,
                               eeprom_wait_ms=args.eeprom_wait_ms,
                               profile_accel=args.profile_accel,
                               profile_vel=args.profile_vel,
                               p_gain=args.p_gain, i_gain=args.i_gain, d_gain=args.d_gain)
        if right_ids:
            ensure_operational(packet, right_port, right_ids,
                               set_mode_3=True,
                               eeprom_wait_ms=args.eeprom_wait_ms,
                               profile_accel=args.profile_accel,
                               profile_vel=args.profile_vel,
                               p_gain=args.p_gain, i_gain=args.i_gain, d_gain=args.d_gain)

    # Send each pose one by one
    for pose_name in seq:
        if pose_name not in srdf_states:
            continue
        print(f"[POSE] {pose_name}")
        apply_pose(packet, left_port, right_port, joint_id_map, offsets_deg, srdf_states, pose_name, dry_run=args.dry_run)
        time.sleep(0.6)

    # Optional: turn off torque after execution
    if args.final_torque_off:
        for i in left_ids:
            cr, er = packet.write1ByteTxRx(left_port, i, ADDR_TORQUE_ENABLE, 0)
            comm_ok(packet, cr, er, f"left torque off id={i}")
        for i in right_ids:
            cr, er = packet.write1ByteTxRx(right_port, i, ADDR_TORQUE_ENABLE, 0)
            comm_ok(packet, cr, er, f"right torque off id={i}")

    # End: do not automatically turn off torque, avoid arm sagging; if needed, add manually
    for ph in ports.values():
        try:
            ph.closePort()
        except Exception:
            pass


if __name__ == '__main__':
    main()


