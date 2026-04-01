#!/usr/bin/env python3

"""

  python3 flo_humanoid/scripts/dxl_self_test.py \
    --ports /dev/ttyUSB0 /dev/ttyUSB1 \
    --ids 111 112 211 212 \
    --baudrate 1000000 \
    --write-test

"""

import argparse
import sys
import time
from typing import Dict, List, Tuple

try:
    from dynamixel_sdk import PortHandler, PacketHandler
except Exception as exc:
    print("[FATAL] cannot import dynamixel_sdk, please execute: pip install dynamixel-sdk\nerror: {}".format(exc))
    sys.exit(1)


# Control table addresses (X-series / Protocol 2.0)
ADDR_MODEL_NUMBER = 0              # 2 bytes
ADDR_DRIVE_MODE = 10               # 1 byte
ADDR_OPER_MODE = 11                # 1 byte
ADDR_TORQUE_ENABLE = 64            # 1 byte
ADDR_HARDWARE_ERROR_STATUS = 70    # 1 byte
ADDR_POSITION_D_GAIN = 80          # 2 bytes
ADDR_POSITION_I_GAIN = 82          # 2 bytes
ADDR_POSITION_P_GAIN = 84          # 2 bytes
ADDR_PROFILE_ACCELERATION = 108    # 4 bytes
ADDR_PROFILE_VELOCITY = 112        # 4 bytes
ADDR_PRESENT_POSITION = 132        # 4 bytes
ADDR_PRESENT_INPUT_VOLTAGE = 144   # 2 bytes (0.1V)
ADDR_PRESENT_TEMPERATURE = 146     # 1 byte (C)


def comm_ok(packet: PacketHandler, comm_result: int, dxl_error: int, intro: str = "") -> bool:
    if comm_result != 0 or dxl_error != 0:
        print("[ERR] {} comm={}({}), err={}({})".format(
            intro,
            comm_result, packet.getTxRxResult(comm_result),
            dxl_error, packet.getRxPacketError(dxl_error)
        ))
        return False
    return True


def read_diag(port: PortHandler, packet: PacketHandler, dxl_id: int) -> Dict[str, float]:
    # Hardware Error
    dxl_comm_result, dxl_err, hw = _read1(packet, port, dxl_id, ADDR_HARDWARE_ERROR_STATUS)
    comm_ok(packet, dxl_comm_result, dxl_err, "read HW_ERR(70)")
    print("  - HW_ERR(70): {}".format(hw))

    # Voltage
    dxl_comm_result, dxl_err, vin = _read2(packet, port, dxl_id, ADDR_PRESENT_INPUT_VOLTAGE)
    comm_ok(packet, dxl_comm_result, dxl_err, "read Vin(144)")
    print("  - Vin(144): {:.1f} V".format(vin / 10.0))

    # Temperature
    dxl_comm_result, dxl_err, temp = _read1(packet, port, dxl_id, ADDR_PRESENT_TEMPERATURE)
    comm_ok(packet, dxl_comm_result, dxl_err, "read Temp(146)")
    print("  - Temp(146): {} C".format(temp))

    # Torque enable
    dxl_comm_result, dxl_err, torque = _read1(packet, port, dxl_id, ADDR_TORQUE_ENABLE)
    comm_ok(packet, dxl_comm_result, dxl_err, "read Torque(64)")
    print("  - Torque(64): {}".format(torque))

    # Operating Mode
    dxl_comm_result, dxl_err, opm = _read1(packet, port, dxl_id, ADDR_OPER_MODE)
    comm_ok(packet, dxl_comm_result, dxl_err, "read OperMode(11)")
    print("  - OperMode(11): {}".format(opm))

    # Drive Mode
    dxl_comm_result, dxl_err, drv = _read1(packet, port, dxl_id, ADDR_DRIVE_MODE)
    comm_ok(packet, dxl_comm_result, dxl_err, "read DriveMode(10)")
    print("  - DriveMode(10): {}".format(drv))

    return {
        "hw_err": hw,
        "vin": vin,
        "temp": temp,
        "torque": torque,
        "opm": opm,
        "drv": drv,
    }


def _read1(packet: PacketHandler, port: PortHandler, dxl_id: int, addr: int) -> Tuple[int, int, int]:
    val, dxl_comm_result, dxl_error = packet.read1ByteTxRx(port, dxl_id, addr)
    if isinstance(val, tuple):  # Compatible with the return order of some versions
        val, dxl_comm_result, dxl_error = val
    return dxl_comm_result, dxl_error, int(val)


def _read2(packet: PacketHandler, port: PortHandler, dxl_id: int, addr: int) -> Tuple[int, int, int]:
    val, dxl_comm_result, dxl_error = packet.read2ByteTxRx(port, dxl_id, addr)
    if isinstance(val, tuple):
        val, dxl_comm_result, dxl_error = val
    return dxl_comm_result, dxl_error, int(val)


def _read4(packet: PacketHandler, port: PortHandler, dxl_id: int, addr: int) -> Tuple[int, int, int]:
    val, dxl_comm_result, dxl_error = packet.read4ByteTxRx(port, dxl_id, addr)
    if isinstance(val, tuple):
        val, dxl_comm_result, dxl_error = val
    return dxl_comm_result, dxl_error, int(val)


def _write1(packet: PacketHandler, port: PortHandler, dxl_id: int, addr: int, value: int) -> Tuple[int, int]:
    dxl_comm_result, dxl_error = packet.write1ByteTxRx(port, dxl_id, addr, int(value))
    return dxl_comm_result, dxl_error


def _write2(packet: PacketHandler, port: PortHandler, dxl_id: int, addr: int, value: int) -> Tuple[int, int]:
    dxl_comm_result, dxl_error = packet.write2ByteTxRx(port, dxl_id, addr, int(value))
    return dxl_comm_result, dxl_error


def _write4(packet: PacketHandler, port: PortHandler, dxl_id: int, addr: int, value: int) -> Tuple[int, int]:
    dxl_comm_result, dxl_error = packet.write4ByteTxRx(port, dxl_id, addr, int(value))
    return dxl_comm_result, dxl_error


def test_one_motor(port: PortHandler, packet: PacketHandler, dxl_id: int, args: argparse.Namespace) -> None:
    print("\n===== Test ID {} =====".format(dxl_id))

    # Ping（get model number）
    try:
        model_number, dxl_comm_result, dxl_error = packet.ping(port, dxl_id)
    except TypeError:
        # compatible with old SDK return style
        dxl_comm_result, dxl_error, model_number = packet.ping(port, dxl_id)
    if dxl_comm_result != 0 or dxl_error != 0:
        print("[ERR] Ping failed: comm={}({}), err={}({})".format(
            dxl_comm_result, packet.getTxRxResult(dxl_comm_result),
            dxl_error, packet.getRxPacketError(dxl_error)))
        return
    print("  - Ping OK, ModelNumber: {}".format(model_number))

    # basic diagnosis
    diag = read_diag(port, packet, dxl_id)

    if not args.write_test:
        return

    # read original parameters
    _, _, orig_opm = _read1(packet, port, dxl_id, ADDR_OPER_MODE)
    _, _, orig_prof_acc = _read4(packet, port, dxl_id, ADDR_PROFILE_ACCELERATION)
    _, _, orig_prof_vel = _read4(packet, port, dxl_id, ADDR_PROFILE_VELOCITY)
    _, _, orig_p = _read2(packet, port, dxl_id, ADDR_POSITION_P_GAIN)
    _, _, orig_d = _read2(packet, port, dxl_id, ADDR_POSITION_D_GAIN)

    # turn off torque (torque off)
    dxl_comm_result, dxl_error = _write1(packet, port, dxl_id, ADDR_TORQUE_ENABLE, 0)
    if not comm_ok(packet, dxl_comm_result, dxl_error, "torque off"):
        return
    # confirm torque is off
    _, _, torque_val = _read1(packet, port, dxl_id, ADDR_TORQUE_ENABLE)
    if torque_val != 0:
        print("[ERR] Torque is not off, abort write test.")
        return

    # write position mode(11) = 3 (only when needed or forced test)
    if args.force_set_mode or orig_opm != 3:
        dxl_comm_result, dxl_error = _write1(packet, port, dxl_id, ADDR_OPER_MODE, 3)
        if not comm_ok(packet, dxl_comm_result, dxl_error, "set opmode=3"):
            return
        time.sleep(args.eeprom_wait_ms / 1000.0)  # EEPROM backup wait

    # write Profile Accel / Vel (RAM)
    test_acc = args.profile_accel
    test_vel = args.profile_vel

    # write test values
    dxl_comm_result, dxl_error = _write4(packet, port, dxl_id, ADDR_PROFILE_ACCELERATION, test_acc)
    if not comm_ok(packet, dxl_comm_result, dxl_error, "write profile accel"):
        return
    dxl_comm_result, dxl_error = _write4(packet, port, dxl_id, ADDR_PROFILE_VELOCITY, test_vel)
    if not comm_ok(packet, dxl_comm_result, dxl_error, "write profile vel"):
        return

    # write P/D (leave I gain unchanged)
    dxl_comm_result, dxl_error = _write2(packet, port, dxl_id, ADDR_POSITION_P_GAIN, orig_p)
    if not comm_ok(packet, dxl_comm_result, dxl_error, "write P gain (same)"):
        return
    dxl_comm_result, dxl_error = _write2(packet, port, dxl_id, ADDR_POSITION_D_GAIN, orig_d)
    if not comm_ok(packet, dxl_comm_result, dxl_error, "write D gain (same)"):
        return

    # read back and confirm
    _, _, chk_acc = _read4(packet, port, dxl_id, ADDR_PROFILE_ACCELERATION)
    _, _, chk_vel = _read4(packet, port, dxl_id, ADDR_PROFILE_VELOCITY)
    _, _, chk_p = _read2(packet, port, dxl_id, ADDR_POSITION_P_GAIN)
    _, _, chk_d = _read2(packet, port, dxl_id, ADDR_POSITION_D_GAIN)
    print("  - after write check: Accel={}, Vel={}, P={}, D={}".format(chk_acc, chk_vel, chk_p, chk_d))

    # optional torque cycle
    if args.torque_cycle:
        for val in (1, 0):
            dxl_comm_result, dxl_error = _write1(packet, port, dxl_id, ADDR_TORQUE_ENABLE, val)
            comm_ok(packet, dxl_comm_result, dxl_error, f"torque={val}")
            time.sleep(0.02)

    # restore original parameters
    _write4(packet, port, dxl_id, ADDR_PROFILE_ACCELERATION, orig_prof_acc)
    _write4(packet, port, dxl_id, ADDR_PROFILE_VELOCITY, orig_prof_vel)
    if orig_opm != 3 and args.restore_mode:
        _write1(packet, port, dxl_id, ADDR_OPER_MODE, orig_opm)
        time.sleep(args.eeprom_wait_ms / 1000.0)

    # keep or turn off torque as needed
    final_torque = 0 if args.leave_torque_off else 1
    dxl_comm_result, dxl_error = _write1(packet, port, dxl_id, ADDR_TORQUE_ENABLE, final_torque)
    comm_ok(packet, dxl_comm_result, dxl_error, f"final torque={final_torque}")


def open_ports(ports: List[str], baudrate: int) -> Dict[str, PortHandler]:
    handlers: Dict[str, PortHandler] = {}
    for dev in ports:
        ph = PortHandler(dev)
        if not ph.openPort():
            print(f"[ERR] open port failed: {dev}")
            continue
        if not ph.setBaudRate(baudrate):
            print(f"[ERR] set baudrate failed: {dev} @ {baudrate}")
            ph.closePort()
            continue
        handlers[dev] = ph
        print(f"[OK ] open port: {dev} @ {baudrate}")
    return handlers


def main():
    parser = argparse.ArgumentParser(description="Dynamixel SDK independent self-test script (connection and register read/write)")
    parser.add_argument("--ports", nargs="+", required=True, help="Serial device list, e.g. /dev/ttyUSB0 /dev/ttyUSB1")
    parser.add_argument("--ids", nargs="+", type=int, required=True, help="Dynamixel ID list, e.g. 111 112 211 212")
    parser.add_argument("--baudrate", type=int, default=1000000, help="Baud rate, default 1000000")
    parser.add_argument("--protocol", type=float, default=2.0, help="Protocol version, default 2.0")
    parser.add_argument("--write-test", action="store_true", help="Execute write test (torque off)")
    parser.add_argument("--force-set-mode", action="store_true", help="Force write OPER_MODE even if it's already 3")
    parser.add_argument("--restore-mode", action="store_true", help="After test, restore OPER_MODE to original value")
    parser.add_argument("--leave-torque-off", action="store_true", help="Keep torque off after test")
    parser.add_argument("--torque-cycle", action="store_true", help="Do one torque on/off cycle test")
    parser.add_argument("--profile-accel", type=int, default=50, help="Test Profile Accel, default 50")
    parser.add_argument("--profile-vel", type=int, default=50, help="Test Profile Vel, default 50")
    parser.add_argument("--eeprom-wait-ms", type=int, default=50, help="Wait milliseconds after writing EEPROM(OPER_MODE), default 50ms")
    parser.add_argument("--port-map", action="store_true", help="Guess port by ID (simple rule: <200 go first port, >=200 go second port)")

    args = parser.parse_args()

    packet = PacketHandler(args.protocol)

    ports = open_ports(args.ports, args.baudrate)
    if not ports:
        print("[FATAL] No available ports, exit.")
        sys.exit(2)

    # simple port mapping: if --port-map is enabled, use ports[0] for ID < 200, otherwise use ports[1]
    dev_list = list(ports.keys())
    for dxl_id in args.ids:
        target_port: PortHandler = None
        if args.port_map and len(dev_list) >= 2:
            target_port = ports[dev_list[0 if dxl_id < 200 else 1]]
        else:
            # default to use first port
            target_port = ports[dev_list[0]]

        test_one_motor(target_port, packet, dxl_id, args)

    # close ports
    for ph in ports.values():
        try:
            ph.closePort()
        except Exception:
            pass


if __name__ == "__main__":
    main()

