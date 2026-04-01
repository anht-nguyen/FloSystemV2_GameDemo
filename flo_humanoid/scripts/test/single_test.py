import sys
from time import sleep
from pathlib import Path
from dynamixel_sdk import PortHandler, PacketHandler

repo_root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(repo_root / "utility"))
from device_paths import get_device_paths

port = get_device_paths()["motors"]; baud=1000000; dxl_id=112
ph=PortHandler(port); ph.openPort(); ph.setBaudRate(baud)
pk=PacketHandler(2.0)

# Disable torque first to avoid continued overcurrent
pk.write1ByteTxRx(ph, dxl_id, 64, 0)
# Reboot and wait 0.7s
pk.reboot(ph, dxl_id); sleep(0.7)

def r1(a): return pk.read1ByteTxRx(ph, dxl_id, a)[0]
def r2(a): return pk.read2ByteTxRx(ph, dxl_id, a)[0]

hw = r1(70); vin = r2(144); temp = r1(146); tq = r1(64); opm = r1(11)
print(f"HW_ERR(70)={hw} (bin {hw:08b})  Vin={vin/10.0:.1f}V  Temp={temp}C  Torque(64)={tq}  OperMode(11)={opm}")

ph.closePort()
