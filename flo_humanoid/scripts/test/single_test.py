from time import sleep
from dynamixel_sdk import PortHandler, PacketHandler
port='/dev/ttyUSB0'; baud=1000000; dxl_id=112
ph=PortHandler(port); ph.openPort(); ph.setBaudRate(baud)
pk=PacketHandler(2.0)

# 先关扭矩，避免过流继续发生
pk.write1ByteTxRx(ph, dxl_id, 64, 0)
# 重启并等待0.7s
pk.reboot(ph, dxl_id); sleep(0.7)

def r1(a): return pk.read1ByteTxRx(ph, dxl_id, a)[0]
def r2(a): return pk.read2ByteTxRx(ph, dxl_id, a)[0]

hw = r1(70); vin = r2(144); temp = r1(146); tq = r1(64); opm = r1(11)
print(f"HW_ERR(70)={hw} (bin {hw:08b})  Vin={vin/10.0:.1f}V  Temp={temp}C  Torque(64)={tq}  OperMode(11)={opm}")

ph.closePort()