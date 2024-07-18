import usys
import umath as m
from pybricks.hubs import PrimeHub


hub = PrimeHub()

def ident():
    print(f"System name: {hub.system.name()}")
    print(f"Version: {usys.version}")      
    print(f"Implementation: {usys.implementation}")      
    print(f"Version Info: {usys.version_info}")
    print(f"Battery Voltage: {hub.battery.voltage()}mv") 
    while 1:
        ax, ay, az = hub.imu.acceleration() / 9810
        pitch = m.degrees(m.atan2(az, ax))
        roll = m.degrees(m.atan2(az, ay))
        print(f"ax, ay, az: {ax:6.2f} {ay:6.2f} {az:6.2f} PR: {90-pitch:6.2f} {90-roll:6.2f}")
    print(f"IMU Tilt: {hub.imu.tilt()} degrees")

# pybricksdev run ble -n bubble slerp.py    
if __name__ == "__main__":
    #q1 = make_quat(1, 0, 0, 0)
    #q2 = make_quat(0, 1, 0, 0)
    #for i in range(11):
    #    print(f"{i}: {slerp(q1, q2, i/10)}")
    ident()