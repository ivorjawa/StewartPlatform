import usys
import umath as m
from usys import stdin
from uselect import poll

from pybricks.hubs import PrimeHub
#from pybricks.pupdevices import Motor
#from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
#from pybricks.tools import Matrix, vector, cross

hub = PrimeHub()
millis = StopWatch().time

def identify():
    print(f"System name: {hub.system.name()}")
    print(f"Version: {usys.version}")      
    print(f"Implementation: {usys.implementation}")      
    print(f"Version Info: {usys.version_info}")
    print(f"Battery Voltage: {hub.battery.voltage()}mv") 
    ax, ay, az = hub.imu.acceleration() / 9810
    pitch = m.degrees(m.atan2(ay, m.sqrt(ax**2 + ay**2)))
    roll = m.degrees(m.atan2(ax, m.sqrt(ay**2 + ay**2)))
    
    #print(hub.imu.acceleration() / 9810)
    print(f"pitch: {pitch:0.3f} roll: {roll:0.3f} ax: {ax:0.3f} ay: {ay:0.3f} az: {az:0.3f}")

class RoboShellServer(object):
    def __init__(self, poller, stream):
        # io stuff
        self.keyboard = poller
        self.stream = stream
        self.buf = []
    def readout(self):
        retval = ''.join(self.buf)
        self.buf = []
        return retval
    def poll(self):
        #print("poll()")
        # actually reads the input buffer
        if self.keyboard.poll(0):
          # Read the key and print it.
          key = self.stream.read(1)
          #print(f"robot got key {key}")
          if key == '\n':
              return True
          else:
              self.buf.append(key)
        return False 
      
def run_remote():
    poller = poll()
    # Register the standard input so we can read keyboard presses.
    poller.register(stdin)
    
    rss = RoboShellServer(poller, stdin)
    
    identify()
    print("<awake/>")
    
    while True:
        # input loop scan, expects 30-60Hz inputs
        if rss.poll():
            result = rss.readout()
            print(f"buflen: {len(result)}: {result}")
    
if __name__ == "__main__":
    # pybricksdev run ble -n jawaspike stuart.py
    try:
        run_remote() # full talks to remote run under ./rotorbase.py
    except Exception as e:
        print("General failure: ", e)
    identify()   