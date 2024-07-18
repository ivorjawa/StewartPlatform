import usys
import umath as m
from usys import stdin
from uselect import poll

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.tools import Matrix, vector, cross
from pybricks.parameters import Axis


hub = TechnicHub()

from linear import xaxis, yaxis, zaxis, rotate
import linear as lin

from joycode import JoyProtocol
#from swashplate import Swashplate

millis = StopWatch().time

# C is star, B is port, A is front, D is rotor
# max speed is about 1300

#rotor = Motor(Port.D)
#star = Motor(Port.C)
#port = Motor(Port.B)
#front = Motor(Port.A)

class Enum(object):
    # this is good enough for state machines in micropython
    def __init__(self, enumname, enums, start=1):
        self.__enumname = enumname
        for i, e in enumerate(enums):
            setattr(self, e, i+start)

class MoveSM(object):
    def __init__(self):
        print("new MoveSM")
        self.states = Enum("movestates", ['started', 'finished']) 
        self.state = self.states.finished
        self.start_time = 0
    def tick(self):
        #print(f"MoveSM.tick(): {self.state} start: {self.start_time}")
        if self.state == self.states.started:
            if millis() > (self.start_time + 1000):
                self.state = self.states.finished
                print("<taskdone/>")
            else:
                print("not done yet.")
        else:
            # state is finished
            pass
    def moveto(self):
        self.start_time = millis()
        self.state = self.states.started
        print(f"moveto() start_time: {self.start_time}")
        
                              
def run_remote():
    poller = poll()
    # Register the standard input so we can read keyboard presses.
    poller.register(stdin)
    
    wvars = ['roll', 'pitch', 'yaw', 'coll', 'glyph']
    wirep = JoyProtocol(wvars, 2, poller, stdin)
    
    #jsman = JSMan()
    last_input = 0 # last input from base station
    
    #disk_def = 8 # degrees +-
    #coll_range = .15 # % +-
    
    #threshold = memory_calibrate() # max travel of cylinders, degrees
    # arm radius, min cylinder, max cylinder
    #rot = Rotor(60, 160, 200, threshold) # real robot
    
    identify()
    print("<awake/>")
    msm = MoveSM()
    
    while True:
        if wirep.poll():
            try:    
                last_input = millis() 

                wirep.decode_wire()
                #print(wirep.vals)
                if wirep.decode_raw('glyph') == 255:
                    pass
                    #print("got keepalive")
                else:
                    print("got moveto")
                    msm.moveto()
                msm.tick()
                    
            except Exception as e:
                print("failure in main loop:")
                print(e)
            
            #rot.actuate()


def identify():
    print(f"System name: {hub.system.name()}")
    print(f"Version: {usys.version}")      
    print(f"Implementation: {usys.implementation}")      
    print(f"Version Info: {usys.version_info}")
    print(f"Battery Voltage: {hub.battery.voltage()}mv") 
    #pitch, roll = hub.imu.tilt()
    # are we getting raw IMU values here? FIXME
    #ax, ay, az = hub.imu.acceleration() / 9810
    #pitch = m.degrees(m.atan2(ay, m.sqrt(ax**2 + ay**2)))
    #roll = m.degrees(m.atan2(ax, m.sqrt(ay**2 + ay**2)))
    #print(hub.imu.acceleration() / 9810)
    #print(f"pitch: {pitch:0.3f} roll: {roll:0.3f} ax: {ax:0.3f} ay: {ay:0.3f} az: {az:0.3f}")
    while 1:
        #print(f"imu: {dir(hub.imu)}")
        ax, ay, az = hub.imu.acceleration() / 9810
        pitch = m.degrees(m.atan2(az, ax))
        roll = m.degrees(m.atan2(az, ay))
        yaw = m.degrees(m.atan2(ay, ax))
        print(f"ax, ay, az: {ax:6.2f} {ay:6.2f} {az:6.2f} PR: {90-pitch:6.2f} {90-roll:6.2f}")
        #print(f"(x, y, z) {hub.imu.rotation(Axis.X):6.2f}, {hub.imu.rotation(Axis.Y):6.2f}, {hub.imu.rotation(Axis.Z):6.2f} (ax, ay, az): {ax:6.2f} {ay:6.2f} {az:6.2f}")
        #print(hub.imu.tilt())
        #print(f"y: )
        #print(f"z: {hub.imu.rotation(Axis.Z):6.2f}")
        
if __name__ == "__main__":
    # pybricksdev run ble -n bubble bubble.py
    #teststorage()
    #testwritestore()
    #run_remote()
    try:
        pass # when running above tests
        #run_remote() # full talks to remote run under ./rotorbase.py
        #full_calibrate() # should be done with rotors and scissor link detached
    except Exception as e:
        print("General failure: ", e)
    identify()     