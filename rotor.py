import usys
import umath as m
from usys import stdin
from uselect import poll

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.tools import Matrix, vector, cross

hub = TechnicHub()

from linear import xaxis, yaxis, zaxis, rotate
import linear as lin

from joycode import JoyProtocol
from swashplate import Swashplate

millis = StopWatch().time

# C is star, B is port, A is front, D is rotor
# max speed is about 1300

rotor = Motor(Port.D)
star = Motor(Port.C)
port = Motor(Port.B)
front = Motor(Port.A)

speed = 1000

# encode 16-bit int for system storage
def enc16(i):
    lb = i & 255
    hb = i >> 8
    return bytes([hb, lb])

# decode 16-bit int from system storage
def dec16(b):
    return b[0]*256+b[1]

# read maximum cylinder position from storage
def readthresh():
    thresh = hub.system.storage(0, read=2)  
    return dec16(thresh)
    
def teststorage():
    print("Read Threshold")
    print(f"Threshold is {readthresh()}") 
    identify() 
    
def testwritestore():
    print("Write Threshold")
    hub.system.storage(0, write=enc16(1701))
    identify()

def runtostall(cmots, spd):
    fin = [False, False, False]
    done = False
    for cm in cmots:
        cm.run(spd)
    while not done:
        for i, cm in enumerate(cmots):
            if cm.stalled():
                cm.dc(0)
                cm.reset_angle(0)
                fin[i] = True
        if fin[0] and fin[1] and fin[2]:
            done = True
    
def calib_core(cmots):
    # runs motors until stalled at fully retracted, resets zero
    
    st = [40, 30] # degrees a second, time ms
    for cm in cmots:
        cm.control.stall_tolerances(*st)
    

    print(f"Starting motor test, stall tolerances: {front.control.stall_tolerances()}")
    
    runtostall(cmots, -speed/3)
        
def neutral_coll(cmots, angle):
    "set all three cylinders to one position"
    for cm in cmots:
        cm.track_target(angle)

# read from data previously written by calibrate() below
def memory_calibrate():
    print("Starting soft calibration")
    threshold = readthresh()
    if (threshold < 0) or (threshold > 2000):
        raise ValueError(f"Recorded threshold of {threshold} degrees invalid, recalibrate")
    cmots = [front, star, port]
    calib_core(cmots)
    print("Sending collective neutral")
    neutral_coll(cmots, threshold/2)
    print(f"Soft Calibration complete, threshold {threshold} degrees")
    return threshold
    
# for the actual actuation, we'll use track_target(degrees) command
# make safe operating limit smaller than max so it doesn't stick.
def full_calibrate():
    print("Calibration of actuator ranges, rotors and scissor-link should be detatched")
    print(f"Current threshold: {readthresh()} degrees")
    cmots = [front, star, port]
    calib_core(cmots)
   
    print("Stop detected, reversing")
    runtostall(cmots, speed/3)

    print(f"Star: {star.angle()}, Port: {port.angle()}, Rear: {front.angle()}")
    big_angle = min([star.angle(), port.angle(), front.angle()])
    if big_angle > 1500:
        half = big_angle/2
        neutral_coll(cmots, half)
        wait(3000)
        print("I did it!")
        for cm in cmots:
            cm.dc(0)
            
        print(f"Writing Threshold: {big_angle} degrees")
        hub.system.storage(0, write=enc16(big_angle))
        identify()
        
    else: 
        print(f"Big angle too small: {big_angle}")



class Rotor(Swashplate):
    def __init__(self, rotor_rad, c_min, c_max, threshang):
        super().__init__(rotor_rad, c_min, c_max)
        self.threshang = threshang # top of cylinder, degrees
        self.angle_mul = self.threshang / self.Crange # mm to degree
        
    def pos_ang(self, position):
        # decode desired cylinder length into rotations
        # threshold max angle, degrees
        # mincyl, maxcyl, position mm
        if (position < self.Cmin) or (position > self.Cmax):
            raise ValueError(f"{position} is not in {self.c_min}...{self.c_max}")
        offset = position - self.Cmin
        return offset * self.angle_mul # angle in degrees, can be > 360
    
    def actuate(self):
        front.track_target(self.pos_ang(self.cfc))  
        port.track_target(self.pos_ang(self.cpc))  
        star.track_target(self.pos_ang(self.csc))  
          
    def calculate(self, pitch, roll, collpct):
        try:
            self.solve(pitch, roll, collpct)
            print(self.cfc, self.cpc, self.csc)
            #print(f"front: {self.pos_ang(self.cfc)}  port: {self.pos_ang(self.cpc)}  star: {self.pos_ang(self.csc)}")
        except ValueError as e:
            label = f"Range Error P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} {e}"
            print(label)
            
                       
def run_remote():
    poller = poll()
    # Register the standard input so we can read keyboard presses.
    poller.register(stdin)
    
    wvars = ['coll', 'roll', 'pitch', 'yaw', 'glyph']
    wirep = JoyProtocol(wvars, 2, poller, stdin)
    
    #jsman = JSMan()
    last_input = 0 # last input from base station
    
    disk_def = 8 # degrees +-
    coll_range = .15 # % +-
    
    threshold = memory_calibrate() # max travel of cylinders, degrees
    # arm radius, min cylinder, max cylinder
    rot = Rotor(60, 160, 200, threshold) # real robot
    
    while True:
        if wirep.poll():
            try:    
                last_input = millis() 

                wirep.decode_wire()
                coll = .6+wirep.vals['coll']*coll_range # offset so bottom collective is neutral
                roll = 1*wirep.vals['roll']*disk_def
                pitch = -1*wirep.vals['pitch']*disk_def
                yaw = wirep.vals['yaw']
                glyph = wirep.decode_raw('glyph')
                
                #print("c: %0.2f, r: %0.2f, p: %0.2f, y: %0.2f:  g: %d" % 
                #print("c: %d, r: %d, p: %d, y: %d:  g: %d" % 
                #    (coll, roll, pitch, yaw, glyph))
                
                rot.calculate(pitch, roll, coll)
                
                if((glyph & 40) == 40): # X '0b0101000' SB
                    print(f"<goodbye/>")
                    return None
                    #break
                if((glyph & 72) == 72): #sc
                    rotor.run(-75)
                if((glyph & 68) == 68): #sd
                    rotor.stop()
                    
            except Exception as e:
                print("failure in main loop:")
                print(e)
            
            rot.actuate()


def identify():
    print(f"System name: {hub.system.name()}")
    print(f"Version: {usys.version}")      
    print(f"Implementation: {usys.implementation}")      
    print(f"Version Info: {usys.version_info}")
    print(f"Battery Voltage: {hub.battery.voltage()}mv") 
        
if __name__ == "__main__":
    # pybricksdev run ble -n rotor rotor.py
    #teststorage()
    #testwritestore()
    #run_remote()
    try:
        #pass # when running above tests
        run_remote() # full talks to remote run under ./rotorbase.py
        #full_calibrate() # should be done with rotors and scissor link detached
    except Exception as e:
        print("General failure: ", e)
    identify()     