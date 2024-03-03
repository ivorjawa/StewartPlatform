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

from swashplate import Swashplate

millis = StopWatch().time

# C is star, B is port, A is front, D is rotor
# max speed is about 1300

rotor = Motor(Port.D)
star = Motor(Port.C)
port = Motor(Port.B)
front = Motor(Port.A)

speed = 1000
step = 530

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

def calib_core(cmots):
    # runs motors until stalled at fully retracted, resets zero
    
    st = [40, 30] # degrees a second, time ms
    for cm in cmots:
        cm.control.stall_tolerances(*st)
    
    print("Calibration of actuator ranges, rotors should be detatched")
    print(f"Current threshold: {readthresh()} degrees")
    print(f"Starting motor test, stall tolerances: {front.control.stall_tolerances()}")
    
    # FIXME duplicate code, could factor this better
    fin = [False, False, False]
    done = False
    for cm in cmots:
        cm.run(-speed/3)
    while not done:
        for i, cm in enumerate(cmots):
            if cm.stalled():
                cm.dc(0)
                cm.reset_angle(0)
                fin[i] = True
        if fin[0] and fin[1] and fin[2]:
            done = True
        
def neutral_coll(cmots, angle):
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
def calibrate():
    cmots = [front, star, port]
    calib_core(cmots)
   
    print("Stop detected, reversing")
    done = False
    fin = [False, False, False]
    for cm in cmots:
        cm.run(speed/3)
    while not done:
        for i, cm in enumerate(cmots):
            if cm.stalled():
                cm.dc(0)
                fin[i] = True
        if fin[0] and fin[1] and fin[2]:
            done = True
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

    
# joystick input stuff
class JSMan(object):
    def __init__(self):
        self.buf = list("x" * 22) # control input buffer
        self.pos = 0              # control input buffer pointer
        self.pint = 0
        # Register the standard input so we can read keyboard presses.
        self.keyboard = poll()
        self.keyboard.register(stdin)
    
    def hbyte(self,arr, off):
        i = 256*int(arr[off+0],16) +16*int(arr[off+1],16) + int(arr[off+2],16)
        return i    
    
        
    def decode_js(self, js_uint12):
        # scale 8-bit unsigned into range [-1.0,1.0]
        # both axes reversed logically, so going up and right decrease
        jsf = js_uint12/128.0 - 1
        
        if(abs(jsf)<(5.0/128)): # control the dead zone
            jsf = 0
            
        #jsf = -jsf
        return jsf
    
    def poll(self):
      if(self.pint > 255):
          #print("Memory info")
          #micropython.mem_info()
          #print("poll buf:  ", self.pos, self.buf)
          self.pint = 0
      else: 
          self.pint += 1
      if self.keyboard.poll(0):
          # Read the key and print it.
          key = stdin.read(1)
          #print("key: ", key)
          if key == '>':
              self.pos = 0
          self.buf[self.pos] = key
          self.pos += 1
          if (self.pos==22):
              self.pos = 0
              if (key == '<'):
                  return True 
              else:
                  print("buffer bad")
      return False
      
    def decode(self, offset, inverted=False):
        inp = self.hbyte(self.buf, offset)
        raw = self.decode_js(inp)
        #raw = inp # just int-decode it for now
        if inverted:
            raw = -raw
        return raw # may not need the sigmoid with control algorithm.

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
        pass
        front.track_target(self.pos_ang(self.cfc))  
        port.track_target(self.pos_ang(self.cpc))  
        star.track_target(self.pos_ang(self.csc))  
          
    def calculate(self, pitch, roll, collpct):
        try:
            (cf, cp, cs, coll) = self.solve(pitch, roll, collpct)
            print(self.cfc, self.cpc, self.csc)
            #print(f"front: {self.pos_ang(self.cfc)}  port: {self.pos_ang(self.cpc)}  star: {self.pos_ang(self.csc)}")
        except ValueError as e:
            label = f"Range Error P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} {e}"
            print(label)
            
                       
def run_remote():
    jsman = JSMan()
    last_input = 0 # last input from base station
    
    disk_def = 8 # degrees +-
    coll_range = .15 # % +-
    
    threshold = memory_calibrate() # max travel of cylinders, degrees
    # arm radius, min cylinder, max cylinder
    rot = Rotor(60, 160, 200, threshold) # real robot
    
    while True:
        if jsman.poll():
            try:    
                last_input = millis() 
                #output = ">c%03xr%03xp%03xy%03xg%03x<" % (coll, roll, pitch, yaw, glyph)       
                # >c000r3e7p3f4y3fcg000<
                # 0123456789ABCDEFGHIJKL
                #   ^   ^   ^   ^   ^
                #   2   6   10  14  18
                coll = .6+jsman.decode(2)*coll_range # offset so bottom collective is neutral
                roll = -1*jsman.decode(6)*disk_def
                pitch = -1*jsman.decode(10)*disk_def
                yaw = jsman.decode(14)
                #glyph = jsman.decode(18)
                glyph = jsman.hbyte(jsman.buf, 18)
                
                #print("c: %0.2f, r: %0.2f, p: %0.2f, y: %0.2f:  g: %d" % 
                #print("c: %d, r: %d, p: %d, y: %d:  g: %d" % 
                #    (coll, roll, pitch, yaw, glyph))
                
                rot.calculate(pitch, roll, coll)
                
                if((glyph & 40) == 40): # X '0b0101000' SB
                    print(f"goodbye.")
                    return None
                    #break
                if((glyph & 72) == 72): #sc
                    rotor.run(60)
                if((glyph & 68) == 68): #sd
                    rotor.stop()
                    
            except Exception as e:
                print("failure in main loop:")
                print(e)
            
            rot.actuate()
            
            # all below are millimiters
            #self.cfc = np.mag(cf - self.cyl_front)
            #self.cpc = np.mag(cp - self.cyl_port)
            #self.csc = np.mag(cs - self.cyl_star)


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
    #testlinalg()
    #test_rmat()
    #run_remote()
    try:
        #pass # when running above tests
        run_remote() # full talks to remote run under ./rotorbase.py
        #calibrate() # should be done with rotors detached
    except Exception as e:
        print("General failure: ", e)
    identify()     