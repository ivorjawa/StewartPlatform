import usys
import umath as m
from usys import stdin
from uselect import poll

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.tools import Matrix, vector, cross

import unumpy as np

hub = TechnicHub()

millis = StopWatch().time

# C is star, B is port, A is front, D is rotor
# max speed is about 1300

rotor = Motor(Port.D)
star = Motor(Port.C)
port = Motor(Port.B)
front = Motor(Port.A)

speed = 1000
step = 530

xaxis = vector(1, 0, 0)
yaxis = vector(0, 1, 0)
zaxis = vector(0, 0, 1)
def rmat(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    #axis = np.asarray(axis)
    #axis = axis / m.sqrt(np.dot(axis, axis))
    axis = np.linalg.norm(axis) # this is wrong, should be normalize as above
    a = m.cos(theta / 2.0)
    b, c, d = -axis * m.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def rotate(axis, theta, point):
    rm = rmat(axis, theta)
    #print(f"rotation matrix axis: {axis} theta: {theta}\nrmat: {rm}")
    return rm * point # in pybricks you multiply instead of dot
    #return (np.dot(rm, point))
    

def enc16(i):
    lb = i & 255
    hb = i >> 8
    return bytes([hb, lb])

def dec16(b):
    return b[0]*256+b[1]

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
        # scale 12-bit unsigned into range [-1.0,1.0]
        # both axes reversed logically, so going up and right decrease
        jsf = js_uint12/1024.0 - 1
        
        if(abs(jsf)<(5.0/1024)): # control the dead zone
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

class Rotor(object):
    # coordinate system is +z-up +x-forward
    def __init__(self, rotor_rad, c_min, c_max, threshang):
        self.c_min = c_min
        self.c_max = c_max
        self.c_range = c_max - c_min
        self.threshang = threshang # top of cylinder, degrees
        self.angle_mul = self.threshang / self.c_range # mm to degree
        self.rotor_rad = rotor_rad
        self.coll = vector(0, 0, 0)
        self.cyl_front = vector(-rotor_rad, 0, 0)
        #print(f"front: {self.cyl_front}")
        self.cyl_port = rotate(zaxis, m.radians(120), self.cyl_front)
        #print(f"port: {self.cyl_port}")
        self.cyl_star = rotate(zaxis, m.radians(-120), self.cyl_front)
        #print(f"star: {self.cyl_star}")
        self.cyls = [self.cyl_front, self.cyl_port, self.cyl_star]
        tmat = vector(0, 0, .5*self.c_range + self.c_min)
        #print(f"translation matrix: {tmat}")
        
        self.old_coll = self.coll + tmat
        self.old_cf = self.cyl_front + tmat
        self.old_cs = self.cyl_star + tmat
        self.old_cp = self.cyl_port + tmat
        
        #print(f"front: {self.cyl_front}, port: {self.cyl_port}, startboard: {self.cyl_star}")
    def pos_ang(self, position):
        # threshold max angle, degrees
        # mincyl, maxcyl, position mm
        if (position < self.c_min) or (position > self.c_max):
            raise ValueError(f"{position} is not in {self.c_min}...{self.c_max}")
        offset = position - self.c_min
        return offset * self.angle_mul # angle in degrees, can be > 360
        
    def solve(self, pitch, roll, collpct):
        pitch_v = rotate(yaxis, m.radians(pitch), vector(1, 0, 0))
        roll_v = rotate(xaxis, m.radians(roll), vector(0, 1, 0))
        cyclic_norm = np.cross(pitch_v, roll_v)
        cyclic_norm_n = np.linalg.norm(cyclic_norm)
        #print(f"pitch_v: {pitch_v}, roll_v: {roll_v}, cyclic_norm_n: {cyclic_norm_n}")
        coll_v = vector(0, 0, self.c_min + collpct*self.c_range) # top of mast at collective setting
        arms = []
        for i, cyl in enumerate(self.cyls):
            #print(f"i: {i}, cyl: {cyl}")
            cyl_norm = np.cross(cyl, coll_v)
            cyl_norm_n = np.linalg.norm(cyl_norm)
            isect = np.cross(cyclic_norm, cyl_norm) # should be plane intersection
            isect_n = np.linalg.norm(isect)
            #print(f"cylinder: {i}, cyl_norm_n: {cyl_norm_n}, intersection_n: {isect_n}")
            arm_v = (self.rotor_rad * isect_n) + coll_v
            arms.append(arm_v)
            cyl_len = np.mag(arm_v - cyl)
            if cyl_len < self.c_min:
                raise ValueError(f"too short! Cyl: {cyl_len:{4}.{4}} min: {self.c_min:{4}}")
            elif cyl_len > self.c_max:
                raise ValueError(f"too long! Cyl: {cyl_len:{4}.{4}} max: {self.c_max:{4}}")
        (cf, cp, cs) = arms
        coll = coll_v        
        #self.validate(cf, cp, cs, coll, pitch, roll, collpct)
        return (cf, cp, cs, coll)
    
    def actuate(self):
          front.track_target(self.pos_ang(self.cfc))  
          port.track_target(self.pos_ang(self.cpc))  
          star.track_target(self.pos_ang(self.csc))  
          
    def calculate(self, pitch, roll, collpct):
        try:
            (cf, cp, cs, coll) = self.solve(pitch, roll, collpct)
            self.old_cf = cf
            self.old_cp = cp
            self.old_cs = cs
            self.old_coll = coll
            self.cfc = np.mag(cf - self.cyl_front)
            self.cpc = np.mag(cp - self.cyl_port)
            self.csc = np.mag(cs - self.cyl_star)
            print(f"front: {self.pos_ang(self.cfc)}  port: {self.pos_ang(self.cpc)}  star: {self.pos_ang(self.csc)}")
        except ValueError as e:
            label = f"Range Error P: {pitch:{4}.{3}}, R: {roll:{4}.{3}}, C%: {collpct:{3}.{3}} {e}"
            print(label)
            cf = self.old_cf
            cp = self.old_cp
            cs = self.old_cs
            coll = self.old_coll
            
                       
def run_remote():
    jsman = JSMan()
    last_input = 0 # last input from base station
    
    disk_def = 5 # degrees +-
    coll_range = .1 # % +-
    
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
                roll = jsman.decode(6)*disk_def
                pitch = jsman.decode(10)*disk_def
                yaw = jsman.decode(14)
                #glyph = jsman.decode(18)
                glyph = jsman.hbyte(jsman.buf, 18)
                
                #print("c: %0.2f, r: %0.2f, p: %0.2f, y: %0.2f:  g: %d" % 
                #print("c: %d, r: %d, p: %d, y: %d:  g: %d" % 
                #    (coll, roll, pitch, yaw, glyph))
                
                rot.calculate(pitch, roll, coll)
                
                if(glyph == 40): # X '0b0101000' SB
                    print("goodbye.")
                    #return None
                    break
                elif(glyph == 72): #sc
                    rotor.run(60)
                elif(glyph == 68): #sd
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

def testlinalg():
    a = vector(2, 2, 2)
    b = vector(3, 3, 3)
    print(f"b shape: {b.shape}")
    print(f"b magnitude: {np.mag(b)}")
    print(f"b norm: {np.linalg.norm(b)}")
    print(f"mag(norm(b): {np.mag(np.linalg.norm(b))}")
    print(f"{a} dot {b}: {np.dot(a, b)}")
     
    twerk = Matrix([[1, 0, 0]])
    print(f"twerk: {twerk} shape: {twerk.shape}")
       
    twox = 2*xaxis
    axis1 = twox / m.sqrt(np.dot(twox, twox))
    axis2 = np.linalg.norm(twox)
    print(f"twox: {twox}")
    print(f"axis1: {axis1}")
    print(f"axis2: {axis2}")

    zrot = rmat(zaxis, m.radians(50))
    print(f"zrot 50 degrees: {zrot}")


    
    rot = Rotor(60, 160, 200, 1900) # real robot
    identify()  
    
def test_rmat():
    cyl_front = vector(-100, 0, 0)
    cyl_port = rotate(zaxis, m.radians(120), cyl_front)  
    print(f"front: {cyl_front}, port: {cyl_port}")
        
if __name__ == "__main__":
    # pybricksdev run ble -n rotor rotor.py
    #teststorage()
    #testwritestore()
    #testlinalg()
    #test_rmat()
    try:
        #pass # when running above tests
        run_remote() # full talks to remote run under ./rotorbase.py
        #calibrate() # should be done with rotors detached
    except Exception as e:
        print("General failure: ", e)     