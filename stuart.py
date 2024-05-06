#!/usr/bin/env python

import usys
import umath as m
from usys import stdin
from uselect import poll

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.tools import Matrix, vector, cross

hub = PrimeHub()

from linear import xaxis, yaxis, zaxis, rotate
import linear as lin

from joycode import JoyProtocol
from StewartPlatform import StewartPlatform

millis = StopWatch().time


M1 = Motor(Port.A)
M2 = Motor(Port.B)
M3 = Motor(Port.C)
M4 = Motor(Port.D)
M5 = Motor(Port.E)
M6 = Motor(Port.F)

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

def runtostall(cmots, spd, reset=True):
    fin = [False, False, False, False, False, False]
    done = False
    for cm in cmots:
        cm.run(spd)
    while not done:
        for i, cm in enumerate(cmots):
            if cm.stalled():
                cm.dc(0)
                if reset:
                    cm.reset_angle(0)
                fin[i] = True
        if fin[0] and fin[1] and fin[2] and fin[3] and fin[4] and fin[5]:
            done = True
    
def calib_core(cmots):
    # runs motors until stalled at fully retracted, resets zero
    
    st = [40, 30] # degrees a second, time ms
    for cm in cmots:
        cm.control.stall_tolerances(*st)
    

    print(f"Starting motor test, stall tolerances: {M1.control.stall_tolerances()}")
    
    runtostall(cmots, -speed/3)
        
def neutral_coll(cmots, angle):
    "set all three cylinders to one position"
    for cm in cmots:
        cm.track_target(angle)

# read from data previously written by calibrate() below
def memory_calibrate():
    print("Starting soft calibration")
    threshold = readthresh()
    if (threshold < 0) or (threshold > 3500):
        raise ValueError(f"Recorded threshold of {threshold} degrees invalid, recalibrate")
    cmots = [M1, M2, M3, M4, M5, M6]
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
    cmots = [M1, M2, M3, M4, M5, M6]
    calib_core(cmots)
   
    print("Stop detected, reversing")
    runtostall(cmots, speed/3, False)

    for (i, m) in enumerate(cmots):
        print(f"M{i+1}: {m.angle()}")
    #print(f"M1: {M1.angle()}, M2: {M2.angle()}, M3: {M3.angle()}")
    big_angle = min([m.angle() for m in cmots])
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

def identify():
    print(f"System name: {hub.system.name()}")
    print(f"Version: {usys.version}")      
    print(f"Implementation: {usys.implementation}")      
    print(f"Version Info: {usys.version_info}")
    print(f"Battery Voltage: {hub.battery.voltage()}mv") 

class Enum(object):
    # this is good enough for state machines in micropython
    def __init__(self, enumname, enums, start=1):
        self.__enumname = enumname
        for i, e in enumerate(enums):
            setattr(self, e, i+start)
            
class MoveSM(object):
    def __init__(self, testfn):
        print("new MoveSM")
        self.states = Enum("movestates", ['started', 'finished']) 
        self.state = self.states.finished
        self.start_time = 0
        self.testfn = testfn
    def tick(self):
        #print(f"MoveSM.tick(): {self.state} start: {self.start_time}")
        if self.state == self.states.started:
            if self.testfn():
                self.state = self.states.finished
                print(f"elapsed {millis() - self.start_time}")
                print("<taskdone/>")
            else:
                pass
                #print("<notdoneyet/>") # gets stuck if I don't report FIXME
        else:
            # state is finished
            pass
    def moveto(self):
        self.start_time = millis()
        self.state = self.states.started
        #print(f"moveto() start_time: {self.start_time}")
        
class Stewart(StewartPlatform):
    def __init__(self, inner_r, outer_r, footprint, min_cyl, max_cyl, threshang):
        super().__init__(inner_r, outer_r, footprint, min_cyl, max_cyl)
        self.threshang = threshang # top of cylinder, degrees
        self.angle_mul = self.threshang / self.Crange # mm to degree
        self.cmots = [M1, M2, M3, M4, M5, M6]
        
    def pos_ang(self, position):
        # decode desired cylinder length into rotations
        # threshold max angle, degrees
        # mincyl, maxcyl, position mm
        if (position < self.Cmin) or (position > self.Cmax):
            raise ValueError(f"{position} is not in {self.Cmin}...{self.Cmax}")
        offset = position - self.Cmin
        return offset * self.angle_mul # angle in degrees, can be > 360
    
    def actuate(self):
        for i, cyl in enumerate(self.cyls):
            #print(f"motor {i} wants cylinder length {cyl}")
            target = self.pos_ang(cyl)
            self.cmots[i].track_target(target)
 
    def is_moved(self):
        thresh = 5
        done = [False for i in self.cyls]
        for i, cyl in enumerate(self.cyls):
            target = self.pos_ang(cyl)
            current = self.cmots[i].angle()
            speed = self.cmots[i].speed()
            motordone = self.cmots[i].done()
            mathdone = abs(current-target) < thresh
            done[i] = mathdone
            #print(f"cyl: {i} target: {target} current: {current} speed: {speed} motord: {motordone} mathd: {mathdone} d:{done[i]}")
        return sum(done) == len(self.cyls)
        
    def calculate(self, roll, pitch, yaw, coll, glyph):
        try:
            #print(f"calculate roll: {roll}, pitch: {pitch} yaw: {yaw} glyph: {glyph}")
            (coll_v, sa, sb, sc) = self.solve(roll, pitch, yaw, coll, glyph) # sets self.cyls
            #print(f"coll_v: {coll_v}, sa: {sa}, sb: {sb}, sc{sc}")
            #self.solve(pitch, roll, collpct)
            #print(self.cfc, self.cpc, self.csc)
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
    
    disk_def = 10 # degrees +-
    coll_range = 1 # % +-
    
    threshold = memory_calibrate() # max travel of cylinders, degrees
    # arm radius, min cylinder, max cylinder
    Stew = Stewart(40, 120, 120, 240, 308, threshold) #inner, outer radius, footprint, min, max cylinder extension
    
    identify()
    print("<awake/>")    
    msm = MoveSM(Stew.is_moved)
        
    while True:
        if wirep.poll():
            try:    
                last_input = millis() 

                wirep.decode_wire()
                #print("wirep: ", wirep.vals)
                coll = (wirep.vals['coll']/2 + .5)*coll_range   
                #coll = .5              
                #print(f"wire coll: {wirep.vals['coll']} calculated: {coll}")
                pitch = -1*wirep.vals['roll']*disk_def
                roll = -1*wirep.vals['pitch']*disk_def
                yaw = -1*wirep.vals['yaw']*23
                glyph = wirep.decode_raw('glyph')
                
                if glyph == 255:
                    # keepalive
                    pass
                else:
                    Stew.calculate(roll, pitch, yaw, coll, glyph)
                    msm.moveto()
                    #print(f"roll:{roll: 3.1f}, pitch:{pitch: 3.1f}, yaw:{yaw: 3.1f} coll:{coll: 3.1f} glyph:{glyph}", end="\n")
                    #for i, cyl in enumerate(Stew.cyls):
                    #    print(f" Cyl {i}:{cyl: 3.1f}mm", end="")
                    #print("")
                
                    if((glyph & 40) == 40): # X '0b0101000' SB
                        print(f"<goodbye/>")
                        return None
                        #break
                    
            except Exception as e:
                print("failure in main loop:")
                print(e)
        
            try:
                Stew.actuate()
            except Exception as e:
                print("actuate failed: ", e)
                print(f"<goodbye/>")
                return None
        msm.tick()
                
if __name__ == "__main__":
    # pybricksdev run ble -n rotor rotor.py
    #teststorage()
    #testwritestore()
    #run_remote()
    try:
        #pass # when running above tests
        run_remote() # full talks to remote run under ./rotorbase.py
        #full_calibrate() 
    except Exception as e:
        print("General failure: ", e)
    identify()   