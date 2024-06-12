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
from statemachine import StateMachine
import dataforslerp as slerpdat
import slerp

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
    
    #st = [30, 30] # degrees a second, time ms
    #for cm in cmots:
    #    cm.control.stall_tolerances(*st)
    

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


class SlerpSM(StateMachine):
    def __init__(self, stew):
        super().__init__()
        self.build("slerpstate", ['loadframe', 'segstart', 'segend', 'wait', 'advance'])
        self.rcube = slerpdat.rcube # cube rotations
        self.scube = slerpdat.scube # cube positions

        self.cubelength = len(self.scube) #-8
        self.stew = stew
        self.cubedex = 0
        self.framedex = 0
        self.rotor1 = None
        self.rotor2 = None
        self.segstime = millis()
        self.segcount = 10
        print(f"rcube: {self.rcube}")
        print("SlerpSM()")
    def loadframe(self):
        # FIXME need to calculate both  LERP distance and SLERP distance
        # and use them to derive number of segments needed.
        # SLERP and LERP should terminate same time
        print(f"loadframe {self.cubedex}")
        cubedex1 = self.cubedex
        cubedex2 = (self.cubedex+1) % self.cubelength
        rcube1 = self.rcube[cubedex1] 
        rcube2 = self.rcube[cubedex2]
        
        self.rotor1 = slerp.euler_quat(m.radians(rcube1[0]), m.radians(rcube1[1]), m.radians(rcube1[2]))
        self.rotor2 = slerp.euler_quat(m.radians(rcube2[0]), m.radians(rcube2[1]), m.radians(rcube2[2]))
               
        self.scube1 = self.scube[cubedex1]
        self.scube2 = self.scube[cubedex2]
        
        self.cubefract = (self.scube2-self.scube1)
        self.state = self.states.segstart
        
    def segstart(self):
        print("segstart")
        self.segstime = millis()
        lerpcube = self.scube1 + (self.framedex/(1.0*self.segcount))*self.cubefract
        rotor = slerp.slerp(self.rotor1, self.rotor2, self.framedex/(1.0*self.segcount))
        (r, p, y) = slerp.to_euler(rotor) # (roll, pitch, yaw)
        r = -m.degrees(r) # FIXME seems opposite direction of intention?
        p = -m.degrees(p)
        y = m.degrees(y)
        
        print(f"r: {r:5.2f} p: {p:5.2f} y: {y:5.2f}")
        try:    
            colspokes = self.stew.solve4(rotor, *lerpcube)
            if len(colspokes) == 4:
                
                (coll_v, sa, sb, sc) = colspokes
                self.stew.actuate()
        except ValueError as e:
            label = f"Range Error solve6: RPY:({m.radians(roll)},{m.radians(pitch)},{m.radians(yaw)}) {e}"
            print(label)
            
        self.state = self.states.segend
    def segend(self):
        dt = millis()-self.segstime
        #print(f"segend {self.framedex} dt: {dt:5.3f}")
        timeout = False
        if(dt) > 50:
            print(f"move timeout: {dt}")
            timeout = True
        if self.stew.is_moved() or timeout:
            self.framedex += 1
            if self.framedex == self.segcount+1:
                self.framedex = 0
                self.state = self.states.wait
                self.segstime = millis()
            else:
                self.state = self.states.segstart
            print(f"next state: {self.state}")
    def wait(self):
        dt = millis()-self.segstime
        if(dt) > 1000:
            print("waited")
            self.state = self.states.advance
    def advance(self):
        print("advance")
        self.cubedex += 1
        if(self.cubedex == self.cubelength):
            self.cubedex = 0
            #raise(Exception("moopsy!")) # kill after one cycle
        self.state = self.states.loadframe
                        
class MoveSM(StateMachine):
    def __init__(self, stew):
        print("new MoveSM")
        super().__init__()
        self.build("movestates", ['finished', 'started']) 
        self.start_time = 0
        self.stew = stew

    def started(self):
        if self.stew.is_moved():
            self.state = self.states.finished
            self.stew.stopall()
            print(f"elapsed {millis() - self.start_time}")
            print("<taskdone/>")
        else:
            pass
            #print("<notdoneyet/>") # gets stuck if I don't report FIXME
    def finished(self):
        # state is finished
        pass
    def moveto(self, roll, pitch, yaw, coll, glyph):
        self.stew.calculate(roll, pitch, yaw, coll, glyph)
        self.start_time = millis()
        self.state = self.states.started
        self.stew.actuate() 
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
    def stopall(self):
        for m in self.cmots:
            m.dc(0)
    def is_moved(self):
        thresh = 25
        done = [False for i in self.cyls]
        for i, cyl in enumerate(self.cyls):
            target = self.pos_ang(cyl)
            current = self.cmots[i].angle()
            speed = self.cmots[i].speed()
            motordone = self.cmots[i].done()
            mathdone = abs(current-target) < thresh
            #done[i] = mathdone
            #done[i] = motordone
            done[i] = motordone or mathdone
            #print(f"cyl: {i} target: {target} current: {current} speed: {speed} motord: {motordone} mathd: {mathdone} d:{done[i]}")
        return sum(done) == len(self.cyls)
     
    def calculate6(self, roll, pitch, yaw, x, y, z):
        try:
            (coll_v, sa, sb, sc) = self.solve6(roll, pitch, yaw, x, y, z) # sets self.cyls  
        except ValueError as e:
            label = f"Range Error solve6: RPY:({m.radians(roll)},{m.radians(pitch)},{m.radians(yaw)}) {e}"
            print(label)
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
    
    # FIXME need to add x, y, z and flag in glyph for quatmode
    # put wvars into a python file everrything can include
    #wvars = ['coll', 'roll', 'pitch', 'yaw', 'glyph'] 
    from stewart_wvars import wvars
    wirep = JoyProtocol(wvars, 2, poller, stdin)
    
    #jsman = JSMan()
    last_input = 0 # last input from base station
    
    disk_def = 10 # degrees +-
    coll_range = 1 # % +-
    
    threshold = memory_calibrate() # max travel of cylinders, degrees
    # arm radius, min cylinder, max cylinder
    #Stew = Stewart(40, 120, 120, 240, 308, threshold) #inner, outer radius, footprint, min, max cylinder extension
    Stew = Stewart(57, 98, 120, 250, 314, threshold)
    identify()
    print("<awake/>")    
    
    msm = MoveSM(Stew)
    ssm = SlerpSM(Stew)
    sm = msm   
    last_sm_tick = 0 
    while True:
        # input loop scan, expects 30-60Hz inputs
        if wirep.poll():
            try:    
                last_input = millis() 

                wirep.decode_wire()
                #print("wirep: ", wirep.vals)
                coll = (wirep.vals['coll']/2 + .5)*coll_range   
                pitch = -1*wirep.vals['roll']*disk_def
                roll = -1*wirep.vals['pitch']*disk_def
                yaw = -1*wirep.vals['yaw']*23
                glyph = wirep.decode_raw('glyph')
                
                if glyph == 255:
                    # keepalive
                    # FIXME: remove.  not necessary anymore with SMs independent of input
                    pass
                else:
                    #print(f"roll:{roll: 3.1f}, pitch:{pitch: 3.1f}, yaw:{yaw: 3.1f} coll:{coll: 3.1f} glyph:{glyph}", end="\n")
                    #for i, cyl in enumerate(Stew.cyls):
                    #    print(f" Cyl {i}:{cyl: 3.1f}mm", end="")
                    #print("")
                    if((glyph & 24) == 24):
                        print("switch a on")
                        sm = ssm
                    else:
                        sm = msm
                        try:
                            msm.moveto(roll, pitch, yaw, coll, glyph)
                        except Exception as e:
                            print("moveto/actuate failed: ", e)
                            print(f"<goodbye/>")
                            return None
                            
                    if((glyph & 40) == 40): # X '0b0101000' SB
                        print(f"<goodbye/>")
                        return None
                        #break
                    
            except Exception as e:
                print("failure in main loop:")
                print(e)
        
        # update current state machine at 100Hz
        ticktime = millis()
        if (ticktime - last_sm_tick) > 10: # 100 hz
            last_sm_tick = ticktime
            sm.tick()

        
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