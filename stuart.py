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
    if (threshold < 0) or (threshold > 2000):
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
    
if __name__ == "__main__":
    # pybricksdev run ble -n rotor rotor.py
    #teststorage()
    #testwritestore()
    #run_remote()
    try:
        #pass # when running above tests
        #run_remote() # full talks to remote run under ./rotorbase.py
        full_calibrate() # should be done with rotors and scissor link detached
    except Exception as e:
        print("General failure: ", e)
    identify()   