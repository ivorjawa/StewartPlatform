#!/usr/bin/env python

import sys
import math as m
import time
import numpy as np
#import cv2

import controllers as ctrl

decode_report = ctrl.decode_taranis
gamepad = ctrl.open_taranis()

t0 = time.time()

output = []
if __name__ == "__main__":
    count = 0
    counting = False
    while 1:
        report = gamepad.read(64)
        #report = None
        if report:
            # Taranis major axes 3 5 7 9 lx = 9, ly = 7, rx = 3, ry = 5 
            rd = decode_report(report)
            scale = 256.0
            coll_in = rd['coll']/scale
            roll_in = rd['roll']/scale          
            pitch_in = rd['pitch']/scale
            yaw_in = rd['yaw']/scale
            glyph = rd['glyph']
            if glyph == 24: 
                counting = True
                print("recording")
            if counting:
                output.append([time.time()-t0, coll_in, roll_in, pitch_in, yaw_in, glyph])
                count += 1
                if (count % 100) == 0:
                    print(f"count: {count}")
            if count == 500:
                with open("joylog.csv", "w") as f:
                    print("time,coll,roll,pitch,yaw,glyph", file=f)
                    for line in output:
                        print(",".join([str(x) for x in line]),file=f)
                sys.exit(0)
            
            print(f"time: {time.time() - t0} coll: {coll_in} roll: {roll_in} pitch: {pitch_in} yaw: {yaw_in} glyph: {glyph}")