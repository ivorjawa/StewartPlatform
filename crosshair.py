import sys
import time
import math as m

#from rich import print as rp

import numpy as np
import cv2
import uvc

import pyglet

import linear as lin
from camscan import find_mode

joysticks = pyglet.input.get_joysticks()
assert joysticks, 'No joystick device is connected'
joystick = joysticks[0]
joystick.open()

class crossout(object):
    def __init__(self):
       device = uvc.device_list()[0]
       self.cap = uvc.Capture(device["uid"])
       self.width = 640
       self.height = 480
       self.cap.frame_mode = find_mode(self.cap, self.width, self.height, 30) 
    def loop(self):
        while True:
            frame = self.cap.get_frame()
            frame = frame.bgr
            #ret, frame = cam.read()
            #if not ret:
            #    print("eof?")
            #    break
            
            axes = joystick.device.get_controls()[24:]
    
            coll_in = axes[2].value/2047.0
            roll_in = axes[0].value/1024.0 - 1
            pitch_in = axes[1].value/1024.0 - 1
            yaw_in = axes[3].value/1024.0 - 1
            pitch_in *= -1
            x = np.intp((roll_in+1)*(self.width/2))
            y = np.intp((pitch_in+1)*(self.height/2))
            #print(f"coll: {coll_in:3.3f}, roll: {roll_in:3.3f} pitch: {pitch_in:3.3f} yaw: {yaw_in:3.3f} x: {x} y: {y}")
            print(f"roll: {roll_in:3.3f} pitch: {pitch_in:3.3f} x: {x} y: {y}     ", end='\r')
            cv2.line(frame, np.intp((0, self.height/2)), np.intp((self.width, self.height/2)), (0, 0, 255), 1)
            cv2.line(frame, np.intp((self.width/2, 0)), np.intp((self.width/2, self.height)), (0, 0, 255), 1)
            cv2.line(frame, np.intp((0, y)), np.intp((self.width, y)), (0, 255, 0), 1)
            cv2.line(frame, np.intp((x, 0)), np.intp((x, self.height)), (0, 255, 0), 1)
            cv2.imshow("crapture", frame)
            if cv2.pollKey() == 27:
                break
                
if __name__ == "__main__":
    cr = crossout() 
    cr.loop()