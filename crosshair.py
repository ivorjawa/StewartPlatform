#!/usr/bin/env python

import sys
import time
import math as m
from enum import Enum

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

# https://community.infineon.com/t5/USB-superspeed-peripherals/question-about-UVC-auto-exposure-mode/td-p/52722
# the `time` parameter should be provided in units of 0.0001 seconds (e.g., use the value 100 for a 10ms exposure period). 
# Auto exposure should be set to `manual` or `shutter_priority`before attempting to change this setting.


class AutoExpModes(Enum):
    MANUAL = 1
    AUTO = 2
    SHUTTER_PRI = 4
    APERTURE_PRI = 8

class crossout(object):
    def __init__(self):
       device = uvc.device_list()[0]
       self.cap = uvc.Capture(device["uid"])
       self.width = 640
       self.height = 480
       self.cap.frame_mode = find_mode(self.cap, self.width, self.height, 30) 
       self.cont_dict = {}
       for i, c in enumerate(self.cap.controls):
           key = '_'.join(c.display_name.lower().split(' '))
           self.cont_dict[key] = c
           print(f"{key}: {c.value}")
       self.cont_dict['auto_focus'].value = 0 
       self.cont_dict['absolute_focus'].value = 0 # absolute focus, 1 ... 200
       self.cont_dict['auto_exposure_mode'].value = 1
       exp_time = 5
       self.cont_dict['absolute_exposure_time'].value = exp_time 
       self.cont_dict['sharpness'].value = 255
       
       for i, c in enumerate(self.cap.controls):
           key = '_'.join(c.display_name.lower().split(' '))
           print(f"out: {key}: {c.value}")
    
    def loop(self):
        lastfocus = 0   
        while True:
            #print("getting frame")
            frame = self.cap.get_frame()
            frame = frame.bgr
            #print("got bgr frame")
            #ret, frame = cam.read()
            #if not ret:
            #    print("eof?")
            #    break
            
            axes = joystick.device.get_controls()[24:]
    
            coll_in = axes[2].value/2047.0
            focus = np.intp(coll_in*20)
            #self.cont_dict['absolute_focus'].value = focus # absolute focus, 1 ... 200
            self.cont_dict['absolute_exposure_time'].value = focus 
            
            if focus != lastfocus:
                print(f"set focus to {focus}")
                lastfocus = focus
            
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