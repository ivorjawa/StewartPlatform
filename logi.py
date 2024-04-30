#!/usr/bin/env python

import os
import cv2
import pyglet
import numpy as np

joysticks = pyglet.input.get_joysticks()
assert joysticks, 'No joystick device is connected'
joystick = joysticks[0]
joystick.open()

def logi():
    width = 640
    height = 480
    cam = cv2.VideoCapture(0)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cam.set(cv2.CAP_PROP_FPS, 30)
    #cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    #cam.set(cv2.CAP_PROP_FOCUS, 0) # 0 should be as close as it gets    
    #cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # should be manual
    #cam.set(cv2.CAP_PROP_EXPOSURE, 100) # 15ms
    
    os.system("./uvc-util -I 0 -s auto-focus=0")
    os.system("./uvc-util -I 0 -s focus-abs=0")
    os.system("./uvc-util -I 0 -s auto-exposure-mode=1")
    os.system("./uvc-util -I 0 -s exposure-time-abs=150")
    
    print(f"exp: {cam.get(cv2.CAP_PROP_EXPOSURE)}")
    # 
    #
    #width = 640
    #height = 480
    
    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    #cv2.resizeWindow("output", width, height) 
    
    last_exp = 0
    while 1:
        axes = joystick.device.get_controls()[24:]

        #coll_in = axes[2].value/2047.0
        #focus = np.intp(coll_in*20)
        exp_time = axes[2].value # 3 .. 2047
        if(exp_time != last_exp):
            os.system(f"./uvc-util -I 0 -s exposure-time-abs={exp_time}")
            print(f"setting exp_time to {exp_time}")
            last_exp = exp_time
            #cam.set(cv2.CAP_PROP_EXPOSURE, exp_time) # 15ms
        
        
        ret, frame = cam.read()
        if not ret:
            print("eof?")
            break
        cv2.line(frame, np.intp((0, height/2)), np.intp((width, height/2)), (0, 0, 255), 1)
        cv2.line(frame, np.intp((width/2, 0)), np.intp((width/2, height)), (0, 0, 255), 1)
            
        cv2.imshow("output", frame)
        if cv2.pollKey() == 27:
            break
            
if __name__ == "__main__":
    logi()           