#!/usr/bin/env python

import numpy as np
import cv2
import uvc

#camprops = list(filter(lambda x: x.find("CAP_") == 0, dir(cv2)))
#cam = cv2.VideoCapture(0)
#cam = cv2.VideoCapture("/Users/kujawa/Desktop/goodballdata.mov")



#cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
#cam.set(cv2.CAP_PROP_FPS, 60)
#cam.set(cv2.CAP_PROP_AUTOFOCUS, 1)

#for i, prop in enumerate(camprops):
#    val = cam.get(getattr(cv2, prop))
#    if val != 0:
#        print(i, prop, val)
 
def find_mode(cap, width, height, fps):
    """
    for mode in cap.available_modes:
        print(
            f"MODE: {mode.width} x {mode.height} @ {mode.fps} ({mode.format_name}) {mode.__class__}"
        )
    """
    for mode in cap.available_modes:
        if (mode.width == width) and (mode.height == height) and (mode.fps == fps):
            return mode
                
def gronkulate():   
    device = uvc.device_list()[0]
    cap = uvc.Capture(device["uid"])
    for c in cap.controls:
        print(f"Control: {c.display_name:35} min: {c.min_val:6} max {c.max_val:6} value: {c.value:6}")
    cap.frame_mode = find_mode(cap, 640, 480, 30) 
    print(f"frame mode: {cap.frame_mode}")
     
    while True:
        frame = cap.get_frame()
        
        #ret, frame = cam.read()
        #if not ret:
        #    print("eof?")
        #    break
        cv2.imshow("crapture", frame.bgr)
        if cv2.pollKey() == 27:
            break

def gupervate():
    device = uvc.device_list()[0]
    cap = uvc.Capture(device["uid"])
    width = 640
    height = 480
    cap.frame_mode = find_mode(cap, width, height, 30) 
    
    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("output", width, height) 
    
    while 1:
        canvas = np.zeros((height, width, 3), np.uint8)
        #cv2.line(img, ituple(p1), ituple(p2), ituple(c), width)
        frame = cap.get_frame().bgr
        
        res = cv2.resize(frame,(int(width/2), int(height/2)), interpolation = cv2.INTER_CUBIC)
        
        #>>> ball = img[280:340, 330:390]
        #>>> img[273:333, 100:160] = ball
        #ba = res[0:0, 240:320]
        #print("ba", ba.shape, ba.dtype)
        canvas[0:240, 0:320] = res
        #print('canvas', canvas.shape, canvas.dtype)
        #print("res", res.shape, res.dtype)
        
        cv2.line(canvas, (0,0), (int(width), int(height)), (0, 0, 255), 1)
        cv2.imshow("output", canvas)
        if cv2.pollKey() == 27:
            break

def kypertate():
    cam = cv2.VideoCapture("/Users/kujawa/Desktop/goodballdata.mov")

    width = 640
    height = 480
    
    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("output", width, height) 
    
    while 1:
        
        ret, frame = cam.read()
        if not ret:
            print("eof?")
            break
        
        canvas = np.zeros((height, width, 3), np.uint8)
        #cv2.line(img, ituple(p1), ituple(p2), ituple(c), width)
        #frame = cap.get_frame().bgr
        
        res = cv2.resize(frame,(int(width/2), int(height/2)), interpolation = cv2.INTER_CUBIC)
        blue = cv2.cvtColor(res[:, :, 0],cv2.COLOR_GRAY2BGR)
        green = cv2.cvtColor(res[:, :, 1],cv2.COLOR_GRAY2BGR)
        red = cv2.cvtColor(res[:, :, 2],cv2.COLOR_GRAY2BGR)
        
        #>>> ball = img[280:340, 330:390]
        #>>> img[273:333, 100:160] = ball
        #ba = res[0:0, 240:320]
        #print("ba", ba.shape, ba.dtype)
        canvas[0:240, 0:320] = res
        canvas[240:480, 0:320] = blue
        canvas[0:240, 320:640] = red
        canvas[240:480, 320:640] = green
        
        #print('canvas', canvas.shape, canvas.dtype)
        #print("res", res.shape, res.dtype)
        
        cv2.line(canvas, (0,0), (int(width), int(height)), (0, 0, 255), 1)
        cv2.imshow("output", canvas)
        if cv2.pollKey() == 27:
            break            
 
def cavitate():
    img = cv2.imread('roi.jpg')  
    print("roi", img.shape, img.dtype) 
    # row_low:row_high, col_low:col_high
    ball = img[280:340, 330:390]
    print("ball", ball.shape, ball.dtype) 
    img[273:333, 100:160] = ball  
    
    while 1:
        cv2.imshow("output", img) 
        if cv2.pollKey() == 27:
            break         

if __name__ == "__main__":
    #gronkulate()
    #gupervate()
    #cavitate()
    kypertate()
    
    