#!/usr/bin/env python

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
            
if __name__ == "__main__":
    gronkulate()
    pass
    