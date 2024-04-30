import cv2
import os

#props = [x for x in dir(cv2) if str(x).upper().find('CAP_PROP')!=-1]
props = ['CAP_PROP_EXPOSURE', 'CAP_PROP_AUTO_EXPOSURE', 'CAP_PROP_IOS_DEVICE_EXPOSURE']

cam = cv2.VideoCapture(0)

os.system("./uvc-util -I 0 -s exposure-time-abs=100")
"""cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam.set(cv2.CAP_PROP_FPS, 30)
cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cam.set(cv2.CAP_PROP_FOCUS, 0) # 0 should be as close as it gets    
cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # should be manual
cam.set(cv2.CAP_PROP_EXPOSURE, 100) # 15ms
"""

for prop in props:
    pa = getattr(cv2, prop)
    value = cam.get(pa)
    if value != 0.0:
        print(f"{prop}: {value}")
        
    