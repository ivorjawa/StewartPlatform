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

def notch(a, min, max):
    over = np.array(np.greater(a, min), dtype='uint8')
    under = np.array(np.less(a, max), dtype='uint8')
    mask = over*under
    return a * mask, mask

def color_filter(img, crange):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    
    oh, omask = notch(h, *crange)
    
    vmask, vmm = notch(v,240,256)
    
    # only the bright pixels in our selected hue
    oh = oh*vmm
    
    # flatten out saturation
    s = 255 * vmm
    
    final_hsv = cv2.merge((oh, s, vmask))
    img2 = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img2
        
def darkitate(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    #lim = 255 - value
    #v[v > lim] = 255
    #v[v <= lim] += value
    #v = v*.5
    ret,v = cv2.threshold(v,240,255,cv2.THRESH_BINARY)
    #v = np.sqrt(v)
    #v = v*.5
    #v = v*v
    #v = np.array(v, dtype='uint8')

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img
    
def kypertate():
    #cam = cv2.VideoCapture("/Users/kujawa/Desktop/goodballdata.mov")
    cam = cv2.VideoCapture("/Users/kujawa/Desktop/color_ring_crop.mov")
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
        #blue = cv2.cvtColor(res[:, :, 0],cv2.COLOR_GRAY2BGR)
        #green = cv2.cvtColor(res[:, :, 1],cv2.COLOR_GRAY2BGR)
        #red = cv2.cvtColor(res[:, :, 2],cv2.COLOR_GRAY2BGR)
        #print("blue", blue.shape, blue.dtype)
        
        #canny thresholds
        lt = 100
        ut = 200
        blin = res[:, :, 0]
        blcan = cv2.Canny(blin, lt, ut)
        #blue = cv2.cvtColor(blcan,cv2.COLOR_GRAY2BGR)
        
        blblur = img = cv2.medianBlur(blin,5)
        blue = cv2.cvtColor(blblur,cv2.COLOR_GRAY2BGR)
        
        circles = cv2.HoughCircles(blblur,cv2.HOUGH_GRADIENT,1,20,param1=50,param2=30,minRadius=0,maxRadius=0)
        circles = None
        if np.any(circles):
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                 # draw the outer circle
                 cv2.circle(blue,(i[0],i[1]),i[2],(0,255,0),2)
                 # draw the center of the circle
                 cv2.circle(blue,(i[0],i[1]),2,(0,0,255),3)
        
        grin = res[:, :, 0]
        grin = cv2.Canny(grin, lt, ut)
        green = cv2.cvtColor(grin,cv2.COLOR_GRAY2BGR)
        rin = res[:, :, 0]
        rin = cv2.Canny(rin, lt, ut)
        red = cv2.cvtColor(rin,cv2.COLOR_GRAY2BGR)
        
        
        orange = [161, 176] 
        yellow = [28, 32]
        #blue = darkitate(blue)
        #blue = color_filter(res, orange)
        #red = darkitate(red)
        #red = color_filter(res, yellow)
        #green = darkitate(green)
        
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

                
def redmenace():
    #cam = cv2.VideoCapture("/Users/kujawa/Desktop/goodballdata.mov")
    cam = cv2.VideoCapture("/Users/kujawa/Desktop/color_ring_crop.mov")
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
        
        res = cv2.resize(frame,(int(width), int(height)), interpolation = cv2.INTER_CUBIC)
        #blue = cv2.cvtColor(res[:, :, 0],cv2.COLOR_GRAY2BGR)
        #green = cv2.cvtColor(res[:, :, 1],cv2.COLOR_GRAY2BGR)
        #red = cv2.cvtColor(res[:, :, 2],cv2.COLOR_GRAY2BGR)
        #print("blue", blue.shape, blue.dtype)
        
        blin = res[:, :, 0]
        grin = res[:, :, 1]
        rin = res[:, :, 2]
        
        #canny thresholds
        lt = 128
        ut = 255
        #rcan = cv2.Canny(rin, lt, ut)
        #red = cv2.cvtColor(rcan,cv2.COLOR_GRAY2BGR)
        
        rblur = cv2.medianBlur(rin,5)
        red = cv2.cvtColor(rblur,cv2.COLOR_GRAY2BGR)
        
        circles = cv2.HoughCircles(rblur,cv2.HOUGH_GRADIENT,1,20,param1=130,param2=30,minRadius=15,maxRadius=80)
        #circles = None
        if np.any(circles):
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                 # draw the outer circle
                 print(f"circle ({i[0]}, {i[1]}), r = {i[2]}")
                 cv2.circle(red,(i[0],i[1]),i[2],(0,255,0),2)
                 # draw the center of the circle
                 cv2.circle(red,(i[0],i[1]),2,(0,0,255),3)
        
        #red = cv2.cvtColor(rcan,cv2.COLOR_GRAY2BGR)
        
        canvas[0:480, 0:640] = red
        
        cv2.line(canvas, (0,0), (int(width), int(height)), (0, 0, 255), 1)
        cv2.imshow("output", canvas)
        if cv2.pollKey() == 27:
            break            

class Clacker(object):
    def __init__(self):
        self.image = None
        self.hasimage = False

    def click(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print(f"click!  x: {x} y: {y}")
            if self.hasimage:
                point = self.image[y, x]
                hsv = cv2.cvtColor(np.uint8([[point]]), cv2.COLOR_BGR2HSV)
                print(f"clack! BGR: {point} HSV: {hsv}")
                
        
        
    def clickitate(self):    
        cam = cv2.VideoCapture("/Users/kujawa/Desktop/color_ring_crop.mov")

        width = 640
        height = 480
    
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("output", width, height) 
        cv2.setMouseCallback("output", self.click)
    
        while 1:
        
            ret, frame = cam.read()
            if not ret:
                print("eof?")
                break
                
            frame = cv2.resize(frame,(int(width), int(height)), interpolation = cv2.INTER_CUBIC)
            
            self.image = frame
            self.hasimage = True

            cv2.imshow("output", frame)
            key = cv2.waitKey(0)
            if key == 27:
                break
                                
def clickitate():
    clack = Clacker()
    clack.clickitate()
            
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
    #kypertate() # 4-by
    redmenace() # 1-by red
    #clickitate() # HSV clicker
    
    