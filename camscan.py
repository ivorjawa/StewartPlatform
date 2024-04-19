#!/usr/bin/env python

import sys
import time
import math as m

from rich import print as rp

import numpy as np
import cv2
import uvc

import linear as lin

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
    """
    Control[0], auto exposure mode
    https://community.infineon.com/t5/USB-superspeed-peripherals/question-about-UVC-auto-exposure-mode/td-p/52722
    1: manual mode
    2: auto mode
    4: shutter priority
    8: aperture priority
    
    
    Control[0]: Auto Exposure Mode                  min:      1 max      8 value:      8
    Doc: 

    Control[1]: Absolute Exposure Time              min:      2 max    500 value:     84
    Doc: The `time` parameter should be provided in units of 0.0001 seconds (e.g., use the value 100 for a 10ms exposure period). Auto exposure should be set to
    `manual` or `shutter_priority`before attempting to change this setting.

    Control[2]: Auto Focus                          min:      0 max      1 value:      1
    Doc: Enable the Auto Focus

    Control[3]: Absolute Focus                      min:      0 max    200 value:     49
    Doc: Set Absolute Focus

    Control[4]: Backlight Compensation              min:      0 max      4 value:      1
    Doc: The Backlight Compensation Control is used to specify the backlight compensation.

    Control[5]: Brightness                          min:   -128 max    127 value:      0
    Doc: This is used to specify the brightness.

    Control[6]: Contrast                            min:     60 max    255 value:    136
    Doc: This is used to specify the contrast value.

    Control[7]: Power Line frequency                min:      0 max      2 value:      2
    Doc: This control allows the host software to specify the local power line frequency,for the implementing anti-flicker processing.

    Control[8]: Hue                                 min:   -128 max    127 value:      0
    Doc: This is used to specify the hue setting.

    Control[9]: Saturation                          min:      0 max    255 value:    150
    Doc: This is used to specify the saturation setting.

    Control[10]: Sharpness                           min:      0 max    255 value:     50
    Doc: This is used to specify the sharpness setting.

    Control[11]: Gamma                               min:     72 max    500 value:    100
    Doc: This is used to specify the gamma setting.

    Control[12]: White Balance temperature           min:   2800 max   6500 value:   4434
    Doc: This is used to specify the white balance setting as a color temperature in degrees Kelvin.

    Control[13]: White Balance temperature,Auto      min:      0 max      1 value:      1
    Doc: The White Balance Temperature Auto Control setting determines whether the device will provide automatic adjustment of the related control.1 indicates 
    automatic adjustment is enabled
    
    """
    device = uvc.device_list()[0]
    cap = uvc.Capture(device["uid"])
    for i, c in enumerate(cap.controls):
        print(f"Control[{i}]: {c.display_name:35} min: {c.min_val:6} max {c.max_val:6} value: {c.value:6}")
        rp("[green bold]Doc: %s[/green bold]\n" % c.doc)
        
    cap.controls[0] = 0 # 1 manual, 2 auto, 4 aperture priority,  8 shutter priority
    cap.controls[1] = 100 # exposure time 10ms
    cap.controls[2] = 0 # auto focus
    cap.controls[3] = 2000 # absolute focus, 0 ... 200
    
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

# intersection between line(p1, p2) and line(p3, p4)
def intersect(p1, p2, p3, p4):
    x1,y1,_ = p1
    x2,y2,_ = p2
    x3,y3,_ = p3
    x4,y4,_ = p4
    denom = (y4-y3)*(x2-x1) - (x4-x3)*(y2-y1)
    if denom == 0: # parallel
        return None
    ua = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / denom
    if ua < 0 or ua > 1: # out of range
        return None
    ub = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / denom
    if ub < 0 or ub > 1: # out of range
        return None
    x = x1 + ua * (x2-x1)
    y = y1 + ua * (y2-y1)
    return (x,y)
    
def triangle_defuckery(Leglen, Ared, Bblue, Ggreen):
    print(f"Ared: {Ared}, Bblue: {Bblue}, Ggreen: {Ggreen}")
    
    A = lin.vector(*Ared[:2], 0)
    B = lin.vector(*Bblue[:2], 0)
    G = lin.vector(*Ggreen[:2], 0)
    print(f"A red: {A}, B blue: {B}, G green: {G}")
    

    # angle between AB and AG
    AB = B-A
    AG = G-A
    BG = G-B
    BA = -AB
    GA = -AG
    GB = -BG
    print(f"AB len: {lin.vmag(AB)}")
    print(f"AG len: {lin.vmag(AG)}")
    print(f"BG len: {lin.vmag(BG)}")
    
    BAG = lin.included_angle(AB, AG)
    AGB = lin.included_angle(GA, GB)
    ABG = lin.included_angle(BG, BA)
    print(f"AB: {AB}, AG: {AG} Angle BAG: {m.degrees(BAG)}")
    print(f"AG: {AG}, BG: {BG} Angle AGB: {m.degrees(AGB)}")
    print(f"AB: {AB}, BG: {BG} Angle ABG: {m.degrees(ABG)}")
    
    #midpoint of lines
    AGc = A+AG/2
    ABc = A+AB/2
    print(f"AGc: {AGc}, ABc: {ABc}")
    
    # center of triangle
    GABc = ABc-G
    BAGc = AGc-B 
    C = intersect(ABc, G, AGc, B)
    #C = lin.cross(GABc, BAGc)
    
    print(f"GABc: {GABc}, BAGc: {BAGc}")
    #print(f"center: {C}")
    return AGc.astype('int16'), ABc.astype('int16'), np.array(C, dtype='int16')
    #sys.exit(0)
    
            
def redmenace():
    #cam = cv2.VideoCapture("/Users/kujawa/Desktop/goodballdata.mov")
    cam = cv2.VideoCapture("/Users/kujawa/Desktop/color_ring_crop.mov")
    width = 640
    height = 480
    
    cv2.namedWindow("output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("output", width, height) 
    
    good_circ_run = [] # how many times in a row have we got a good solution
    #out = cv2.VideoWriter('output.avi', -1, 30.0, (640,480))
    out = cv2.VideoWriter('output.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 10, (width,height))
    while 1:
        halt = False
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
            bigcircles = []
            smallcircles = []
            circles = np.uint16(np.around(circles))
            for i in circles[0,:]:
                 # draw the outer circle
                 r = i[2]
                 if (r > 22) and (r < 60):
                     #print(f"dud rejected with radius {r}")
                     continue
                     
                 if r <= 22:
                     smallcircles.append(i)
                 else:
                     bigcircles.append(i)
                 #print(f"circle ({i[0]}, {i[1]}), r = {i[2]}")
                 cv2.circle(red,(i[0],i[1]),i[2],(0,255,0),2)
                 # draw the center of the circle
                 cv2.circle(red,(i[0],i[1]),2,(0,0,255),3)
                 
            # big ball can fall off table
            if (len(smallcircles) == 3) and (len(bigcircles) == 1):
                # FIXME this time() is bogus, should be frame dt from camera
                # but it will be correct when doing realtime
                good_circ_run.append((time.time(), bigcircles[0], smallcircles))
                print(f"Potential solution found! Current run {len(good_circ_run)}")
            else:
                good_circ_run.clear()
        
            gcr = len(good_circ_run)
            if gcr > 0:
                lastthree = good_circ_run[-3:]
                if gcr >= 1:
                    # we can calculate position
                    t1, big1, small1 = lastthree[0]
                    p1, p2, p3 = small1
                    cv2.line(red, (p1[0],p1[1]), (p2[0],p2[1]), (0, 0, 255), 3)
                    cv2.line(red, (p2[0],p2[1]), (p3[0],p3[1]), (0, 0, 255), 3)
                    cv2.line(red, (p1[0],p1[1]), (p3[0],p3[1]), (0, 0, 255), 3)
                    #triangle_defuckery(Leglen, Ared, Bblue, Ggreen)
                    # try to figure out red later for real order
                    AGc, BGc, C = triangle_defuckery(130, p1, p2, p3)
                    print(f"AGc: {AGc}, BGc: {BGc}")
                    cv2.line(red, (p2[0],p2[1]), (AGc[0],AGc[1]), (128, 128, 128), 3) # blue to AGc (yellow)
                    cv2.line(red, (p3[0],p3[1]), (BGc[0],BGc[1]), (128, 128, 128), 3) # green to BGc (cyan)
                    cv2.circle(red, (AGc[0], AGc[1]), 10, (0, 255, 255), -1) 
                    cv2.circle(red, (BGc[0], BGc[1]), 10, (255, 255, 0), -1)
                    cv2.circle(red, (C[0], C[1]), 10, (255, 255, 255), -1)
                    
                    
                    cv2.circle(red,(p1[0],p1[1]),10,(0,0,255),-1)
                    cv2.circle(red,(p2[0],p2[1]),10,(255,0,0),-1)
                    cv2.circle(red,(p3[0],p3[1]),10,(0,255,0),-1)
                    
                    halt = True
                if gcr >= 2:
                    # we can calculate position and velocity
                    t2, big2, small2 = lastthree[1]
                    s1 = lin.vector(big1[:2])
                    s2 = lin.vector(big2[:2])
                    ds1 = lin.vmag(s2-s1)
                    dt1 = t2-t1
                    v1 = ds1/dt1
                    if ds1 < 50:
                        print(f"v1: {v1:3.2f} pix/s ds1: {ds1:3.2f} dt1: {dt1:3.4f}") #  vs: {s2-s1} s1: {s1} s2: {s2}
                        cv2.line(red, *s2, *(s2+(s2-s1)), (255, 255, 0), 2)
                    else:
                        print(f"bad ds {ds1:3.2f} rejected")
                
                    # draw entire path here
                    _, path1, _ = good_circ_run[0]
                    for _, path2, _ in good_circ_run[1:]:
                        cv2.line(red, (path1[0],path1[1]), (path2[0],path2[1]), (255, 0, 255), 3)
                        path1 = path2
                    
                if gcr >= 3:
                    # we can calculate position, velocity, and acceleration
                    #
                    t3, big3, small3 = lastthree[2]
                    if ds1 < 50: # reject bad calculations
                        s3 = lin.vector(big2[:2])
                        ds2 = lin.vmag(s3-s2)
                        dt2 = t3-t2
                        v2 = ds2/dt2
                        dv = v2 - v1
                        a = dv/dt2
                        if ds2 < 50:
                            print(f"v2: {v2:3.2f} pix/s a: {a:3.2f} pix/s^2 ds2: {ds2:3.2f} dt2: {dt2:3.4f}") #  vs: {s2-s1} s1: {s1} s2: {s2}
                            cv2.line(red, *s3, *(s3+(s3-s2)), (255, 255, 0), 2)
                        else:
                            print(f"bad ds {ds1:3.2f} rejected")
                    
                
 
            
        #red = cv2.cvtColor(rcan,cv2.COLOR_GRAY2BGR)
        out.write(red)
        canvas[0:480, 0:640] = red
        
        #cv2.line(canvas, (0,0), (int(width), int(height)), (0, 0, 255), 1)
        cv2.imshow("output", canvas)
        if halt:
            cv2.waitKey(0)
            sys.exit(0)
        if cv2.pollKey() == 27:
            break 
    out.release()           

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
"""
            Red Ball is 50mm.  Color circles are 15mm.
            Distance from sensor to color wheels is 24.5cm at 50% collective
            
        x   * Bubble level and new brick, communicate async with new brick while reading video
        x   * aruco markers for ring if possible
            * Camera calibration -- use statemachine() to run through camera calibration smoothly
            * use statemachine and quaternions slerp to animate through a path smoothly
            * IMU in new brick vs bubble
        x   * Recognition of bubble
            * Recognition of Lego pneumatic gauge
            * Calibration checkerboard for platform and program to control and calibrate on new brick
        x   * Pybricks asynchronous comm with opencv camera read and pyglet.
            * Write 6DOF isolation demo like video on parallel and serial robots
            * Fix joystick dead zone so no step at zero — subtract n steps from each side. 
            * Benchmark rotation rates to estimate needed number of points for slerp 
            * Degrees per second on isolated axes
            * Do boneheaded elevator animation in f360 -- Servo, wire, horn, hinge. 
            https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html
            https://pysource.com/2021/05/28/measure-size-of-an-object-with-opencv-aruco-marker-and-python/
            https://github.com/niconielsen32/ComputerVision/tree/master
             
            
"""

if __name__ == "__main__":
    gronkulate() # camera interrogation
    #gupervate()
    #cavitate()
    #kypertate() # 4-by
    #redmenace() # 1-by red
    #clickitate() # HSV clicker
    
    