#!/usr/bin/env python
import sys
import os
import datetime
import logging, logging.handlers
import multiprocessing as mp
import asyncio
import time
from enum import Enum
import random
import pickle

import numpy as np
import cv2
#import uvc 

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus
import bleak

from rich import print as rprint

from joycode import JoyProtocol
#from statemachine import BasicSM, Transition, State
from rotorbase import LoggingBricksHub

import StewartPlatform


class Calibrator(object):
    def __init__(self, sqx=8, sqy=6, width=640, height=480):            
        # termination criteria
        self.criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
 
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        self.sqx = sqx
        self.sqy = sqy
        self.objp = np.zeros((sqx*sqy,3), np.float32)
        self.objp[:,:2] = np.mgrid[0:sqx,0:sqy].T.reshape(-1,2)
 
        # Arrays to store object points and image points from all the images.
        self.objpoints = [] # 3d point in real world space
        self.imgpoints = [] # 2d points in image plane.
 
        self.width = width
        self.height = height
        #self.device = uvc.device_list()[0]
        #self.cap = uvc.Capture(self.device["uid"]) 
        #self.cap.frame_mode = find_mode(self.cap, width, height, 30)
        self.sufficient_pts = False 
        #print(f"frame mode: {self.cap.frame_mode}")
    def calibrate_frame(self, img):
        #img = self.cap.get_frame()
        #print (img)
        #img = img.bgr
        print("calibrate_frame")
        self.gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #gray = img.gray
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(self.gray, (self.sqx,self.sqy), None)

        # If found, add object points, image points (after refining them)
        waittime = 1
        if ret == True:
            print("found chessboard")
            self.objpoints.append(self.objp)


            corners2 = cv2.cornerSubPix(self.gray,corners, (11,11), (-1,-1), self.criteria)
            self.imgpoints.append(corners2)

            # Draw and display the corners
            cv2.drawChessboardCorners(img, (self.sqx,self.sqy), corners2, ret)
            #waittime = 500
    
        cv2.line(img, (int(0),int(self.height/2)), (int(self.width), int(self.height/2)), (0, 0, 255), 1)
        cv2.line(img, (int(self.width/2),int(0)), (int(self.width/2), int(self.height)), (0, 0, 255), 1)
        cv2.imshow('calibrator', img)
        return ret, True
    def calculate(self):
        numpts = len(self.objpoints)
        rprint(f"now I have {numpts} points", end='\r')
        if numpts >= 50:
            print("")
            self.sufficient_pts = True
            ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(self.objpoints, self.imgpoints, self.gray.shape[::-1], None, None)
            if ret:
                self.mtx = mtx
                self.dist = dist
                self.rvecs = rvecs
                self.tvecs = tvecs
                rprint(f"Successful calibration")
                rprint(f"Camera matrix: {mtx}")
                rprint(f"Distortion coefficients: {dist}")
                caminfo = {"cam_mtx": mtx, "distortion": dist}
                pfilename = "caminfo.pickle"
                with open(pfilename, "wb") as f:
                    pickle.dump(caminfo, f)
                print(f"saved as {pfilename}")
                #print(f"Rotation vectors: {rvecs}")
                #print(f"Translation vectors: {tvecs}") 
            return
                
class AEnum(object):
    # this is good enough for state machines in micropython
    def __init__(self, enumname, enums, start=1):
        self.__enumname = enumname
        for i, e in enumerate(enums):
            setattr(self, e, i+start)

#"maps -1.0..1.0 to 0..255"
one28 = lambda x: int(((x+1)/2)*255)

     
class CalibSM(object):
    """
    Generate pose and signal
    Wait for movement to complete
    Scan for board and save if found
    Loop until points is 50
    """
    """
    command full left roll
    wait for completion
    calculate time
    seek pose
    command full right roll
    wait for completion 
    calculate time
    seek pose 
    ...
    """
    def __init__(self, fromq, toq):
        self.fromq = fromq
        self.toq = toq

        self.start_time = 0
        self.end_time = 0
        self.woke = False
        self.moving = False
        self.poses = []
        self.posegoal = 50
        
        self.calibrator = Calibrator()
        #self.cur_pitch = -0.5
        
        self.width = 640
        self.height = 480
        #self.device = uvc.device_list()[0]
        #self.cap = uvc.Capture(self.device["uid"]) 
        #self.cap.frame_mode = find_mode(self.cap, self.width, self.height, 30)
        #self.cont_dict = {}
        #for i, c in enumerate(self.cap.controls):
        #    key = '_'.join(c.display_name.lower().split(' '))
        #    self.cont_dict[key] = c
        #self.cont_dict['auto_focus'].value = 0 
        #self.cont_dict['absolute_focus'].value = 0 # absolute focus, 1 ... 200
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cam.set(cv2.CAP_PROP_FPS, 30)
        #cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        #cam.set(cv2.CAP_PROP_FOCUS, 0) # 0 should be as close as it gets    
        #cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # should be manual
        #cam.set(cv2.CAP_PROP_EXPOSURE, 100) # 15ms
    
        os.system("./uvc-util -I 0 -s auto-focus=0")
        os.system("./uvc-util -I 0 -s focus-abs=0")
        os.system("./uvc-util -I 0 -s auto-exposure-mode=1")
        os.system("./uvc-util -I 0 -s exposure-time-abs=150")
        
        self.states = Enum('cstates', ['wait_start', 'gen_sig', 'wait_move', 'scan', 'count', 'done'])
        self.state = self.states.wait_start
    
    def recognize(self):
        print("recognizer")
        ret, pose = self.calibrator.calibrate_frame(self.frame)
        return ret, pose
        
    def tick(self):
        #print(f"tick(): self.state: {self.state} |{self.state == self.states.wait_start}|")
        if self.state == self.states.wait_start:
            #print("asleep")
            if self.woke:
                self.state = self.states.gen_sig
                print("generating signal")
        elif self.state == self.states.gen_sig:
            #StewartPlatform.cSD = Flatmode
            #StewartPlatform.cSC set is cup motion, otherwise dome motion
            #.cSB is kill switch
            modeglyph = StewartPlatform.cSC|StewartPlatform.cSD
            cdict = {
                'roll': one28(random.random()-.5), 
                'pitch': one28(random.random()-.5), 
                'yaw': one28(random.random()-.5), 
                'coll': one28(random.random()-.5), 
                'glyph': modeglyph
            }
            #print(f"cdict: {cdict}")
            #self.cur_pitch += 1/self.posegoal
            self.toq.put_nowait(cdict)
            self.moving = True
            self.start_time = time.time()
            print("sent command")
            self.state = self.states.wait_move
        elif self.state == self.states.wait_move:
            now = time.time()
            if self.moving == True:
                if (now - self.start_time > 5):
                    print("Timeout waiting for robot to move, kicking")
                    self.state = self.states.gen_sig
            elif self.moving == False:
                self.end_time = now
                self.state = self.states.scan
                print(f"got movement, took {self.end_time-self.start_time:3.3f}s")
        elif self.state == self.states.scan:
            ret, pose = self.recognize()
            if ret:
                self.poses.append(pose)
                print(f"appended pose {len(self.poses)}")
            self.state = self.states.count
        elif self.state == self.states.count:
            if len(self.poses) >= self.posegoal:
                print("enough poses found")
                self.calibrator.calculate()
                self.state = self.states.done
            else:
                self.state = self.states.gen_sig           
        else:
            #print(f"sufficient poses found {self.state}")
            pass
    
    def loop(self):
        while 1:
            #self.frame = self.cap.get_frame().bgr
            ret, self.frame = self.cam.read()
            if not ret:
                print("eof?")
                break

            cv2.line(self.frame, np.intp((0, self.height/2)), np.intp((self.width, self.height/2)), (0, 0, 255), 1)
            cv2.line(self.frame, np.intp((self.width/2, 0)), np.intp((self.width/2, self.height)), (0, 0, 255), 1)
            cv2.imshow('calibrator', self.frame)
            try:
                token = self.fromq.get_nowait()
                print(f"got token {token}")
                if token == "<awake/>":
                    self.woke = True
                elif token == "<taskdone/>":
                    self.moving = False
                else:
                    print(f"got unknown token: {token}")
            except Exception as e:
                pass
            self.tick()
            if cv2.pollKey() == 27:
                break
        cv2.destroyAllWindows()
        return        

def load_img(fromq, toq):
    csm = CalibSM(fromq, toq)
    csm.loop()

class  LoggingQueuedBricksHub(PybricksHub):
    """
        A PybricksHub that can act on things received by _line_handler.
        <report> (lines) </report> will create a log file from 
        logfilename, ideally comma-separated values.
        <goodbye/> is the disconnect single from the hub.
    """
    def __init__(self, logfilename, toq):
        super().__init__()
        self.csvfile = None
        self.csv_stemname = logfilename
        self.toq = toq
        
    def _line_handler(self, line: bytes) -> None:
        try:
            l = line.decode()
            logging.debug(f"Hub Sent:  {l}")
            
            if l == "<report>":
                fn = "%s_%s.csv" % (self.csv_stemname,
                                    str(datetime.datetime.now().isoformat()))
                self.csvfile = open(fn, "w")
                logging.info("logging to ", fn)
                return
            elif l == "</report>":
                if(self.csvfile):
                    self.csvfile.close()
                    self.csvfile = None
                    logging.info("done logging")
            elif l == "<goodbye/>":
                sys.exit(1)
            elif l == "<awake/>":
                self.toq.put_nowait(l)
            elif l == "<taskdone/>":
                self.toq.put_nowait(l) 
                print("queued taskdone")           
            if(self.csvfile):
                print(l, file=self.csvfile)
        except Exception as e:
            logging.error(f"_line_handler error: {e}")
        
 
class BaseStation(object):
    def __init__(self, fromq, toq):
        self.fromq = fromq
        self.toq = toq
        self.last_sent = time.time()
        #self.gamepad = ctrl.TaranisX9d()
    
    async def send_data(self, hub):
        lastout = ""
        wvars = ['coll', 'roll', 'pitch', 'yaw', 'glyph']
        wirep = JoyProtocol(wvars, 2, None, sys.stdin)
        logging.info("starting main loop")
        while 1:
        
            #report = self.gamepad.report()
            cdict = {'roll': 0, 'pitch': 0, 'yaw': 0, 'coll': 0, 'glyph': 255} # keepawake
            try:
                cdict = self.fromq.get_nowait()
                #report["framenum"] = framenum
                #print(f"got frame framenum")
            except Exception as e:
                pass
            #print(report)  
            output = wirep.encode(cdict)

            if (output != lastout) or (((time.time()-self.last_sent)*1000) > 16):
                lastout = output
                logging.debug(output)
                #print(f"cdict before send: {cdict}")        
                #print(f"output: {output}")
                try:
                    await hub.write(bytearray(output, 'ascii'))
                    self.last_sent = time.time()
                except bleak.exc.BleakError as e:
                    logging.debug(f"BLE communication error: {e}")
                except Exception as e:
                    logging.error(f"Other error in hub.write(): {e}")
                    #sys.exit(0)
    
    async def go(self, logbasename, brickaddress, pyprog):
        hub = LoggingQueuedBricksHub(logbasename, self.toq)
        address = await find_device(brickaddress)
        await hub.connect(address)   

        try:
            await hub.run(pyprog, wait=False)
            await asyncio.gather(self.send_data(hub))

        except Exception as e:
            logging.error("script gather failed:  ", e)

        await hub.disconnect()
    
    def engage(self, logbasename, brickaddress, pyprog):
        asyncio.run(self.go(logbasename, brickaddress, pyprog))

def slurnk(fromq, toq):
    bubblebase = BaseStation(fromq, toq)
    bubblebase.engage('stewbase', 'jawaspike', 'stuart.py') # make it so
    #bubblebase.engage("bubble", "bubble", "bubble.py") # make it so on a dummy machine
      
               
if __name__ == '__main__':
    mp.set_start_method('spawn')
    cvq = mp.Queue()
    brickq = mp.Queue()
    #q = mp.Queue()
    p = mp.Process(target=load_img, args=(brickq, cvq))
    p.start()
    p2 = mp.Process(target=slurnk, args=(cvq, brickq))
    p2.start()
    #print(q.get())
    p.join()
    p2.join()