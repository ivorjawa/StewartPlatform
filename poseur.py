#!/usr/bin/env python

import sys
import os
import datetime
import logging, logging.handlers
import multiprocessing as mp
import queue
import asyncio
import time
import random
import pickle
import math as m

import numpy as np
import cv2

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus
import bleak

from rich import print as rprint

import PID
PID.setmillis(lambda: time.time()*1000) # must be set before creating any PID objects
            

from joycode import JoyProtocol
from statemachine import StateMachine
import StewartPlatform
from pose_est import Recognizer

#"maps -1.0..1.0 to 0..255"
one28 = lambda x: int(((x+1)/2)*255)
     
class TrackerSM(StateMachine):
    def __init__(self, fromq, toq):
        super().__init__()
        #self.build('cstates', ['scan', 'wait_start', 'wait_move'])
        self.build('cstates', ['wait_start', 'scan', 'wait_move'])
        
        self.fromq = fromq
        self.toq = toq

        self.start_time = 0
        self.end_time = 0
        self.woke = False
        self.moving = False
        
        self.width = 640
        self.height = 480

        self.rec = Recognizer(self.height, self.width)
        
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cam.set(cv2.CAP_PROP_FPS, 30)

        # initialize camera, set infinite focus, turn off auto focus, set exposure time
        os.system("./uvc-util -I 0 -s auto-focus=0")
        os.system("./uvc-util -I 0 -s focus-abs=0")
        os.system("./uvc-util -I 0 -s auto-exposure-mode=1")
        os.system("./uvc-util -I 0 -s exposure-time-abs=150")
        
        # rotational, controlling degrees
        rKp = 0.025
        rKi = 0.015
        rKd = 0
        # translational, controlling mm
        tKp = 0.01
        tKi = 0.002
        tKd = 0
        
        # roll and pitch, input pixel offset, controls limited degrees
        bKp = 0.02
        bKi = 0.001
        bKd = 0
        
        self.x_pid = PID.PID(0, tKp, tKi, tKd, PID.PID.P_ON_E, PID.PID.DIRECT)
        self.y_pid = PID.PID(0, tKp, tKi, tKd, PID.PID.P_ON_E, PID.PID.DIRECT)
        self.heading_pid = PID.PID(0, rKp, rKi, rKd, PID.PID.P_ON_E, PID.PID.DIRECT)
        self.roll_pid = PID.PID(0, bKp, bKi, bKd, PID.PID.P_ON_E, PID.PID.DIRECT)
        self.pitch_pid = PID.PID(0, bKp, bKi, bKd, PID.PID.P_ON_E, PID.PID.DIRECT)
        self.pids = [
            self.x_pid, 
            self.y_pid, 
            self.heading_pid, 
            self.roll_pid,
            self.pitch_pid,
        ]
        for pid in self.pids:
            pid.mySetpoint = 0  
            pid.SetOutputLimits(-1.0, 1.0)
            pid.SetSampleTime(33.3) # ms, 30 FPS
            pid.SetMode(pid.AUTOMATIC)
        
    def wait_start(self):
        #print("asleep")
        if self.woke:
            self.state = self.states.scan
            print("scanning")
 
    def scan(self):
        ret, img = self.cam.read()
        if ret:
            try:
                pose_info = self.rec.recognize(img)

                #cv2.line(self.rec.output, (int(0),int(self.height/2)), (int(self.width), int(self.height/2)), (0, 0, 255), 1)
                #cv2.line(self.rec.output, (int(self.width/2),int(0)), (int(self.width/2), int(self.height)), (0, 0, 255), 1)

                canvas = np.zeros((self.height, self.width*2, 3), np.uint8)
                canvas[0:480, 0:640] = self.rec.output
                canvas[0:480, 640:1280] = self.rec.red
                cv2.imshow('Estimated Pose', canvas)
                
                xerr = self.rec.dxp
                yerr = self.rec.dyp
                headerr = pose_info.heading
                
                #print(f"heading_pid auto state: {self.heading_pid.inAuto}")
                xpe = self.x_pid.Compute(xerr)
                ype = self.y_pid.Compute(yerr)
                hpe = self.heading_pid.Compute(headerr)
                #print(f"pid results: {[xpe, ype, hpe]}")
                
                
                #angle = time.time() % m.tau # circle in tau seconds
                #pitch = m.sin(angle)
                #roll = m.cos(angle)
                pitch = 0
                roll = 0
                
                if self.rec.have_ball:
                    # FIXME rec.detect_ball is what triggers logging
                    # rec.start_logging and stop_logging control it
                    # need to add log type for collected PID and error data,
                    # possibly need to inherit Recognizer and override rec.log()
                    # ... or, something.  ugly.
                    
                    ballrad = m.sqrt(self.rec.ball_dxp**2 + self.rec.ball_dyp**2);
                    ballang = m.atan2(self.rec.ball_dyp, self.rec.ball_dxp)
                    ballangd = m.degrees(ballang)
                    #ballradscale = (ballrad/130)*.65 # used without PI
                    ballradscale = ballrad
                    print(f"ball found rad: {ballrad:5.2f} angle: {ballangd:5.2f} output: {ballradscale:5.2f}")
                    #pitch = m.sin(ballang)*ballradscale
                    #roll = m.cos(ballang)*ballradscale
                    ppe = pitcherr = m.sin(ballang)*ballradscale
                    rpe = rollerr = m.cos(ballang)*ballradscale
                    self.pitch_pid.Compute(pitcherr)
                    self.roll_pid.Compute(rollerr)
                    rprint(f"[white on blue]pid results: {np.array([rpe, ppe])}")
                    
                print(f"insert PID magic here xerr: {xerr}=>{self.x_pid.myOutput:5.3f} yerr: {yerr}=>{self.y_pid.myOutput:5.3f} headerr: {headerr:5.1f}=>{self.heading_pid.myOutput:5.3f}")
                # initial strategy: want to make dxp and dyp and heading 0 with z at 50%
                modeglyph = StewartPlatform.cSC # select 6-DOF absolute / precision mode
                # in precision mode, 
                # z: coll
                # x: LS
                # y: RS
                # yaw: S1
                cdict = {
                    #'roll': one28(0), 
                    #'roll': one28(roll),
                    'roll': one28(self.roll_pid.myOutput),
                    #'pitch': one28(0),  
                    #'pitch': one28(pitch),
                    'pitch': one28(self.pitch_pid.myOutput),
                    'S1': one28(self.heading_pid.myOutput), 
                    #'S1': one28(0), 
                    'coll': one28(0), # middle
                    'LS': one28(self.x_pid.myOutput),
                    'RS': one28(self.y_pid.myOutput),                    
                    #'LS': one28(0),
                    #'RS': one28(0),
                    'glyph': modeglyph
                }
                #self.state = self.states.wait_move
                self.toq.put_nowait(cdict)
                self.moving = True
                self.start_time = time.time()
                
                key = cv2.waitKey(1)
                if key == 27:
                    # FIXME make this send other thread / robot termination message
                    self.toq.put_nowait({'glyph':StewartPlatform.cSB}) # kill packet
                    time.sleep(1)
                    sys.exit(0)
                elif key == ord('l'):
                    self.rec.start_logging()
                elif key == ord('s'):
                    self.rec.stop_logging()
                    
                print("sent command")
            except Exception as e:
                print(f"recognizer failed: {e}")
                 
        
    def wait_move(self):
        now = time.time()
        if self.moving == True:
            if (now - self.start_time > 5):
                print("Timeout waiting for robot to move, kicking")
                self.state = self.states.scan
        elif self.moving == False:
            self.end_time = now
            self.state = self.states.scan
            print(f"got movement, took {self.end_time-self.start_time:3.3f}s")
            
    def loop(self):
        while 1:
            self.tick()
            try:
                token = self.fromq.get_nowait()
                print(f"got token {token}")
                if token == "<awake/>":
                    self.woke = True
                elif token == "<taskdone/>":
                    self.moving = False
                elif token == "<goodbye/>":
                    print("robot requested exit")
                    sys.exit(1)
                else:
                    print(f"got unknown token: {token}")
            except queue.Empty as e:
                pass
                #print(f"Tracker tick exception: {e}")
                #raise
        cv2.destroyAllWindows()
        return        

def tracker(fromq, toq):
    tsm = TrackerSM(fromq, toq)
    tsm.loop()

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
            logging.warning(f"Hub Sent:  {l}")
            
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
                self.toq.put_nowait(l)
                time.sleep(1)
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
        #wvars = ['coll', 'roll', 'pitch', 'yaw', 'glyph']
        from stewart_wvars import wvars
        wirep = JoyProtocol(wvars, 2, None, sys.stdin)
        logging.info("starting main loop")
        while 1:
        
            #report = self.gamepad.report()
            #for x in [24, 40, 72, 68, 255]: print(f"{x:#010b}") 
            """
            >>> for x in [24, 40, 72, 68, 128, 129, 130, 131, 255]: print(f"{x:#010b}")
            ... 
            0b00011000
            0b00101000
            0b01001000
            0b01000100
            0b10000000
            0b10000001
            0b10000010
            0b10000011
            0b11111111
            
            if we treat bit 7 as a command flag, we've got 2 bits for modes
            (3 if we consider keepawake a mode, 4 if keepawake is handled separately)
            Glyph still has to die.
            129 will signify "slerp to next pose"
            130 will be "twitch (no interpolation) to next pose" 
            Well, eventually.  we just send cSC now for precision 6DOF
            """
            cdict = {'roll': 0, 'pitch': 0, 'yaw': 0, 'coll': 0, 'glyph': 1} #FIXME glyph was 255 keepawake, now 1 null
            # get command from opencv recognizer process
            track_packet = False
            try:
                cdict = self.fromq.get_nowait()
                #print(f"got cdict from tracker: {cdict}")
                track_packet = True
            except Exception as e:
                pass
            #print(report)  
            output = wirep.encode(cdict)
            #if track_packet:
            #    print(f"command output: {output}")
            
            # may need this throttling, not sure yet, recognizer locked to 30 hz
            #dtms = (time.time() - self.last_sent) * 1000
            #if (dtms > 16): # only send 60 fps
            
            dtms = (time.time() - self.last_sent) * 1000
            if (dtms > 16) or track_packet: # only send keepalive 60 fps
            #if (output != lastout) or (((time.time()-self.last_sent)*1000) > 16):
                #lastout = output
                logging.debug(output)
                #print(f"cdict before send: {cdict}")        
                #print(f"output: {output}")
                try:
                    #if track_packet:
                    #    print("about to send track packet")
                    await hub.write(bytearray(output, 'ascii'))
                    self.last_sent = time.time()
                    #if track_packet:
                    #    print("sent track packet")
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

def robotlink(fromq, toq):
    robotbase = BaseStation(fromq, toq)
    robotbase.engage('stewbase', 'jawaspike', 'stuart.py') # make it so
    #robotbase.engage("bubble", "bubble", "bubble.py") # make it so on a dummy machine
      
               
if __name__ == '__main__':
    mp.set_start_method('spawn')
    
    cvq = mp.Queue()
    brickq = mp.Queue()
    
    p = mp.Process(target=tracker, args=(brickq, cvq))
    p.start()
    p2 = mp.Process(target=robotlink, args=(cvq, brickq))
    p2.start()
    
    p.join()
    p2.join()