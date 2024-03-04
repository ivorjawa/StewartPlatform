#!/usr/bin/env python 

import sys
import datetime, time
import logging, logging.handlers
from asyncio import gather, sleep, run

import bleak

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus

import controllers as ctrl

logging.basicConfig(level=logging.INFO, format='%(message)s [%(levelname)s:%(name)s]')

gamepad = ctrl.TaranisX9d()

class  LoggingBricksHub(PybricksHub):
    """
        A PybricksHub that can act on things received by _line_handler.
        <report> (lines) </report> will create a log file from 
        logfilename, ideally comma-separated values.
        <goodbye/> is the disconnect single from the hub.
    """
    def __init__(self, logfilename):
        super().__init__()
        self.csvfile = None
        self.csv_stemname = logfilename
        
    def _line_handler(self, line: bytes) -> None:
        try:
            l = line.decode()
            logging.info(f"Hub Sent:  {l}")
            
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
            if(self.csvfile):
                print(l, file=self.csvfile)
        except Exception as e:
            logging.error(f"_line_handler error: {e}")
        
last_sent = time.time()
async def send_data(hub):
    global last_sent 
    lastout = ""

    logging.info("starting main loop")
    while 1:
        
        report = gamepad.report()
        if report:            
            coll = report['coll'] 
            roll = report['roll']          
            pitch = report['pitch']
            yaw = report['yaw']
            glyph = report['glyph']
            
            output = ">c%03xr%03xp%03xy%03xg%03x<" % (coll, roll, pitch, yaw, glyph)
            if (output != lastout) or (((time.time()-last_sent)*1000) > 16):
                lastout = output
                logging.info(output)
                try:
                    await hub.write(bytearray(output, 'ascii'))
                    last_sent = time.time()
                except bleak.exc.BleakError as e:
                    logging.debug(f"BLE communication error: {e}")
                except Exception as e:
                    logging.error(f"Other error in hub.write(): {e}")
                    #sys.exit(0)
    
async def go():
    hub = LoggingBricksHub('rotorbase')
    address = await find_device('rotor')
    await hub.connect(address)   

    try:
        await hub.run('rotor.py', wait=False)
        await gather(send_data(hub))

    except Exception as e:
        logging.error("script gather failed:  ", e)

    await hub.disconnect()

run(go())