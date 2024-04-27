#!/usr/bin/env python 

import sys
import datetime, time
import logging, logging.handlers
import asyncio
#from asyncio import gather, sleep, run

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus
import bleak

import controllers as ctrl
from joycode import JoyProtocol

logging.basicConfig(level=logging.INFO, format='%(message)s [%(levelname)s:%(name)s]')

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
        
class BaseStation(object):
    def __init__(self):
        self.last_sent = time.time()
    
    async def send_data(self, hub):
        self.gamepad = ctrl.TaranisX9d() # pyglet doesn't work under async?  Try running in own process like qtest
        lastout = ""
        wvars = ['coll', 'roll', 'pitch', 'yaw', 'glyph']
        wirep = JoyProtocol(wvars, 2, None, sys.stdin)
        logging.info("starting main loop")
        while 1:
        
            report = self.gamepad.report()
            #report = None
            if report:  
                #print(report)          
                output = wirep.encode(report)
                if (output != lastout) or (((time.time()-self.last_sent)*1000) > 16):
                    lastout = output
                    logging.debug(output)
                    try:
                        await hub.write(bytearray(output, 'ascii'))
                        self.last_sent = time.time()
                    except bleak.exc.BleakError as e:
                        logging.debug(f"BLE communication error: {e}")
                    except Exception as e:
                        logging.error(f"Other error in hub.write(): {e}")
                        #sys.exit(0)
    
    async def go(self, hubname, brickaddress, pyprog):
        hub = LoggingBricksHub(hubname)
        address = await find_device(brickaddress)
        await hub.connect(address)   

        try:
            await hub.run(pyprog, wait=False)
            await asyncio.gather(self.send_data(hub))

        except Exception as e:
            logging.error("script gather failed:  ", e)

        await hub.disconnect()
    
    def engage(self, hubname, brickaddress, pyprog):
        asyncio.run(self.go(hubname, brickaddress, pyprog))
        
if __name__ == "__main__":
    rotor = BaseStation()
    #rotor = BaseStation('rotorbase', 'rotor', 'rotor.py')
    #rotor = BaseStation('stewbase', 'jawaspike', 'stuart.py')
    rotor.engage('stewbase', 'jawaspike', 'stuart.py') # make it so
    