#!/usr/bin/env python

import sys
import datetime, time
import logging, logging.handlers
import multiprocessing as mp
import queue
import asyncio
import code, readline

#from asyncio import gather, sleep, run

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus
import bleak

from rich import print as rprint

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
        print(f"Created LoggingBricksHub")
        self.csv_stemname = logfilename
        
    def _line_handler(self, line: bytes) -> None:
        try:
            l = line.decode()
            #logging.info(f"Hub Sent:  {l}")
            rprint(f"[#000000 on #00FF00]Hub Sent:  {l}")
        except Exception as e:
            logging.error(f"_line_handler error: {e}")
            
class BaseStation(object):
    def __init__(self, fromq):
        self.fromq = fromq
            
    async def send_data(self, hub):
        while 1:
            
            output = None
            try:
                output = self.fromq.get_nowait()
            except Exception as e:
                pass

            if output is not None:
                try:
                    #print(f"sending to brick: {output}")
                    await hub.write(bytearray(output, 'ascii'))
                except bleak.exc.BleakError as e:
                    logging.debug(f"BLE communication error: {e}")
                except Exception as e:
                    logging.error(f"Other error in hub.write(): {e}")
                    #sys.exit(0)
    
    async def go(self, logbasename, brickaddress, pyprog):
        hub = LoggingBricksHub(logbasename)
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

def robotlink(fromq):
    robotbase = BaseStation(fromq)
    robotbase.engage('rsserver', 'jawaspike', 'rsserver.py') # make it so
    #robotbase.engage("bubble", "bubble", "bubble.py") # make it so on a dummy machine
    
# https://bernsteinbear.com/blog/simple-python-repl/
# https://docs.pybricks.com/en/stable/micropython/builtins.html
# TODO: make shell that can talk to brick by sending source
class RoboShell(code.InteractiveConsole):
    def __init__(self, roboq):
        self.roboq = roboq
        sys_locals = {
            'roboq': roboq,
            #'wobbler': wobbler,
            #'StewartPlatform': StewartPlatform
        }
        super().__init__(locals=sys_locals)
        #self.wobbler = wobbler
    def runsource(self, source, filename="<input>", symbol="single"):
        # TODO: Integrate your compiler/interpreter
        #if not source.endswith(";"):
        #    return True
        #print("source:", source)
        self.roboq.put(source+'\n')
        #print("sent to roboq")
        #return super().runsource(source, filename, symbol)
    def loop(self):
        self.interact(banner="STU>>>", exitmsg="DAVENO!")
        
def gorsh():
    mp.set_start_method('spawn')
    
    roboq = mp.Queue() # input from shell task to robotlink task 
    p1 = mp.Process(target=robotlink, args=(roboq,)) # add jsk
    p1.start()
    #p3 = mp.Process(target=jslink, args=(jsq, brickq)) # jslink is another input to tracker
    #p3.start()
    
    repl = RoboShell(roboq)
    repl.loop()
    
    time.sleep(1)
    for p in [p1]:
        try:
            p.terminate()
            p.join()
        except Exception as e:
            print(f"Join unhappy: {e}")

if __name__ == '__main__':
    try:
        gorsh()
    except Exception as e:
        rprint(f"[#FFFFFF on #00FF00]gorsh: {e}")
        raise
    rprint("[#FFFFFF on #FF0000]hAvE A niCE dAy.")        