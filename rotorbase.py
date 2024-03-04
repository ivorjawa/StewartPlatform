#!/usr/bin/env python 

from pybricksdev.connections.pybricks import PybricksHub
from pybricksdev.ble import find_device, nus
from asyncio import gather, sleep, run

import sys
import datetime, time
import logging

import controllers as ctrl

logging.basicConfig(level=logging.INFO)

decode_report = ctrl.decode_taranis
gamepad = ctrl.open_taranis()

flogfile = None    
class  MybricksHub(PybricksHub):
    def _line_handler(self, line: bytes) -> None:
        global flogfile
        try:
            l = line.decode()
            logging.info(f"MybricksHub got:  {l}")
            
            if l == "<report>":
                fn = "rotorbase%s.csv" % str(datetime.datetime.now().isoformat())
                flogfile = open(fn, "w")
                logging.info("logging to ", fn)
                return
            elif l == "</report>":
                if(flogfile):
                    flogfile.close()
                    flogfile = None
                    logging.info("done logging")
            elif l == "goodbye.":
                sys.exit(1)            
            if(flogfile):
                print(l, file=flogfile)
        except Exception as e:
            logging.error(f"_line_handler error: {e}")
        
last_sent = time.time()
async def send_data(hub): 
    lastout = ""

    logging.info("starting main loop")
    while 1:
        
        report = gamepad.read(64)
        if report:            
            rd = decode_report(report)

            coll = rd['coll'] 
            roll = rd['roll']          
            pitch = rd['pitch']
            yaw = rd['yaw']
            glyph = rd['glyph']
            
            output = ">c%03xr%03xp%03xy%03xg%03x<" % (coll, roll, pitch, yaw, glyph)
            if (output != lastout) or (((time.time()-last_sent)*1000) > 16):
                lastout = output
                logging.info(output)
                try:
                    await hub.write(bytearray(output, 'ascii'))
                    last_sent = time.time()
                except Exception as e:
                    print(f"e class{e.__class__}")
                    logging.warning(f"write_gatt_char failed: {e}")
                    sys.exit(0)
                    #print("write_gatt_char failed:  ", e)
    
async def go():
    hub = MybricksHub()
    address = await find_device('rotor')
    await hub.connect(address)   


    try:
        await hub.run('rotor.py', wait=False)
        await gather(send_data(hub))

    except Exception as e:
        logging.error("script gather failed:  ", e)

    await hub.disconnect()

run(go())