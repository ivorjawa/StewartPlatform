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
    #def line_handler(self, line):
        global flogfile, readyq, readytriggered
        try:
            l = line.decode()
            print("MybricksHub got:  ", l)
            
            if l == "<report>":
                fn = "rotorbase%s.csv" % str(datetime.datetime.now().isoformat())
                flogfile = open(fn, "w")
                print("logging to ", fn)
                return
            elif l == "</report>":
                if(flogfile):
                    flogfile.close()
                    flogfile = None
                    print("done logging")
            elif l == "goodbye.":
                sys.exit(1)            
            if(flogfile):
                print(l, file=flogfile)
        except Exception as e:
            print("_line_handler error:")
            print(e)
        
last_sent = time.time()
async def send_data(hub):
    #setup()  
    #clock = pygame.time.Clock()  
    lastout = ""

    print("starting main loop")
    while 1:
        #await sleep(1)
        #polljoy()
        #clock.tick()
        report = gamepad.read(64)
        if report:
            # Taranis major axes 3 5 7 9 lx = 9, ly = 7, rx = 3, ry = 5
            # PS4 major axes lx = 1, ly = 2, rx = 3, ry = 4
            
            # PS4
            #crab = report[1] # crab / lx
            #spin = report[3] # spin / rx
            #fwdback = report[4] # fb / ry
            #glyph = report[5]
            
            # Taranis
            #crab = report[9] # crab / lx
            #spin = report[3] # spin / rx
            #fwdback = report[5] # fb / ry
            #glyph = 0 #report[5] # Need to configure switches on transmitter
            
            rd = decode_report(report)

            coll = rd['coll'] 
            roll = rd['roll']          
            pitch = rd['pitch']
            yaw = rd['yaw']
            glyph = rd['glyph']
            
            output = ">c%03xr%03xp%03xy%03xg%03x<" % (coll, roll, pitch, yaw, glyph)
            if (output != lastout) or (((time.time()-last_sent)*1000) > 16):
                lastout = output
                print(output)
                try:
                    #await hub.client.write_gatt_char(nus.NUS_RX_UUID, bytearray(b"HELLO!"), True)
                    #await hub.client.write_gatt_char(nus.NUS_RX_UUID, bytearray(output, 'ascii'), True)
                    await hub.write(bytearray(output, 'ascii'))
                    last_sent = time.time()
                except Exception as e:
                    print("write_gatt_char failed:  ", e)
                    #return
    
async def go():
    hub = MybricksHub()
    address = await find_device('rotor')
    await hub.connect(address)   


    try:
        await hub.run('rotor.py', wait=False)
        await gather(send_data(hub))

    except Exception as e:
        print("script gather failed:  ", e)

    await hub.disconnect()

run(go())