import usys
import umath as m
from usys import stdin
from uselect import poll

from pybricks.hubs import TechnicHub
from pybricks.pupdevices import Motor
from pybricks.parameters import Port
from pybricks.tools import wait, StopWatch
from pybricks.tools import Matrix, vector, cross

hub = TechnicHub()

from linear import xaxis, yaxis, zaxis, rotate
import linear as lin

from joycode import JoyProtocol
#from swashplate import Swashplate

millis = StopWatch().time

# C is star, B is port, A is front, D is rotor
# max speed is about 1300

#rotor = Motor(Port.D)
#star = Motor(Port.C)
#port = Motor(Port.B)
#front = Motor(Port.A)


                       
def run_remote():
    poller = poll()
    # Register the standard input so we can read keyboard presses.
    poller.register(stdin)
    
    wvars = ['roll', 'pitch', 'yaw', 'coll', 'glyph']
    wirep = JoyProtocol(wvars, 2, poller, stdin)
    
    #jsman = JSMan()
    last_input = 0 # last input from base station
    
    #disk_def = 8 # degrees +-
    #coll_range = .15 # % +-
    
    #threshold = memory_calibrate() # max travel of cylinders, degrees
    # arm radius, min cylinder, max cylinder
    #rot = Rotor(60, 160, 200, threshold) # real robot
    
    identify()
    while True:
        if wirep.poll():
            try:    
                last_input = millis() 

                wirep.decode_wire()
                print(wirep.vals)
                print("<taskdone/>")
                    
            except Exception as e:
                print("failure in main loop:")
                print(e)
            
            #rot.actuate()


def identify():
    print(f"System name: {hub.system.name()}")
    print(f"Version: {usys.version}")      
    print(f"Implementation: {usys.implementation}")      
    print(f"Version Info: {usys.version_info}")
    print(f"Battery Voltage: {hub.battery.voltage()}mv") 
        
if __name__ == "__main__":
    # pybricksdev run ble -n bubble bubble.py
    #teststorage()
    #testwritestore()
    #run_remote()
    try:
        #pass # when running above tests
        run_remote() # full talks to remote run under ./rotorbase.py
        #full_calibrate() # should be done with rotors and scissor link detached
    except Exception as e:
        print("General failure: ", e)
    identify()     