try:
    import math as m
except ImportError:
    import umath as m

from StewartPlatform import StewartPlatform
import slerp
import linear as lin
#import dataforslerp as slerpdata

scube1 = [
    [0, 0, .5],
    [0, 0, .5],
    [0, 0, .5],
    [0, 0, .5],    
    [0, 0, .5],
    [0, 0, .5],
    [0, 0, .5],
    [0, 0, .5],
]

scube12 = [
   [0, 0, .5],
   [0, 0, .5], 
]
# yaw pitch roll
rcube1 = [
    [0, 0, 0],
    [0, 10, 0],
    [0, 0, 0], 
    [0, -10, 0],    
    [0, 0, 0],
    [0, 0, 10],
    [0, 0, 0], 
    [0, 0, -10],
]

rcube12 = [
    [0, 15, 15], 
    [0, -15, -15],
]
scube = [lin.vector(*v) for v in scube1]
rcube = [lin.vector(*v) for v in rcube1]


Stew = StewartPlatform(57, 98, 120, 250, 314) #inner, outer radius, footprint, min, max cylinder extension
def test1():
    roll = 0
    pitch = 0
    yaw = 0
    x = 1
    y = 1
    z = .1
    print(f"RPY: ({roll: 3.1f}, {pitch: 3.1f}, {yaw: 3.1f}) XYZ: ({x: 3.1f}, {y: 3.1f}, {z: 3.1f})")
    try:
        coll_v, sa, sb, sc = Stew.solve6(roll, pitch, yaw, x, y, z)
        print(f"COLL_v: ({coll_v[0]: 3.1f}, {coll_v[1]: 3.1f}, {coll_v[2]: 3.1f})")
        for i, c in enumerate(Stew.cyls):
            print(f"Cyl {i}: {c: 3.1f}mm")
    except Exception as e:
        print(e)

def fv3(q):
    return f"({q[0]: 6.3f}, {q[1]: 6.3f}, {q[2]: 6.3f})"
    
def test2():
    #scube = slerpdata.scube
    #rcube = slerpdata.rcube
    
    for cubedex in range(len(scube)):
        cubedex1 = cubedex
        cubedex2 = (cubedex+1) % len(scube)
        
        rcube1 = rcube[cubedex1]
        rcube2 = rcube[cubedex2]
    
        #euler_quat(heading, pitch, roll) yaw, pitch, roll
        rotor1 = slerp.euler_quat(m.radians(rcube1[0]), m.radians(rcube1[1]), m.radians(rcube1[2]))
        rotor2 = slerp.euler_quat(m.radians(rcube2[0]), m.radians(rcube2[1]), m.radians(rcube2[2]))

        scube1 = scube[cubedex1]
        scube2 = scube[cubedex2]
        cubefract = (scube2-scube1)
        print(f"\ncubedex: {cubedex}")
        print(f"scube1: {fv3(scube1)}, rcube1: {fv3(rcube1)}")
        print(f"scube2: {fv3(scube2)}, rcube2: {fv3(rcube2)}")
        #print(f"cubefract: {cubefract}")
        print(f"rotor1: {slerp.fq(rotor1)}")
        print(f"rotor2: {slerp.fq(rotor2)}")
        for framedex in range(11):
            rotor = slerp.slerp(rotor1, rotor2, framedex/10.0)
            (r, p, y) = slerp.to_euler(rotor) # (roll, pitch, yaw)
            roll = m.degrees(r)
            pitch = m.degrees(p)
            yaw = m.degrees(y)
            print(f"frame: {framedex:2} rotor: {slerp.fq(rotor)} r: {roll:6.2f} p: {pitch:6.2f} y: {yaw:6.2f}")

# pybricksdev run ble -n jawaspike gatestew.py    
            
if __name__ == "__main__":
    #test1()
    test2()