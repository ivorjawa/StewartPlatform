#!/usr/bin/env python

import math as m
import linear as lin
import slerp

def fqe(rotor):
    (r, p, y) = slerp.to_euler(rotor) # (roll, pitch, yaw)
    r = m.degrees(r)
    p = m.degrees(p)
    y = m.degrees(y)
    return f"r: {r: 6.3f}, p: {p: 6.3f}, y: {y: 6.3f}"

from StewartPlatform import StewartPlatform
Stew = StewartPlatform(40, 120, 120, 240, 308) #inner, outer radius, footprint, min, max cylinder extension

cube = lin.Matrix([
    [0, 0, 0], 
    #[0, 1, 0], 
    #[1, 1, 0], 
    #[1, 0, 0],      
    #[0, 0, 1], 
    [0, 1, 1], 
    #[1, 1, 1], 
    #[1, 0, 1],     
])

rcube = lin.Matrix([
    [0, 15, 15], 
    #[0, 0, 0],
    [0, -15, -15],
    #[0, 0, 0],
    
    #[0, 15, 15], 
    
])
    
#rcube = cube * 15
scube = cube - lin.vector(.5, .5, 0)
scube = scube * .75
scube = scube + lin.vector(0, 0, .5)

#print(f"rcube: {rcube}")
print(f"scube: {scube}")

rcube1 = rcube[0]
rcube2 = rcube[1]
    
for i in range(11):
    framedex = i/10.0
    #euler_quat(heading, pitch, roll) yaw, pitch, roll
    rotor1 = slerp.euler_quat(m.radians(rcube1[0]), m.radians(rcube1[1]), m.radians(rcube1[2]))
    rotor2 = slerp.euler_quat(m.radians(rcube2[0]), m.radians(rcube2[1]), m.radians(rcube2[2]))
    rotor = slerp.slerp(rotor1, rotor2, framedex)
    print(f"i: {framedex:3.2f} r1: {slerp.fq(rotor1)} {fqe(rotor1)}")
    print(f"i: {framedex:3.2f} r2: {slerp.fq(rotor2)} {fqe(rotor2)}")
    print(f"i: {framedex:3.2f}  r: {slerp.fq(rotor)} {fqe(rotor)}\n")
    #print(f"\n\nr")

#print(f"cube shape: {cube.shape}")


#for i in range(cube.shape[0]):
#    colspokes = Stew.solve6(*rcube[i], *scube[i])
#    if len(colspokes) == 4:
#        coll, sa, sb, sc = colspokes
#        print(f"i: {i}, sa: {sa}, sb: {sb}, sc: {sc}, coll: {coll}")
#    else:
#        print(f"i: {i}: bad")

