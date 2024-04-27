#!/usr/bin/env python

import linear as lin

try:
    import numpy as np
    import quaternion
    import math as m
    is_pybrics = False
    make_quat = np.quaternion
    qnorm = lambda q: q.normalized()
    q2vec = lambda q: q.components
    clamp = np.clip
    np.set_printoptions(suppress=True, formatter={'float_kind':'{:3.2f}'.format})  
except Exception as e:
    import umath as m
    import quat
    is_pybrics = True
    make_quat = quat.Quaternion
    qnorm = lambda q: q.normalise()
    q2vec = lambda q: lin.vec4(*q)
    def clamp(n, min, max): 
        if n < min: 
            return min
        elif n > max: 
            return max
        else: 
            return n
            

# https://stackoverflow.com/questions/44706591/how-to-test-quaternion-slerp


        
def slerp(quat1, quat2, t):
    """Spherically interpolates between quat1 and quat2 by t.
    The parameter t is clamped to the range [0, 1]
    """

    # https://en.wikipedia.org/wiki/Slerp

    #v0 = normalise(quat1)
    #v1 = normalise(quat2)
    # in pybricks quat, .normalise
    v0 = qnorm(quat1)
    v1 = qnorm(quat2)

    #dot = vector4.dot(v0, v1)
    # in np.components, in pyrbricks quat, lin.vector(quat[0:])
    dot = lin.dot(q2vec(v0), q2vec(v1))

    # TODO: fixlater
    # If the inputs are too close for comfort,
    # linearly interpolate and normalize the result.
    # if abs(dot) > 0.9995:
    #     pass
    # https://en.wikipedia.org/wiki/Spherical_law_of_cosines
    # https://www.movable-type.co.uk/scripts/latlong.html

    # If the dot product is negative, the quaternions
    # have opposite handed-ness and slerp won't take
    # the shorter path. Fix by reversing one quaternion.
    if dot < 0.0:
        v1 = -v1
        dot = -dot

    # clamp
    dot = clamp(dot, -1.0, 1.0)
    theta = m.acos(dot) * t

    v2 = v1 - v0 * dot
    res = v0 * m.cos(theta) + v2 * m.sin(theta)
    return res
    
    
"""
>>> import numpy as np
>>> import quaternion
>>> q1 = np.quaternion(1, 0, 0, 0)
>>> q2 = np.quaternion(0, 1, 0, 0)
>>> quaternion.slerp_evaluate(q1, q2, .0)
quaternion(1, 0, 0, 0)
>>> quaternion.slerp_evaluate(q1, q2, .2)
quaternion(0.951056516295154, 0.309016994374947, 0, 0)
>>> quaternion.slerp_evaluate(q1, q2, .4)
quaternion(0.809016994374947, 0.587785252292473, 0, 0)
>>> quaternion.slerp_evaluate(q1, q2, .6)
quaternion(0.587785252292473, 0.809016994374947, 0, 0)
>>> quaternion.slerp_evaluate(q1, q2, .8)
quaternion(0.309016994374947, 0.951056516295154, 0, 0)
>>> quaternion.slerp_evaluate(q1, q2, 1.)
quaternion(6.12323399573677e-17, 1, 0, 0)
"""

def ident():
    print(f"System name: {hub.system.name()}")
    print(f"Version: {usys.version}")      
    print(f"Implementation: {usys.implementation}")      
    print(f"Version Info: {usys.version_info}")
    print(f"Battery Voltage: {hub.battery.voltage()}mv") 

# pybricksdev run ble -n bubble slerp.py    
if __name__ == "__main__":
    q1 = make_quat(1, 0, 0, 0)
    q2 = make_quat(0, 1, 0, 0)
    for i in range(11):
        print(f"{i}: {slerp(q1, q2, i/10)}")
    #a = lin.vector(1, 2, 3)
    #b = lin.Matrix([[1, 2, 3, 4]]).T
    #b = lin.vec4(1, 2, 3, 4)
    #print(f"a: {a}, a.shape: {a.shape}, b: {b}, b.shape: {b.shape}")
    #ident()
    