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
    # A rotation quaternion is a unit quaternion i.e. magnitude == 1
    def rotator_quat(theta=0, x=0, y=0, z=0):
        print(f"rotator theta: {theta} @ ({x}, {y}, {z})")
        s = m.sin(theta/2)
        print(f"s: {s}")
        mm = m.sqrt(x*x + y*y + z*z)  # Convert to unit vector
        print(f"mm: {mm}")
        if mm > 0:
            return np.quaternion(m.cos(theta/2), s*x/mm, s*y/mm, s*z/mm)
        else:
            return np.quaternion(1, 0, 0, 0)  # Identity quaternion

    def euler_quat(heading, pitch, roll):
        cy = m.cos(heading * 0.5);
        sy = m.sin(heading * 0.5);
        cp = m.cos(pitch * 0.5);
        sp = m.sin(pitch * 0.5);
        cr = m.cos(roll * 0.5);
        sr = m.sin(roll * 0.5);

        w = cr * cp * cy + sr * sp * sy;
        x = sr * cp * cy - cr * sp * sy;
        y = cr * sp * cy + sr * cp * sy;
        z = cr * cp * sy - sr * sp * cy;
        return np.quaternion(w, x, y, z)  # Tait-Bryan angles but z == towards sky
    def fq(q):
        return f"w: {q.w: 6.3f}, x: {q.x: 6.3f}, y: {q.y: 6.3f}, z: {q.z: 6.3f}" 
    def qrotate(q, p):
        return (q * p * q.conjugate())
        
    
except Exception as e:
    import umath as m
    import quat
    is_pybrics = True
    make_quat = quat.Quaternion
    rotator_quat = quat.Rotator
    euler_quat = quat.Euler
    qnorm = lambda q: q.normalise()
    q2vec = lambda q: lin.vec4(*q)
    def fq(q):  
        return f"w: {q.d[0]: 6.3f}, x: {q.d[1]: 6.3f}, y: {q.d[2]: 6.3f}, z: {q.d[3]: 6.3f}"
    def qrotate(q, p):
        return p @ q # matmul
    def clamp(n, min, max): 
        if n < min: 
            return min
        elif n > max: 
            return max
        else: 
            return n
            

# https://stackoverflow.com/questions/44706591/how-to-test-quaternion-slerp

def quat2vec3(q):
    return lin.vector(q.x, q.y, q.z)

def point(x, y, z):
    return make_quat(0, x, y, z)
 
def slerp(one, two, t):
    #https://splines.readthedocs.io/en/latest/rotation/slerp.html
    """Spherical Linear intERPolation."""
    return (two * one.inverse())**t * one
           

#https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles    
def to_euler( q):
    try:
        w = q.w
        x = q.x
        y = q.y
        z = q.z
    except Exception as e:
        w = q.d[0]
        x = q.d[1]
        y = q.d[2]
        z = q.d[3]
    # roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = m.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = m.sqrt(1 + 2 * (w * y - x * z))
    cosp = m.sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * m.atan2(sinp, cosp) - (m.pi / 2);

    # yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = m.atan2(siny_cosp, cosy_cosp)

    return (roll, pitch, yaw)
    
def to_euler_d(q):
    rpy = to_euler(q)
    return [m.degrees(a) for a in rpy]

    
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
    #r = rotator_quat(m.pi/2, 0, 0, 1)
    #print(f"r: {r}")
    #p = point(1, 0, 0)
    #print(f"p: {p}")
    #p2 = qrotate(r, p)
    #x = p2.x
    #y = p2.y
    #z = p2.z
    #print(f"p2: ({x:3.3f}, {y:3.3f}, {z:3.3f})")
    
    q1 = euler_quat(m.radians(0), m.radians(15), m.radians(15))
    q2 = euler_quat(m.radians(0), m.radians(-15), m.radians(-15))
    #q1 = make_quat(0, 1, 0, 0)
    #print(f"q1: {q1} q1.d: {q1.d.T}")
    #2 = make_quat(0, 0, 1, 0)
    #print(f"q2: {q2} q2.d: {q2.d.T}")
    
    #q1 = make_quat(0, .13, .13, -0.02)
    #q2 = make_quat(0, -.13, -.13, -0.02)
    print(f"q1: {fq(q1)} euler: {to_euler_d(q1)}")
    for i in range(11):
        print(f"{i}: {fq(slerp(q1, q2, i/10))}")
    print(f"q2: {fq(q2)} euler: {to_euler_d(q2)}")
    
    #a = lin.vector(1, 2, 3)
    #b = lin.Matrix([[1, 2, 3, 4]]).T
    #b = lin.vec4(1, 2, 3, 4)
    #print(f"a: {a}, a.shape: {a.shape}, b: {b}, b.shape: {b.shape}")
    #ident()
    