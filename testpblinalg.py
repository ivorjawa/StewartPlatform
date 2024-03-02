import usys
import umath as m

from pybricks.tools import Matrix, vector, cross

import unumpy as np

from pybricks.hubs import TechnicHub

hub = TechnicHub()

xaxis = vector(1, 0, 0)
yaxis = vector(0, 1, 0)
zaxis = vector(0, 0, 1)
def rmat(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    #axis = np.asarray(axis)
    #axis = axis / m.sqrt(np.dot(axis, axis))
    axis = np.linalg.norm(axis) # this is wrong, we want to normalize
    a = m.cos(theta / 2.0)
    b, c, d = -axis * m.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return np.array([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def rotate(axis, theta, point):
    rm = rmat(axis, theta)
    #print(f"rotation matrix axis: {axis} theta: {theta}\nrmat: {rm}")
    return rm * point # in pybricks you multiply instead of dot
    #return (np.dot(rm, point))
    
def identify():
    print(f"System name: {hub.system.name()}")
    print(f"Version: {usys.version}")      
    print(f"Implementation: {usys.implementation}")      
    print(f"Version Info: {usys.version_info}")
    print(f"Battery Voltage: {hub.battery.voltage()}mv") 

fm = lambda v: ''.join(str(v).split(chr(0x0a)))

def testlinalg():
    a = vector(2, 2, 2)
    #hd = lambda s: '.'.join([str(hex(ord(c))[2:]) for c in s])
    #sa = str(a)
    #print(f"sa: |{sa}|")
    #print(f"hd(sa): |{hd(sa)}|")
    #print(f"fm(a): {fm(a)}")
    
    b = vector(3, 3, 3)
    #print(f"\n\n{dir(b)}\n\n")
    print(f"b: {fm(b)}")
    print(f"b shape: {b.shape}")
    print(f"b.T: {fm(b.T)}")
    print(f"b.T shape: {b.T.shape}")
    
    print(f"b magnitude: {np.mag(b)}")
    print(f"b norm: {fm(np.linalg.norm(b))}")
    print(f"mag(norm(b): {np.mag(np.linalg.norm(b))}")
    print(f"{fm(a)} dot {fm(b)}: {fm(np.dot(a, b))}")
     
    twerk = Matrix([[1, 0, 0]])
    print(f"twerk: {fm(twerk)} shape: {twerk.shape}")
    #print(f"\n\n{dir(twerk)}\n\n")
      
    twox = 2*xaxis
    axis1 = twox / m.sqrt(np.dot(twox, twox))
    axis2 = np.linalg.norm(twox)
    print(f"twox: {fm(twox)}")
    print(f"axis1: {fm(axis1)}")
    print(f"axis2: {fm(axis2)}")

    zrot = rmat(zaxis, m.radians(50))
    print(f"zrot 50 degrees: {fm(zrot)}")


        
def test_rmat():
    cyl_front = vector(-100, 0, 0)
    cyl_port = rotate(zaxis, m.radians(120), cyl_front)  
    print(f"front: {fm(cyl_front)}, port: {fm(cyl_port)}")
    
if __name__ == "__main__":
    # pybricksdev run ble -n rotor testpblinalg.py
    identify()
    testlinalg()
    test_rmat()
