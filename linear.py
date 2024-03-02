#an attempt to provide a consistent interface between pybricks.tools.Matrix, vector, etc and np

def vector(*args):
    # n by 1 vector convenience function
    return vector_(args)

def vlen(v): 
    "number of elements in v"
    # len doesn't work on a pybricks "vector"
    return vlen_(v)

def dot(a, b):
    # pybricks doesn't have one
    return dot_(a, b)
    
def vmag(v):
    # magnitude of vector, np.linalg.norm(v)
    # m.sqrt(dot(v,v)) on pybricks
    return vmag_(v)

def normalize(v):
    # I hate that norm() and normalize() are so close linguistically
    return normalize_(v)
        
def fv(v):
    # format n by 1 vector on one line
    core = [f"{float(f):.3f}" for f in v]
    return f"[{' '.join(core)}]"

def fm(m):
    # format matrix
    return fm_(m)

def matmul(a, b):
    return matmul_(a, b)  
      
Matrix = None
cross = None
       
try:
    import numpy as np
    import math as m
    print("Numpy detected")
    def vector_(args):
        return np.array(args)
    #vlen_ = lambda v: len(v)
    vlen_ = lambda v: v.shape[0] # len also works, shape is (3,) on np and (3,1) on pybricks
    Matrix = np.array
    cross = np.cross
    dot_ = lambda a, b: np.dot(a, b)
    vmag_ = lambda v: np.linalg.norm(v)
    normalize_ = lambda v: v/np.linalg.norm(v)
    matmul_ = lambda a, b: np.matmul(a, b)
    def fm_(m):
        core = [fv(v) for v in m] 
        nl = '\n'
        return f"[{nl.join(core)}]"
    #def testprec_():
    #    np.set_printoptions(precision = 3, floatmode="fixed")
except Exception as e:
    #print(f"Numpy not detected: {e}")
    try:
        from pybricks.tools import vector as pbvector
        from pybricks.tools import Matrix as pbmatrix
        from pybricks.tools import cross as pbcross
        import umath as m
        def vector_(args):
            return pbvector(*args)
        print("PyBricks detected")
        vlen_ = lambda v: v.shape[0]
        Matrix = pbmatrix
        cross = pbcross
        def dot_(a, b):
            return sum( [ a[i]*b[i] for i in range(vlen(a)) ] )
        def vmag_(v):
            return m.sqrt(dot(v, v))
        normalize_ = lambda v: v/vmag(v)
        matmul_ = lambda a, b: a*b
        def fm_(m):
            rows = [ [m[3*z], m[3*z+1], m[3*z+2]] for z in range(3)]
            core = [fv(v) for v in rows] 
            nl = '\n'
            return f"[{nl.join(core)}]"
            #return fv(m)
    except Exception as e:
        print(f"linear algebra not detected: {e}")
        raise

# https://stackoverflow.com/questions/6802577/rotation-of-3d-vector
xaxis = vector(1, 0, 0)
yaxis = vector(0, 1, 0)
zaxis = vector(0, 0, 1)
def rmat(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = normalize(axis)
    a = m.cos(theta / 2.0)
    b, c, d = -axis * m.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return Matrix([[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]])

def rotate(axis, theta, point):
    rm = rmat(axis, theta)
    return np.matmul(rm, point)
     
def test():
    v = vector(1.0, 2.0, 3.0)
    w = vector(3, -2, 9)
    u = cross(v, w)
    print(f"fv: {fv(v)} v.shape[0]: {v.shape[0]}, v[0]: {v[0]} vlen(v): {vlen(v)}")
    print(f"vmag(v): {vmag(v):.3f} fw: {fv(w)} fu: {fv(u)} normalize(u): {fv(normalize(u))}")
    xaxis45 = rmat(xaxis, m.radians(45))
    rotv = matmul(xaxis45, v)
    print(f"xaxis45: {fm(xaxis45)} fv(rotv): {fv(rotv)}")
           
if __name__ == "__main__":
    if(vector):
        test()
        