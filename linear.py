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

Matrix = None
cross = None
       
try:
    import numpy as np
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
        #print("pybricks detected")
        vlen_ = lambda v: v.shape[0]
        Matrix = pbmatrix
        cross = pbcross
        def dot_(a, b):
            return sum( [ a[i]*b[i] for i in range(vlen(a)) ] )
        def vmag_(v):
            return m.sqrt(dot(v, v))
        normalize_ = lambda v: v/vmag(v)
    except Exception as e:
        print(f"linear algebra not detected: {e}")
        raise
 
def test():
    v = vector(1.0, 2.0, 3.0)
    w = vector(3, -2, 9)
    u = cross(v, w)
    print(f"fv: {fv(v)} v.shape[0]: {v.shape[0]}, v[0]: {v[0]} vlen(v): {vlen(v)}")
    print(f"vmag(v): {vmag(v):.3f} fw: {fv(w)} fu: {fv(u)} normalize(u): {fv(normalize(u))}")
           
if __name__ == "__main__":
    if(vector):
        test()
        