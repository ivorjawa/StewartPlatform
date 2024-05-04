# quat.py "Micro" Quaternion class

# Released under the MIT License (MIT). See LICENSE.
# Copyright (c) 2020 Peter Hinch

from umath import sqrt, sin, cos, acos, asin, atan2, pi
from isclose import isclose
import umath as m

#from array import array
import linear as lin

mdelta = 0.001  # 0.1% Minimum difference considered significant for graphics
adelta = 0.001  # Absolute tolerance for components near 0

def _arglen(arg):
    length = 0
    try:
        length = len(arg)
    except TypeError:
        pass
    if length not in (0, 3, 4):
        raise ValueError('Sequence length must be 3 or 4')
    return length

# Convert a rotation quaternion to Euler angles. Beware:
# https://github.com/moble/quaternion/wiki/Euler-angles-are-horrible
def euler(q):  # Return (heading, pitch, roll)
    if not q.isrot():
        raise ValueError('Must be a rotation quaternion.')
    w, x, y, z = q
    pitch = asin(2*(w*y - x*z))
    if isclose(pitch, pi/2, rel_tol = mdelta):
        return -2 * atan2(x, w), pitch, 0
    if isclose(pitch, -pi/2, rel_tol = mdelta):
        return 2 * atan2(x, w), pitch, 0
    roll = atan2(2*(w*x + y*z), w*w - x*x - y*y + z*z)
    #roll = atan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
    hdg = atan2(2*(w*z + x*y), w*w + x*x - y*y - z*z)
    #hdg = atan2(2*(w*z + x*y), 1 - 2 *(y*y + z*z))
    return hdg, pitch, roll

        
""" 
        @property
        def w(self):
            return self[0]

        @w.setter
        def w(self, v):
            self[0] = v

        @property
        def x(self):
            return self[1]

        @x.setter
        def x(self, v):
            self[1] = v

        @property
        def y(self):
            return self[2]

        @y.setter
        def y(self, v):
            self[2] = v

        @property
        def z(self):
            return self[3]

        @z.setter
        def z(self, v):
            self[3] = v   
"""
    
class Quaternion:
    
        
    def __init__(self, w=1, x=0, y=0, z=0):  # Default: the identity quaternion
        #self.d = array('f', (w, x, y, z))
        self.d = lin.vec4(w, x, y, z)
        
    def normalise(self):
        if self[0] == 1:  # acos(1) == 0. Identity quaternion: no rotation
            return Quaternion(1, 0, 0, 0)
        m = abs(self)  # Magnitude
        assert m > 0.1  # rotation quaternion should have magnitude ~= 1
        if isclose(m, 1.0, rel_tol=mdelta):
            return self  # No normalisation necessary
        return Quaternion(*(a/m for a in self))

    def __getitem__(self, key):
        #print(f"key: {key} d: {type(self.d)}")
        #print(dir(self.d))
        try:
            ret =  self.d[key]
        except TypeError as e:
            #ql = [x for x in self.d] # this REVERSES it???
            ql = []
            for i in range(lin.vlen(self.d)): # oh what the actual HELL???
                ql.append(self.d[i])
                #print(f"d[{i}]: {self.d[i]}")
                #print(f"ql[{i}]: {ql[i]}")
            #print(f"ql: {ql}")
            ret = ql[key]
        #print(f"key: {key} ret: {ret}")
        return ret

    def __setitem__(self, key, v):
        try:
            v1 = lin.vec4(*v)
        except TypeError:  # Scalar
            v1 = v
        self.d[key] = v1

    def copy(self):
        return Quaternion(*self)

    def __abs__(self):  # Return magnitude
        return sqrt(sum((d*d for d in self)))

    def __len__(self):
        return 4
    # Comparison: == and != perform equality test of all elements
    def __eq__(self, other):
        return all((isclose(a, b, rel_tol=mdelta, abs_tol=adelta) for a, b in zip(self, other)))

    def __ne__(self, other):
        return not self == other

    # < and > comparisons compare magnitudes.
    def __gt__(self, other):
        return abs(self) > abs(other)

    def __lt__(self, other):
        return abs(self) < abs(other)

    # <= and >= return True for complete equality otherwise magnitudes are compared.
    def __ge__(self, other):
        return True if self == other else abs(self) > abs(other)

    def __le__(self, other):
        return True if self == other else abs(self) < abs(other)

    def to_angle_axis(self):
        q = self.normalise()
        if isclose(q[0], 1.0, rel_tol = mdelta):
            return 0, 1, 0, 0
        theta = 2*acos(q[0])
        s = sin(theta/2)
        return [theta] + [a/s for a in q[1:]]

    def conjugate(self):
        return Quaternion(self[0], *(-a for a in self[1:]))

    def inverse(self):  # Reciprocal
        return self.conjugate()/sum((d*d for d in self))

    def __str__(self):
        return 'w = {:4.2f} x = {:4.2f} y = {:4.2f} z = {:4.2f}'.format(*self)

    def __pos__(self):
        return Quaternion(*self)

    def __neg__(self):
        return Quaternion(*(-a for a in self))

    def __truediv__(self, scalar):
        if isinstance(scalar, Quaternion):  # See docs for reason
            raise ValueError('Cannot divide by Quaternion')
        return Quaternion(*(a/scalar for a in self))

    def __rtruediv__(self, other):
        return self.inverse() * other

    # Multiply by quaternion, list, tuple, or scalar: result = self * other
    def __mul__(self, other):
        if isinstance(other, Quaternion):
            w1, x1, y1, z1 = self
            w2, x2, y2, z2 = other
            w = w1*w2 - x1*x2 - y1*y2 - z1*z2
            x = w1*x2 + x1*w2 + y1*z2 - z1*y2
            y = w1*y2 - x1*z2 + y1*w2 + z1*x2
            z = w1*z2 + x1*y2 - y1*x2 + z1*w2
            return Quaternion(w, x, y, z)
        length = _arglen(other)
        if length == 0:  # Assume other is scalar
            return Quaternion(*(a * other for a in self))
        elif length == 3:
            return Quaternion(0, *(a * b for a, b in zip(self[1:], other)))
        # length == 4:
        return Quaternion(*(a * b for a, b in zip(self, other)))

    def __rmul__(self, other):
        return self * other  # Multiplication by scalars and tuples is commutative

    def __add__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(*(a + b for a, b in zip(self, other)))
        length = _arglen(other)
        if length == 0:  # Assume other is scalar
            return Quaternion(self[0] + other, *self[1:]) # ? Is adding a scalar meaningful?
        elif length == 3:
            return Quaternion(0, *(a + b for a, b in zip(self[1:], other)))
        # length == 4:
        return Quaternion(*(a + b for a, b in zip(self, other)))

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(*(a - b for a, b in zip(self, other)))
        length = _arglen(other)
        if length == 0:  # Assume other is scalar
            return Quaternion(self[0] - other, *self[1:]) # ? Is this meaningful?
        elif length == 3:
            return Quaternion(0, *(a - b for a, b in zip(self[1:], other)))
        # length == 4:
        return Quaternion(*(a - b for a, b in zip(self, other)))

    def __rsub__(self, other):
        return other + self.__neg__()  # via __radd__

    def isrot(self):
        return isclose(abs(self), 1.0, rel_tol = mdelta)

    def isvec(self):
        return isclose(self[0], 0, abs_tol = adelta)

    def __matmul__(self, rot):
        return rot * self * rot.conjugate()

    def rrot(self, rot):
        return rot.conjugate() * self * rot
        
    #https://github.com/Kent-H/blue3D/blob/master/Blue3D/src/blue3D/type/QuaternionF.java
    #https://math.stackexchange.com/questions/939229/unit-quaternion-to-a-scalar-power
    def scale(self, n):
        return Quaternion(*(n*self.d))
        
    def ln(self): 
        #w, x, y, z = self.d
        w = self.d[0]
        x = self.d[1]
        y = self.d[2]
        z = self.d[3]
        #print("ln wxyz:",w,x,y,z)
        r  = m.sqrt(x*x+y*y+z*z)
        #print(f"r: {r}")
        #print(f"w: {w}")
        t  = (r>0.00001) and (m.atan2(r,w)/r) or 0
        #print(f"t: {t}")
        #w = 0.5*m.log(lin.dot(self.d, self.d));
        w = 0.5*m.log(w*w+x*x+y*y+z*z);
        #w = 0.5*m.log(abs(self))
        x = x*t
        y = y*t
        z = z*t
        return Quaternion(w, x, y, z)

    def exp(self):
        #w, x, y, z = self.d
        w = self.d[0]
        x = self.d[1]
        y = self.d[2]
        z = self.d[3]
        #print("exp wxyz:",w,x,y,z)
        r  = m.sqrt(x*x+y*y+z*z)
        et = m.exp(w)
        s  = (r>=0.00001) and (et*m.sin(r)/r) or 0

        w = et*m.cos(r)
        x = x*s
        y = y*s
        z = z*s
        return Quaternion(w, x, y, z)
        
    def __pow__(self, n):
        return self.ln().scale(n).exp()
        
# A vector quaternion has real part 0. It can represent a point in space.
def Vector(x, y, z):
    return Quaternion(0, x, y, z)

Point = Vector

# A rotation quaternion is a unit quaternion i.e. magnitude == 1
def Rotator(theta=0, x=0, y=0, z=0):
    s = sin(theta/2)
    m = sqrt(x*x + y*y + z*z)  # Convert to unit vector
    if m > 0:
        return Quaternion(cos(theta/2), s*x/m, s*y/m, s*z/m)
    else:
        return Quaternion(1, 0, 0, 0)  # Identity quaternion

def Euler(heading, pitch, roll):
    cy = cos(heading * 0.5);
    sy = sin(heading * 0.5);
    cp = cos(pitch * 0.5);
    sp = sin(pitch * 0.5);
    cr = cos(roll * 0.5);
    sr = sin(roll * 0.5);

    w = cr * cp * cy + sr * sp * sy;
    x = sr * cp * cy - cr * sp * sy;
    y = cr * sp * cy + sr * cp * sy;
    z = cr * cp * sy - sr * sp * cy;
    return Quaternion(w, x, y, z)  # Tait-Bryan angles but z == towards sky
    
if __name__ == "__main__":
    def sgn(P):
        a = abs(p)
        if a != 0:
            return P/a            
        else:
            return 0
    
    def arg(P):
        a = p.d[0]
        return m.acos(a/abs(p))
    def exp(P):
        pbar = P.conjugate()
        uv = (P-pbar)/2   
        a = P.d[0] 
        expa = m.exp(a)
        absu = abs(uv) 
        cosu = m.cos(absu)
        sgnu = sgn(uv) 
        sinu = m.sin(absu)
        ret = expa*cosu+sgnu*sinu
        #print(f"P: {P}, pbar: {pbar} uv: {uv} a: {a} expa: {expa} absu: {absu} cosu: {cosu} sgnu: {sgnu} sinu: {sinu} ret: {ret}")
        return ret 
    def ln(P):
        lnpa = m.log(abs(P))
        
        pbar = P.conjugate()
        uv = (P-pbar)/2 
        sgnu = sgn(uv) 
        argp = arg(P)
        
        ret = lnpa + sgnu * argp
        #print(f"lnpa: {lnpa} sgnu: {sgnu} argp: {argp} ret: {ret}")
        return ret
          
        
    #v = lin.vec4(5, 5, 5, 5)
    #w, x, y, z = v
    #p = Quaternion(1, 1, 1, 1)
    p = Rotator(m.radians(45), 1, 0, 1)
    print(f"p: {p}")
    pbar = p.conjugate()
    print(f"pbar: {pbar}")
    uv = (p-pbar)/2
    print(f"uv: {uv}")
    
    sp = sgn(p)
    print(f"sgn(p): {sp}")
    ap = arg(p)
    print(f"arg(p): {ap}")
    expp = exp(p)
    print(f"exp(p): {expp}")
    lnp = ln(p)
    print(f"ln(p): {lnp}")
    n = .1
    pn = exp(n*ln(p))
    print(f"pn: {pn}")
    
    
    
    #qs = q.scale(5)
    #print(qs)
    #print(qs.ln())