from pybricks.tools import Matrix, cross, vector
import umath as m

array = Matrix

# this isn't quite right
# in numpy to rotate you dot(matrix, vector)
# in pybricks to rotate you Matrix*vector
def dot(a, b):
    return sum( [ a[i]*b[i] for i in range(vlen(a)) ] )

def vlen(v):
    return v.shape[0]

def mag(a):
    return m.sqrt(dot(a, a))

# sin is ugly, what this does is normalizes       
class linalg(object):
    def norm(x):
        return x/mag(x)
        
