#!/usr/bin/env python
import math as m
import numpy as np

import scipy
import scipy.constants
sic = scipy.constants
import scipy.signal as signal
import control


def get_j():
    # https://www.bricklink.com/v2/catalog/catalogitem.page?P=41250#T=C
    # Ball, Hard Plastic 52mm D, page has mass
    ms = 13.9/1000 # ball mass: g->kg 
    R = 26/1000 # ball radius: mm->m
    # https://www.quora.com/What-is-the-volume-of-plastic-in-a-4x2-Lego-brick
    ρ_ABS = 1070 # density of ABS plastic: 1070 kg/m^3
    v_ABS = ms/ρ_ABS # volume of ABS in m^3
    vsph = lambda r: (4/3)*m.pi*r**3
    osm = vsph(R)*ρ_ABS
    #osm * 1000 # 52 mm sphere of solid LEGO ABS in g
    rvol = lambda M, ρ: ((3*M)/(4*m.pi*ρ))**(1/3)
    Ris = rvol(osm-ms, ρ_ABS) # inner shell to subtracted
    #Ris * 1000 # mm
    jsphsh = lambda r1, r2, m: (2/5)*m*( (r2**5 - r1**5) / (r2**3 - r1**3) )
    J = jsphsh(Ris, R, ms)
    H = -ms*sic.g/(J/(R**2)+ms)
    return J, H

def rscale(a_, b_, c_, d_, k_):
    "This function will find the scale factor for a full-state feedback system to eliminate the steady-state error."
    # https://www.ee.usyd.edu.au/tutorials_online/matlab/extras/rscale.html
    # http://octave-online.net
    # https://docs.scipy.org/doc/numpy-dev/user/numpy-for-matlab-users.html
    s = a_.shape[0]
    print("s:", s)
    z = np.concatenate((np.zeros((1, s))[0], [1]), axis=0)
    print("z:", z)
    top = np.concatenate((a_, b_), axis=1)
    bot = np.concatenate((c_, d_), axis=0)
    tot = np.vstack((top, bot))
    print("tot:", tot)
    N = np.linalg.inv(tot).dot(z)
    print("N: ", N)
    Nx = N[0:s]
    print("Nx", Nx)
    Nu = N[s]
    print("Nu:", Nu)
    Nbar = Nu + k_.dot(Nx)
    print("Nbar", Nbar)
    return Nbar[0]


class BallMPC(object):    
    def __init__(self, ts = 66/1000, x0 = np.array([25/1000, 100/1000])):
        print(f"sic.g: {sic.g}")
        #g = -9.807 #m/s^2
        J, H = get_j()
    
        print(f"J: {J}, H: {H}")
    
        A = np.array([ [0, 1],
                    [0, 0]] )
        B = np.array([[0], [H]])
        C = np.array([1,  0,  ])
        D = np.array([0])
        ball_ss = signal.StateSpace(A, B, C, D)
        print(f"ball_ss: {ball_ss}")
        p1 = -2+2j
        p2 = -2-2j

        K = control.place(A, B, [p1, p2])
        print(f"K: {K}")
    
        Nbar = rscale(A, B, C, D, K)
        print(f"Nbar: {Nbar}")
    
        #ts = 60/1000
        syst = control.ss(A-B*K, B*Nbar, C, D)
        self.sysd = control.c2d(syst, ts) # converting allows us to assume constant step time
        self.x = x0
    
    # https://github.com/scipy/scipy/blob/main/scipy/signal/_ltisys.py
    # line 1932 shows continuous computation methods
    # Zero-order hold
            # Algorithm: to integrate from time 0 to time dt, we solve
            #   xdot = A x + B u,  x(0) = x0
            #   udot = 0,          u(0) = u0.
            #
            # Solution is
            #   [ x(dt) ]       [ A*dt   B*dt ] [ x0 ]
            #   [ u(dt) ] = exp [  0     0    ] [ u0 ]
            
    # Linear interpolation between steps
            # Algorithm: to integrate from time 0 to time dt, with linear
            # interpolation between inputs u(0) = u0 and u(dt) = u1, we solve
            #   xdot = A x + B u,        x(0) = x0
            #   udot = (u1 - u0) / dt,   u(0) = u0.
            #
            # Solution is
            #   [ x(dt) ]       [ A*dt  B*dt  0 ] [  x0   ]
            #   [ u(dt) ] = exp [  0     0    I ] [  u0   ]
            #   [u1 - u0]       [  0     0    0 ] [u1 - u0]
            
    def compute(self, u):
        x = self.sysd.A@self.x +(self.sysd.B*u).reshape(2)
        y = self.sysd.C@x
        #xout.append(x)
        #yout.append(y)
        self.x = x
        return y
    def computeux(self, u, x): # apparently I'm French now
        xout = self.sysd.A@x +(self.sysd.B*u).reshape(2)
        yout = self.sysd.C@xout
        return yout, xout
    
if __name__ == "__main__":
    #x0 = np.array([25/1000, 100/1000]) # initial position 25mm, initial velocity 100mm/s
    #bmpc = BallMPC(x0)
    bmpc = BallMPC()
    u = 50/1000 # requested distance mm -> m
    print(f"bmpc.compute(u): {[m.degrees(bmpc.compute(u)) for x in range(5)]}")