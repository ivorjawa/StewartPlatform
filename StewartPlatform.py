import linear as lin
m = lin.m
from linear import xaxis, yaxis, zaxis, rotate, vector
import slerp

# FIXME more glyph button bullshit, oh how I hate the PS3 controller
cSA = 24
cSB = 40
cSC = 72
cSD = 68

# FIXME we can get this out of linear or slerp
def euler_rotation_matrix(alpha,beta,gamma):
    """
    https://automaticaddison.com/how-to-describe-the-rotation-of-a-robot-in-3d/
    Generate a full three-dimensional rotation matrix from euler angles
 
    Input
    :param alpha: The roll angle (radians) - Rotation around the x-axis
    :param beta: The pitch angle (radians) - Rotation around the y-axis
    :param alpha: The yaw angle (radians) - Rotation around the z-axis
 
    Output
    :return: A 3x3 element matix containing the rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
 
    """
    # First row of the rotation matrix
    r00 = m.cos(gamma) * m.cos(beta)
    r01 = m.cos(gamma) * m.sin(beta) * m.sin(alpha) - m.sin(gamma) * m.cos(alpha)
    r02 = m.cos(gamma) * m.sin(beta) * m.cos(alpha) + m.sin(gamma) * m.sin(alpha)
     
    # Second row of the rotation matrix
    r10 = m.sin(gamma) * m.cos(beta)
    r11 = m.sin(gamma) * m.sin(beta) * m.sin(alpha) + m.cos(gamma) * m.cos(alpha)
    r12 = m.sin(gamma) * m.sin(beta) * m.cos(alpha) - m.cos(gamma) * m.sin(alpha)
     
    # Third row of the rotation matrix
    r20 = -m.sin(beta)
    r21 = m.cos(beta) * m.sin(alpha)
    r22 = m.cos(beta) * m.cos(alpha)
     
    # 3x3 rotation matrix
    rot_matrix = lin.Matrix([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
                            
    return rot_matrix
    
class StewartPlatform(object): # millimeters
    def __init__(self, inner_r, outer_r, footprint, min_cyl, max_cyl):
        self.Cmin = min_cyl
        self.Cmax = max_cyl
        self.Crange = max_cyl - min_cyl
        self.inner_r = inner_r
        self.outer_r = outer_r
        self.p0 = lin.vector(0, 0, 0)
        self.modelabel = "init"
        #big_line = lin.vector(outer_r, 0, 0)  
        small_line = lin.vector(inner_r, 0, 0)
        fp_line = lin.vector(footprint/2.0, 0, 0)
        
        # lowest and highest platform can get
        min_height_hyp = m.sqrt( min_cyl**2 - (footprint/2.0)**2 )
        self.min_height = m.sqrt( min_height_hyp**2 - (outer_r-inner_r)**2 )
        max_height_hyp = m.sqrt( max_cyl**2 - (footprint/2.0)**2  )
        self.max_height = m.sqrt( max_height_hyp**2 - (outer_r-inner_r)**2  )
        self.plat_range = self.max_height - self.min_height
        self.cube_unit_guess = min(self.inner_r, self.plat_range) # guess at limit for expanding unit xyz movement
        print(f"min_cyl: {min_cyl} max_cyl: {max_cyl} inner_r: {inner_r} outer_r: {outer_r}")
        print(f"min_height_hyp: {min_height_hyp:6.1f} min_height: {self.min_height:6.1f}")
        print(f"max_height_hyp: {max_height_hyp:6.1f} max_height: {self.max_height:6.1f}")
        print(f"Height Range: +-{self.plat_range/2:6.1f}")
        print(f"cube unit guess: {self.cube_unit_guess}")
        #sys.exit(0)

        
        # three spokes of inner wheel are sA, sB, sC ccw
        # three spokes of outer wheel are s1-s6
        # three feet of outer wheel are fA, fB, fC
        
        # starting disk positions
        rotoff = 30 # align with roll and pitch: B -- front, C -- port, A -- starboard
        self.sA = rotate(zaxis, m.radians(30+rotoff), small_line)
        self.sB = rotate(zaxis, m.radians(150+rotoff), small_line)
        self.sC = rotate(zaxis, m.radians(-90+rotoff), small_line)
        
        # default cylinder positions
        self.cyls = [self.Cmin + .5* self.Crange for i in range(6)]
        
        # fixed ends of cylinders
        self.s1 = rotate(zaxis, m.radians(-60+rotoff), fp_line) + outer_r*lin.normalize(self.sA)
        self.s2 = rotate(zaxis, m.radians(120+rotoff), fp_line) + outer_r*lin.normalize(self.sA)
        self.s3 = rotate(zaxis, m.radians(60+rotoff), fp_line) + outer_r*lin.normalize(self.sB)
        self.s4 = rotate(zaxis, m.radians(240+rotoff), fp_line) + outer_r*lin.normalize(self.sB)
        self.s5 = rotate(zaxis, m.radians(180+rotoff), fp_line) + outer_r*lin.normalize(self.sC)
        self.s6 = rotate(zaxis, m.radians(0+rotoff), fp_line) + outer_r*lin.normalize(self.sC)
        
        # caching stuff, this needs to be handled better, should be in motor actuate function in subclass
        self.old_sA = self.sA
        self.old_sB = self.sB
        self.old_sC = self.sC
        self.old_coll_v = self.p0 *(self.min_height + .5 * self.plat_range)
        self.old_cyls = self.cyls.copy()
    
    def solve6(self, roll, pitch, yaw, x, y, z, use_guess=True):
        "rpy in degrees, xyz -1.0..1.0 * guess or xyz in relative mm to 0"
        #(heading, pitch, roll)
        rotor = slerp.euler_quat(m.radians(yaw), m.radians(pitch), m.radians(roll))
        return self.solve4(rotor, x, y, z, use_guess)
        
    def solve4(self, rotor, x, y, z, use_guess=True):
        cubescale = 1
        if use_guess:
            cubescale = self.cube_unit_guess
        saq = slerp.point(*self.sA)
        sbq = slerp.point(*self.sB)
        scq = slerp.point(*self.sC)
        sa = slerp.quat2vec3(slerp.qrotate(rotor, saq))
        sb = slerp.quat2vec3(slerp.qrotate(rotor, sbq))
        sc = slerp.quat2vec3(slerp.qrotate(rotor, scq))
        coll_v = (lin.vector(x, y, z) * cubescale) + lin.vector(0, 0, self.min_height)
        sa += coll_v
        sb += coll_v
        sc += coll_v
        spokecyls = self.spoke_solve(sa, sb, sc)
        if len(spokecyls) == 4:
            sa, sb, sc, cyls = spokecyls
            self.cyls = cyls # necessary for movement.
            return ((coll_v, sa, sb, sc))
        else:
            return (())
    
    def solve(self, roll, pitch, yaw, coll, glyph): 
        """
        https://stackoverflow.com/questions/26289972/use-numpy-to-multiply-a-matrix-across-an-array-of-points
        
        transforming multiple points:
        >> mat = lin.rmat(lin.xaxis, m.radians(90))
        >> pts = np.random.random((5,3))
        >> pts
        array([[0.73548668, 0.82505642, 0.24109958],
               [0.16282707, 0.05095367, 0.48493043],
               [0.86938809, 0.17692427, 0.47028215],
               [0.7015419 , 0.59625183, 0.30894065],
               [0.71625289, 0.5231511 , 0.45795695]])
        >> lin.matmul(pts, mat.T)
        array([[ 0.73548668, -0.24109958,  0.82505642],
               [ 0.16282707, -0.48493043,  0.05095367],
               [ 0.86938809, -0.47028215,  0.17692427],
               [ 0.7015419 , -0.30894065,  0.59625183],
               [ 0.71625289, -0.45795695,  0.5231511 ]])
        
        """
        #white = white
        #red = red
        #green = green
        #print(f"solve(roll: {roll:.3f}, pitch: {pitch:.3f}, yaw: {yaw:.3f}, coll: {coll:.3f}, glyph: {glyph})")
        Vp = lin.vector(m.cos(m.radians(pitch)), 0, m.sin(m.radians(pitch)))
        Vr = lin.vector(0, m.cos(m.radians(roll)), m.sin(m.radians(roll)))
        Vq = lin.normalize(Vp + Vr)
        #print(f"Pitch: {pitch} Vp: {Vp} roll: {roll} Vr: {Vr} Vq: {Vq}")
        
        # Normal of rotor disk
        Vdisk = lin.cross(Vp, Vr) 
        Vdisk_n = lin.normalize(Vdisk)
        
        # starts like helicopter rotor, straight up and down at selected percent of range
        coll_p = coll*self.plat_range+self.min_height
        #print(f"plat_range: {self.plat_range} min_height: {self.min_height}, collective: {coll} coll_p: {coll_p}")
        
        # bend that vector by the disk deflection
        coll_v = coll_p * Vdisk_n         
        
        # once we rewrite this, glyph can go away FIXME
        flatmode = False
        if (glyph & cSD) == cSD:
            flatmode = True
            #print("using flat motion")
            coll_v = lin.vector(coll_v[0], coll_v[1], coll_p)
            self.modelabel = f"flat motion {glyph}"
        
        if not flatmode:
            
            self.modelabel = f"sphere motion {glyph}"
            oily = euler_rotation_matrix(m.radians(roll),m.radians(-pitch),0) # sphere motion
            
            # cup motion was never really useful except as a test.
            # but keeping the logic around anyway
            #if (glyph & cSC) == cSC: 
            #    #print("using cup motion") 
            #    self.modelabel = f"cup motion {glyph}"     
            #    oily = euler_rotation_matrix(m.radians(-roll),m.radians(pitch),0) # cup motion
            #else:
            #    self.modelabel = f"sphere motion {glyph}"
            #    #print("using sphere motion")
            #    oily = euler_rotation_matrix(m.radians(roll),m.radians(-pitch),0) # sphere motion
                
        
        if flatmode:
            sa = rotate(zaxis, m.radians(yaw), self.sA)
            sb = rotate(zaxis, m.radians(yaw), self.sB)
            sc = rotate(zaxis, m.radians(yaw), self.sC)
            sa = sa+coll_v
            sb = sb+coll_v
            sc = sc+coll_v          
        else:
            sa = lin.matmul(oily, self.sA)+coll_v
            sb = lin.matmul(oily, self.sB)+coll_v
            sc = lin.matmul(oily, self.sC)+coll_v
            sa = rotate(Vdisk_n, m.radians(yaw), sa)
            sb = rotate(Vdisk_n, m.radians(yaw), sb)
            sc = rotate(Vdisk_n, m.radians(yaw), sc)
        
        spokes = self.spoke_solve(sa, sb, sc)
        #print(f"solved spokes: {spokes}")
        if len(spokes) == 4:
            #print("have new spokes")
            sa, sb, sc, cyls = spokes
            self.old_sA = sa
            self.old_sB = sb
            self.old_sC = sc
            self.old_coll_v = coll_v
            self.old_cyls = cyls
        else:
            #print("usikng old spokes")
            sa = self.old_sA
            sb = self.old_sB
            sc = self.old_sC
            coll_v = self.old_coll_v
            cyls = self.old_cyls
        self.cyls = cyls
        return (coll_v, sa, sb, sc)
        
    def spoke_solve(self, sa, sb, sc):
        triads = [
            [sa, self.s1, self.s2],
            [sb, self.s3, self.s4],
            [sc, self.s5, self.s6],
        ]
        cyls = []
        #print(f"triads: {triads}")
        #for (top, left, right) in triads:
        for triad in triads:
            (top, left, right) = triad
            #print(f"triad: {triad}")
            #print(f"top: {top} left: {left} right: {right}")
            c1 = lin.vmag(top-left)
            cyls.append(c1)
            c2 = lin.vmag(top-right)
            cyls.append(c2)
            
            if min(c1, c2, self.Cmin) != self.Cmin:
                print(f"cylinder too short: c1: {c1} c2: {c2} Cmin: {self.Cmin}")
                return(())
                
            if max(c1, c2, self.Cmax) != self.Cmax:
                print(f"cylinder too long: c1: {c1} c2: {c2} Cmin: {self.Cmax}")
                return(())
        return((sa, sb, sc, cyls))
            

def probe():
    np = lin.np
    from rich.pretty import pprint as rpp
    #inner_r, outer_r, footprint, min_cyl, max_cyl: mm
    Stew = StewartPlatform(57, 98, 120, 250, 314)
    def max_r(hpct):
        retval = (0, (Stew.plat_range) * hpct)
        for r in range(0, 200, 1):
            for d in range(0, 360, 20):
                roll = 0
                pitch = 0
                yaw = 0
                x = r*m.cos(m.radians(d))
                y = r*m.sin(m.radians(d))
                z = (Stew.plat_range) * hpct
                #print(f"Radius: {r} Angle: {d} RPY: ({roll}, {pitch} {yaw}) XYZ: ({x:5.1f}, {y:5.1f}, {z:5.1f}) ", end='')
                rv = Stew.solve6(roll, pitch, yaw, x, y, z, False)
                if  rv != ():
                    retval = (r, z)
                    #print(f"cyls: {np.intp(Stew.cyls)}")
                else:
                    #print(f"solve failed, returning radius {retval}")
                    return retval
    #h = .5
    #r = max_r(.5)
    #print(f"height percent: {h} max radius: {r}")
    rd = []
    for h in np.arange(0, 1.01, .01):
        rd.append(max_r(h))
    rpp(rd)
    retval = np.array(rd)
    print(retval)
    #for k, v in enumerate(rd):
    #    #r, z = v
    #    #print(f"{k:3.1f}% R: {v[0]:5.1f} z: {v[1]:5.1f}")
    #    print(f"{k:3.1f}% v: {v}")
    return retval, Stew
if __name__ == "__main__":
    probe()
    
    
        

    