# Swashplate models a swashplate with 3 linear actuators spaced evenly around a circle.
# parameters are control arm radius and minimum and maximum length of linear actuators
# solve calculates the length of the cylinders for a given roll and pitch (degrees) and collective(%)

import linear as lin
m = lin.m
from linear import vector

class Swashplate(object):
    # coordinate system is +z:up +x:forward +y:port
    def __init__(self, rotor_rad, c_min, c_max):
        self.Cmin = c_min
        self.Cmax = c_max
        self.Crange = c_max - c_min
        self.Rsw = rotor_rad
        self.P0 = vector(0, 0, 0)
        
        # Feet 
        self.Ff = vector(rotor_rad, 0, 0) # Front 
        self.Fp = vector(rotor_rad*m.cos(m.radians(120)), rotor_rad*m.sin(m.radians(120)), 0)
        self.Fs = vector(rotor_rad*m.cos(m.radians(-120)), rotor_rad*m.sin(m.radians(-120)), 0)
        
        self.feet = [self.Ff, self.Fp, self.Fs]
        
        # default 50% collective, position after calibration
        tmat = vector(0, 0, .5*self.Crange + self.Cmin)
        
        self.old_Vmast = tmat
        self.old_Cf = self.Ff + tmat
        self.old_Cp = self.Fp + tmat
        self.old_Cs = self.Fs + tmat

    def solve(self, pitch, roll, collpct):
        Vp = vector(m.cos(m.radians(pitch)), 0, m.sin(m.radians(pitch)))
        Vr = vector(0, m.cos(m.radians(roll)), m.sin(m.radians(roll)))
        
        # Normal of rotor disk
        Vdisk = lin.cross(Vp, Vr) 
        Vdisk_n = lin.normalize(Vdisk)
        
        Vmast = [0, 0, self.Cmin + collpct*self.Crange] # top of mast at collective setting
        arms = []
        for i, Fn in enumerate(self.feet):
            # Vcn is the plane the cylinder rotates on its foot in, Foot X Mast
            Vcn = lin.cross(Fn, Vmast)
            Vcn_n = lin.normalize(Vcn)

            # Visect is the intersection of the Rotor Disk plane and the Cylinder rotation plane
            Visect = lin.cross(Vdisk_n, Vcn_n) # should be plane intersection
            Visect_n = lin.normalize(Visect)

            # Va is the arm vector as rotated in the cylinder rotation plane
            Va = (self.Rsw * Visect_n) + Vmast
            
            arms.append(Va)
            cyl_len = lin.vmag(Va - Fn)
            if cyl_len < self.Cmin:
                raise ValueError(f"too short! Cyl: {cyl_len:{4}.{4}} min: {self.Cmin:{4}}")
            elif cyl_len > self.Cmax:
                raise ValueError(f"too long! Cyl: {cyl_len:{4}.{4}} max: {self.Cmax:{4}}")
        
        (Cf, Cp, Cs) = arms        
        #self.validate(Cf, Cp, Cs, Vmast, pitch, roll, collpct)
        return (Cf, Cp, Cs, Vmast)