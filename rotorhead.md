







#### Table of Variables

#####invariants
P_O: Origin of XYZ system
R_sw: radius of swashplate arms from P_coll
C_min: cylinder minimum length
C_max: cylinder maximum length
A_cyl: separation of cylinders, 120∘
F_f, F_s, F_p, F_n: Foot vectors
V_mast: Vector of mast
P_f_f, P_f_0: End of front swashplate foot
P_f_p, P_f_1: End of port swashplate foot
P_f_s, P_f_2: End of starboard swashplate foot
V_cN: normal vector plane of cylinder foot and mast
#####inputs
P_coll: height of collective above origin
A_p, A_r: Pitch and Roll angle derived from collective

#####outputs
C_f, C_0: Front linear actuator, vector representing length
C_p, C_1: Port linear actuator
C_s, C_2: Starboard linear actuator
P_a_f, P_a_0: End of front swashplate arm
P_a_p, P_a_1: End of port swashplate arm
P_a_s, P_a_2: End of starboard swashplate arm

#####calculations
Vp, V^p: Pitch vector from cyclic, normalized
Vr, V^p: Roll vector from cyclic, normalized
Vdisk: Normal vector of rotor plane, Vp X Vr
Visectn, hat: intersection of rotor plane and cylinder plane, Vdisk X V_cN
Van: vector of arm N, Rsw*Visctn+ Vvmast
Cn: Cylinder Vector: Van-Vf



### Algorithm
The point Pcoll is first chosen from the collective input.  The pitch vector Vp is the x axis rotated
around the y axis by the pitch input.  The roll vector Vr is the y axis rotated along the x axis by
the roll input.

The normal vector representing the plane of the rotor then Vr X Vx.

Then for each of the cylinder, the Mast X the vector from the orgin to the foot of the cylinder
is normal to the plane the cylinder can travel in.  The cross product of this vector and the rotor
plane vector is the vector of intersection of these two planes.  This vector is normalized and then
multiplied by radius R and added to Pc to locate the top point of the cylinder.

Finally, the length of the cylinder vector is checked to make sure it fits within the limits.


