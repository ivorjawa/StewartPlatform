### Coordinate systems

P0 is origin of coordinate system, centered under the mast of the helicopter.
Positive X is pointing to the front of the helicopter.
Positive Y is pointing starboard.
Positive Z is pointing up.

### Definitions

The primary controls of a helicopter are a stick, called the cyclic, a lever, called the collective,
and a pair of pedals, called antitorque pedals.  The throttle is typically managed mechanically to 
keep the rotor RPM constant.
The cyclic stick controls roll and pitch of the helicopter.  The antitorque pedals provide yaw,
and work to counteract the counter-rotation the fuselage experiences from the rotation of the
main rotor.  The collective changes the angle of attack of the main rotor, to regulate the thrust
of the rotor, but can really be thought of as the "gas pedal" of the helicopter.

The main rotors of the helicopter are connected through a hub called the rotorhead to the prop shaft
from the helicopter's transmission.

### System Description

For the purposes of this demonstration, a rotorhead consists of:
A two-part rotating swashplate, the bottom half of which is represented as a Y-shaped frame
rotating around a spherical bearing, which in turn surrounds a bushing through which the
drive shaft runs.  This allows the swashplate to pitch and roll around the spherical bearing, 
which in turn moves up and down on the drive shaft.  The arms of the Y-shaped frame will be
referred to as swashpate arms, each with a length of Rsw.  The center of the Y-shaped frame is
referred to as Pcoll, and represents the hight of the rotorhead over the Origin.  Pcoll can vary
in Z-height regulated by the collective position.  The ends of the arms are referred to as P_a_N,
with N proceding clockwise from the front.
The arms are connected to the helicopter with linear actuators, which will be referred to as
cylinders.  The cylinders are connected to the ends of the arms with ball joints, and to the base 
XY plane by hinges on "feet", which have an identical Y-shape to the swashplate frame.  
Thus, the cylinders can rotate towards the origin along their feet.  The length of these feet
is the same as the length of the arms Rsw, but needn't be.  The cylinders are located 120∘ around
a circle around the Origin, but don't need to be regularly distributed.
The ends of the feet are P_f_n.

For convenience, variables are typed in python without underscores, and will be used interchangably
outside the definitions in the table of variables below.

∠Pff PO Pfp = 120∘
∠Pff PO Pfs = 120∘

For convenience, cylinders also have position names.  C_0 is C_f, C_1 is C_p, and C_2 is C_s.  Similarly, P_a_f is equivalent to P_a_0, P_f_f is P_f_0, and so on, with "f", "p", and "s" abbreviating front, port, and starboard.

Each cylinder has a minimum and maximum length, C_min and C_max

The cyclic input is used to make Pitch and Roll vectors Vp and Vr, and collective the scalar
P_coll.

The plane of the rotor is defined by Pcoll, the nominal height of the collective, as well as the
three points Paf, Pap, and Pas.  This plane is manipulated around the Y axis by the pitch input
of the cyclic, and around the x axis by the roll input of the cyclic.

#### Table of Variables
P_O: Origin of XYZ system
P_coll: height of collective above origin
R_sw: radius of swashplate arms from P_coll
C_f, C_0: Front linear actuator, vector representing length
C_p, C_1: Port linear actuator
C_s, C_2: Starboard linear actuator
P_a_f, P_a_0: End of front swashplate arm
P_a_p, P_a_1: End of port swashplate arm
P_a_s, P_a_2: End of starboard swashplate arm
P_f_f, P_f_0: End of front swashplate foot
P_f_p, P_f_1: End of port swashplate foot
P_f_s, P_f_2: End of starboard swashplate foot
C_min: cylinder minimum length
C_max: cylinder maximum length
Vp: Pitch vector from cyclic
Vr: Roll vector from cyclic



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

##Code Cleanup
Can remove rotation matrices by just using plane old trig in the two planes that matter. r sine cosine theta, man.


