###Rotorhead coordinate system


The rotorhead is configured as three linear manipulators angled 120∘ apart in a circle of radius R.
The are hinged towards the center point P0 at the bottom and are attached to a Y-shaped frame at the
top with ball-and socket joints.  The mast extends vertically from P0.  The top frame can move 
freely around a spherical bearing which in turn can slide up and down on the mast, providing the
necessary degrees of freedom for both cyclic and collective.

P0 is origin of coordinate system.
Positive X is pointing to the front of the system.
Positive Y is pointing starboard.
Positive Z is pointing up.

∠Pf-P0-Pp = 120∘
∠Pf-P0-Ps = 120∘

The three linear actuators ("cylinders") are identified as Cf, Cp, and Cs, for the front, port, and
starboard respectively.  Each cylinder is represented by two points, a fixed point at the bottom
and a movable one at the connection to the rotorhead Y-frame.  For the front Cylinder, the vector
Cf is Pf'-Pf, with the primed point being movable.  Each cylinder has a minimum and maximum length.

The plane of the rotor is defined by Pc, the nominal height of the collective, as well as the
three points Pf', Pp', and Ps'.  This plane is manipulated around the Y axis by the pitch input
of the cyclic, and around the x axis by the roll input of the collective.

#### Algorithm
The point Pc is first chosen from the collective input.  The pitch vector Vp is the x axis rotated
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