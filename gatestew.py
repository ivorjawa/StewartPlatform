from StewartPlatform import StewartPlatform

Stew = StewartPlatform(57, 98, 120, 250, 314) #inner, outer radius, footprint, min, max cylinder extension
roll = 0
pitch = 0
yaw = 0
x = 1
y = 1
z = .1
print(f"RPY: ({roll: 3.1f}, {pitch: 3.1f}, {yaw: 3.1f}) XYZ: ({x: 3.1f}, {y: 3.1f}, {z: 3.1f})")
try:
    coll_v, sa, sb, sc = Stew.solve6(roll, pitch, yaw, x, y, z)
    print(f"COLL_v: ({coll_v[0]: 3.1f}, {coll_v[1]: 3.1f}, {coll_v[2]: 3.1f})")
    for i, c in enumerate(Stew.cyls):
        print(f"Cyl {i}: {c: 3.1f}mm")
except Exception as e:
    print(e)