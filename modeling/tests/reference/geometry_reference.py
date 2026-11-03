"""Writes reference results from Python build123d for the .NET port's differential tests.

Run with the reference environment (see tests/reference/README.md):

    PYTHONPATH=../build123d/src .deps/py/Scripts/python -I tests/reference/geometry_reference.py \
        tests/Kerf.Modeling.Tests/Reference/geometry.json

Inputs are generated from a fixed seed, so the file only changes when build123d's results do.
"""

import json
import math
import random
import sys

from OCP.gp import gp_Ax2, gp_Dir, gp_Pnt, gp_Trsf

from build123d import (
    Axis,
    Color,
    Extrinsic,
    Intrinsic,
    Location,
    Plane,
    Rotation,
    Vector,
)

rng = random.Random(123)
ORDERS = [("Intrinsic", o.name) for o in Intrinsic] + [("Extrinsic", o.name) for o in Extrinsic]


def order_enum(kind, name):
    return (Intrinsic if kind == "Intrinsic" else Extrinsic)[name]


def vec(v):
    return [v.X, v.Y, v.Z]


def rand_vec(scale=10.0):
    return [rng.uniform(-scale, scale) for _ in range(3)]


def plane_out(p):
    return {"origin": vec(p.origin), "x": vec(p.x_dir), "y": vec(p.y_dir), "z": vec(p.z_dir)}


cases = {}

# Locations from Euler angles in every order
cases["location_euler"] = []
for kind, name in ORDERS:
    for _ in range(4):
        pos, angles = rand_vec(), [rng.uniform(-360, 360) for _ in range(3)]
        loc = Location(pos, angles, order_enum(kind, name))
        cases["location_euler"].append(
            {"pos": pos, "angles": angles, "kind": kind, "order": name,
             "position": vec(loc.position), "orientation": vec(loc.orientation),
             "plane": plane_out(Plane(loc))}
        )

# Locations about an axis, composed, inverted and raised to a power
cases["location_ops"] = []
for _ in range(20):
    a = Location(rand_vec(), rand_vec(1.0), rng.uniform(-180, 180))
    b = Location(rand_vec(), [rng.uniform(-180, 180) for _ in range(3)])
    n = rng.randint(-3, 3)
    cases["location_ops"].append(
        {"a_pos": vec(a.position), "a_orient": vec(a.orientation),
         "b_pos": vec(b.position), "b_orient": vec(b.orientation), "n": n,
         "product": [vec(x) for x in (a * b)], "inverse": [vec(x) for x in a.inverse()],
         "power": [vec(x) for x in a ** n], "negated": [vec(x) for x in -a]}
    )

# Mirror across planes (kept separate so the plane is recorded)
cases["location_mirror"] = []
for _ in range(20):
    loc = Location(rand_vec(), [rng.uniform(-180, 180) for _ in range(3)])
    plane_origin, plane_z = rand_vec(), rand_vec(1.0)
    mirrored = loc.mirror(Plane(plane_origin, z_dir=plane_z))
    cases["location_mirror"].append(
        {"pos": vec(loc.position), "orient": vec(loc.orientation),
         "plane_origin": plane_origin, "plane_z": plane_z,
         "position": vec(mirrored.position), "orientation": vec(mirrored.orientation)}
    )

# Planes from mirrored (left-handed) transformations
cases["plane_from_mirror"] = []
for _ in range(10):
    p, d = rand_vec(), rand_vec(1.0)
    trsf = gp_Trsf()
    trsf.SetMirror(gp_Ax2(gp_Pnt(*p), gp_Dir(*d)))
    cases["plane_from_mirror"].append({"point": p, "dir": d, "plane": plane_out(Plane(Location(trsf)))})

# Planes from origin and directions, positional and through points
cases["plane_dirs"] = []
for _ in range(20):
    o, x, z = rand_vec(), rand_vec(1.0), rand_vec(1.0)
    cases["plane_dirs"].append({"origin": o, "x": x, "z": z, "plane": plane_out(Plane(o, x, z)),
                                "no_x": plane_out(Plane(o, z_dir=z))})

cases["plane_rotated"] = []
for kind, name in ORDERS[::3]:
    o, x, z = rand_vec(), rand_vec(1.0), rand_vec(1.0)
    angles = [rng.uniform(-180, 180) for _ in range(3)]
    p = Plane(o, x, z).rotated(angles, order_enum(kind, name))
    cases["plane_rotated"].append({"origin": o, "x": x, "z": z, "angles": angles, "kind": kind,
                                   "order": name, "plane": plane_out(p)})

# Vector angles
cases["vector_angles"] = []
for _ in range(30):
    a, b, n = rand_vec(), rand_vec(), rand_vec()
    va, vb = Vector(a), Vector(b)
    cases["vector_angles"].append({"a": a, "b": b, "n": n, "angle": va.get_angle(vb),
                                   "signed": va.get_signed_angle(vb), "signed_n": va.get_signed_angle(vb, Vector(n))})

# Axis intersections (crossing axes built from a shared point)
cases["axis_intersect"] = []
for _ in range(20):
    point, d1, d2 = rand_vec(), rand_vec(1.0), rand_vec(1.0)
    p1 = (Vector(point) + Vector(d1) * rng.uniform(-5, 5))
    p2 = (Vector(point) + Vector(d2) * rng.uniform(-5, 5))
    hit = Axis(p1, d1) & Axis(p2, d2)
    cases["axis_intersect"].append({"p1": vec(p1), "d1": d1, "p2": vec(p2), "d2": d2, "point": vec(hit)})

# Colours: exact tuples and names
cases["colors_rgb"] = []
for _ in range(30):
    r, g, b, a = (rng.random() for _ in range(4))
    c = Color(r, g, b, a)
    cases["colors_rgb"].append({"in": [r, g, b, a], "out": list(tuple(c)), "str": str(c)})
cases["colors_named"] = [
    {"name": name, "out": list(tuple(Color(name))), "str": str(Color(name))}
    for name in ["red", "steelblue", "GoldenRod", "BLUE1", "grey", "#123", "#12345678", "#abcdef"]
]
cases["colors_categorical"] = [
    {"count": n, "hue": h, "out": [list(tuple(c)) for c in Color.categorical_set(n, h)]}
    for n, h in [(5, 0.0), (7, 0.3), (10, 0.95)]
]

with open(sys.argv[1], "w", encoding="utf8") as f:
    json.dump(cases, f, indent=1)
