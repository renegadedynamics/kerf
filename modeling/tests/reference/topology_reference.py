"""Writes reference results from Python build123d's direct (topology) API for the .NET port's
differential tests: for each case, the volume, area, centre, bounding box and topology counts of
the shape it builds.

Run with the reference environment (see tests/reference/README.md):

    PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=../build123d/src .deps/py/Scripts/python -W ignore \
        tests/reference/topology_reference.py tests/Kerf.Modeling.Tests/Reference/topology.json

Each case is named; tests/Kerf.Modeling.Tests/Topology/TopologyReferenceTests.cs builds the same shape
with the port. Keep the two lists in step.
"""

import json
import sys

from build123d import (
    AngularDirection,
    Axis,
    Edge,
    Face,
    Keep,
    Kind,
    Location,
    Plane,
    Pos,
    Side,
    Solid,
    Vertex,
    Wire,
)


def summary(shape):
    bb = shape.bounding_box()
    out = {
        "type": type(shape).__name__,
        "volume": shape.volume,
        "area": shape.area,
        "bbox_min": [bb.min.X, bb.min.Y, bb.min.Z],
        "bbox_max": [bb.max.X, bb.max.Y, bb.max.Z],
        "vertices": len(shape.vertices()),
        "edges": len(shape.edges()),
        "faces": len(shape.faces()),
        "solids": len(shape.solids()),
        "valid": shape.is_valid,
    }
    try:
        c = shape.center()
        out["center"] = [c.X, c.Y, c.Z]
    except Exception:  # pylint: disable=broad-except
        pass
    if hasattr(shape, "length") and type(shape).__name__ in ("Edge", "Wire"):
        out["length"] = shape.length
    return out


def box(l, w, h, at=(0, 0, 0)):
    return Solid.make_box(l, w, h, Plane(at))


cases = {}

# Primitives
cases["box"] = box(1, 2, 3)
cases["cylinder"] = Solid.make_cylinder(2, 5)
cases["partial_cylinder"] = Solid.make_cylinder(2, 5, Plane.XZ, angle=120)
cases["cone"] = Solid.make_cone(3, 1, 4)
cases["sphere"] = Solid.make_sphere(2.5)
cases["partial_sphere"] = Solid.make_sphere(2, angle1=-45, angle2=60, angle3=270)
cases["torus"] = Solid.make_torus(5, 1)
cases["wedge"] = Solid.make_wedge(4, 3, 2, 1, 0.5, 3, 1.5)

# Booleans
cases["box_minus_cylinder"] = box(10, 10, 10, (-5, -5, -5)) - Solid.make_cylinder(3, 10, Plane((0, 0, -5)))
cases["box_plus_box"] = box(2, 2, 2) + box(2, 2, 2, (1, 1, 1))
cases["box_and_sphere"] = box(4, 4, 4, (-2, -2, -2)) & Solid.make_sphere(2.5)
cases["disjoint_fuse"] = box(1, 1, 1) + box(1, 1, 1, (3, 0, 0))
cases["pocket"] = box(20, 20, 10) - box(10, 10, 10, (5, 5, 5))

# Features
block = box(10, 10, 10)
cases["fillet_top"] = block.fillet(1.5, block.edges().group_by(Axis.Z)[-1])
cases["chamfer_vertical"] = block.chamfer(1, None, block.edges().filter_by(Axis.Z))
cases["chamfer_asymmetric"] = block.chamfer(2, 1, block.edges().group_by(Axis.Z)[-1])
cases["hollow"] = block.hollow([block.faces().sort_by(Axis.Z)[-1]], -1)
cases["offset_3d"] = block.offset_3d(None, 1)

# Faces and extrusions
rect = Face.make_rect(4, 2)
cases["rect_face"] = rect
cases["extrude_rect"] = Solid.extrude(rect, (0, 0, 3))
cases["extrude_taper"] = Solid.extrude_taper(Face.make_rect(4, 4), (0, 0, 2), 10)
cases["revolve_rect"] = Solid.revolve(Face.make_rect(1, 2, Plane.XZ.offset(0).shift_origin((3, 0))), 270, Axis.Z)
holed = Face(Wire.make_rect(6, 6), [Wire.make_circle(1)])
cases["holed_face"] = holed
cases["extrude_holed"] = Solid.extrude(holed, (0, 0, 1))
cases["thicken_face"] = Solid.thicken(Face.make_rect(3, 3), 0.5)
cases["face_from_spline_wire"] = Face(
    Wire([Edge.make_spline([(0, 0), (2, 1), (4, 0)]), Edge.make_line((4, 0), (0, 0))])
)

# Lofts and sweeps
cases["loft"] = Solid.make_loft([Wire.make_rect(4, 4), Wire.make_circle(1, Plane.XY.offset(5))])
cases["loft_ruled"] = Solid.make_loft([Wire.make_rect(2, 2), Wire.make_rect(1, 1, Plane.XY.offset(3))], ruled=True)
cases["loft_to_vertex"] = Solid.make_loft([Wire.make_rect(2, 2), Vertex(0, 0, 3)])
path = Wire([Edge.make_line((0, 0, 0), (0, 0, 5)), Edge.make_three_point_arc((0, 0, 5), (0.6, 0, 6.2), (2, 0, 7))])
cases["sweep_circle"] = Solid.sweep(Wire.make_circle(0.5), path)
cases["sweep_helix"] = Solid.sweep(
    Wire.make_circle(0.2, Plane((3, 0, 0), z_dir=(0, 1, 0))), Wire([Edge.make_helix(2, 6, 3)]), is_frenet=True
)

# Edges and wires
cases["line"] = Edge.make_line((0, 0, 0), (3, 4, 0))
cases["circle"] = Edge.make_circle(2)
cases["arc"] = Edge.make_circle(2, start_angle=30, end_angle=200)
cases["arc_cw"] = Edge.make_circle(2, start_angle=30, end_angle=200, angular_direction=AngularDirection.CLOCKWISE)
cases["arc_xz"] = Edge.make_circle(1.5, Plane.XZ, 0, 90)
cases["ellipse"] = Edge.make_ellipse(3, 1)
cases["ellipse_arc"] = Edge.make_ellipse(1, 3, start_angle=10, end_angle=250)
cases["spline"] = Edge.make_spline([(0, 0, 0), (1, 2, 0), (3, 1, 1), (4, 3, 0)])
cases["spline_tangents"] = Edge.make_spline([(0, 0, 0), (2, 1, 0), (4, 0, 0)], tangents=[(0, 1, 0), (0, -1, 0)])
cases["bezier"] = Edge.make_bezier((0, 0, 0), (1, 2, 0), (3, 2, 0), (4, 0, 0))
cases["helix"] = Edge.make_helix(1, 4, 2)
cases["conical_helix"] = Edge.make_helix(1, 4, 2, angle=15)
cases["three_point_arc"] = Edge.make_three_point_arc((0, 0, 0), (1, 1, 0), (2, 0, 0))
cases["tangent_arc"] = Edge.make_tangent_arc((0, 0, 0), (1, 0, 0), (2, 2, 0))
cases["trimmed_spline"] = Edge.make_spline([(0, 0, 0), (1, 2, 0), (3, 1, 1), (4, 3, 0)]).trim(0.2, 0.7)
cases["polygon"] = Wire.make_polygon([(0, 0), (3, 0), (3, 2), (1, 3)])
cases["rect_wire"] = Wire.make_rect(5, 3, Plane.XZ)
cases["offset_wire"] = Wire.make_rect(4, 2).offset_2d(0.5)
cases["offset_wire_intersection"] = Wire.make_rect(4, 2).offset_2d(-0.5, kind=Kind.INTERSECTION)
cases["offset_arc"] = Edge.make_circle(3, start_angle=0, end_angle=120).offset_2d(0.5)
cases["offset_arc_left"] = Edge.make_circle(3, start_angle=0, end_angle=120).offset_2d(0.5, side=Side.LEFT)
square = Wire.make_rect(4, 4)
cases["chamfer_wire"] = square.chamfer_2d(1, 0.5, square.vertices()[:2])

# Transforms
cases["mirror_box"] = box(1, 2, 3, (1, 1, 1)).mirror(Plane.YZ)
cases["rotate_box"] = box(1, 2, 3).rotate(Axis.Z, 30)
cases["scale_box"] = box(1, 2, 3).scale(2)
cases["scale_box_xyz"] = box(1, 2, 3).scale((1, 2, 3))
cases["transformed_box"] = box(1, 2, 3).transformed((10, 20, 30), (1, 2, 3))
cases["located_cylinder"] = Solid.make_cylinder(1, 2).located(Location((1, 2, 3), (30, 0, 45)))

# Splits
cases["split_top"] = box(2, 2, 2).split(Plane((0, 0, 0.5)))
cases["split_bottom"] = box(2, 2, 2).split(Plane((0, 0, 0.5)), keep=Keep.BOTTOM)

# Clean
cases["clean_fuse"] = (box(1, 1, 1) + box(1, 1, 1, (1, 0, 0))).clean()

result = {}
for name, shape in cases.items():
    if isinstance(shape, list):
        shape = shape[0]
    result[name] = summary(shape)

with open(sys.argv[1], "w", encoding="utf8") as f:
    json.dump(result, f, indent=1)
print(f"{len(result)} cases")
