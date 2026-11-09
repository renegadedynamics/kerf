"""build123d's ExportDXF on a sweep of curve kinds: the entities ezdxf reads back from each file, as JSON.

Usage: dxf_reference.py out.json

Each case is a shape built here and in DxfReferenceTests.cs alike; the test writes it with Kerf's ExportDxf and
compares the entities (kinds, layers, counts and geometry) with these.
"""

import io
import json
import math
import sys

import ezdxf
from build123d import *
from build123d.exporters import ColorIndex, LineType


def spline():
    return Edge.make_spline([(0, 0), (5, 3), (9, -1), (12, 4)])


FLIPPED = Plane((0, 0, 0), x_dir=(1, 0, 0), z_dir=(0, 0, -1))

CASES = {
    "rectangle": lambda: Rectangle(20, 10),
    "plate_with_hole": lambda: Rectangle(50, 20) - Circle(4),
    "circle_moved": lambda: Pos(15, 5) * Circle(3),
    "arc_ccw": lambda: Edge.make_circle(5, start_angle=10, end_angle=100),
    "arc_cw": lambda: Edge.make_circle(5, start_angle=10, end_angle=100, angular_direction=AngularDirection.CLOCKWISE),
    "arc_placed": lambda: Location((2, 3, 0), (0, 0, 30)) * Edge.make_circle(4, start_angle=200, end_angle=350),
    "arc_flipped": lambda: Edge.make_circle(5, FLIPPED, start_angle=20, end_angle=120),
    "arc_wrapping": lambda: Edge.make_circle(3, start_angle=300, end_angle=60),
    "ellipse": lambda: Edge.make_ellipse(6, 3),
    "ellipse_arc": lambda: Edge.make_ellipse(6, 3, start_angle=30, end_angle=200),
    "ellipse_arc_flipped": lambda: Edge.make_ellipse(6, 3, FLIPPED, start_angle=30, end_angle=200),
    "ellipse_sketch_rotated": lambda: Rot(0, 0, 40) * Ellipse(8, 3),
    "spline": spline,
    "spline_periodic": lambda: Edge.make_spline([(0, 0), (5, 3), (9, -1), (4, -4)], periodic=True),
    "spline_trimmed": lambda: spline().trim(0.2, 0.7),
    "bezier": lambda: Edge.make_bezier((0, 0), (3, 6), (8, -2), (10, 3)),
    "polyline": lambda: Polyline((0, 0), (4, 0), (6, 3), (2, 5)),
    "slot": lambda: SlotOverall(20, 6),
    "rounded_rectangle": lambda: RectangleRounded(30, 20, 4),
    "hexagon": lambda: RegularPolygon(5, 6),
    # build123d approximates these to polynomial cubics (to_splines, 1e-3); Kerf writes them exactly
    "bezier_rational": lambda: Edge.make_bezier((0, 0), (5, 8), (10, 0), weights=[1, 3, 1]),
    "bspline_degree5": lambda: Edge.make_bspline(
        [(0, 0), (2, 4), (4, -1), (6, 5), (8, 0), (10, 3), (12, 1)], [0, 0, 0, 0, 0, 0, 0.5, 1, 1, 1, 1, 1, 1], 5),
    "circle_arc_rational": lambda: Edge.make_bspline(
        [(1, 0), (1, 1), (0, 1)], [0, 0, 0, 1, 1, 1], 2, weights=[1, math.sqrt(0.5), 1]),
}


def write(exporter):
    stream = io.BytesIO()
    exporter.write(stream)
    return stream.getvalue()


def entities(doc):
    out = []
    for e in doc.modelspace():
        t = e.dxftype()
        d = {"type": t, "layer": e.dxf.layer}
        if t == "LINE":
            d["start"] = list(e.dxf.start)[:2]
            d["end"] = list(e.dxf.end)[:2]
        elif t == "CIRCLE":
            d["center"] = list(e.dxf.center)[:2]
            d["radius"] = e.dxf.radius
        elif t == "ARC":
            d["center"] = list(e.dxf.center)[:2]
            d["radius"] = e.dxf.radius
            d["start_angle"] = e.dxf.start_angle
            d["end_angle"] = e.dxf.end_angle
        elif t == "ELLIPSE":
            d["center"] = list(e.dxf.center)[:2]
            d["major_axis"] = list(e.dxf.major_axis)[:2]
            d["ratio"] = e.dxf.ratio
            d["start_param"] = e.dxf.start_param
            d["end_param"] = e.dxf.end_param
        elif t == "SPLINE":
            d["degree"] = e.dxf.degree
            d["knots"] = list(e.knots)
            d["control_points"] = [list(p)[:2] for p in e.control_points]
            d["weights"] = list(e.weights)
        # Points along a spline, for the cases build123d approximates, compared by shape rather than by data
        try:
            d["points"] = [list(p)[:2] for p in e.flattening(0.002)] if t == "SPLINE" else []
        except AttributeError:
            d["points"] = []
        out.append(d)
    return out


def tables(doc):
    layers = [{"name": l.dxf.name, "color": l.dxf.color, "linetype": l.dxf.linetype, "lineweight": l.dxf.lineweight}
              for l in doc.layers]
    linetypes = [{"name": l.dxf.name, "description": l.dxf.description,
                  "pattern": list(l.pattern_tags.tags and [t.value for t in l.pattern_tags.tags if t.code in (40, 49)] or [])}
                 for l in doc.linetypes]
    return layers, linetypes


def main(path):
    result = {"ezdxf": ezdxf.__version__, "cases": {}}
    for name, make in CASES.items():
        exporter = ExportDXF()
        exporter.add_shape(make())
        doc = ezdxf.read(io.StringIO(write(exporter).decode("utf-8")))
        result["cases"][name] = {"entities": entities(doc)}

    # Layers, colours, line types and weights, and the units in the header
    for unit_name, unit in (("mm", Unit.MM), ("inch", Unit.IN)):
        exporter = ExportDXF(unit=unit, color=ColorIndex.BLUE, line_weight=0.5, line_type=LineType.CENTER)
        exporter.add_layer("hidden", color=ColorIndex.GRAY, line_weight=0.18, line_type=LineType.HIDDEN)
        exporter.add_layer("iso", color=ColorIndex.RED, line_type=LineType.ISO_DASH_DOT)
        exporter.add_layer("plain")
        exporter.add_shape(Rectangle(20, 10))
        exporter.add_shape(Circle(3), layer="hidden")
        exporter.add_shape(Edge.make_line((0, 0), (5, 5)), layer="iso")
        exporter.add_shape(Edge.make_line((0, 0), (-5, 5)), layer="plain")
        doc = ezdxf.read(io.StringIO(write(exporter).decode("utf-8")))
        layers, linetypes = tables(doc)
        result["cases"]["layers_" + unit_name] = {
            "entities": entities(doc),
            "layers": layers,
            "linetypes": linetypes,
            "insunits": doc.header.get("$INSUNITS"),
            "measurement": doc.header.get("$MEASUREMENT"),
        }

    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(result, f, indent=1)
        f.write("\n")


if __name__ == "__main__":
    main(sys.argv[1])
