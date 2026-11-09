// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/Visualization/TKV3d/StdPrs/StdPrs_BRepFont.cxx (renderGlyph, buildFaces), redesigned (unit 13D)

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Fonts;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Fonts;

/// <summary>
/// A glyph's outline as shapes in the XY plane, at a size (OCCT's <c>StdPrs_BRepFont</c>): each contour a wire of lines
/// and Bézier curves, each outer contour a face with the contours directly inside it as its holes, oriented to face +Z.
/// A single-stroke font's contours are open wires, with no faces. Built once per glyph and shared by its uses.
/// </summary>
/// <remarks>
/// OCCT turns each contour counter-clockwise by classifying a point at infinity, and finds which contour is inside which
/// by classifying each edge's middle against every other wire through <c>BRepTopAdaptor_FClass2d</c>. Here a contour's
/// orientation is its exact signed area (Green's theorem, which is exact for lines and Bézier curves), and a contour is
/// inside another when every one of its segments' middles is inside the other's outline flattened to a polygon, OCCT's rule
/// for "inside" with the same classification of overlapping contours (neither inside the other). A contour enclosing no
/// area is left out, where OCCT would make a face of no area.
/// </remarks>
internal sealed class GlyphShapes
{
    private const double Precision = 1e-7;   // OCCT's Precision::Confusion, StdPrs_BRepFont's precision
    private const int FlattenSteps = 16;     // per curve, for the containment test only

    private readonly Font _font;
    private readonly double _scale;
    private readonly double _slant;
    private readonly bool _singleStroke;
    private readonly PlaneSurface _plane = new(new Plane3D());
    private readonly Dictionary<int, Shape> _cache = [];

    public GlyphShapes(Font font, double size, double slant, bool singleStroke)
    {
        _font = font;
        _scale = size / font.UnitsPerEm;
        _slant = Math.Tan(slant * Math.PI / 180);
        _singleStroke = singleStroke;
    }

    /// <summary>The glyph's shape at the origin, or a null shape for a blank glyph; a failure for a damaged one.</summary>
    public Result<Shape> Of(int glyph)
    {
        if (_cache.TryGetValue(glyph, out Shape shape))
        {
            return shape;
        }

        Result<GlyphOutline> outline = _font.Outline(glyph);
        if (!outline.IsSuccess)
        {
            return Result<Shape>.Failure(outline.Diagnostics);
        }

        shape = Build(outline.Value);
        _cache[glyph] = shape;
        return shape;
    }

    private Point2D Map(Point2D p) => new(_scale * (p.X + _slant * p.Y), _scale * p.Y);

    private Shape Build(GlyphOutline outline)
    {
        var contours = new List<List<OutlineSegment>>();
        foreach (Contour raw in outline.Raw)
        {
            var segments = new List<OutlineSegment>(raw.Segments.Length);
            int count = raw.Segments.Length - (_singleStroke && raw.ClosedByLine ? 1 : 0);
            for (int i = 0; i < count; i++)
            {
                OutlineSegment s = raw.Segments[i];
                var mapped = new OutlineSegment(s.Kind, Map(s.Start), Map(s.Control1), Map(s.Control2), Map(s.End));
                if (!IsDegenerate(mapped))
                {
                    segments.Add(mapped);
                }
            }

            // Joined again where a segment of no length was left out
            for (int i = 1; i < segments.Count; i++)
            {
                if (segments[i].Start != segments[i - 1].End)
                {
                    segments[i] = WithStart(segments[i], segments[i - 1].End);
                }
            }

            if (!_singleStroke && segments.Count > 0 && segments[^1].End != segments[0].Start)
            {
                segments[^1] = WithEnd(segments[^1], segments[0].Start);
            }

            if (segments.Count > 0)
            {
                contours.Add(segments);
            }
        }

        return _singleStroke ? Strokes(contours) : Faces(contours);
    }

    private static bool IsDegenerate(OutlineSegment s) => s.Kind switch
    {
        OutlineSegmentKind.Line => s.Start.DistanceTo(s.End) <= Precision,
        _ => s.Start.DistanceTo(s.End) <= Precision && s.Start.DistanceTo(s.Control1) <= Precision && s.Start.DistanceTo(s.Control2) <= Precision,
    };

    private static OutlineSegment WithStart(OutlineSegment s, Point2D start) =>
        new(s.Kind, start, s.Kind == OutlineSegmentKind.Line ? start : s.Control1, s.Control2, s.End);

    private static OutlineSegment WithEnd(OutlineSegment s, Point2D end) =>
        new(s.Kind, s.Start, s.Control1, s.Kind == OutlineSegmentKind.Line ? end : s.Control2, end);

    private Shape Strokes(List<List<OutlineSegment>> contours)
    {
        var wires = new List<Shape>(contours.Count);
        foreach (List<OutlineSegment> contour in contours)
        {
            wires.Add(Wire(contour, closed: false));
        }

        return wires.Count switch
        {
            0 => default,
            _ => Builder.MakeCompound(wires),
        };
    }

    private Shape Faces(List<List<OutlineSegment>> contours)
    {
        // Each contour counter-clockwise; one enclosing no area is no boundary
        var rings = new List<Ring>();
        foreach (List<OutlineSegment> contour in contours)
        {
            double area = SignedArea(contour);
            if (Math.Abs(area) <= Precision * Precision || contour.Count < 2)
            {
                continue;
            }

            List<OutlineSegment> ccw = area > 0 ? contour : Reversed(contour);
            rings.Add(new Ring(ccw, Flatten(ccw)));
        }

        // Which ring is inside which: each ring's depth is the number of rings around it
        int n = rings.Count;
        var inside = new bool[n, n]; // [i, j]: ring j is inside ring i
        var depth = new int[n];
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i != j && IsInside(rings[j], rings[i]))
                {
                    inside[i, j] = true;
                    depth[j]++;
                }
            }
        }

        // Faces for the rings at an even depth, deepest first, each taking as holes the rings inside it no deeper face has
        // taken (StdPrs_BRepFont::buildFaces)
        var used = new bool[n];
        var faces = new List<Shape>();
        foreach (int outer in Enumerable.Range(0, n).Where(i => depth[i] % 2 == 0).OrderByDescending(i => depth[i]).ThenBy(i => i))
        {
            used[outer] = true;
            var holes = new List<int>();
            for (int j = 0; j < n; j++)
            {
                if (inside[outer, j] && !used[j])
                {
                    holes.Add(j);
                    used[j] = true;
                }
            }

            faces.Add(Face(rings[outer].Segments, holes.Select(h => rings[h].Segments)));
        }

        return faces.Count switch
        {
            0 => default,
            1 => faces[0],
            _ => Builder.MakeCompound(faces),
        };
    }

    private Shape Face(List<OutlineSegment> outer, IEnumerable<List<OutlineSegment>> holes)
    {
        var builder = new BrepBuilder();
        var face = new Face();
        builder.MakeFace(ref face, _plane, Precision);
        builder.Add(face, Wire(outer, closed: true));
        foreach (List<OutlineSegment> hole in holes)
        {
            builder.Add(face, Wire(Reversed(hole), closed: true));
        }

        return face;
    }

    private Shape Wire(List<OutlineSegment> segments, bool closed)
    {
        var builder = new BrepBuilder();
        var vertices = new Vertex[segments.Count + 1];
        for (int i = 0; i < segments.Count; i++)
        {
            builder.MakeVertex(ref vertices[i], To3D(segments[i].Start), Precision);
        }

        if (closed)
        {
            vertices[^1] = vertices[0];
        }
        else
        {
            builder.MakeVertex(ref vertices[^1], To3D(segments[^1].End), Precision);
        }

        var edges = new List<Shape>(segments.Count);
        for (int i = 0; i < segments.Count; i++)
        {
            edges.Add(Edge(builder, segments[i], vertices[i], vertices[i + 1]));
        }

        return Builder.MakeWire(edges);
    }

    private Edge Edge(BrepBuilder builder, OutlineSegment s, Vertex v1, Vertex v2)
    {
        Curve3D curve;
        Curve2D pcurve;
        double last;
        if (s.Kind == OutlineSegmentKind.Line)
        {
            last = s.Start.DistanceTo(s.End);
            var direction = new Direction2D(s.End.X - s.Start.X, s.End.Y - s.Start.Y);
            pcurve = new LineCurve2D(s.Start, direction);
            curve = new LineCurve3D(To3D(s.Start), new Direction3D(direction.X, direction.Y, 0));
        }
        else
        {
            Point2D[] poles = s.Kind == OutlineSegmentKind.Quadratic ? [s.Start, s.Control1, s.End] : [s.Start, s.Control1, s.Control2, s.End];
            pcurve = new BezierCurve2D(poles);
            curve = new BezierCurve3D([.. poles.Select(To3D)]);
            last = 1;
        }

        Edge edge = new LibraryMakeEdge(curve, v1, v2, 0, last).InternalEdge();
        builder.UpdateEdge(edge, pcurve, _plane, new Placement(), Precision);
        return edge;
    }

    private static Point3D To3D(Point2D p) => new(p.X, p.Y, 0);

    private static List<OutlineSegment> Reversed(List<OutlineSegment> contour)
    {
        var reversed = new List<OutlineSegment>(contour.Count);
        for (int i = contour.Count - 1; i >= 0; i--)
        {
            OutlineSegment s = contour[i];
            reversed.Add(new OutlineSegment(s.Kind, s.End, s.Control2, s.Control1, s.Start));
        }

        return reversed;
    }

    /// <summary>The contour's signed area, positive counter-clockwise: exact for lines and Bézier curves.</summary>
    internal static double SignedArea(IEnumerable<OutlineSegment> contour)
    {
        static double Cross(Point2D a, Point2D b) => a.X * b.Y - a.Y * b.X;
        double twice = 0;
        foreach (OutlineSegment s in contour)
        {
            twice += s.Kind switch
            {
                OutlineSegmentKind.Line => Cross(s.Start, s.End),
                OutlineSegmentKind.Quadratic =>
                    (2 * Cross(s.Start, s.Control1) + 2 * Cross(s.Control1, s.End) + Cross(s.Start, s.End)) / 3,
                _ => (6 * Cross(s.Start, s.Control1) + 3 * Cross(s.Start, s.Control2) + Cross(s.Start, s.End)
                      + 3 * Cross(s.Control1, s.Control2) + 3 * Cross(s.Control1, s.End) + 6 * Cross(s.Control2, s.End)) / 10,
            };
        }

        return twice / 2;
    }

    private static Point2D At(OutlineSegment s, double t)
    {
        double u = 1 - t;
        return s.Kind switch
        {
            OutlineSegmentKind.Line => new(u * s.Start.X + t * s.End.X, u * s.Start.Y + t * s.End.Y),
            OutlineSegmentKind.Quadratic => new(
                u * u * s.Start.X + 2 * u * t * s.Control1.X + t * t * s.End.X,
                u * u * s.Start.Y + 2 * u * t * s.Control1.Y + t * t * s.End.Y),
            _ => new(
                u * u * u * s.Start.X + 3 * u * u * t * s.Control1.X + 3 * u * t * t * s.Control2.X + t * t * t * s.End.X,
                u * u * u * s.Start.Y + 3 * u * u * t * s.Control1.Y + 3 * u * t * t * s.Control2.Y + t * t * t * s.End.Y),
        };
    }

    private static Point2D[] Flatten(List<OutlineSegment> contour)
    {
        var points = new List<Point2D>();
        foreach (OutlineSegment s in contour)
        {
            if (s.Kind == OutlineSegmentKind.Line)
            {
                points.Add(s.Start);
                continue;
            }

            for (int k = 0; k < FlattenSteps; k++)
            {
                points.Add(At(s, k / (double)FlattenSteps));
            }
        }

        return [.. points];
    }

    // Whether every segment's middle of the inner ring is inside the outer's polygon (OCCT's classifyWW, which gives
    // "unknown", so neither inside, where the middles disagree)
    private static bool IsInside(Ring inner, Ring outer)
    {
        foreach (OutlineSegment s in inner.Segments)
        {
            if (!Contains(outer.Polygon, At(s, 0.5)))
            {
                return false;
            }
        }

        return true;
    }

    private static bool Contains(Point2D[] polygon, Point2D p)
    {
        bool inside = false;
        for (int i = 0, j = polygon.Length - 1; i < polygon.Length; j = i++)
        {
            Point2D a = polygon[i], b = polygon[j];
            if ((a.Y > p.Y) != (b.Y > p.Y) && p.X < (b.X - a.X) * (p.Y - a.Y) / (b.Y - a.Y) + a.X)
            {
                inside = !inside;
            }
        }

        return inside;
    }

    private sealed record Ring(List<OutlineSegment> Segments, Point2D[] Polygon);
}
