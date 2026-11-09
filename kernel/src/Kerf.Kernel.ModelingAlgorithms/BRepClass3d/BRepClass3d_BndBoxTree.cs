// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_BndBoxTree.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_BndBoxTree.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Classification;

/// <summary>
/// Selects, in the tree of the boxes of a solid's edges and vertices, a shape whose tolerance contains a point (and
/// stops at the first one).
/// </summary>
/// <remarks>A working object (decision 21) over the explorer's map of edges and vertices.</remarks>
internal sealed class BndBoxTreeSelectorPoint : SpatialTree<int, Box>.Selector
{
    private readonly IndexedSet<Shape> _mapOfShape; // shapes (vertices + edges)
    private Point3D _p = new();

    /// <summary>A selector over the shapes of <paramref name="mapOfShape"/> (the tree's objects are their indices).</summary>
    public BndBoxTreeSelectorPoint(IndexedSet<Shape> mapOfShape)
    {
        _mapOfShape = mapOfShape;
    }

    /// <summary>True if the box doesn't contain the point.</summary>
    public override bool Reject(in Box box) => (box.IsOut(_p));

    /// <summary>True (and stops) if the point is within the tolerance of the edge or vertex.</summary>
    public override bool Accept(in int obj)
    {
        // Box-point collision.
        if (obj > _mapOfShape.Count)
        {
            return false;
        }

        Shape shp = _mapOfShape[obj - 1];
        ShapeType sht = shp.Type;
        if (sht == ShapeType.Edge)
        {
            Edge e = ShapeCasts.Edge(shp);
            double edgeTSq = Tool.Tolerance(e);
            if (SplineHull.KeepsAway(e, edgeTSq, _p, null))
            {
                return false;
            }
            edgeTSq *= edgeTSq;
            var c = new EdgeCurveAdaptor(e);
            Tool.Range(e, out double f, out double l);

            // Edge-Point interference.
            var extPC = new CurvePointExtrema(_p, c, f, l);
            if (extPC.IsDone && extPC.ExtCount > 0)
            {
                for (int i = 1; i <= extPC.ExtCount; i++)
                {
                    if (extPC.SquareDistance(i) < edgeTSq)
                    {
                        _stop = true; // exit from selector
                        return true;
                    }
                }
            }
        }
        else if (sht == ShapeType.Vertex)
        {
            Vertex v = ShapeCasts.Vertex(shp);
            Point3D vPnt = Tool.Pnt(v);
            double vertTSq = Tool.Tolerance(v);
            vertTSq *= vertTSq;
            // Vertex-Point interference.
            if (vPnt.SquareDistanceTo(_p) < vertTSq)
            {
                _stop = true;
                return true;
            }
        }

        return false;
    }

    /// <summary>Sets the point.</summary>
    public void SetCurrentPoint(in Point3D p) => _p = p;
}

/// <summary>
/// Selects, in the tree of the boxes of a solid's edges and vertices, those a segment of a line passes within the
/// tolerance of, with the parameters of the nearest points.
/// </summary>
/// <remarks>A working object (decision 21) over the explorer's map of edges and vertices.</remarks>
internal sealed class BndBoxTreeSelectorLine : SpatialTree<int, Box>.Selector
{
    /// <summary>An edge met by the line: the parameters on the edge and on the line.</summary>
    public readonly record struct EdgeParam(Edge E, double myParam, double LParam);

    /// <summary>A vertex met by the line: the parameter on the line.</summary>
    public readonly record struct VertParam(Vertex myV, double LParam);

    private readonly IndexedSet<Shape> _mapOfShape; // shapes (vertices + edges)
    private Line3D _l = new();
    private readonly NCollection_Sequence<EdgeParam> _ep = new(); // output result (edge vs line)
    private readonly NCollection_Sequence<VertParam> _vp = new(); // output result (vertex vs line)
    private readonly CurveAdaptor _lc = new();
    private bool _isValid;

    /// <summary>A selector over the shapes of <paramref name="mapOfShape"/> (the tree's objects are their indices).</summary>
    public BndBoxTreeSelectorLine(IndexedSet<Shape> mapOfShape)
    {
        _mapOfShape = mapOfShape;
        _isValid = true;
    }

    /// <summary>True if the box doesn't meet the line.</summary>
    public override bool Reject(in Box box) => (box.IsOut(_l));

    /// <summary>True if the segment passes within the tolerance of the edge or vertex (recording the parameters).</summary>
    public override bool Accept(in int obj)
    {
        // box-line collision
        if (obj > _mapOfShape.Count)
        {
            return false;
        }

        Shape shp = _mapOfShape[obj - 1];
        ShapeType sht = shp.Type;
        if (sht == ShapeType.Edge)
        {
            Edge e = ShapeCasts.Edge(shp);
            double edgeTSq = Tool.Tolerance(e);
            if (SplineHull.KeepsAway(e, edgeTSq, _l.Location, _l.Direction))
            {
                return false;
            }
            edgeTSq *= edgeTSq;
            var c = new EdgeCurveAdaptor(e);
            Tool.Range(e, out double f, out double l);

            // Edge-Line interference.
            var extCC = new ExtCC(c, _lc, f, l, _lc.FirstParameter(), _lc.LastParameter());
            if (extCC.IsDone)
            {
                if (extCC.IsParallel)
                {
                    // Tangent case is invalid for classification
                    _isValid = false;
                }
                else if (extCC.ExtCount > 0)
                {
                    bool isInside = false;
                    for (int i = 1; i <= extCC.ExtCount; i++)
                    {
                        if (extCC.SquareDistance(i) < edgeTSq)
                        {
                            extCC.Points(i, out POnCurv p1, out POnCurv p2);

                            // Original curve is the first parameter, the line the second.
                            var ep = new EdgeParam(e, p1.Parameter(), p2.Parameter());
                            _ep.Append(ep);

                            isInside = true;
                        }
                    }

                    if (isInside)
                    {
                        return true;
                    }
                }
            }
        }
        else if (sht == ShapeType.Vertex)
        {
            Vertex v = ShapeCasts.Vertex(shp);
            double vertTSq = Tool.Tolerance(v);
            vertTSq *= vertTSq;
            // Vertex-Line interference.
            var extPL = new ExtPElC(Tool.Pnt(v),
                                            _l,
                                            global::Kerf.Engine.Foundation.Precision.Confusion(),
                                            -global::Kerf.Engine.Foundation.Precision.Infinite(),
                                            global::Kerf.Engine.Foundation.Precision.Infinite());
            if (extPL.IsDone && extPL.ExtCount > 0)
            {
                if (extPL.SquareDistance(1) < vertTSq)
                {
                    POnCurv pp;
                    double paramL;
                    pp = extPL.Point(1);
                    paramL = pp.Parameter();
                    var vp = new VertParam(v, paramL);
                    _vp.Append(vp);
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>Sets the line, and the segment of it from -PConfusion to <paramref name="maxParam"/> tested against the edges.</summary>
    public void SetCurrentLine(in Line3D l, double maxParam)
    {
        _l = l;
        _lc.Load(new LineCurve3D(l), -global::Kerf.Engine.Foundation.Precision.PConfusion(), maxParam);
    }

    /// <summary>The <paramref name="i"/>-th edge met, with the parameters on it and on the line.</summary>
    public void GetEdgeParam(int i, out Edge outE, out double outParam, out double outLParam)
    {
        EdgeParam ep = _ep.Value(i);
        outE = ep.E;
        outParam = ep.myParam;
        outLParam = ep.LParam;
    }

    /// <summary>The <paramref name="i"/>-th vertex met, with the parameter on the line.</summary>
    public void GetVertParam(int i, out Vertex outV, out double outLParam)
    {
        VertParam vp = _vp.Value(i);
        outV = vp.myV;
        outLParam = vp.LParam;
    }

    /// <summary>The number of edges met.</summary>
    public int GetNbEdgeParam() => _ep.Length();

    /// <summary>The number of vertices met.</summary>
    public int GetNbVertParam() => _vp.Length();

    /// <summary>Forgets the edges and vertices met.</summary>
    public void ClearResults()
    {
        _ep.Clear();
        _vp.Clear();
        _isValid = true;
    }

    /// <summary>True if a correct classification is possible (no edge was parallel to the line).</summary>
    public bool IsCorrect => _isValid;
}

/// <summary>
/// Kerf (unit 13K): whether an edge's spline keeps further than a distance from a point, or from a line, by its poles alone.
/// </summary>
/// <remarks>
/// A spline lies in the hull of its poles, as its weights are positive, and each span in the hull of the Degree + 1 poles that
/// make it; a hull is inside the box of its poles, and seen along a line, inside the box of the poles seen so. If the point
/// or line is outside every span's box by more than the distance, no point of the edge is within it, and the extrema the
/// selectors would ask, a global search on a spline, find none that counts: they're asked otherwise, as before, so what a
/// selector keeps is the same. Those extrema were nearly all of a classifier's time on a result whose section curves are
/// splines with a loose tolerance, as a Boolean of cylinders nearly crossing gives (decision 1924's points).
/// </remarks>
internal static class SplineHull
{
    // The spline edges this thread's selectors kept away by their poles, and those whose extrema they still asked: for the tests
    // that hold a classifier's cost down
    [ThreadStatic]
    internal static int s_keptAway;

    [ThreadStatic]
    internal static int s_extremaAsked;

    /// <summary>True if every point of <paramref name="e"/>'s spline is further than <paramref name="distance"/> from
    /// <paramref name="origin"/>, or from the line through it along <paramref name="along"/>; false if that isn't shown, or
    /// the edge's curve isn't a spline.</summary>
    public static bool KeepsAway(in Edge e, double distance, in Point3D origin, Direction3D? along)
    {
        Curve3D? curve = Tool.Curve(e, out Placement placement, out _, out _);
        if (curve is TrimmedCurve3D trimmed)
        {
            curve = trimmed.BasisCurve;
        }
        NCollection_Array1<Point3D> poles;
        int span;
        switch (curve)
        {
            case BSplineCurve3D spline:
                poles = spline.InternalPoles();
                span = spline.IsPeriodic ? poles.Length() : spline.Degree + 1;
                break;
            case BezierCurve3D bezier:
                poles = bezier.InternalPoles();
                span = poles.Length();
                break;
            default:
                return false;
        }

        // Axes to measure along: across the line, or the three of space
        Vector3D first, second, third;
        if (along is { } direction)
        {
            var d = new Vector3D(direction);
            first = d.Crossed(Math.Abs(direction.X) < 0.6 ? new Vector3D(1, 0, 0) : new Vector3D(0, 1, 0)).Normalized();
            second = d.Crossed(first);
            third = Vector3D.Zero;
        }
        else
        {
            (first, second, third) = (new Vector3D(1, 0, 0), new Vector3D(0, 1, 0), new Vector3D(0, 0, 1));
        }
        bool moved = !placement.IsIdentity;
        Transform3D move = placement.Transformation;
        double reach = distance + global::Kerf.Engine.Foundation.Precision.Confusion();
        int lower = poles.Lower(), count = poles.Length();
        if (count == 0 || span > count)
        {
            return false;
        }
        for (int start = 0; start + span <= count; start++)
        {
            double minX = double.MaxValue, maxX = double.MinValue, minY = double.MaxValue, maxY = double.MinValue;
            double minZ = double.MaxValue, maxZ = double.MinValue;
            for (int i = start; i < start + span; i++)
            {
                Point3D pole = poles[lower + i];
                if (moved)
                {
                    pole = pole.Transformed(move);
                }
                var offset = new Vector3D(pole.X - origin.X, pole.Y - origin.Y, pole.Z - origin.Z);
                double x = offset.Dot(first), y = offset.Dot(second), z = offset.Dot(third);
                minX = Math.Min(minX, x);
                maxX = Math.Max(maxX, x);
                minY = Math.Min(minY, y);
                maxY = Math.Max(maxY, y);
                minZ = Math.Min(minZ, z);
                maxZ = Math.Max(maxZ, z);
            }
            if (minX <= reach && maxX >= -reach && minY <= reach && maxY >= -reach && minZ <= reach && maxZ >= -reach)
            {
                s_extremaAsked++;
                return false;
            }
        }
        s_keptAway++;
        return true;
    }
}
