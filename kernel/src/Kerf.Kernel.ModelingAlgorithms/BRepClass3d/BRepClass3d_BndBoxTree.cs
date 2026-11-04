// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_BndBoxTree.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_BndBoxTree.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

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
