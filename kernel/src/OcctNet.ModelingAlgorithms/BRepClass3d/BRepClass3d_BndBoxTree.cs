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
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// Selects, in the tree of the boxes of a solid's edges and vertices, a shape whose tolerance contains a point (and
/// stops at the first one).
/// </summary>
/// <remarks>A working object (decision 21) over the explorer's map of edges and vertices.</remarks>
public sealed class BRepClass3d_BndBoxTreeSelectorPoint : SpatialTree<int, Box>.Selector
{
    private readonly NCollection_IndexedMap<Shape> myMapOfShape; // shapes (vertices + edges)
    private Point3D myP = new();

    /// <summary>A selector over the shapes of <paramref name="theMapOfShape"/> (the tree's objects are their indices).</summary>
    public BRepClass3d_BndBoxTreeSelectorPoint(NCollection_IndexedMap<Shape> theMapOfShape)
    {
        myMapOfShape = theMapOfShape;
    }

    /// <summary>True if the box doesn't contain the point.</summary>
    public override bool Reject(in Box theBox) => (theBox.IsOut(myP));

    /// <summary>True (and stops) if the point is within the tolerance of the edge or vertex.</summary>
    public override bool Accept(in int theObj)
    {
        // Box-point collision.
        if (theObj > myMapOfShape.Extent())
        {
            return false;
        }

        Shape shp = myMapOfShape[theObj];
        ShapeType sht = shp.ShapeType();
        if (sht == ShapeType.Edge)
        {
            Edge E = ShapeCasts.Edge(shp);
            double EdgeTSq = Tool.Tolerance(E);
            EdgeTSq *= EdgeTSq;
            var C = new EdgeCurveAdaptor(E);
            Tool.Range(E, out double f, out double l);

            // Edge-Point interference.
            var ExtPC = new CurvePointExtrema(myP, C, f, l);
            if (ExtPC.IsDone && ExtPC.ExtCount > 0)
            {
                for (int i = 1; i <= ExtPC.ExtCount; i++)
                {
                    if (ExtPC.SquareDistance(i) < EdgeTSq)
                    {
                        _stop = true; // exit from selector
                        return true;
                    }
                }
            }
        }
        else if (sht == ShapeType.Vertex)
        {
            Vertex V = ShapeCasts.Vertex(shp);
            Point3D VPnt = Tool.Pnt(V);
            double VertTSq = Tool.Tolerance(V);
            VertTSq *= VertTSq;
            // Vertex-Point interference.
            if (VPnt.SquareDistance(myP) < VertTSq)
            {
                _stop = true;
                return true;
            }
        }

        return false;
    }

    /// <summary>Sets the point.</summary>
    public void SetCurrentPoint(in Point3D theP) => myP = theP;
}

/// <summary>
/// Selects, in the tree of the boxes of a solid's edges and vertices, those a segment of a line passes within the
/// tolerance of, with the parameters of the nearest points.
/// </summary>
/// <remarks>A working object (decision 21) over the explorer's map of edges and vertices.</remarks>
public sealed class BRepClass3d_BndBoxTreeSelectorLine : SpatialTree<int, Box>.Selector
{
    /// <summary>An edge met by the line: the parameters on the edge and on the line.</summary>
    public readonly record struct EdgeParam(Edge myE, double myParam, double myLParam);

    /// <summary>A vertex met by the line: the parameter on the line.</summary>
    public readonly record struct VertParam(Vertex myV, double myLParam);

    private readonly NCollection_IndexedMap<Shape> myMapOfShape; // shapes (vertices + edges)
    private Line3D myL = new();
    private readonly NCollection_Sequence<EdgeParam> myEP = new(); // output result (edge vs line)
    private readonly NCollection_Sequence<VertParam> myVP = new(); // output result (vertex vs line)
    private readonly CurveAdaptor myLC = new();
    private bool myIsValid;

    /// <summary>A selector over the shapes of <paramref name="theMapOfShape"/> (the tree's objects are their indices).</summary>
    public BRepClass3d_BndBoxTreeSelectorLine(NCollection_IndexedMap<Shape> theMapOfShape)
    {
        myMapOfShape = theMapOfShape;
        myIsValid = true;
    }

    /// <summary>True if the box doesn't meet the line.</summary>
    public override bool Reject(in Box theBox) => (theBox.IsOut(myL));

    /// <summary>True if the segment passes within the tolerance of the edge or vertex (recording the parameters).</summary>
    public override bool Accept(in int theObj)
    {
        // box-line collision
        if (theObj > myMapOfShape.Extent())
        {
            return false;
        }

        Shape shp = myMapOfShape[theObj];
        ShapeType sht = shp.ShapeType();
        if (sht == ShapeType.Edge)
        {
            Edge E = ShapeCasts.Edge(shp);
            double EdgeTSq = Tool.Tolerance(E);
            EdgeTSq *= EdgeTSq;
            var C = new EdgeCurveAdaptor(E);
            Tool.Range(E, out double f, out double l);

            // Edge-Line interference.
            var ExtCC = new ExtCC(C, myLC, f, l, myLC.FirstParameter(), myLC.LastParameter());
            if (ExtCC.IsDone)
            {
                if (ExtCC.IsParallel)
                {
                    // Tangent case is invalid for classification
                    myIsValid = false;
                }
                else if (ExtCC.ExtCount > 0)
                {
                    bool IsInside = false;
                    for (int i = 1; i <= ExtCC.ExtCount; i++)
                    {
                        if (ExtCC.SquareDistance(i) < EdgeTSq)
                        {
                            ExtCC.Points(i, out POnCurv P1, out POnCurv P2);

                            // Original curve is the first parameter, the line the second.
                            var EP = new EdgeParam(E, P1.Parameter(), P2.Parameter());
                            myEP.Append(EP);

                            IsInside = true;
                        }
                    }

                    if (IsInside)
                    {
                        return true;
                    }
                }
            }
        }
        else if (sht == ShapeType.Vertex)
        {
            Vertex V = ShapeCasts.Vertex(shp);
            double VertTSq = Tool.Tolerance(V);
            VertTSq *= VertTSq;
            // Vertex-Line interference.
            var ExtPL = new ExtPElC(Tool.Pnt(V),
                                            myL,
                                            global::Kerf.Engine.Foundation.Precision.Confusion(),
                                            -global::Kerf.Engine.Foundation.Precision.Infinite(),
                                            global::Kerf.Engine.Foundation.Precision.Infinite());
            if (ExtPL.IsDone && ExtPL.ExtCount > 0)
            {
                if (ExtPL.SquareDistance(1) < VertTSq)
                {
                    POnCurv PP;
                    double paramL;
                    PP = ExtPL.Point(1);
                    paramL = PP.Parameter();
                    var VP = new VertParam(V, paramL);
                    myVP.Append(VP);
                    return true;
                }
            }
        }

        return false;
    }

    /// <summary>Sets the line, and the segment of it from -PConfusion to <paramref name="theMaxParam"/> tested against the edges.</summary>
    public void SetCurrentLine(in Line3D theL, double theMaxParam)
    {
        myL = theL;
        myLC.Load(new LineCurve3D(theL), -global::Kerf.Engine.Foundation.Precision.PConfusion(), theMaxParam);
    }

    /// <summary>The <paramref name="i"/>-th edge met, with the parameters on it and on the line.</summary>
    public void GetEdgeParam(int i, out Edge theOutE, out double theOutParam, out double outLParam)
    {
        EdgeParam EP = myEP.Value(i);
        theOutE = EP.myE;
        theOutParam = EP.myParam;
        outLParam = EP.myLParam;
    }

    /// <summary>The <paramref name="i"/>-th vertex met, with the parameter on the line.</summary>
    public void GetVertParam(int i, out Vertex theOutV, out double outLParam)
    {
        VertParam VP = myVP.Value(i);
        theOutV = VP.myV;
        outLParam = VP.myLParam;
    }

    /// <summary>The number of edges met.</summary>
    public int GetNbEdgeParam() => myEP.Length();

    /// <summary>The number of vertices met.</summary>
    public int GetNbVertParam() => myVP.Length();

    /// <summary>Forgets the edges and vertices met.</summary>
    public void ClearResults()
    {
        myEP.Clear();
        myVP.Clear();
        myIsValid = true;
    }

    /// <summary>True if a correct classification is possible (no edge was parallel to the line).</summary>
    public bool IsCorrect() => myIsValid;
}
