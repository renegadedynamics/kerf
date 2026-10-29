// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_BndBoxTree.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_BndBoxTree.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Selects, in the tree of the boxes of a solid's edges and vertices, a shape whose tolerance contains a point (and
/// stops at the first one).
/// </summary>
/// <remarks>A working object (decision 21) over the explorer's map of edges and vertices.</remarks>
public sealed class BRepClass3d_BndBoxTreeSelectorPoint : NCollection_UBTree<int, Bnd_Box>.Selector
{
    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapOfShape; // shapes (vertices + edges)
    private gp_Pnt myP = new();

    /// <summary>A selector over the shapes of <paramref name="theMapOfShape"/> (the tree's objects are their indices).</summary>
    public BRepClass3d_BndBoxTreeSelectorPoint(NCollection_IndexedMap<TopoDS_Shape> theMapOfShape)
    {
        myMapOfShape = theMapOfShape;
    }

    /// <summary>True if the box doesn't contain the point.</summary>
    public override bool Reject(in Bnd_Box theBox) => (theBox.IsOut(myP));

    /// <summary>True (and stops) if the point is within the tolerance of the edge or vertex.</summary>
    public override bool Accept(in int theObj)
    {
        // Box-point collision.
        if (theObj > myMapOfShape.Extent())
        {
            return false;
        }

        TopoDS_Shape shp = myMapOfShape[theObj];
        TopAbs_ShapeEnum sht = shp.ShapeType();
        if (sht == TopAbs_EDGE)
        {
            TopoDS_Edge E = TopoDS.Edge(shp);
            double EdgeTSq = BRep_Tool.Tolerance(E);
            EdgeTSq *= EdgeTSq;
            var C = new BRepAdaptor_Curve(E);
            BRep_Tool.Range(E, out double f, out double l);

            // Edge-Point interference.
            var ExtPC = new Extrema_ExtPC(myP, C, f, l);
            if (ExtPC.IsDone() && ExtPC.NbExt() > 0)
            {
                for (int i = 1; i <= ExtPC.NbExt(); i++)
                {
                    if (ExtPC.SquareDistance(i) < EdgeTSq)
                    {
                        myStop = true; // exit from selector
                        return true;
                    }
                }
            }
        }
        else if (sht == TopAbs_VERTEX)
        {
            TopoDS_Vertex V = TopoDS.Vertex(shp);
            gp_Pnt VPnt = BRep_Tool.Pnt(V);
            double VertTSq = BRep_Tool.Tolerance(V);
            VertTSq *= VertTSq;
            // Vertex-Point interference.
            if (VPnt.SquareDistance(myP) < VertTSq)
            {
                myStop = true;
                return true;
            }
        }

        return false;
    }

    /// <summary>Sets the point.</summary>
    public void SetCurrentPoint(in gp_Pnt theP) => myP = theP;
}

/// <summary>
/// Selects, in the tree of the boxes of a solid's edges and vertices, those a segment of a line passes within the
/// tolerance of, with the parameters of the nearest points.
/// </summary>
/// <remarks>A working object (decision 21) over the explorer's map of edges and vertices.</remarks>
public sealed class BRepClass3d_BndBoxTreeSelectorLine : NCollection_UBTree<int, Bnd_Box>.Selector
{
    /// <summary>An edge met by the line: the parameters on the edge and on the line.</summary>
    public readonly record struct EdgeParam(TopoDS_Edge myE, double myParam, double myLParam);

    /// <summary>A vertex met by the line: the parameter on the line.</summary>
    public readonly record struct VertParam(TopoDS_Vertex myV, double myLParam);

    private readonly NCollection_IndexedMap<TopoDS_Shape> myMapOfShape; // shapes (vertices + edges)
    private gp_Lin myL = new();
    private readonly NCollection_Sequence<EdgeParam> myEP = new(); // output result (edge vs line)
    private readonly NCollection_Sequence<VertParam> myVP = new(); // output result (vertex vs line)
    private readonly GeomAdaptor_Curve myLC = new();
    private bool myIsValid;

    /// <summary>A selector over the shapes of <paramref name="theMapOfShape"/> (the tree's objects are their indices).</summary>
    public BRepClass3d_BndBoxTreeSelectorLine(NCollection_IndexedMap<TopoDS_Shape> theMapOfShape)
    {
        myMapOfShape = theMapOfShape;
        myIsValid = true;
    }

    /// <summary>True if the box doesn't meet the line.</summary>
    public override bool Reject(in Bnd_Box theBox) => (theBox.IsOut(myL));

    /// <summary>True if the segment passes within the tolerance of the edge or vertex (recording the parameters).</summary>
    public override bool Accept(in int theObj)
    {
        // box-line collision
        if (theObj > myMapOfShape.Extent())
        {
            return false;
        }

        TopoDS_Shape shp = myMapOfShape[theObj];
        TopAbs_ShapeEnum sht = shp.ShapeType();
        if (sht == TopAbs_EDGE)
        {
            TopoDS_Edge E = TopoDS.Edge(shp);
            double EdgeTSq = BRep_Tool.Tolerance(E);
            EdgeTSq *= EdgeTSq;
            var C = new BRepAdaptor_Curve(E);
            BRep_Tool.Range(E, out double f, out double l);

            // Edge-Line interference.
            var ExtCC = new Extrema_ExtCC(C, myLC, f, l, myLC.FirstParameter(), myLC.LastParameter());
            if (ExtCC.IsDone())
            {
                if (ExtCC.IsParallel())
                {
                    // Tangent case is invalid for classification
                    myIsValid = false;
                }
                else if (ExtCC.NbExt() > 0)
                {
                    bool IsInside = false;
                    for (int i = 1; i <= ExtCC.NbExt(); i++)
                    {
                        if (ExtCC.SquareDistance(i) < EdgeTSq)
                        {
                            ExtCC.Points(i, out Extrema_POnCurv P1, out Extrema_POnCurv P2);

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
        else if (sht == TopAbs_VERTEX)
        {
            TopoDS_Vertex V = TopoDS.Vertex(shp);
            double VertTSq = BRep_Tool.Tolerance(V);
            VertTSq *= VertTSq;
            // Vertex-Line interference.
            var ExtPL = new Extrema_ExtPElC(BRep_Tool.Pnt(V),
                                            myL,
                                            Precision.Confusion(),
                                            -Precision.Infinite(),
                                            Precision.Infinite());
            if (ExtPL.IsDone() && ExtPL.NbExt() > 0)
            {
                if (ExtPL.SquareDistance(1) < VertTSq)
                {
                    Extrema_POnCurv PP;
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
    public void SetCurrentLine(in gp_Lin theL, double theMaxParam)
    {
        myL = theL;
        myLC.Load(new Geom_Line(theL), -Precision.PConfusion(), theMaxParam);
    }

    /// <summary>The <paramref name="i"/>-th edge met, with the parameters on it and on the line.</summary>
    public void GetEdgeParam(int i, out TopoDS_Edge theOutE, out double theOutParam, out double outLParam)
    {
        EdgeParam EP = myEP.Value(i);
        theOutE = EP.myE;
        theOutParam = EP.myParam;
        outLParam = EP.myLParam;
    }

    /// <summary>The <paramref name="i"/>-th vertex met, with the parameter on the line.</summary>
    public void GetVertParam(int i, out TopoDS_Vertex theOutV, out double outLParam)
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
