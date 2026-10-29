// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeAnalysis_TransferParametersProj on an edge whose 3d curve (a straight cubic Bezier with uneven speed) and pcurve
/// (a line) aren't parametrised alike, so that projection and the linear transfer differ.
/// </summary>
public class ShapeAnalysis_TransferParametersProjTests
{
    private const double Tol = 1e-7;

    /// <summary>x(s) of the Bezier with poles at x = 0, 0.1, 0.2, 2 (y = z = 0).</summary>
    private static double BezierX(double s)
    {
        double t = 1 - s;
        return 3 * t * t * s * 0.1 + 3 * t * s * s * 0.2 + s * s * s * 2.0;
    }

    private static (TopoDS_Edge Edge, TopoDS_Face Face) BezierEdge(bool theSameParameter, TopoDS_Face? theFace = null)
    {
        var aB = new BRep_Builder();
        TopoDS_Face aFace = theFace ?? new();
        if (theFace is null)
        {
            aB.MakeFace(ref aFace, new Geom_Plane(new gp_Pln()), Tol);
        }

        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(0.1, 0, 0);
        aPoles[3] = new gp_Pnt(0.2, 0, 0);
        aPoles[4] = new gp_Pnt(2, 0, 0);
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE, new Geom_BezierCurve(aPoles), Tol);
        TopoDS_Vertex aV1 = new(), aV2 = new();
        aB.MakeVertex(ref aV1, new gp_Pnt(0, 0, 0), Tol);
        aB.MakeVertex(ref aV2, new gp_Pnt(2, 0, 0), Tol);
        aB.Add(anE, aV1.Oriented(TopAbs_FORWARD));
        aB.Add(anE, aV2.Oriented(TopAbs_REVERSED));
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), aFace, Tol);
        aB.Range(anE, aFace, 0.0, 2.0);
        aB.Range(anE, 0.0, 1.0, true);
        aB.SameRange(anE, false);
        aB.SameParameter(anE, theSameParameter);
        return (anE, aFace);
    }

    [Fact]
    public void Perform_ProjectsWhenNotSameParameter()
    {
        (TopoDS_Edge anE, TopoDS_Face aFace) = BezierEdge(false);
        var aTool = new ShapeAnalysis_TransferParametersProj(anE, aFace);
        Assert.False(aTool.IsSameRange());
        // 3d to 2d: the pcurve parameter is the point's x
        Assert.Equal(BezierX(0.5), aTool.Perform(0.5, true), 1e-7);
        Assert.Equal(BezierX(0.25), aTool.Perform(0.25, true), 1e-7);
        // 2d to 3d: the inverse
        Assert.Equal(0.5, aTool.Perform(BezierX(0.5), false), 1e-6);

        var aParams = new NCollection_HSequence<double>();
        aParams.Append(0.25);
        aParams.Append(0.5);
        aParams.Append(0.75);
        NCollection_HSequence<double> aRes = aTool.Perform(aParams, true);
        Assert.Equal(3, aRes.Length());
        Assert.Equal(BezierX(0.25), aRes.Value(1), 1e-7);
        Assert.Equal(BezierX(0.5), aRes.Value(2), 1e-7);
        Assert.Equal(BezierX(0.75), aRes.Value(3), 1e-7);
    }

    [Fact]
    public void Perform_IsLinearOnSameParameterEdges_UnlessForced()
    {
        (TopoDS_Edge anE, TopoDS_Face aFace) = BezierEdge(true);
        var aTool = new ShapeAnalysis_TransferParametersProj(anE, aFace);
        // linear: [0, 1] to [0, 2]
        Assert.Equal(1.0, aTool.Perform(0.5, true), 1e-12);
        aTool.ForceProjection() = true;
        Assert.True(aTool.ForceProjection());
        Assert.Equal(BezierX(0.5), aTool.Perform(0.5, true), 1e-7);
    }

    [Fact]
    public void TransferRange_ProjectsTheEnds()
    {
        (TopoDS_Edge anE, TopoDS_Face aFace) = BezierEdge(false);
        (TopoDS_Edge aNew, _) = BezierEdge(false, aFace);

        var aTool = new ShapeAnalysis_TransferParametersProj(anE, aFace);
        aTool.TransferRange(aNew, 0.25, 0.75, false);
        BRep_Tool.Range(aNew, out double aF, out double aL);
        Assert.Equal(0.25, aF);
        Assert.Equal(0.75, aL);
        BRep_Tool.Range(aNew, aFace, out aF, out aL);
        Assert.Equal(BezierX(0.25), aF, 1e-7);
        Assert.Equal(BezierX(0.75), aL, 1e-7);
        Assert.False(BRep_Tool.SameRange(aNew));
    }

    [Fact]
    public void TransferRange_ThrowsOnAFrozenEdge()
    {
        (TopoDS_Edge anE, TopoDS_Face aFace) = BezierEdge(false);
        (TopoDS_Edge aNew, _) = BezierEdge(false);
        aNew.TShape()!.FreezeGraph();
        var aTool = new ShapeAnalysis_TransferParametersProj(anE, aFace);
        Assert.ThrowsAny<Exception>(() => aTool.TransferRange(aNew, 0.25, 0.75, false));
    }

    /// <summary>An unfrozen straight edge from (0, 0, 0) to (4, 0, 0), built with BRep_Builder.</summary>
    private static TopoDS_Edge LineEdge()
    {
        var aB = new BRep_Builder();
        TopoDS_Vertex aV1 = new(), aV2 = new();
        aB.MakeVertex(ref aV1, new gp_Pnt(0, 0, 0), Tol);
        aB.MakeVertex(ref aV2, new gp_Pnt(4, 0, 0), Tol);
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE, new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), Tol);
        aB.Add(anE, aV1.Oriented(TopAbs_FORWARD));
        aB.Add(anE, aV2.Oriented(TopAbs_REVERSED));
        aB.Range(anE, 0.0, 4.0);
        return anE;
    }

    [Fact]
    public void CopyNMVertex_CopiesAnInternalVertexFrozen()
    {
        var aB = new BRep_Builder();
        TopoDS_Edge anE = LineEdge();
        TopoDS_Edge aTo = LineEdge();
        TopoDS_Vertex aV = new();
        aB.MakeVertex(ref aV, new gp_Pnt(1, 0, 0), Tol);
        aB.UpdateVertex(aV, 1.0, anE, Tol);
        TopoDS_Vertex anInternal = TopoDS.Vertex(aV.Oriented(TopAbs_INTERNAL));
        anE.TShape()!.FreezeGraph();
        aV.TShape()!.FreezeGraph();

        TopoDS_Vertex aCopy = ShapeAnalysis_TransferParametersProj.CopyNMVertex(anInternal, aTo, anE);
        Assert.False(aCopy.IsNull());
        Assert.False(aCopy.IsSame(aV));
        Assert.True(aCopy.TShape()!.IsFrozen());
        Assert.True(BRep_Tool.Pnt(aCopy).IsEqual(new gp_Pnt(1, 0, 0), 0.0));
        Assert.True(BRep_Tool.Parameter(aCopy, aTo, out double aPar));
        Assert.Equal(1.0, aPar, 1e-12);

        // a FORWARD vertex isn't copied
        Assert.True(ShapeAnalysis_TransferParametersProj.CopyNMVertex(aV, aTo, anE).IsNull());

        // the target edge must be writable: BRep_Builder.UpdateVertex refuses a locked (frozen) edge, as OCCT does
        aTo.TShape()!.FreezeGraph();
        Assert.Throws<TopoDS_LockedShape>(() => ShapeAnalysis_TransferParametersProj.CopyNMVertex(anInternal, aTo, anE));
    }
}
