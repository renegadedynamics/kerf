// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

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

    private static (Edge Edge, Face Face) BezierEdge(bool theSameParameter, Face? theFace = null)
    {
        var aB = new BrepBuilder();
        Face aFace = theFace ?? new();
        if (theFace is null)
        {
            aB.MakeFace(ref aFace, new PlaneSurface(new Plane3D()), Tol);
        }

        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(0.1, 0, 0);
        aPoles[3] = new Point3D(0.2, 0, 0);
        aPoles[4] = new Point3D(2, 0, 0);
        Edge anE = new();
        aB.MakeEdge(ref anE, new BezierCurve3D(aPoles), Tol);
        Vertex aV1 = new(), aV2 = new();
        aB.MakeVertex(ref aV1, new Point3D(0, 0, 0), Tol);
        aB.MakeVertex(ref aV2, new Point3D(2, 0, 0), Tol);
        aB.Add(anE, aV1.Oriented(Forward));
        aB.Add(anE, aV2.Oriented(Reversed));
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aFace, Tol);
        aB.Range(anE, aFace, 0.0, 2.0);
        aB.Range(anE, 0.0, 1.0, true);
        aB.SameRange(anE, false);
        aB.SameParameter(anE, theSameParameter);
        return (anE, aFace);
    }

    [Fact]
    public void Perform_ProjectsWhenNotSameParameter()
    {
        (Edge anE, Face aFace) = BezierEdge(false);
        var aTool = new TransferParametersProjAnalysis(anE, aFace);
        Assert.False(aTool.IsSameRange);
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
        (Edge anE, Face aFace) = BezierEdge(true);
        var aTool = new TransferParametersProjAnalysis(anE, aFace);
        // linear: [0, 1] to [0, 2]
        Assert.Equal(1.0, aTool.Perform(0.5, true), 1e-12);
        aTool.ForceProjection() = true;
        Assert.True(aTool.ForceProjection());
        Assert.Equal(BezierX(0.5), aTool.Perform(0.5, true), 1e-7);
    }

    [Fact]
    public void TransferRange_ProjectsTheEnds()
    {
        (Edge anE, Face aFace) = BezierEdge(false);
        (Edge aNew, _) = BezierEdge(false, aFace);

        var aTool = new TransferParametersProjAnalysis(anE, aFace);
        aTool.TransferRange(aNew, 0.25, 0.75, false);
        Tool.Range(aNew, out double aF, out double aL);
        Assert.Equal(0.25, aF);
        Assert.Equal(0.75, aL);
        Tool.Range(aNew, aFace, out aF, out aL);
        Assert.Equal(BezierX(0.25), aF, 1e-7);
        Assert.Equal(BezierX(0.75), aL, 1e-7);
        Assert.False(Tool.SameRange(aNew));
    }

    [Fact]
    public void TransferRange_ThrowsOnAFrozenEdge()
    {
        (Edge anE, Face aFace) = BezierEdge(false);
        (Edge aNew, _) = BezierEdge(false);
        aNew.TShape()!.FreezeGraph();
        var aTool = new TransferParametersProjAnalysis(anE, aFace);
        Assert.ThrowsAny<Exception>(() => aTool.TransferRange(aNew, 0.25, 0.75, false));
    }

    /// <summary>An unfrozen straight edge from (0, 0, 0) to (4, 0, 0), built with BRep_Builder.</summary>
    private static Edge LineEdge()
    {
        var aB = new BrepBuilder();
        Vertex aV1 = new(), aV2 = new();
        aB.MakeVertex(ref aV1, new Point3D(0, 0, 0), Tol);
        aB.MakeVertex(ref aV2, new Point3D(4, 0, 0), Tol);
        Edge anE = new();
        aB.MakeEdge(ref anE, new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), Tol);
        aB.Add(anE, aV1.Oriented(Forward));
        aB.Add(anE, aV2.Oriented(Reversed));
        aB.Range(anE, 0.0, 4.0);
        return anE;
    }

    [Fact]
    public void CopyNMVertex_CopiesAnInternalVertexFrozen()
    {
        var aB = new BrepBuilder();
        Edge anE = LineEdge();
        Edge aTo = LineEdge();
        Vertex aV = new();
        aB.MakeVertex(ref aV, new Point3D(1, 0, 0), Tol);
        aB.UpdateVertex(aV, 1.0, anE, Tol);
        Vertex anInternal = ShapeCasts.Vertex(aV.Oriented(Internal));
        anE.TShape()!.FreezeGraph();
        aV.TShape()!.FreezeGraph();

        Vertex aCopy = TransferParametersProjAnalysis.CopyNMVertex(anInternal, aTo, anE);
        Assert.False(aCopy.IsNull);
        Assert.False(aCopy.IsSame(aV));
        Assert.True(aCopy.TShape()!.IsFrozen);
        Assert.True(Tool.Pnt(aCopy).IsEqual(new Point3D(1, 0, 0), 0.0));
        Assert.True(Tool.Parameter(aCopy, aTo, out double aPar));
        Assert.Equal(1.0, aPar, 1e-12);

        // a FORWARD vertex isn't copied
        Assert.True(TransferParametersProjAnalysis.CopyNMVertex(aV, aTo, anE).IsNull);

        // the target edge must be writable: BRep_Builder.UpdateVertex refuses a locked (frozen) edge, as OCCT does
        aTo.TShape()!.FreezeGraph();
        Assert.Throws<LockedShapeException>(() => TransferParametersProjAnalysis.CopyNMVertex(anInternal, aTo, anE));
    }
}
