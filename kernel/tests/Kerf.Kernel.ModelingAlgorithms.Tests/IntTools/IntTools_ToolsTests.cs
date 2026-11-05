// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>IntTools_Tools: known answers for each static.</summary>
public class IntTools_ToolsTests
{
    private static Vertex Vertex(double theX, double theY, double theZ, double theTol)
    {
        Vertex aV = new MakeVertex(new Point3D(theX, theY, theZ)).Vertex();
        Vertex aCopy = ShapeCasts.Vertex(TestCopies.ExactCopyForWrite(aV, out _));
        new BrepBuilder().UpdateVertex(aCopy, theTol);
        return aCopy;
    }

    private static Edge Segment(double theX1, double theX2) =>
        new MakeEdge(new Point3D(theX1, 0, 0), new Point3D(theX2, 0, 0)).Edge();

    [Fact]
    public void ComputeVV_compares_with_the_sum_of_tolerances()
    {
        Assert.Equal(0, IntersectionTools.ComputeVV(Vertex(0, 0, 0, 0.1), Vertex(0.15, 0, 0, 0.1)));
        Assert.Equal(-1, IntersectionTools.ComputeVV(Vertex(0, 0, 0, 0.1), Vertex(0.25, 0, 0, 0.1)));
    }

    [Fact]
    public void IsVertex_overloads()
    {
        Edge aE = Segment(0, 10);
        Assert.True(IntersectionTools.IsVertex(aE, 0.0));
        Assert.True(IntersectionTools.IsVertex(aE, 10.0));
        Assert.False(IntersectionTools.IsVertex(aE, 1.0e-5)); // 1e-6 squared, whatever the vertex's tolerance
        Vertex aV = Vertex(5, 0, 0, 0.5);
        Assert.True(IntersectionTools.IsVertex(aE, aV, 5.4));
        Assert.False(IntersectionTools.IsVertex(aE, aV, 5.6));
        Assert.True(IntersectionTools.IsVertex(new Point3D(5.7, 0, 0), 0.2, aV));
        Assert.False(IntersectionTools.IsVertex(new Point3D(5.8, 0, 0), 0.2, aV));

        CommonPrt aCP = new();
        aCP.SetEdge1(aE);
        aCP.SetEdge2(Segment(-3, 3));
        aCP.SetRange1(2, 4);
        aCP.AppendRange2(2, 4);
        Assert.False(IntersectionTools.IsVertex(aCP)); // middles 3 on both edges (x = 3 and x = 0)
        aCP.SetRange1(-1, 1);
        Assert.True(IntersectionTools.IsVertex(aCP)); // middle 0 on aE is its vertex
    }

    [Fact]
    public void HasInternalEdge_MakeFaceFromWireAndFace_ClassifyPointByFace()
    {
        Face aF = ShapeCasts.Face(new Explorer(new MakeBox(2, 2, 2).Shape(), ShapeType.Face).Current());
        Wire aW = ShapeCasts.Wire(new Explorer(aF, ShapeType.Wire).Current());
        Assert.False(IntersectionTools.HasInternalEdge(aW));
        Wire aWI = new();
        BrepBuilder aBB = new();
        aBB.MakeWire(ref aWI);
        aBB.Add(aWI, Segment(0, 1).Oriented(Internal));
        Assert.True(IntersectionTools.HasInternalEdge(aWI));

        IntersectionTools.MakeFaceFromWireAndFace(aW, ShapeCasts.Face(aF.Reversed()), out Face aFNew);
        Assert.Equal(Forward, aFNew.Orientation);
        Assert.Same(Tool.Surface(aF), Tool.Surface(aFNew));
        Assert.False(aFNew.IsSame(aF));
        Assert.Equal(1, new Iterator(aFNew).More() ? 1 : 0);

        FaceSurfaceAdaptor aBAS = new(aF);
        double aUm = 0.5 * (aBAS.FirstUParameter() + aBAS.LastUParameter());
        double aVm = 0.5 * (aBAS.FirstVParameter() + aBAS.LastVParameter());
        Assert.Equal(In, IntersectionTools.ClassifyPointByFace(aF, new Point2D(aUm, aVm)));
        Assert.Equal(Out, IntersectionTools.ClassifyPointByFace(aF, new Point2D(aUm + 10, aVm)));
    }

    [Fact]
    public void IsMiddlePointsEqual()
    {
        Assert.True(IntersectionTools.IsMiddlePointsEqual(Segment(0, 10), Segment(0, 10)));
        Assert.False(IntersectionTools.IsMiddlePointsEqual(Segment(0, 10), Segment(0, 12)));
    }

    [Fact]
    public void IsDirsCoinside_equal_or_opposite()
    {
        Direction3D aX = new(1, 0, 0);
        Assert.True(IntersectionTools.IsDirsCoinside(aX, new Direction3D(-1, 0, 0)));
        Assert.True(IntersectionTools.IsDirsCoinside(aX, new Direction3D(1, 1.0e-4, 0)));
        Assert.False(IntersectionTools.IsDirsCoinside(aX, new Direction3D(1, 1.0e-3, 0)));
        Assert.True(IntersectionTools.IsDirsCoinside(aX, new Direction3D(1, 1.0e-3, 0), 0.01));
        Assert.False(IntersectionTools.IsDirsCoinside(aX, new Direction3D(0, 1, 0), 0.01));
    }

    [Fact]
    public void IsClosed_SplitCurve_RejectLines()
    {
        CircleCurve3D aCircle = new(new Frame3D(), 2.0);
        Assert.False(IntersectionTools.IsClosed(aCircle)); // not bounded
        TrimmedCurve3D aClosed = new(aCircle, 0, 2 * Math.PI);
        Assert.True(IntersectionTools.IsClosed(aClosed));
        Assert.False(IntersectionTools.IsClosed(new TrimmedCurve3D(aCircle, 0, 3)));

        NCollection_Sequence<IntersectionCurve> aCvs = new();
        Assert.Equal(0, IntersectionTools.SplitCurve(new IntersectionCurve(new TrimmedCurve3D(aCircle, 0, 3), null, null, 1e-7, 1e-7), aCvs));
        Assert.Equal(2, IntersectionTools.SplitCurve(new IntersectionCurve(aClosed, null, null, 1e-6, 1e-5), aCvs));
        Assert.Equal(Math.PI, aCvs.Value(1).Curve()!.LastParameter);
        Assert.Equal(Math.PI, aCvs.Value(2).Curve()!.FirstParameter);
        Assert.Equal(1e-6, aCvs.Value(2).Tolerance());
        Assert.Equal(1e-5, aCvs.Value(2).TangentialTolerance());

        // lines: the first one, then the first non-parallel one
        LineCurve3D aL1 = new(new Point3D(), new Direction3D(1, 0, 0));
        LineCurve3D aL2 = new(new Point3D(0, 1, 0), new Direction3D(-1, 0, 0));
        LineCurve3D aL3 = new(new Point3D(), new Direction3D(0, 1, 0));
        NCollection_Sequence<IntersectionCurve> aIn = new();
        aIn.Append(new IntersectionCurve(new TrimmedCurve3D(aL1, 0, 1), null, null, 0, 0));
        aIn.Append(new IntersectionCurve(aL2, null, null, 0, 0));
        aIn.Append(new IntersectionCurve(aL3, null, null, 0, 0));
        NCollection_Sequence<IntersectionCurve> aOut = new();
        IntersectionTools.RejectLines(aIn, aOut);
        Assert.Equal(2, aOut.Length());
        // the trimmed line is replaced by its basis (a copy of aL1, as Geom_TrimmedCurve copies), in the input too (OCCT)
        Assert.IsType<LineCurve3D>(aOut.Value(1).Curve());
        Assert.Same(aOut.Value(1).Curve(), aIn.Value(1).Curve());
        Assert.Same(aL3, aOut.Value(2).Curve());
        // a non-line before a non-parallel line: everything
        NCollection_Sequence<IntersectionCurve> aIn2 = new();
        aIn2.Append(new IntersectionCurve(aL1, null, null, 0, 0));
        aIn2.Append(new IntersectionCurve(aCircle, null, null, 0, 0));
        aIn2.Append(new IntersectionCurve(aL3, null, null, 0, 0));
        IntersectionTools.RejectLines(aIn2, aOut);
        Assert.Equal(3, aOut.Length());
    }

    [Fact]
    public void CurveTolerance_of_a_trimmed_parabola()
    {
        ParabolaCurve3D aP = new(new Frame3D(), 0.5);
        Assert.Equal(1e-3, IntersectionTools.CurveTolerance(aP, 1e-3)); // not trimmed
        Assert.Equal(1e-3, IntersectionTools.CurveTolerance(null, 1e-3));
        // at parameter t the point is (t^2/(4f), t): its projection on the axis is x = t^2/(4f) = t^2/2
        double aTol = IntersectionTools.CurveTolerance(new TrimmedCurve3D(aP, -1, 2), 1e-3);
        Assert.Equal(1e-3 * Math.Sqrt(0.5 * 2.0 / 0.5), aTol, 1e-15);
        Assert.Equal(1e-3, IntersectionTools.CurveTolerance(new TrimmedCurve3D(new LineCurve3D(new Point3D(), new Direction3D(1, 0, 0)), 0, 1), 1e-3));
    }

    [Fact]
    public void CheckCurve_rejects_short_curves()
    {
        LineCurve3D aL = new(new Point3D(), new Direction3D(1, 0, 0));
        Box aBox = new();
        Assert.True(IntersectionTools.CheckCurve(new IntersectionCurve(new TrimmedCurve3D(aL, 0, 1), null, null, 1e-7, 1e-7), ref aBox));
        Assert.False(aBox.IsVoid);
        aBox = new Box();
        Assert.False(IntersectionTools.CheckCurve(new IntersectionCurve(new TrimmedCurve3D(aL, 0, 1e-8), null, null, 1e-7, 1e-7), ref aBox));
        aBox = new Box();
        Assert.False(IntersectionTools.CheckCurve(new IntersectionCurve(), ref aBox));
    }

    [Fact]
    public void Ranges_and_paves()
    {
        ParameterRange aR = new(1, 2);
        Assert.True(IntersectionTools.IsOnPave(1.05, aR, 0.1));
        Assert.False(IntersectionTools.IsOnPave(1.5, aR, 0.1));
        Assert.True(IntersectionTools.IsOnPave1(1.5, aR, 0.0));
        Assert.True(IntersectionTools.IsOnPave1(2.05, aR, 0.1));
        Assert.False(IntersectionTools.IsOnPave1(2.2, aR, 0.1));
        Assert.True(IntersectionTools.IsInRange(aR, new ParameterRange(1.9, 5), 0.0));
        Assert.True(IntersectionTools.IsInRange(aR, new ParameterRange(2.05, 5), 0.1));
        Assert.False(IntersectionTools.IsInRange(aR, new ParameterRange(0, 5), 0.1)); // OCCT tests only the ends

        CommonPrt aCP = new();
        aCP.SetRange1(0, 2);
        aCP.AppendRange2(4, 8);
        aCP.SetVertexParameter1(0.5);
        aCP.SetVertexParameter2(9);
        IntersectionTools.VertexParameters(aCP, out double aT1, out double aT2);
        Assert.Equal(0.5, aT1);
        Assert.Equal(6.0, aT2);
        IntersectionTools.VertexParameter(aCP, out double aT);
        Assert.Equal(0.5, aT);
        aCP.SetVertexParameter1(3);
        IntersectionTools.VertexParameter(aCP, out aT);
        Assert.Equal(1.0, aT);
    }

    [Fact]
    public void SegPln_return_codes()
    {
        Line3D aLin = new(new Point3D(0, 0, -1), new Direction3D(0, 0, 1));
        Plane3D aPln = new(new Point3D(0, 0, 0.5), new Direction3D(0, 0, 1));
        Point3D aP = new();
        double aT = 0, aTolP = 0, aTmin = 0, aTmax = 0;
        Assert.Equal(0, IntersectionTools.SegPln(aLin, 0, 3, 1e-3, aPln, 1e-2, ref aP, ref aT, ref aTolP, ref aTmin, ref aTmax));
        Assert.Equal(1.5, aT);
        Assert.Equal(0.5, aP.Z);
        Assert.Equal(1e-3 + 1e-2, aTolP);
        Assert.Equal(1.5 - 1e-2, aTmin);
        Assert.Equal(1.5 + 1e-2, aTmax);
        Assert.Equal(2, IntersectionTools.SegPln(aLin, 2, 3, 1e-3, aPln, 1e-2, ref aP, ref aT, ref aTolP, ref aTmin, ref aTmax));
        Assert.Equal(1, IntersectionTools.SegPln(aLin, 0, 1, 1e-3, aPln, 1e-2, ref aP, ref aT, ref aTolP, ref aTmin, ref aTmax));
        // in the plane: both signed distances below the tolerance
        Line3D aInPlane = new(new Point3D(0, 0, 0.5), new Direction3D(1, 0, 0));
        Assert.Equal(1, IntersectionTools.SegPln(aInPlane, 0, 3, 1e-3, aPln, 1e-2, ref aP, ref aT, ref aTolP, ref aTmin, ref aTmax));
    }

    [Fact]
    public void ComputeTolerance_of_a_curve_on_a_plane()
    {
        PlaneSurface aPlane = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        LineCurve3D aL3 = new(new Point3D(0, 0, 0.01), new Direction3D(1, 0, 0));
        LineCurve2D aL2 = new(new Point2D(0, 0), new Direction2D(1, 0));
        Assert.True(IntersectionTools.ComputeTolerance(aL3, aL2, aPlane, 0, 1, out double aD, out double aT));
        Assert.Equal(0.01 * (1.0 + 1.0e-5), aD, 1e-15);
        Assert.InRange(aT, 0.0, 1.0);
        Assert.True(IntersectionTools.ComputeTolerance(aL3, aL2, aPlane, 0, 1, out double aDP, out double aTP, global::Kerf.Engine.Foundation.Precision.PConfusion(), true));
        Assert.Equal(aD, aDP);
        Assert.Equal(aT, aTP);
    }

    [Fact]
    public void ComputeIntRange()
    {
        Assert.Equal(0.2, IntersectionTools.ComputeIntRange(0.1, 0.2, M_PI_2));
        double anA = Math.PI / 6;
        Assert.Equal(0.1 * Math.Tan(M_PI_2 - anA) + 0.2 / Math.Sin(anA), IntersectionTools.ComputeIntRange(0.1, 0.2, anA));
        Assert.Equal(IntersectionTools.ComputeIntRange(0.1, 0.2, anA), IntersectionTools.ComputeIntRange(0.1, 0.2, Math.PI - anA), 1e-15);
    }
}
