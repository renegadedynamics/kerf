// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Known answers of ShapeAnalysis_Edge on primitives' edges and on hand-built bad edges.</summary>
public class ShapeAnalysisEdgeTests
{
    private const double Conf = 1e-7;

    private static PlaneSurface XOY() => new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    private static IEnumerable<(Edge E, Face F)> EdgesOnFaces(in Shape theShape)
    {
        var aList = new List<(Edge, Face)>();
        for (var aFExp = new Explorer(theShape, ShapeType.Face); aFExp.More(); aFExp.Next())
        {
            Face aF = ShapeCasts.Face(aFExp.Current());
            for (var anEExp = new Explorer(aF, ShapeType.Edge); anEExp.More(); anEExp.Next())
            {
                aList.Add((ShapeCasts.Edge(anEExp.Current()), aF));
            }
        }
        return aList;
    }

    [Fact]
    public void Box_edges_are_consistent_with_their_faces()
    {
        Shape aBox = new MakeBox(10, 20, 30).Shape();
        var sae = new EdgeAnalysis();
        int aCount = 0;
        foreach ((Edge anE, Face aF) in EdgesOnFaces(aBox))
        {
            aCount++;
            Assert.True(sae.HasCurve3d(anE));
            Assert.True(sae.HasPCurve(anE, aF));
            Assert.False(sae.IsSeam(anE, aF));
            Assert.False(sae.IsClosed3d(anE));

            Assert.False(sae.CheckVerticesWithCurve3d(anE));
            Assert.True(sae.Status(ExtendStatus.OK));
            Assert.False(sae.CheckVerticesWithPCurve(anE, aF));
            Assert.True(sae.Status(ExtendStatus.OK));
            // a plane: OCCT doesn't compare the curves
            Assert.False(sae.CheckCurve3dWithPCurve(anE, aF));

            double aMaxDev = -1;
            Assert.False(sae.CheckSameParameter(anE, aF, ref aMaxDev));
            Assert.True(aMaxDev >= 0 && aMaxDev < 1e-9);
            Assert.True(sae.Status(ExtendStatus.OK));
            aMaxDev = -1;
            Assert.False(sae.CheckSameParameter(anE, ref aMaxDev));
            Assert.True(aMaxDev < 1e-9);

            Assert.False(sae.CheckVertexTolerance(anE, aF, out double aTol1, out double aTol2));
            Assert.Equal(Tool.Tolerance(anE), aTol1);
            Assert.Equal(Tool.Tolerance(anE), aTol2);
            Assert.False(sae.CheckVertexTolerance(anE, out aTol1, out aTol2));

            // the pcurve's ends are the vertices' points on the surface, in the edge's direction
            Assert.True(sae.BoundUV(anE, aF, out Point2D aUV1, out Point2D aUV2));
            Surface aS = Tool.Surface(aF)!;
            Assert.True(aS.Value(aUV1.X, aUV1.Y).DistanceTo(Tool.Pnt(sae.FirstVertex(anE))) < 1e-9);
            Assert.True(aS.Value(aUV2.X, aUV2.Y).DistanceTo(Tool.Pnt(sae.LastVertex(anE))) < 1e-9);

            // the tangent at the start points from the first UV point to the last (straight pcurves)
            Assert.True(sae.GetEndTangent2d(anE, aF, false, out Point2D aPos, out Vector2D aTan));
            Assert.True(aPos.DistanceTo(aUV1) < 1e-12);
            Assert.True(aTan.IsParallel(new Vector2D(aUV1, aUV2), 1e-12) && aTan * new Vector2D(aUV1, aUV2) > 0);
            Assert.True(sae.GetEndTangent2d(anE, aF, true, out aPos, out aTan, 0.1));
            Assert.True(aPos.DistanceTo(aUV2) < 1e-12);
            Assert.True(aTan * new Vector2D(aUV1, aUV2) > 0);
        }
        Assert.Equal(24, aCount);
    }

    [Fact]
    public void Orientation_swaps_ranges_and_vertices()
    {
        Edge anE = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Edge aRev = ShapeCasts.Edge(anE.Reversed());
        var sae = new EdgeAnalysis();

        Assert.True(sae.Curve3d(anE, out Curve3D? aC, out double f, out double l));
        Assert.NotNull(aC);
        Assert.Equal(0.0, f);
        Assert.Equal(10.0, l);
        Assert.True(sae.Curve3d(aRev, out _, out f, out l));
        Assert.Equal(10.0, f);
        Assert.Equal(0.0, l);
        Assert.True(sae.Curve3d(aRev, out _, out f, out l, false));
        Assert.Equal(0.0, f);
        Assert.Equal(10.0, l);

        Vertex aV1 = sae.FirstVertex(aRev);
        Vertex aV2 = sae.LastVertex(aRev);
        Assert.Equal(10.0, Tool.Pnt(aV1).X);
        Assert.Equal(0.0, Tool.Pnt(aV2).X);
        Assert.Equal(Forward, aV1.Orientation);
        Assert.Equal(Reversed, aV2.Orientation);
    }

    [Fact]
    public void Curve3d_applies_the_location()
    {
        var aT = new Transform3D();
        aT = aT.WithTranslation(new Vector3D(0, 0, 5));
        Edge anE = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Edge aMoved = ShapeCasts.Edge(anE.Moved(new Placement(aT)));
        var sae = new EdgeAnalysis();
        Assert.True(sae.Curve3d(aMoved, out Curve3D? aC, out double f, out double l));
        Assert.Equal(5.0, aC!.Value(f).Z);
        Assert.Equal(5.0, aC.Value(l).Z);
    }

    [Fact]
    public void Cylinder_seam_has_two_pcurves()
    {
        Shape aCyl = new ApiMakeCylinder(2, 5).Shape();
        var sae = new EdgeAnalysis();
        int aSeams = 0;
        foreach ((Edge anE, Face aF) in EdgesOnFaces(aCyl))
        {
            if (!sae.IsSeam(anE, aF))
            {
                Assert.False(sae.CheckVerticesWithPCurve(anE, aF));
                continue;
            }
            aSeams++;
            Assert.True(sae.BoundUV(anE, aF, out Point2D aF1, out _));
            Assert.True(sae.BoundUV(ShapeCasts.Edge(anE.Reversed()), aF, out Point2D aR1, out _));
            Assert.Equal(2 * Math.PI, Math.Abs(aF1.X - aR1.X), 1e-12);
            Assert.False(sae.CheckCurve3dWithPCurve(anE, aF));
            Assert.True(sae.Status(ExtendStatus.OK));
            double aMaxDev = 0;
            Assert.False(sae.CheckSameParameter(anE, aF, ref aMaxDev));
            Assert.True(aMaxDev < 1e-7);
        }
        Assert.Equal(2, aSeams); // the seam, FORWARD and REVERSED in the lateral face's wire
    }

    [Fact]
    public void Sphere_degenerated_edges()
    {
        Shape aSph = new MakeSphere(3).Shape();
        var sae = new EdgeAnalysis();
        int aDegenerated = 0;
        foreach ((Edge anE, Face aF) in EdgesOnFaces(aSph))
        {
            if (!Tool.Degenerated(anE))
            {
                continue;
            }
            aDegenerated++;
            Assert.False(sae.HasCurve3d(anE));
            Assert.True(sae.HasPCurve(anE, aF));
            double aMaxDev = 5;
            Assert.False(sae.CheckSameParameter(anE, aF, ref aMaxDev));
            Assert.Equal(5.0, aMaxDev); // unchanged for a degenerated edge
            // no 3D curve, but degenerated: no FAIL2
            Assert.False(sae.CheckVertexTolerance(anE, aF, out double aTol1, out _));
            Assert.False(sae.Status(ExtendStatus.FAIL2));
            Assert.Equal(Tool.Tolerance(anE), aTol1);
            Assert.False(sae.CheckVerticesWithCurve3d(anE));
            Assert.True(sae.Status(ExtendStatus.FAIL1));
        }
        Assert.Equal(2, aDegenerated);
    }

    [Fact]
    public void Vertices_off_the_curve_are_reported()
    {
        // a segment from (0,0,0) to (10,0,0) whose vertices are 0.01 and 0.02 off, tolerance 1e-7
        Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0.01, 0));
        Vertex aV2 = TestShapes.Vertex(new Point3D(10, 0, 0.02));
        Edge anE = TestShapes.Edge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10, aV1, aV2);
        var sae = new EdgeAnalysis();

        Assert.True(sae.CheckVerticesWithCurve3d(anE));
        Assert.True(sae.Status(ExtendStatus.DONE1));
        Assert.True(sae.Status(ExtendStatus.DONE2));
        Assert.True(sae.CheckVerticesWithCurve3d(anE, -1, 1));
        Assert.True(sae.Status(ExtendStatus.DONE1));
        Assert.False(sae.Status(ExtendStatus.DONE2));
        Assert.True(sae.CheckVerticesWithCurve3d(anE, 0.015));
        Assert.False(sae.Status(ExtendStatus.DONE1));
        Assert.True(sae.Status(ExtendStatus.DONE2));
        Assert.False(sae.CheckVerticesWithCurve3d(anE, 0.03));

        Assert.True(sae.CheckVertexTolerance(anE, out double aTol1, out double aTol2));
        Assert.True(sae.Status(ExtendStatus.DONE1) && sae.Status(ExtendStatus.DONE2));
        Assert.Equal(1.0000001 * Math.Sqrt(0.01 * 0.01), aTol1);
        Assert.Equal(1.0000001 * Math.Sqrt(0.02 * 0.02), aTol2);
    }

    [Fact]
    public void Edge_without_3d_curve_fails()
    {
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        Edge anE = new();
        aB.MakeEdge(ref anE);
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        aB.Add(anE, TestShapes.Vertex(new Point3D(0, 0, 0)).Oriented(Forward));
        aB.Add(anE, TestShapes.Vertex(new Point3D(4, 0, 0)).Oriented(Reversed));
        aB.Range(anE, aF, 0, 4);

        var sae = new EdgeAnalysis();
        Assert.False(sae.HasCurve3d(anE));
        Assert.False(sae.Curve3d(anE, out Curve3D? aC, out _, out _));
        Assert.Null(aC);
        Assert.False(sae.CheckVerticesWithCurve3d(anE));
        Assert.True(sae.Status(ExtendStatus.FAIL1));
        double aMaxDev = 0;
        Assert.False(sae.CheckSameParameter(anE, ref aMaxDev));
        Assert.True(sae.Status(ExtendStatus.FAIL1));
        Assert.False(sae.CheckVertexTolerance(anE, aF, out _, out _));
        Assert.True(sae.Status(ExtendStatus.FAIL2));
        Assert.False(sae.CheckVerticesWithPCurve(anE, aF));
        Assert.True(sae.Status(ExtendStatus.OK));

        // no pcurve on another plane
        var aB2 = new BrepBuilder();
        Face aG = new();
        aB2.MakeFace(ref aG, new PlaneSurface(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), Conf);
        Assert.False(sae.HasPCurve(anE, aG));
        Assert.False(sae.PCurve(anE, aG, out Curve2D? aPC, out _, out _));
        Assert.Null(aPC);
        Assert.False(sae.BoundUV(anE, aG, out _, out _));
        Assert.False(sae.GetEndTangent2d(anE, aG, false, out _, out Vector2D aTan));
        Assert.Equal(0.0, aTan.Magnitude);
        Assert.False(sae.CheckVerticesWithPCurve(anE, aG));
        Assert.True(sae.Status(ExtendStatus.FAIL1));
    }

    [Fact]
    public void Same_parameter_deviation_is_measured()
    {
        // a segment on the plane z = 0 whose pcurve runs along it at another speed (pcurve over the same range [0, 10],
        // but a parabola in the parameter) and isn't flagged SameParameter
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var aPC = new BSplineCurve2D(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(2, 0), new Point2D(10, 0)]),
                                          NCollection_Array1<double>.From([0.0, 10.0]), NCollection_Array1<int>.From([3, 3]), 2);
        aB.UpdateEdge(anE, aPC, aF, Conf);
        aB.SameParameter(anE, false);

        var sae = new EdgeAnalysis();
        double aMaxDev = 0;
        // not flagged SameParameter: BRepLib_ValidateEdge measures the distance to the 3D curve (by projection), which
        // is 0 here, and the flag gives DONE2
        Assert.True(sae.CheckSameParameter(anE, aF, ref aMaxDev));
        Assert.False(sae.Status(ExtendStatus.DONE1));
        Assert.True(sae.Status(ExtendStatus.DONE2));
        Assert.True(aMaxDev < 1e-7);

        // flagged SameParameter: the deviation at equal parameters, largest between t and the B-spline's
        // x(t) = 0.4 t + 0.06 t^2 over [0, 10], at t = 5: 1.5
        aB.SameParameter(anE, true);
        Assert.True(sae.CheckSameParameter(anE, aF, ref aMaxDev));
        Assert.True(sae.Status(ExtendStatus.DONE1));
        Assert.False(sae.Status(ExtendStatus.DONE2));
        Assert.Equal(1.5, aMaxDev, 1e-2);

        // a plane is skipped
        Assert.False(sae.CheckCurve3dWithPCurve(anE, aF));
    }

    [Fact]
    public void Curve3d_against_pcurve_on_a_cylinder()
    {
        // an arc on a cylinder whose pcurve runs the other way: the ends match reversed (DONE1)
        var aCylS = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1);
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, aCylS, Conf);
        var aCircle = new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1);
        Edge anE = TestShapes.Edge(aCircle, 0, 1);
        var sae = new EdgeAnalysis();

        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        Assert.False(sae.CheckCurve3dWithPCurve(anE, aF));
        Assert.True(sae.Status(ExtendStatus.OK));

        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(1, 0), new Direction2D(-1, 0)), aF, Conf);
        Assert.True(sae.CheckCurve3dWithPCurve(anE, aF));
        Assert.True(sae.Status(ExtendStatus.DONE1));

        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 3), new Direction2D(1, 0)), aF, Conf);
        Assert.True(sae.CheckCurve3dWithPCurve(anE, aF));
        Assert.False(sae.Status(ExtendStatus.DONE1));
    }

    [Fact]
    public void CheckPCurveRange_of_lines_circles_and_trimmed_curves()
    {
        var sae = new EdgeAnalysis();
        var aLine = new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0));
        Assert.True(sae.CheckPCurveRange(-1e10, 1e10, aLine));

        var aCircle = new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);
        Assert.True(sae.CheckPCurveRange(-1, 2 * Math.PI - 1, aCircle));
        Assert.True(sae.CheckPCurveRange(0, 2 * Math.PI + 0.5e-9, aCircle));
        Assert.False(sae.CheckPCurveRange(0, 2 * Math.PI + 1e-6, aCircle));

        // a trimmed circle: its basis is periodic
        var aTrimmed = new TrimmedCurve2D(aCircle, 0, 1);
        Assert.True(sae.CheckPCurveRange(-3, 3, aTrimmed));
        Assert.False(sae.CheckPCurveRange(-4, 4, aTrimmed));

        // a trimmed segment of a B-spline over [0, 1]: the basis's range counts, not the trimming
        var aBS = new BSplineCurve2D(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(1, 1)]),
                                          NCollection_Array1<double>.From([0.0, 1.0]), NCollection_Array1<int>.From([2, 2]), 1);
        var aTrimmedBS = new TrimmedCurve2D(aBS, 0.2, 0.4);
        Assert.True(sae.CheckPCurveRange(0, 1, aTrimmedBS));
        Assert.False(sae.CheckPCurveRange(0, 1.1, aTrimmedBS));
        Assert.False(sae.CheckPCurveRange(-0.1, 0.5, aBS));
    }

    [Fact]
    public void CheckOverlapping_of_collinear_and_parallel_segments()
    {
        Edge aLong = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        Edge aShort = new MakeEdge(new Point3D(2, 0, 0), new Point3D(8, 0, 0)).Edge();
        Edge aParallel = new MakeEdge(new Point3D(0, 1, 0), new Point3D(10, 1, 0)).Edge();
        Edge aCrossing = new MakeEdge(new Point3D(5, -5, 0), new Point3D(5, 5, 0)).Edge();

        var sae = new EdgeAnalysis();
        double aTol = 1e-3;
        Assert.True(sae.CheckOverlapping(aLong, aShort, ref aTol));
        Assert.True(sae.Status(ExtendStatus.DONE3));
        Assert.Equal(1e-3, aTol); // unchanged when the whole edge overlaps

        var sae2 = new EdgeAnalysis();
        aTol = 1e-3;
        Assert.False(sae2.CheckOverlapping(aLong, aParallel, ref aTol));
        Assert.Equal(1e-3, aTol);
        Assert.False(sae2.CheckOverlapping(aLong, aParallel, ref aTol, 1.0));
        Assert.True(sae2.Status(ExtendStatus.OK));

        // crossing segments touch at one point: over a domain of 0.001 around it they are within 1.0 of each other
        aTol = 1.0;
        Assert.True(sae2.CheckOverlapping(aLong, aCrossing, ref aTol, 0.001));
        Assert.True(sae2.Status(ExtendStatus.DONE4));
        Assert.True(aTol < 1e-9); // the distance found
    }

    /// <summary>
    /// A zero-length edge whose vertex lies on the other edge: OCCT sampled the shorter edge with a step of half its
    /// length, 0 here, and never left the loop (occt-issues.md, unit 4S's ShapeAnalysis_Edge row; Kerf unit 10K1, decision
    /// 16504). The edge's one point is checked: it overlaps.
    /// </summary>
    [Fact]
    public void CheckOverlapping_with_a_zero_length_edge_returns()
    {
        Edge aLong = new MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        var b = new BrepBuilder();
        Vertex aV = new();
        b.MakeVertex(ref aV, new Point3D(5, 0, 0), Conf);
        Edge aZero = new();
        b.MakeEdge(ref aZero, new LineCurve3D(new Point3D(5, 0, 0), new Direction3D(1, 0, 0)), Conf);
        b.Add(aZero, aV.Oriented(Forward));
        b.Add(aZero, aV.Oriented(Reversed));
        b.Range(aZero, 0.0, 0.0);

        var sae = new EdgeAnalysis();
        double aTol = 1e-3;
        Assert.True(HangGuard.Within(TimeSpan.FromSeconds(20), () => sae.CheckOverlapping(aLong, aZero, ref aTol)));
        Assert.True(sae.Status(ExtendStatus.DONE3));

        // and away from the other edge, it doesn't
        Vertex aFar = new();
        b.MakeVertex(ref aFar, new Point3D(5, 1, 0), Conf);
        Edge aZeroFar = new();
        b.MakeEdge(ref aZeroFar, new LineCurve3D(new Point3D(5, 1, 0), new Direction3D(1, 0, 0)), Conf);
        b.Add(aZeroFar, aFar.Oriented(Forward));
        b.Add(aZeroFar, aFar.Oriented(Reversed));
        b.Range(aZeroFar, 0.0, 0.0);
        aTol = 1e-3;
        Assert.False(HangGuard.Within(TimeSpan.FromSeconds(20), () => new EdgeAnalysis().CheckOverlapping(aLong, aZeroFar, ref aTol)));
    }

    [Fact]
    public void PCurve_and_HasPCurve_with_surface_and_location()
    {
        Shape aBox = new MakeBox(1, 1, 1).Shape();
        var sae = new EdgeAnalysis();
        foreach ((Edge anE, Face aF) in EdgesOnFaces(aBox))
        {
            Surface aS = Tool.Surface(aF, out Placement aL)!;
            Assert.True(sae.HasPCurve(anE, aS, aL));
            Assert.True(sae.PCurve(anE, aS, aL, out Curve2D? aPC1, out double f1, out double l1));
            Assert.True(sae.PCurve(anE, aF, out Curve2D? aPC2, out double f2, out double l2));
            Assert.Same(aPC1, aPC2);
            Assert.Equal(f1, f2);
            Assert.Equal(l1, l2);
            Assert.True(sae.PCurve(anE, aF, out _, out double f3, out double l3, false));
            if (anE.Orientation == Reversed)
            {
                Assert.Equal(f1, l3);
                Assert.Equal(l1, f3);
            }
            else
            {
                Assert.Equal(f1, f3);
                Assert.Equal(l1, l3);
            }
        }
    }
}
