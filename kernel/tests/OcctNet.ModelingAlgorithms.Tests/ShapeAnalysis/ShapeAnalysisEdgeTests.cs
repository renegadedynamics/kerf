// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Known answers of ShapeAnalysis_Edge on primitives' edges and on hand-built bad edges.</summary>
public class ShapeAnalysisEdgeTests
{
    private const double Conf = 1e-7;

    private static Geom_Plane XOY() => new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));

    private static IEnumerable<(TopoDS_Edge E, TopoDS_Face F)> EdgesOnFaces(in TopoDS_Shape theShape)
    {
        var aList = new List<(TopoDS_Edge, TopoDS_Face)>();
        for (var aFExp = new TopExp_Explorer(theShape, TopAbs_FACE); aFExp.More(); aFExp.Next())
        {
            TopoDS_Face aF = TopoDS.Face(aFExp.Current());
            for (var anEExp = new TopExp_Explorer(aF, TopAbs_EDGE); anEExp.More(); anEExp.Next())
            {
                aList.Add((TopoDS.Edge(anEExp.Current()), aF));
            }
        }
        return aList;
    }

    [Fact]
    public void Box_edges_are_consistent_with_their_faces()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(10, 20, 30).Shape();
        var sae = new ShapeAnalysis_Edge();
        int aCount = 0;
        foreach ((TopoDS_Edge anE, TopoDS_Face aF) in EdgesOnFaces(aBox))
        {
            aCount++;
            Assert.True(sae.HasCurve3d(anE));
            Assert.True(sae.HasPCurve(anE, aF));
            Assert.False(sae.IsSeam(anE, aF));
            Assert.False(sae.IsClosed3d(anE));

            Assert.False(sae.CheckVerticesWithCurve3d(anE));
            Assert.True(sae.Status(ShapeExtend_OK));
            Assert.False(sae.CheckVerticesWithPCurve(anE, aF));
            Assert.True(sae.Status(ShapeExtend_OK));
            // a plane: OCCT doesn't compare the curves
            Assert.False(sae.CheckCurve3dWithPCurve(anE, aF));

            double aMaxDev = -1;
            Assert.False(sae.CheckSameParameter(anE, aF, ref aMaxDev));
            Assert.True(aMaxDev >= 0 && aMaxDev < 1e-9);
            Assert.True(sae.Status(ShapeExtend_OK));
            aMaxDev = -1;
            Assert.False(sae.CheckSameParameter(anE, ref aMaxDev));
            Assert.True(aMaxDev < 1e-9);

            Assert.False(sae.CheckVertexTolerance(anE, aF, out double aTol1, out double aTol2));
            Assert.Equal(BRep_Tool.Tolerance(anE), aTol1);
            Assert.Equal(BRep_Tool.Tolerance(anE), aTol2);
            Assert.False(sae.CheckVertexTolerance(anE, out aTol1, out aTol2));

            // the pcurve's ends are the vertices' points on the surface, in the edge's direction
            Assert.True(sae.BoundUV(anE, aF, out Point2D aUV1, out Point2D aUV2));
            Geom_Surface aS = BRep_Tool.Surface(aF)!;
            Assert.True(aS.Value(aUV1.X(), aUV1.Y()).Distance(BRep_Tool.Pnt(sae.FirstVertex(anE))) < 1e-9);
            Assert.True(aS.Value(aUV2.X(), aUV2.Y()).Distance(BRep_Tool.Pnt(sae.LastVertex(anE))) < 1e-9);

            // the tangent at the start points from the first UV point to the last (straight pcurves)
            Assert.True(sae.GetEndTangent2d(anE, aF, false, out Point2D aPos, out Vector2D aTan));
            Assert.True(aPos.Distance(aUV1) < 1e-12);
            Assert.True(aTan.IsParallel(new Vector2D(aUV1, aUV2), 1e-12) && aTan * new Vector2D(aUV1, aUV2) > 0);
            Assert.True(sae.GetEndTangent2d(anE, aF, true, out aPos, out aTan, 0.1));
            Assert.True(aPos.Distance(aUV2) < 1e-12);
            Assert.True(aTan * new Vector2D(aUV1, aUV2) > 0);
        }
        Assert.Equal(24, aCount);
    }

    [Fact]
    public void Orientation_swaps_ranges_and_vertices()
    {
        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        TopoDS_Edge aRev = TopoDS.Edge(anE.Reversed());
        var sae = new ShapeAnalysis_Edge();

        Assert.True(sae.Curve3d(anE, out Geom_Curve? aC, out double f, out double l));
        Assert.NotNull(aC);
        Assert.Equal(0.0, f);
        Assert.Equal(10.0, l);
        Assert.True(sae.Curve3d(aRev, out _, out f, out l));
        Assert.Equal(10.0, f);
        Assert.Equal(0.0, l);
        Assert.True(sae.Curve3d(aRev, out _, out f, out l, false));
        Assert.Equal(0.0, f);
        Assert.Equal(10.0, l);

        TopoDS_Vertex aV1 = sae.FirstVertex(aRev);
        TopoDS_Vertex aV2 = sae.LastVertex(aRev);
        Assert.Equal(10.0, BRep_Tool.Pnt(aV1).X());
        Assert.Equal(0.0, BRep_Tool.Pnt(aV2).X());
        Assert.Equal(TopAbs_FORWARD, aV1.Orientation());
        Assert.Equal(TopAbs_REVERSED, aV2.Orientation());
    }

    [Fact]
    public void Curve3d_applies_the_location()
    {
        var aT = new Transform3D();
        aT.SetTranslation(new Vector3D(0, 0, 5));
        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        TopoDS_Edge aMoved = TopoDS.Edge(anE.Moved(new Placement(aT)));
        var sae = new ShapeAnalysis_Edge();
        Assert.True(sae.Curve3d(aMoved, out Geom_Curve? aC, out double f, out double l));
        Assert.Equal(5.0, aC!.Value(f).Z());
        Assert.Equal(5.0, aC.Value(l).Z());
    }

    [Fact]
    public void Cylinder_seam_has_two_pcurves()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(2, 5).Shape();
        var sae = new ShapeAnalysis_Edge();
        int aSeams = 0;
        foreach ((TopoDS_Edge anE, TopoDS_Face aF) in EdgesOnFaces(aCyl))
        {
            if (!sae.IsSeam(anE, aF))
            {
                Assert.False(sae.CheckVerticesWithPCurve(anE, aF));
                continue;
            }
            aSeams++;
            Assert.True(sae.BoundUV(anE, aF, out Point2D aF1, out _));
            Assert.True(sae.BoundUV(TopoDS.Edge(anE.Reversed()), aF, out Point2D aR1, out _));
            Assert.Equal(2 * Math.PI, Math.Abs(aF1.X() - aR1.X()), 1e-12);
            Assert.False(sae.CheckCurve3dWithPCurve(anE, aF));
            Assert.True(sae.Status(ShapeExtend_OK));
            double aMaxDev = 0;
            Assert.False(sae.CheckSameParameter(anE, aF, ref aMaxDev));
            Assert.True(aMaxDev < 1e-7);
        }
        Assert.Equal(2, aSeams); // the seam, FORWARD and REVERSED in the lateral face's wire
    }

    [Fact]
    public void Sphere_degenerated_edges()
    {
        TopoDS_Shape aSph = new BRepPrimAPI_MakeSphere(3).Shape();
        var sae = new ShapeAnalysis_Edge();
        int aDegenerated = 0;
        foreach ((TopoDS_Edge anE, TopoDS_Face aF) in EdgesOnFaces(aSph))
        {
            if (!BRep_Tool.Degenerated(anE))
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
            Assert.False(sae.Status(ShapeExtend_FAIL2));
            Assert.Equal(BRep_Tool.Tolerance(anE), aTol1);
            Assert.False(sae.CheckVerticesWithCurve3d(anE));
            Assert.True(sae.Status(ShapeExtend_FAIL1));
        }
        Assert.Equal(2, aDegenerated);
    }

    [Fact]
    public void Vertices_off_the_curve_are_reported()
    {
        // a segment from (0,0,0) to (10,0,0) whose vertices are 0.01 and 0.02 off, tolerance 1e-7
        TopoDS_Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0.01, 0));
        TopoDS_Vertex aV2 = TestShapes.Vertex(new Point3D(10, 0, 0.02));
        TopoDS_Edge anE = TestShapes.Edge(new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 10, aV1, aV2);
        var sae = new ShapeAnalysis_Edge();

        Assert.True(sae.CheckVerticesWithCurve3d(anE));
        Assert.True(sae.Status(ShapeExtend_DONE1));
        Assert.True(sae.Status(ShapeExtend_DONE2));
        Assert.True(sae.CheckVerticesWithCurve3d(anE, -1, 1));
        Assert.True(sae.Status(ShapeExtend_DONE1));
        Assert.False(sae.Status(ShapeExtend_DONE2));
        Assert.True(sae.CheckVerticesWithCurve3d(anE, 0.015));
        Assert.False(sae.Status(ShapeExtend_DONE1));
        Assert.True(sae.Status(ShapeExtend_DONE2));
        Assert.False(sae.CheckVerticesWithCurve3d(anE, 0.03));

        Assert.True(sae.CheckVertexTolerance(anE, out double aTol1, out double aTol2));
        Assert.True(sae.Status(ShapeExtend_DONE1) && sae.Status(ShapeExtend_DONE2));
        Assert.Equal(1.0000001 * Math.Sqrt(0.01 * 0.01), aTol1);
        Assert.Equal(1.0000001 * Math.Sqrt(0.02 * 0.02), aTol2);
    }

    [Fact]
    public void Edge_without_3d_curve_fails()
    {
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        TopoDS_Edge anE = new();
        aB.MakeEdge(ref anE);
        aB.UpdateEdge(anE, new Geom2d_Line(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        aB.Add(anE, TestShapes.Vertex(new Point3D(0, 0, 0)).Oriented(TopAbs_FORWARD));
        aB.Add(anE, TestShapes.Vertex(new Point3D(4, 0, 0)).Oriented(TopAbs_REVERSED));
        aB.Range(anE, aF, 0, 4);

        var sae = new ShapeAnalysis_Edge();
        Assert.False(sae.HasCurve3d(anE));
        Assert.False(sae.Curve3d(anE, out Geom_Curve? aC, out _, out _));
        Assert.Null(aC);
        Assert.False(sae.CheckVerticesWithCurve3d(anE));
        Assert.True(sae.Status(ShapeExtend_FAIL1));
        double aMaxDev = 0;
        Assert.False(sae.CheckSameParameter(anE, ref aMaxDev));
        Assert.True(sae.Status(ShapeExtend_FAIL1));
        Assert.False(sae.CheckVertexTolerance(anE, aF, out _, out _));
        Assert.True(sae.Status(ShapeExtend_FAIL2));
        Assert.False(sae.CheckVerticesWithPCurve(anE, aF));
        Assert.True(sae.Status(ShapeExtend_OK));

        // no pcurve on another plane
        var aB2 = new BRep_Builder();
        TopoDS_Face aG = new();
        aB2.MakeFace(ref aG, new Geom_Plane(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), Conf);
        Assert.False(sae.HasPCurve(anE, aG));
        Assert.False(sae.PCurve(anE, aG, out Geom2d_Curve? aPC, out _, out _));
        Assert.Null(aPC);
        Assert.False(sae.BoundUV(anE, aG, out _, out _));
        Assert.False(sae.GetEndTangent2d(anE, aG, false, out _, out Vector2D aTan));
        Assert.Equal(0.0, aTan.Magnitude());
        Assert.False(sae.CheckVerticesWithPCurve(anE, aG));
        Assert.True(sae.Status(ShapeExtend_FAIL1));
    }

    [Fact]
    public void Same_parameter_deviation_is_measured()
    {
        // a segment on the plane z = 0 whose pcurve runs along it at another speed (pcurve over the same range [0, 10],
        // but a parabola in the parameter) and isn't flagged SameParameter
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, XOY(), Conf);
        TopoDS_Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var aPC = new Geom2d_BSplineCurve(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(2, 0), new Point2D(10, 0)]),
                                          NCollection_Array1<double>.From([0.0, 10.0]), NCollection_Array1<int>.From([3, 3]), 2);
        aB.UpdateEdge(anE, aPC, aF, Conf);
        aB.SameParameter(anE, false);

        var sae = new ShapeAnalysis_Edge();
        double aMaxDev = 0;
        // not flagged SameParameter: BRepLib_ValidateEdge measures the distance to the 3D curve (by projection), which
        // is 0 here, and the flag gives DONE2
        Assert.True(sae.CheckSameParameter(anE, aF, ref aMaxDev));
        Assert.False(sae.Status(ShapeExtend_DONE1));
        Assert.True(sae.Status(ShapeExtend_DONE2));
        Assert.True(aMaxDev < 1e-7);

        // flagged SameParameter: the deviation at equal parameters, largest between t and the B-spline's
        // x(t) = 0.4 t + 0.06 t^2 over [0, 10], at t = 5: 1.5
        aB.SameParameter(anE, true);
        Assert.True(sae.CheckSameParameter(anE, aF, ref aMaxDev));
        Assert.True(sae.Status(ShapeExtend_DONE1));
        Assert.False(sae.Status(ShapeExtend_DONE2));
        Assert.Equal(1.5, aMaxDev, 1e-2);

        // a plane is skipped
        Assert.False(sae.CheckCurve3dWithPCurve(anE, aF));
    }

    [Fact]
    public void Curve3d_against_pcurve_on_a_cylinder()
    {
        // an arc on a cylinder whose pcurve runs the other way: the ends match reversed (DONE1)
        var aCylS = new Geom_CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1);
        var aB = new BRep_Builder();
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aCylS, Conf);
        var aCircle = new Geom_Circle(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1);
        TopoDS_Edge anE = TestShapes.Edge(aCircle, 0, 1);
        var sae = new ShapeAnalysis_Edge();

        aB.UpdateEdge(anE, new Geom2d_Line(new Point2D(0, 0), new Direction2D(1, 0)), aF, Conf);
        Assert.False(sae.CheckCurve3dWithPCurve(anE, aF));
        Assert.True(sae.Status(ShapeExtend_OK));

        aB.UpdateEdge(anE, new Geom2d_Line(new Point2D(1, 0), new Direction2D(-1, 0)), aF, Conf);
        Assert.True(sae.CheckCurve3dWithPCurve(anE, aF));
        Assert.True(sae.Status(ShapeExtend_DONE1));

        aB.UpdateEdge(anE, new Geom2d_Line(new Point2D(0, 3), new Direction2D(1, 0)), aF, Conf);
        Assert.True(sae.CheckCurve3dWithPCurve(anE, aF));
        Assert.False(sae.Status(ShapeExtend_DONE1));
    }

    [Fact]
    public void CheckPCurveRange_of_lines_circles_and_trimmed_curves()
    {
        var sae = new ShapeAnalysis_Edge();
        var aLine = new Geom2d_Line(new Point2D(0, 0), new Direction2D(1, 0));
        Assert.True(sae.CheckPCurveRange(-1e10, 1e10, aLine));

        var aCircle = new Geom2d_Circle(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1);
        Assert.True(sae.CheckPCurveRange(-1, 2 * Math.PI - 1, aCircle));
        Assert.True(sae.CheckPCurveRange(0, 2 * Math.PI + 0.5e-9, aCircle));
        Assert.False(sae.CheckPCurveRange(0, 2 * Math.PI + 1e-6, aCircle));

        // a trimmed circle: its basis is periodic
        var aTrimmed = new Geom2d_TrimmedCurve(aCircle, 0, 1);
        Assert.True(sae.CheckPCurveRange(-3, 3, aTrimmed));
        Assert.False(sae.CheckPCurveRange(-4, 4, aTrimmed));

        // a trimmed segment of a B-spline over [0, 1]: the basis's range counts, not the trimming
        var aBS = new Geom2d_BSplineCurve(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(1, 1)]),
                                          NCollection_Array1<double>.From([0.0, 1.0]), NCollection_Array1<int>.From([2, 2]), 1);
        var aTrimmedBS = new Geom2d_TrimmedCurve(aBS, 0.2, 0.4);
        Assert.True(sae.CheckPCurveRange(0, 1, aTrimmedBS));
        Assert.False(sae.CheckPCurveRange(0, 1.1, aTrimmedBS));
        Assert.False(sae.CheckPCurveRange(-0.1, 0.5, aBS));
    }

    [Fact]
    public void CheckOverlapping_of_collinear_and_parallel_segments()
    {
        TopoDS_Edge aLong = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Edge();
        TopoDS_Edge aShort = new BRepBuilderAPI_MakeEdge(new Point3D(2, 0, 0), new Point3D(8, 0, 0)).Edge();
        TopoDS_Edge aParallel = new BRepBuilderAPI_MakeEdge(new Point3D(0, 1, 0), new Point3D(10, 1, 0)).Edge();
        TopoDS_Edge aCrossing = new BRepBuilderAPI_MakeEdge(new Point3D(5, -5, 0), new Point3D(5, 5, 0)).Edge();

        var sae = new ShapeAnalysis_Edge();
        double aTol = 1e-3;
        Assert.True(sae.CheckOverlapping(aLong, aShort, ref aTol));
        Assert.True(sae.Status(ShapeExtend_DONE3));
        Assert.Equal(1e-3, aTol); // unchanged when the whole edge overlaps

        var sae2 = new ShapeAnalysis_Edge();
        aTol = 1e-3;
        Assert.False(sae2.CheckOverlapping(aLong, aParallel, ref aTol));
        Assert.Equal(1e-3, aTol);
        Assert.False(sae2.CheckOverlapping(aLong, aParallel, ref aTol, 1.0));
        Assert.True(sae2.Status(ShapeExtend_OK));

        // crossing segments touch at one point: over a domain of 0.001 around it they are within 1.0 of each other
        aTol = 1.0;
        Assert.True(sae2.CheckOverlapping(aLong, aCrossing, ref aTol, 0.001));
        Assert.True(sae2.Status(ShapeExtend_DONE4));
        Assert.True(aTol < 1e-9); // the distance found
    }

    [Fact]
    public void PCurve_and_HasPCurve_with_surface_and_location()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(1, 1, 1).Shape();
        var sae = new ShapeAnalysis_Edge();
        foreach ((TopoDS_Edge anE, TopoDS_Face aF) in EdgesOnFaces(aBox))
        {
            Geom_Surface aS = BRep_Tool.Surface(aF, out Placement aL)!;
            Assert.True(sae.HasPCurve(anE, aS, aL));
            Assert.True(sae.PCurve(anE, aS, aL, out Geom2d_Curve? aPC1, out double f1, out double l1));
            Assert.True(sae.PCurve(anE, aF, out Geom2d_Curve? aPC2, out double f2, out double l2));
            Assert.Same(aPC1, aPC2);
            Assert.Equal(f1, f2);
            Assert.Equal(l1, l2);
            Assert.True(sae.PCurve(anE, aF, out _, out double f3, out double l3, false));
            if (anE.Orientation() == TopAbs_REVERSED)
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
