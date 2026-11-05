// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepLib's regularity, sorting, valid range, planar pcurve and face extension statics
/// (ContinuityOfFaces, EncodeRegularity, SortFaces, ReverseSortFaces, FindValidRange, BuildPCurveForEdgeOnPlane,
/// ExtendFace, BoundingVertex), on frozen inputs; the in-place ones on a copy for write.
/// </summary>
public class BRepLibRegularityTests
{
    private const double Conf = 1e-7;

    private static List<Edge> Edges(in Shape S)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(S, ShapeType.Edge, aMap);
        var aList = new List<Edge>();
        for (int i = 0; i < aMap.Count; i++)
        {
            aList.Add(ShapeCasts.Edge(aMap[i]));
        }
        return aList;
    }

    private static IndexedMap<Shape, NCollection_List<Shape>> EdgeFaces(in Shape S)
    {
        var aMap = new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(S, ShapeType.Edge, ShapeType.Face, aMap);
        return aMap;
    }

    /// <summary>
    /// An edge along the line x = 1, y = 0 (0 &lt;= z &lt;= 1) with a pcurve on a cylinder of radius 1 around Z (U = 0)
    /// and one on the plane x = 1 (u = y, v = z), on two faces without wires.
    /// </summary>
    private static (Edge Edge, Face Cylinder, Face Plane) TangentPair()
    {
        var aB = new BrepBuilder();
        var aCyl = new CylindricalSurface(new CoordinateSystem(new Point3D(), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0);
        var aPln = new PlaneSurface(new CoordinateSystem(new Point3D(1, 0, 0), new Direction3D(1, 0, 0), new Direction3D(0, 1, 0)));
        Face aF1 = new(), aF2 = new();
        aB.MakeFace(ref aF1, aCyl, Conf);
        aB.MakeFace(ref aF2, aPln, Conf);
        Edge anE = TestShapes.Edge(new LineCurve3D(new Point3D(1, 0, 0), new Direction3D(0, 0, 1)), 0.0, 1.0);
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1)), aF1, Conf);
        aB.UpdateEdge(anE, new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1)), aF2, Conf);
        return (anE, aF1, aF2);
    }

    [Fact]
    public void The_edges_of_a_box_are_C0()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aMap = EdgeFaces(aBox);
        Assert.Equal(12, aMap.Count);
        for (int i = 0; i < aMap.Count; i++)
        {
            NCollection_List<Shape> aFaces = aMap.ValueAt(i);
            Assert.Equal(2, aFaces.Extent());
            Assert.Equal(C0, BrepLibrary.ContinuityOfFaces(ShapeCasts.Edge(aMap.KeyAt(i)), ShapeCasts.Face(aFaces.First()),
                                                               ShapeCasts.Face(aFaces.Last()), 1e-10));
        }
    }

    [Fact]
    public void A_seam_on_an_elementary_surface_is_CN()
    {
        Face aF = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        Edge aSeam = Edges(aF).Single(e => Tool.IsClosed(e, aF));
        Assert.Equal(CN, BrepLibrary.ContinuityOfFaces(aSeam, aF, aF, 1e-10));
    }

    [Fact]
    public void A_plane_tangent_to_a_cylinder_is_C1()
    {
        (Edge anE, Face aCyl, Face aPln) = TangentPair();
        Assert.Equal(C1, BrepLibrary.ContinuityOfFaces(anE, aCyl, aPln, 1e-10));
        // the plane reversed: the normals oppose
        Assert.Equal(C0, BrepLibrary.ContinuityOfFaces(anE, aCyl, ShapeCasts.Face(aPln.Reversed()), 1e-10));
    }

    [Fact]
    public void Two_faces_on_one_cylinder_are_CN()
    {
        var aB = new BrepBuilder();
        (Edge anE, Face aCyl, _) = TangentPair();
        Face aCyl2 = new();
        aB.MakeFace(ref aCyl2, Tool.Surface(aCyl)!, Conf);
        Assert.Equal(CN, BrepLibrary.ContinuityOfFaces(anE, aCyl, aCyl2, 1e-10));
    }

    [Fact]
    public void EncodeRegularity_throws_on_a_frozen_shape()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        // OCCT swallows the exception of a locked edge; the port lets a frozen one through
        Assert.Throws<LockedShapeException>(() => BrepLibrary.EncodeRegularity(aBox));
        aBefore.AssertUnchanged(aBox);
    }

    [Fact]
    public void EncodeRegularity_on_a_copy_stores_the_continuities()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        Shape aCopy = BrepTools.CopyForWrite(aBox, out _);
        BrepLibrary.EncodeRegularity(aCopy);
        foreach (Edge anE in Edges(aCopy))
        {
            Assert.True(Tool.HasContinuity(anE));
            Assert.Equal(C0, Tool.MaxContinuity(anE));
        }
        foreach (Edge anE in Edges(aBox))
        {
            Assert.False(Tool.HasContinuity(anE));
        }
        aBefore.AssertUnchanged(aBox);
    }

    [Fact]
    public void EncodeRegularity_of_listed_edges_codes_only_those()
    {
        Shape aCopy = BrepTools.CopyForWrite(TestShapes.Frozen(TestShapes.Box(1, 1, 1)), out _);
        List<Edge> anEdges = Edges(aCopy);
        var aList = new NCollection_List<Shape>();
        aList.Append(anEdges[0].Reversed());
        aList.Append(anEdges[5]);
        BrepLibrary.EncodeRegularity(aCopy, aList);
        for (int i = 0; i < anEdges.Count; i++)
        {
            Assert.Equal(i == 0 || i == 5, Tool.HasContinuity(anEdges[i]));
        }
    }

    [Fact]
    public void EncodeRegularity_of_an_edge_between_two_faces()
    {
        (Edge anE, Face aCyl, Face aPln) = TangentPair();
        BrepLibrary.EncodeRegularity(anE, aCyl, aPln);
        Assert.Equal(C1, Tool.Continuity(anE, aCyl, aPln));
        // already coded: nothing is done
        BrepLibrary.EncodeRegularity(anE, aCyl, ShapeCasts.Face(aPln.Reversed()));
        Assert.Equal(C1, Tool.Continuity(anE, aCyl, aPln));
    }

    [Fact]
    public void A_compound_of_one_solid_placed_twice_is_coded_once()
    {
        Solid aBox = TestShapes.Box(1, 1, 1);
        var aT = new Transform3D();
        aT = aT.WithTranslation(new Vector3D(5, 0, 0));
        Compound aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Moved(new Placement(aT))));
        Shape aCopy = BrepTools.CopyForWrite(aC, out _);
        BrepLibrary.EncodeRegularity(aCopy);
        Assert.All(Edges(aCopy), e => Assert.True(Tool.HasContinuity(e)));
    }

    [Fact]
    public void SortFaces_orders_by_surface_complexity()
    {
        Face aPlane = ShapeCasts.Face(BRepGPropTestShapes.Faces(TestShapes.Box(1, 1, 1))[0]);
        Face aCylinder = TestShapes.CylinderFace(1, 1);
        Face aSphere = BRepGPropTestShapes.NaturalFace(new SphericalSurface(new CoordinateSystem(), 1));
        Face aBSpline = BRepGPropTestShapes.NaturalFace(Bezier());
        Face aMesh = BRepGPropTestShapes.MeshFace([new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 1, 0)], (1, 2, 3));
        Face aTrimmedPlane = BRepGPropTestShapes.NaturalFace(
            new TrimmedSurface(new PlaneSurface(new CoordinateSystem()), 0, 1, 0, 1));
        Compound aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aMesh, aBSpline, aSphere, aTrimmedPlane, aCylinder, aPlane));

        IReadOnlyList<Shape> aSorted = BrepLibrary.SortFaces(aC);
        Shape[] anIn = [aTrimmedPlane, aPlane, aCylinder, aSphere, aBSpline, aMesh];
        Assert.Equal("0 1 2 3 4 5", string.Join(" ", aSorted.Select(s => Array.FindIndex(anIn, f => f.IsEqual(s)))));
        Assert.Equal(new Shape[] { aTrimmedPlane, aPlane, aCylinder, aSphere, aBSpline, aMesh }, aSorted.ToList(),
                     EqualityComparer<Shape>.Default);

        // ReverseSortFaces doesn't unwrap the trimmed surface, but its adaptor does
        IReadOnlyList<Shape> aReversed = BrepLibrary.ReverseSortFaces(aC);
        Assert.Equal(new Shape[] { aMesh, aBSpline, aSphere, aCylinder, aTrimmedPlane, aPlane }, aReversed.ToList(),
                     EqualityComparer<Shape>.Default);

        var aList = new NCollection_List<Shape>();
        aList.Append(aMesh);
        BrepLibrary.SortFaces(aC, aList);
        Assert.Equal(6, aList.Extent());
        Assert.True(aList.First().IsEqual(aTrimmedPlane));
    }

    private static BezierSurface Bezier()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles[i, j] = new Point3D(i, j, (i == 2 && j == 2) ? 1 : 0);
            }
        }
        return new BezierSurface(aPoles);
    }

    [Fact]
    public void FindValidRange_leaves_out_the_vertex_tolerances()
    {
        var aB = new BrepBuilder();
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        Exploration.Vertices(anE, out Vertex aV1, out Vertex aV2);
        aB.UpdateVertex(aV1, 0.5);
        aB.UpdateVertex(aV2, 0.25);
        anE = TestShapes.Frozen(anE);

        double aFirst = -1, aLast = -1;
        Assert.True(BrepLibrary.FindValidRange(anE, ref aFirst, ref aLast));
        Assert.Equal(0.5 + Conf, aFirst, 1e-7);
        Assert.Equal(10 - 0.25 - Conf, aLast, 1e-7);

        // the tolerances cover the whole edge
        Edge aShort = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0));
        Exploration.Vertices(aShort, out aV1, out _);
        aB.UpdateVertex(aV1, 0.6);
        aFirst = aLast = -1;
        Assert.False(BrepLibrary.FindValidRange(aShort, ref aFirst, ref aLast));

        // on an adaptor
        var anAC = new EdgeCurveAdaptor(anE);
        Assert.True(BrepLibrary.FindValidRange(anAC, 0.0, 0.0, new Point3D(0, 0, 0), 1.0, 10.0, new Point3D(10, 0, 0), 2.0,
                                           ref aFirst, ref aLast));
        Assert.Equal(1.0, aFirst, 1e-9);
        Assert.Equal(8.0, aLast, 1e-9);
        // an empty range
        Assert.False(BrepLibrary.FindValidRange(anAC, 0.0, 5.0, new Point3D(5, 0, 0), 1.0, 5.0, new Point3D(5, 0, 0), 1.0,
                                            ref aFirst, ref aLast));
    }

    [Fact]
    public void FindValidRange_of_a_BSpline_steps_out_of_a_slow_start()
    {
        // a cubic whose first poles coincide: zero derivative at the start
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(0, 0, 0);
        aPoles[3] = new Point3D(5, 0, 0);
        aPoles[4] = new Point3D(10, 0, 0);
        Edge anE = TestShapes.Frozen(TestShapes.Edge(new BezierCurve3D(aPoles), 0, 1));
        double aFirst = 0, aLast = 0;
        Assert.True(BrepLibrary.FindValidRange(anE, ref aFirst, ref aLast));
        var aC = new EdgeCurveAdaptor(anE);
        Assert.Equal(2 * Conf, aC.Value(aFirst).DistanceTo(new Point3D(0, 0, 0)), 1e-10);
        Assert.Equal(2 * Conf, aC.Value(aLast).DistanceTo(new Point3D(10, 0, 0)), 1e-7);
    }

    [Fact]
    public void BuildPCurveForEdgeOnPlane_computes_and_stores_the_pcurve()
    {
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 1), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, Conf);
        Edge anE = TestShapes.Edge(new Point3D(1, 2, 1), new Point3D(4, 6, 1));
        Compound aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aF, anE));

        BrepLibrary.BuildPCurveForEdgeOnPlane(anE, aF, out Curve2D? aC2D, out bool isToUpdate);
        Assert.True(isToUpdate);
        Assert.NotNull(aC2D);
        Point2D aP = aC2D!.Value(5.0); // the line is parametrised by length
        Assert.Equal(4.0, aP.X, 1e-12);
        Assert.Equal(6.0, aP.Y, 1e-12);
        Assert.Throws<LockedShapeException>(() => BrepLibrary.BuildPCurveForEdgeOnPlane(anE, aF));

        Shape aCopy = BrepTools.CopyForWrite(aC, out History aH);
        Face aFc = ShapeCasts.Face(aH.Modified(aF)[0]);
        Edge anEc = ShapeCasts.Edge(aH.Modified(anE)[0]);
        BrepLibrary.BuildPCurveForEdgesOnPlane(new[] { (Shape)anEc, new Shape() }, aFc);
        BrepLibrary.BuildPCurveForEdgeOnPlane(anEc, aFc, out Curve2D? aStored, out isToUpdate);
        Assert.False(isToUpdate);
        Assert.Equal(4.0, aStored!.Value(5.0).X, 1e-12);
        Assert.NotNull(aCopy.TShape());
    }

    [Fact]
    public void ExtendFace_enlarges_a_plane_face()
    {
        Face aF = new LibraryMakeFace(new Plane3D(new CoordinateSystem()), 0, 1, 0, 2).Face();
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);
        BrepLibrary.ExtendFace(aF, 0.5, true, false, true, true, out Face anExt);
        Assert.True(anExt.TShape()!.IsFrozen);
        BrepTools.UVBounds(anExt, out double u1, out double u2, out double v1, out double v2);
        Assert.Equal(-0.5, u1, 1e-12);
        Assert.Equal(1.0, u2, 1e-12);
        Assert.Equal(-0.5, v1, 1e-12);
        Assert.Equal(2.5, v2, 1e-12);
        aBefore.AssertUnchanged(aF);

        // a reversed face gives a reversed face
        BrepLibrary.ExtendFace(ShapeCasts.Face(aF.Reversed()), 0.5, true, true, true, true, out anExt);
        Assert.Equal(Reversed, anExt.Orientation);
    }

    [Fact]
    public void ExtendFace_of_a_cylinder_stays_within_its_period()
    {
        var aCyl = new Cylinder(new CoordinateSystem(), 2.0);
        Face aF = new LibraryMakeFace(aCyl, 1.0, 2.0, 0, 1).Face();
        BrepLibrary.ExtendFace(aF, 1.0, true, true, false, false, out Face anExt);
        BrepTools.UVBounds(anExt, out double u1, out double u2, out double v1, out double v2);
        double aRes = 2 * Math.Asin(1.0 / 4); // the cylinder's resolution of 1 at radius 2
        Assert.Equal(1.0 - aRes, u1, 1e-12);
        Assert.Equal(2.0 + aRes, u2, 1e-12);
        Assert.Equal(0.0, v1, 1e-12);
        Assert.Equal(1.0, v2, 1e-12);

        // nearly the whole period: the face closes on the basis surface's bounds
        Face aWide = new LibraryMakeFace(aCyl, 0.1, 2 * Math.PI - 0.1, 0, 1).Face();
        BrepLibrary.ExtendFace(aWide, 1.0, true, true, false, false, out anExt);
        BrepTools.UVBounds(anExt, out u1, out u2, out _, out _);
        Assert.Equal(0.0, u1, 1e-12);
        Assert.Equal(2 * Math.PI, u2, 1e-12);
    }

    [Fact]
    public void ExtendFace_of_a_located_Bezier_face_keeps_its_place()
    {
        // a Bezier face moved and turned: its extension contains it, in place
        Face aBase = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(Bezier()));
        Transform3D aT = Transform3D.Rotation(new Axis3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 0)), 0.8)
                                    .Multiplied(Transform3D.Translation(new Vector3D(5, -3, 2)));
        Face aF = ShapeCasts.Face(aBase.Moved(new Placement(aT)));
        BrepLibrary.ExtendFace(aF, 1.0, true, true, true, true, out Face anExt);
        Surface aNew = Tool.Surface(anExt)!;
        var aFace = new FaceSurfaceAdaptor(aF);
        for (int i = 0; i <= 4; i++)
        {
            for (int j = 0; j <= 4; j++)
            {
                double u = aFace.FirstUParameter() + (aFace.LastUParameter() - aFace.FirstUParameter()) * i / 4;
                double v = aFace.FirstVParameter() + (aFace.LastVParameter() - aFace.FirstVParameter()) * j / 4;
                Assert.True(new global::Kerf.Engine.Geometry.Api.ProjectPointOnSurface(aFace.Value(u, v), aNew).LowerDistance() < 1e-7);
            }
        }
    }

    [Fact]
    public void ExtendFace_at_VMin_tests_the_VMin_bound()
    {
        // an extrusion of a segment trimmed to v in ]-infinity, 1]: VMin is infinite, so it isn't extended (OCCT
        // tested VMax for the VMin extension and extended the infinite side, which threw: occt-issues.md, unit 3I's
        // BRepLib::ExtendFace row; decision 1478)
        var aLine = new TrimmedCurve3D(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 2);
        var anExtrusion = new ExtrusionSurface(aLine, new Direction3D(0, 0, 1));
        var aTrimmed = new TrimmedSurface(anExtrusion, 0, 2, -2e100, 1);
        Face aF = new LibraryMakeFace(aTrimmed, 0, 2, -2e100, 1, 1e-7).Face();
        BrepLibrary.ExtendFace(aF, 0.5, false, false, true, false, out Face anExt);
        BrepTools.UVBounds(anExt, out _, out double u2, out double v1, out double v2);
        Assert.Equal(2.0, u2, 1e-9);
        Assert.True(v1 <= -1e100);
        Assert.Equal(1.0, v2, 1e-9);
    }

    [Fact]
    public void ExtendFace_of_a_Bezier_face_extends_its_surface()
    {
        Face aF = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(Bezier()));
        Surface aS = Tool.Surface(aF)!;
        BrepLibrary.ExtendFace(aF, 1.0, false, true, false, false, out Face anExt);
        Surface aNew = Tool.Surface(anExt)!;
        Assert.NotSame(aS, aNew);
        aNew.ParameterBounds(out double u1, out double u2, out _, out _);
        BrepTools.UVBounds(anExt, out double fu1, out double fu2, out _, out _);
        Assert.Equal(u1, fu1, 1e-12);
        Assert.Equal(u2, fu2, 1e-12);
        // the extended end lies one unit further along the surface
        Assert.True(aNew.Value(u2, 0.5).DistanceTo(aS.Value(1, 0.5)) > 0.9);
        Assert.Equal(3.0, ((BezierSurface)aS).Pole(3, 1).X);

        // a face on a surface that isn't bounded is returned as it is
        Face anOffset = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(new OffsetSurface(Bezier(), 0.1)));
        BrepLibrary.ExtendFace(anOffset, 1.0, true, true, true, true, out anExt);
        Assert.True(anExt.IsEqual(anOffset));
    }

    [Fact]
    public void BoundingVertex_covers_the_vertices()
    {
        var aB = new BrepBuilder();
        Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0, 0));
        Vertex aV2 = TestShapes.Vertex(new Point3D(2, 0, 0));
        Vertex aV3 = TestShapes.Vertex(new Point3D(0, 2, 0));
        aB.UpdateVertex(aV1, 0.5);
        var aList = new NCollection_List<Shape>();
        var aC = new Point3D(9, 9, 9);
        double aTol = -1;
        aList.Append(aV1);
        BrepLibrary.BoundingVertex(aList, ref aC, ref aTol);
        Assert.Equal(-1.0, aTol);

        aList.Append(aV2);
        BrepLibrary.BoundingVertex(aList, ref aC, ref aTol);
        // the sphere through both spheres: radius (0.5 + 1e-7 + 2) / 2
        Assert.Equal(0.5 * (0.5 + Conf + 2), aTol, 1e-12);
        Assert.Equal(0.5 * (2 - (0.5 - Conf)), aC.X, 1e-12);

        aList.Append(aV3);
        BrepLibrary.BoundingVertex(aList, ref aC, ref aTol);
        Assert.Equal(2.0 / 3, aC.X, 1e-12);
        Assert.Equal(2.0 / 3, aC.Y, 1e-12);
        Assert.Equal(Math.Sqrt(20.0 / 9) + Conf, aTol, 1e-12);
    }
}
