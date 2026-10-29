// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Bento.Collections;
using OcctNet.ModelingData.Tests;
using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of BRepLib's regularity, sorting, valid range, planar pcurve and face extension statics
/// (ContinuityOfFaces, EncodeRegularity, SortFaces, ReverseSortFaces, FindValidRange, BuildPCurveForEdgeOnPlane,
/// ExtendFace, BoundingVertex), on frozen inputs; the in-place ones on a copy for write.
/// </summary>
public class BRepLibRegularityTests
{
    private const double Conf = 1e-7;

    private static List<TopoDS_Edge> Edges(in TopoDS_Shape S)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(S, TopAbs_EDGE, aMap);
        var aList = new List<TopoDS_Edge>();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(TopoDS.Edge(aMap[i]));
        }
        return aList;
    }

    private static NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> EdgeFaces(in TopoDS_Shape S)
    {
        var aMap = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(S, TopAbs_EDGE, TopAbs_FACE, aMap);
        return aMap;
    }

    /// <summary>
    /// An edge along the line x = 1, y = 0 (0 &lt;= z &lt;= 1) with a pcurve on a cylinder of radius 1 around Z (U = 0)
    /// and one on the plane x = 1 (u = y, v = z), on two faces without wires.
    /// </summary>
    private static (TopoDS_Edge Edge, TopoDS_Face Cylinder, TopoDS_Face Plane) TangentPair()
    {
        var aB = new BRep_Builder();
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 1.0);
        var aPln = new Geom_Plane(new gp_Ax3(new gp_Pnt(1, 0, 0), new gp_Dir(1, 0, 0), new gp_Dir(0, 1, 0)));
        TopoDS_Face aF1 = new(), aF2 = new();
        aB.MakeFace(ref aF1, aCyl, Conf);
        aB.MakeFace(ref aF2, aPln, Conf);
        TopoDS_Edge anE = TestShapes.Edge(new Geom_Line(new gp_Pnt(1, 0, 0), new gp_Dir(0, 0, 1)), 0.0, 1.0);
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1)), aF1, Conf);
        aB.UpdateEdge(anE, new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1)), aF2, Conf);
        return (anE, aF1, aF2);
    }

    [Fact]
    public void The_edges_of_a_box_are_C0()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        var aMap = EdgeFaces(aBox);
        Assert.Equal(12, aMap.Extent());
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            NCollection_List<TopoDS_Shape> aFaces = aMap.FindFromIndex(i);
            Assert.Equal(2, aFaces.Extent());
            Assert.Equal(GeomAbs_C0, BRepLib.ContinuityOfFaces(TopoDS.Edge(aMap.FindKey(i)), TopoDS.Face(aFaces.First()),
                                                               TopoDS.Face(aFaces.Last()), 1e-10));
        }
    }

    [Fact]
    public void A_seam_on_an_elementary_surface_is_CN()
    {
        TopoDS_Face aF = TestShapes.Frozen(TestShapes.CylinderFace(1, 2));
        TopoDS_Edge aSeam = Edges(aF).Single(e => BRep_Tool.IsClosed(e, aF));
        Assert.Equal(GeomAbs_CN, BRepLib.ContinuityOfFaces(aSeam, aF, aF, 1e-10));
    }

    [Fact]
    public void A_plane_tangent_to_a_cylinder_is_C1()
    {
        (TopoDS_Edge anE, TopoDS_Face aCyl, TopoDS_Face aPln) = TangentPair();
        Assert.Equal(GeomAbs_C1, BRepLib.ContinuityOfFaces(anE, aCyl, aPln, 1e-10));
        // the plane reversed: the normals oppose
        Assert.Equal(GeomAbs_C0, BRepLib.ContinuityOfFaces(anE, aCyl, TopoDS.Face(aPln.Reversed()), 1e-10));
    }

    [Fact]
    public void Two_faces_on_one_cylinder_are_CN()
    {
        var aB = new BRep_Builder();
        (TopoDS_Edge anE, TopoDS_Face aCyl, _) = TangentPair();
        TopoDS_Face aCyl2 = new();
        aB.MakeFace(ref aCyl2, BRep_Tool.Surface(aCyl)!, Conf);
        Assert.Equal(GeomAbs_CN, BRepLib.ContinuityOfFaces(anE, aCyl, aCyl2, 1e-10));
    }

    [Fact]
    public void EncodeRegularity_throws_on_a_frozen_shape()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        // OCCT swallows the exception of a locked edge; the port lets a frozen one through
        Assert.Throws<TopoDS_LockedShape>(() => BRepLib.EncodeRegularity(aBox));
        aBefore.AssertUnchanged(aBox);
    }

    [Fact]
    public void EncodeRegularity_on_a_copy_stores_the_continuities()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aBox, out _);
        BRepLib.EncodeRegularity(aCopy);
        foreach (TopoDS_Edge anE in Edges(aCopy))
        {
            Assert.True(BRep_Tool.HasContinuity(anE));
            Assert.Equal(GeomAbs_C0, BRep_Tool.MaxContinuity(anE));
        }
        foreach (TopoDS_Edge anE in Edges(aBox))
        {
            Assert.False(BRep_Tool.HasContinuity(anE));
        }
        aBefore.AssertUnchanged(aBox);
    }

    [Fact]
    public void EncodeRegularity_of_listed_edges_codes_only_those()
    {
        TopoDS_Shape aCopy = BRepTools.CopyForWrite(TestShapes.Frozen(TestShapes.Box(1, 1, 1)), out _);
        List<TopoDS_Edge> anEdges = Edges(aCopy);
        var aList = new NCollection_List<TopoDS_Shape>();
        aList.Append(anEdges[0].Reversed());
        aList.Append(anEdges[5]);
        BRepLib.EncodeRegularity(aCopy, aList);
        for (int i = 0; i < anEdges.Count; i++)
        {
            Assert.Equal(i == 0 || i == 5, BRep_Tool.HasContinuity(anEdges[i]));
        }
    }

    [Fact]
    public void EncodeRegularity_of_an_edge_between_two_faces()
    {
        (TopoDS_Edge anE, TopoDS_Face aCyl, TopoDS_Face aPln) = TangentPair();
        BRepLib.EncodeRegularity(anE, aCyl, aPln);
        Assert.Equal(GeomAbs_C1, BRep_Tool.Continuity(anE, aCyl, aPln));
        // already coded: nothing is done
        BRepLib.EncodeRegularity(anE, aCyl, TopoDS.Face(aPln.Reversed()));
        Assert.Equal(GeomAbs_C1, BRep_Tool.Continuity(anE, aCyl, aPln));
    }

    [Fact]
    public void A_compound_of_one_solid_placed_twice_is_coded_once()
    {
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        var aT = new gp_Trsf();
        aT.SetTranslation(new gp_Vec(5, 0, 0));
        TopoDS_Compound aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aBox, aBox.Moved(new TopLoc_Location(aT))));
        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aC, out _);
        BRepLib.EncodeRegularity(aCopy);
        Assert.All(Edges(aCopy), e => Assert.True(BRep_Tool.HasContinuity(e)));
    }

    [Fact]
    public void SortFaces_orders_by_surface_complexity()
    {
        TopoDS_Face aPlane = TopoDS.Face(BRepGPropTestShapes.Faces(TestShapes.Box(1, 1, 1))[0]);
        TopoDS_Face aCylinder = TestShapes.CylinderFace(1, 1);
        TopoDS_Face aSphere = BRepGPropTestShapes.NaturalFace(new Geom_SphericalSurface(new gp_Ax3(), 1));
        TopoDS_Face aBSpline = BRepGPropTestShapes.NaturalFace(Bezier());
        TopoDS_Face aMesh = BRepGPropTestShapes.MeshFace([new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0), new gp_Pnt(0, 1, 0)], (1, 2, 3));
        TopoDS_Face aTrimmedPlane = BRepGPropTestShapes.NaturalFace(
            new Geom_RectangularTrimmedSurface(new Geom_Plane(new gp_Ax3()), 0, 1, 0, 1));
        TopoDS_Compound aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aMesh, aBSpline, aSphere, aTrimmedPlane, aCylinder, aPlane));

        IBentoList<TopoDS_Shape> aSorted = BRepLib.SortFaces(aC);
        TopoDS_Shape[] anIn = [aTrimmedPlane, aPlane, aCylinder, aSphere, aBSpline, aMesh];
        Assert.Equal("0 1 2 3 4 5", string.Join(" ", aSorted.Select(s => Array.FindIndex(anIn, f => f.IsEqual(s)))));
        Assert.Equal(new TopoDS_Shape[] { aTrimmedPlane, aPlane, aCylinder, aSphere, aBSpline, aMesh }, aSorted.ToList(),
                     EqualityComparer<TopoDS_Shape>.Default);

        // ReverseSortFaces doesn't unwrap the trimmed surface, but its adaptor does
        IBentoList<TopoDS_Shape> aReversed = BRepLib.ReverseSortFaces(aC);
        Assert.Equal(new TopoDS_Shape[] { aMesh, aBSpline, aSphere, aCylinder, aTrimmedPlane, aPlane }, aReversed.ToList(),
                     EqualityComparer<TopoDS_Shape>.Default);

        var aList = new NCollection_List<TopoDS_Shape>();
        aList.Append(aMesh);
        BRepLib.SortFaces(aC, aList);
        Assert.Equal(6, aList.Extent());
        Assert.True(aList.First().IsEqual(aTrimmedPlane));
    }

    private static Geom_BezierSurface Bezier()
    {
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 3, 1, 3);
        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles[i, j] = new gp_Pnt(i, j, (i == 2 && j == 2) ? 1 : 0);
            }
        }
        return new Geom_BezierSurface(aPoles);
    }

    [Fact]
    public void FindValidRange_leaves_out_the_vertex_tolerances()
    {
        var aB = new BRep_Builder();
        TopoDS_Edge anE = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 0, 0));
        TopExp.Vertices(anE, out TopoDS_Vertex aV1, out TopoDS_Vertex aV2);
        aB.UpdateVertex(aV1, 0.5);
        aB.UpdateVertex(aV2, 0.25);
        anE = TestShapes.Frozen(anE);

        double aFirst = -1, aLast = -1;
        Assert.True(BRepLib.FindValidRange(anE, ref aFirst, ref aLast));
        Assert.Equal(0.5 + Conf, aFirst, 1e-7);
        Assert.Equal(10 - 0.25 - Conf, aLast, 1e-7);

        // the tolerances cover the whole edge
        TopoDS_Edge aShort = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(0.5, 0, 0));
        TopExp.Vertices(aShort, out aV1, out _);
        aB.UpdateVertex(aV1, 0.6);
        aFirst = aLast = -1;
        Assert.False(BRepLib.FindValidRange(aShort, ref aFirst, ref aLast));

        // on an adaptor
        var anAC = new BRepAdaptor_Curve(anE);
        Assert.True(BRepLib.FindValidRange(anAC, 0.0, 0.0, new gp_Pnt(0, 0, 0), 1.0, 10.0, new gp_Pnt(10, 0, 0), 2.0,
                                           ref aFirst, ref aLast));
        Assert.Equal(1.0, aFirst, 1e-9);
        Assert.Equal(8.0, aLast, 1e-9);
        // an empty range
        Assert.False(BRepLib.FindValidRange(anAC, 0.0, 5.0, new gp_Pnt(5, 0, 0), 1.0, 5.0, new gp_Pnt(5, 0, 0), 1.0,
                                            ref aFirst, ref aLast));
    }

    [Fact]
    public void FindValidRange_of_a_BSpline_steps_out_of_a_slow_start()
    {
        // a cubic whose first poles coincide: zero derivative at the start
        var aPoles = new NCollection_Array1<gp_Pnt>(1, 4);
        aPoles[1] = new gp_Pnt(0, 0, 0);
        aPoles[2] = new gp_Pnt(0, 0, 0);
        aPoles[3] = new gp_Pnt(5, 0, 0);
        aPoles[4] = new gp_Pnt(10, 0, 0);
        TopoDS_Edge anE = TestShapes.Frozen(TestShapes.Edge(new Geom_BezierCurve(aPoles), 0, 1));
        double aFirst = 0, aLast = 0;
        Assert.True(BRepLib.FindValidRange(anE, ref aFirst, ref aLast));
        var aC = new BRepAdaptor_Curve(anE);
        Assert.Equal(2 * Conf, aC.Value(aFirst).Distance(new gp_Pnt(0, 0, 0)), 1e-10);
        Assert.Equal(2 * Conf, aC.Value(aLast).Distance(new gp_Pnt(10, 0, 0)), 1e-7);
    }

    [Fact]
    public void BuildPCurveForEdgeOnPlane_computes_and_stores_the_pcurve()
    {
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new gp_Ax3(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)));
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, Conf);
        TopoDS_Edge anE = TestShapes.Edge(new gp_Pnt(1, 2, 1), new gp_Pnt(4, 6, 1));
        TopoDS_Compound aC = TestShapes.Frozen(BRepGPropTestShapes.CompoundOf(aF, anE));

        BRepLib.BuildPCurveForEdgeOnPlane(anE, aF, out Geom2d_Curve? aC2D, out bool isToUpdate);
        Assert.True(isToUpdate);
        Assert.NotNull(aC2D);
        gp_Pnt2d aP = aC2D!.Value(5.0); // the line is parametrised by length
        Assert.Equal(4.0, aP.X(), 1e-12);
        Assert.Equal(6.0, aP.Y(), 1e-12);
        Assert.Throws<TopoDS_LockedShape>(() => BRepLib.BuildPCurveForEdgeOnPlane(anE, aF));

        TopoDS_Shape aCopy = BRepTools.CopyForWrite(aC, out BRepTools_History aH);
        TopoDS_Face aFc = TopoDS.Face(aH.Modified(aF)[0]);
        TopoDS_Edge anEc = TopoDS.Edge(aH.Modified(anE)[0]);
        BRepLib.BuildPCurveForEdgesOnPlane(new[] { (TopoDS_Shape)anEc, new TopoDS_Shape() }, aFc);
        BRepLib.BuildPCurveForEdgeOnPlane(anEc, aFc, out Geom2d_Curve? aStored, out isToUpdate);
        Assert.False(isToUpdate);
        Assert.Equal(4.0, aStored!.Value(5.0).X(), 1e-12);
        Assert.NotNull(aCopy.TShape());
    }

    [Fact]
    public void ExtendFace_enlarges_a_plane_face()
    {
        TopoDS_Face aF = new BRepLib_MakeFace(new gp_Pln(new gp_Ax3()), 0, 1, 0, 2).Face();
        ShapeSnapshot aBefore = ShapeSnapshot.Of(aF);
        BRepLib.ExtendFace(aF, 0.5, true, false, true, true, out TopoDS_Face anExt);
        Assert.True(anExt.TShape()!.IsFrozen());
        BRepTools.UVBounds(anExt, out double u1, out double u2, out double v1, out double v2);
        Assert.Equal(-0.5, u1, 1e-12);
        Assert.Equal(1.0, u2, 1e-12);
        Assert.Equal(-0.5, v1, 1e-12);
        Assert.Equal(2.5, v2, 1e-12);
        aBefore.AssertUnchanged(aF);

        // a reversed face gives a reversed face
        BRepLib.ExtendFace(TopoDS.Face(aF.Reversed()), 0.5, true, true, true, true, out anExt);
        Assert.Equal(TopAbs_REVERSED, anExt.Orientation());
    }

    [Fact]
    public void ExtendFace_of_a_cylinder_stays_within_its_period()
    {
        var aCyl = new gp_Cylinder(new gp_Ax3(), 2.0);
        TopoDS_Face aF = new BRepLib_MakeFace(aCyl, 1.0, 2.0, 0, 1).Face();
        BRepLib.ExtendFace(aF, 1.0, true, true, false, false, out TopoDS_Face anExt);
        BRepTools.UVBounds(anExt, out double u1, out double u2, out double v1, out double v2);
        double aRes = 2 * Math.Asin(1.0 / 4); // the cylinder's resolution of 1 at radius 2
        Assert.Equal(1.0 - aRes, u1, 1e-12);
        Assert.Equal(2.0 + aRes, u2, 1e-12);
        Assert.Equal(0.0, v1, 1e-12);
        Assert.Equal(1.0, v2, 1e-12);

        // nearly the whole period: the face closes on the basis surface's bounds
        TopoDS_Face aWide = new BRepLib_MakeFace(aCyl, 0.1, 2 * Math.PI - 0.1, 0, 1).Face();
        BRepLib.ExtendFace(aWide, 1.0, true, true, false, false, out anExt);
        BRepTools.UVBounds(anExt, out u1, out u2, out _, out _);
        Assert.Equal(0.0, u1, 1e-12);
        Assert.Equal(2 * Math.PI, u2, 1e-12);
    }

    [Fact]
    public void ExtendFace_of_a_Bezier_face_extends_its_surface()
    {
        TopoDS_Face aF = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(Bezier()));
        Geom_Surface aS = BRep_Tool.Surface(aF)!;
        BRepLib.ExtendFace(aF, 1.0, false, true, false, false, out TopoDS_Face anExt);
        Geom_Surface aNew = BRep_Tool.Surface(anExt)!;
        Assert.NotSame(aS, aNew);
        aNew.Bounds(out double u1, out double u2, out _, out _);
        BRepTools.UVBounds(anExt, out double fu1, out double fu2, out _, out _);
        Assert.Equal(u1, fu1, 1e-12);
        Assert.Equal(u2, fu2, 1e-12);
        // the extended end lies one unit further along the surface
        Assert.True(aNew.Value(u2, 0.5).Distance(aS.Value(1, 0.5)) > 0.9);
        Assert.Equal(3.0, ((Geom_BezierSurface)aS).Pole(3, 1).X());

        // a face on a surface that isn't bounded is returned as it is
        TopoDS_Face anOffset = TestShapes.Frozen(BRepGPropTestShapes.NaturalFace(new Geom_OffsetSurface(Bezier(), 0.1)));
        BRepLib.ExtendFace(anOffset, 1.0, true, true, true, true, out anExt);
        Assert.True(anExt.IsEqual(anOffset));
    }

    [Fact]
    public void BoundingVertex_covers_the_vertices()
    {
        var aB = new BRep_Builder();
        TopoDS_Vertex aV1 = TestShapes.Vertex(new gp_Pnt(0, 0, 0));
        TopoDS_Vertex aV2 = TestShapes.Vertex(new gp_Pnt(2, 0, 0));
        TopoDS_Vertex aV3 = TestShapes.Vertex(new gp_Pnt(0, 2, 0));
        aB.UpdateVertex(aV1, 0.5);
        var aList = new NCollection_List<TopoDS_Shape>();
        var aC = new gp_Pnt(9, 9, 9);
        double aTol = -1;
        aList.Append(aV1);
        BRepLib.BoundingVertex(aList, ref aC, ref aTol);
        Assert.Equal(-1.0, aTol);

        aList.Append(aV2);
        BRepLib.BoundingVertex(aList, ref aC, ref aTol);
        // the sphere through both spheres: radius (0.5 + 1e-7 + 2) / 2
        Assert.Equal(0.5 * (0.5 + Conf + 2), aTol, 1e-12);
        Assert.Equal(0.5 * (2 - (0.5 - Conf)), aC.X(), 1e-12);

        aList.Append(aV3);
        BRepLib.BoundingVertex(aList, ref aC, ref aTol);
        Assert.Equal(2.0 / 3, aC.X(), 1e-12);
        Assert.Equal(2.0 / 3, aC.Y(), 1e-12);
        Assert.Equal(Math.Sqrt(20.0 / 9) + Conf, aTol, 1e-12);
    }
}
