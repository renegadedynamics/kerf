// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using System.Text.RegularExpressions;
using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The functional twins of BRepLib's in-place statics (design 4.6): each leaves its frozen input unchanged, returns a
/// frozen shape with the values of the static run on a copy for write, and gives back the input's own sub-shapes
/// where the static changed nothing.
/// </summary>
public class BRepLibTwinsTests
{
    private const double Conf = 1e-7;

    // BRep text with the flag lines masked: kept sub-shapes keep their Free and Modified bits, copies have their own.
    private static string Text(in TopoDS_Shape theShape)
    {
        var aWriter = new StringWriter();
        BRepTools.Write(theShape, aWriter);
        return Regex.Replace(aWriter.ToString(), "(?m)^[01]{7}$", "flags");
    }

    private static void AssertTwin(in TopoDS_Shape theInput, Func<TopoDS_Shape, TopoDS_Shape> theTwin, Action<TopoDS_Shape> theStatic)
    {
        ShapeSnapshot aSnap = ShapeSnapshot.Of(theInput);
        TopoDS_Shape aResult = theTwin(theInput);
        aSnap.AssertUnchanged(theInput);
        Assert.True(aResult.TShape()!.IsFrozen);

        TopoDS_Shape aCopy = BRepTools.ExactCopyForWrite(theInput, out _);
        theStatic(aCopy);
        Assert.Equal(Text(aCopy), Text(aResult));
    }

    private static int CountShared(in TopoDS_Shape theA, in TopoDS_Shape theB, TopAbs_ShapeEnum theType)
    {
        var aMapA = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aMapB = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theA, theType, aMapA);
        TopExp.MapShapes(theB, theType, aMapB);
        int n = 0;
        for (int i = 1; i <= aMapA.Extent(); i++)
        {
            if (aMapB.Contains(aMapA[i]))
            {
                n++;
            }
        }
        return n;
    }

    /// <summary>A face on z = 0 bounded by a square, one of whose edges has only its pcurve, frozen.</summary>
    private static TopoDS_Face SquareWithAPcurveOnlyEdge()
    {
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, Conf);
        Point3D[] aP = [new(0, 0, 0), new(2, 0, 0), new(2, 2, 0), new(0, 2, 0)];
        TopoDS_Vertex[] aV = aP.Select(p => TestShapes.Vertex(p)).ToArray();
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        for (int i = 0; i < 3; i++)
        {
            aB.Add(aW, TestShapes.Edge(aV[i], aV[i + 1]));
        }
        TopoDS_Edge aLast = new();
        aB.MakeEdge(ref aLast);
        aB.UpdateEdge(aLast, new Geom2d_Line(new Point2D(0, 2), new Direction2D(0, -1)), aF, Conf);
        aB.Range(aLast, 0, 2);
        aB.Add(aLast, aV[3].Oriented(TopAbs_Orientation.TopAbs_FORWARD));
        aB.Add(aLast, aV[0].Oriented(TopAbs_Orientation.TopAbs_REVERSED));
        aB.Add(aW, aLast);
        aB.Add(aF, aW);
        return TestShapes.Frozen(aF);
    }

    [Fact]
    public void Curves3dBuilt_builds_the_missing_curve_and_keeps_the_other_edges()
    {
        TopoDS_Face aF = SquareWithAPcurveOnlyEdge();
        AssertTwin(aF, s => BRepLib.Curves3dBuilt(s, 1e-5, out _), s => BRepLib.BuildCurves3d(s, 1e-5));

        TopoDS_Shape aResult = BRepLib.Curves3dBuilt(aF, 1e-5, out bool isDone, out BRepTools_History aH);
        Assert.True(isDone);
        Assert.Equal(3, CountShared(aF, aResult, TopAbs_EDGE));
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.NotNull(BRep_Tool.Curve(TopoDS.Edge(anExp.Current()), out _, out _));
        }
        Assert.True(aH.HasModified());
        Assert.True(aH.IsRemoved(aF) == false);
    }

    [Fact]
    public void A_twin_that_changes_nothing_returns_its_input()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        TopoDS_Shape aResult = BRepLib.Curves3dBuilt(aBox, 1e-5, out bool isDone, out BRepTools_History aH);
        Assert.True(isDone);
        Assert.True(aResult.IsEqual(aBox));
        Assert.False(aH.HasModified());

        TopoDS_Shape anEncoded = BRepLib.RegularityEncoded(aBox);
        TopoDS_Shape aTwice = BRepLib.RegularityEncoded(anEncoded);
        Assert.True(aTwice.IsEqual(anEncoded));
    }

    [Fact]
    public void RegularityEncoded_codes_the_edges_of_a_frozen_box()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        AssertTwin(aBox, s => BRepLib.RegularityEncoded(s), s => BRepLib.EncodeRegularity(s));
        TopoDS_Shape aResult = BRepLib.RegularityEncoded(aBox, out BRepTools_History aH);
        var anEF = new NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aResult, TopAbs_EDGE, TopAbs_FACE, anEF);
        for (int i = 1; i <= anEF.Extent(); i++)
        {
            TopoDS_Edge anE = TopoDS.Edge(anEF.FindKey(i));
            Assert.Equal(Continuity.C0,
                         BRep_Tool.Continuity(anE, TopoDS.Face(anEF[i].First()), TopoDS.Face(anEF[i].Last())));
        }
        // every edge changed, the vertices didn't
        Assert.Equal(0, CountShared(aBox, aResult, TopAbs_EDGE));
        Assert.Equal(8, CountShared(aBox, aResult, TopAbs_VERTEX));
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            Assert.Single(aH.Modified(anExp.Current()));
        }
    }

    [Fact]
    public void RegularityEncoded_of_listed_edges_codes_only_those()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Shape anEdge = new TopExp_Explorer(aBox, TopAbs_EDGE).Current();
        TopoDS_Shape aResult = BRepLib.RegularityEncoded(aBox, [anEdge], out BRepTools_History aH);
        Assert.Single(aH.Modified(anEdge));
        Assert.Equal(11, CountShared(aBox, aResult, TopAbs_EDGE));
    }

    [Fact]
    public void SameParametered_makes_the_pcurve_follow_the_3d_curve()
    {
        var aB = new BRep_Builder();
        var aPlane = new Geom_Plane(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        TopoDS_Face aF = new();
        aB.MakeFace(ref aF, aPlane, Conf);
        TopoDS_Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var aBent = new Geom2d_BSplineCurve(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(2, 0), new Point2D(10, 0)]),
                                            NCollection_Array1<double>.From([0.0, 10.0]), NCollection_Array1<int>.From([3, 3]), 2);
        aB.UpdateEdge(anE, aBent, aF, Conf);
        aB.SameParameter(anE, false);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        TestShapes.Frozen(aF);

        AssertTwin(aF, s => BRepLib.SameParametered(s, 1e-5), s => BRepLib.SameParameter(s, 1e-5));
        TopoDS_Shape aResult = BRepLib.SameParametered(aF, out BRepTools_History aH, 1e-5);
        TopoDS_Edge aNewE = TopoDS.Edge(new TopExp_Explorer(aResult, TopAbs_EDGE).Current());
        Assert.True(BRep_Tool.SameParameter(aNewE));
        Assert.False(BRep_Tool.SameParameter(anE));
        Assert.Single(aH.Modified(anE));
        Point2D aUV = BRep_Tool.CurveOnSurface(aNewE, TopoDS.Face(aResult), out _, out _)!.Value(5);
        Assert.Equal(5, aUV.X(), 1e-5);
    }

    [Fact]
    public void SameParametered_of_a_same_parameter_box_only_moves_its_tolerances()
    {
        // OCCT's UpdateTolerances (which SameParameter ends with) adds its rounding guard to every vertex tolerance, so
        // the vertices change by two units in the last place, the edges holding them and the faces with them; the
        // curves and surfaces are the box's own
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        AssertTwin(aBox, s => BRepLib.SameParametered(s), s => BRepLib.SameParameter(s));
        TopoDS_Shape aResult = BRepLib.SameParametered(aBox, out BRepTools_History aH);
        Assert.Equal(0, CountShared(aBox, aResult, TopAbs_VERTEX));
        for (var anExp = new TopExp_Explorer(aBox, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            TopoDS_Edge anE = TopoDS.Edge(anExp.Current());
            TopoDS_Edge aNewE = TopoDS.Edge(Assert.Single(aH.Modified(anE)));
            Assert.Same(BRep_Tool.Curve(anE, out _, out _), BRep_Tool.Curve(aNewE, out _, out _));
            Assert.Equal(BRep_Tool.Tolerance(anE), BRep_Tool.Tolerance(aNewE));
        }
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            Assert.Equal(Conf + 2 * Epsilon(Conf), BRep_Tool.Tolerance(TopoDS.Vertex(anExp.Current())));
        }
    }

    [Fact]
    public void TolerancesUpdated_raises_the_vertices_to_their_edges()
    {
        var aB = new BRep_Builder();
        TopoDS_Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        aB.UpdateEdge(anE, 1e-3);
        TestShapes.Frozen(anE);
        AssertTwin(anE, s => BRepLib.TolerancesUpdated(s), s => BRepLib.UpdateTolerances(s));
        TopoDS_Shape aResult = BRepLib.TolerancesUpdated(anE, out BRepTools_History aH);
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            Assert.True(BRep_Tool.Tolerance(TopoDS.Vertex(anExp.Current())) >= 1e-3);
        }
        for (var anExp = new TopExp_Explorer(anE, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            Assert.Equal(Conf, BRep_Tool.Tolerance(TopoDS.Vertex(anExp.Current())));
            Assert.Single(aH.Modified(anExp.Current()));
        }
    }

    [Fact]
    public void InnerTolerancesUpdated_covers_a_vertex_off_the_curve()
    {
        var aB = new BRep_Builder();
        TopoDS_Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0, 0));
        TopoDS_Vertex aV2 = new();
        aB.MakeVertex(ref aV2, new Point3D(1, 0.01, 0), Conf);
        TopoDS_Edge anE = TestShapes.Edge(new Geom_Line(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 1, aV1, aV2);
        TestShapes.Frozen(anE);
        AssertTwin(anE, s => BRepLib.InnerTolerancesUpdated(s), s => BRepLib.UpdateInnerTolerances(s));
        TopoDS_Shape aResult = BRepLib.InnerTolerancesUpdated(anE);
        TopExp.Vertices(TopoDS.Edge(aResult), out _, out TopoDS_Vertex aNewV2);
        Assert.True(BRep_Tool.Tolerance(aNewV2) >= 0.01);

        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        AssertTwin(aBox, s => BRepLib.InnerTolerancesUpdated(s), s => BRepLib.UpdateInnerTolerances(s));
    }

    [Fact]
    public void ClosedSolidOriented_reverses_a_reversed_box_without_copying()
    {
        TopoDS_Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        TopoDS_Solid aReversed = TopoDS.Solid(aBox.Reversed());
        TopoDS_Solid aResult = BRepLib.ClosedSolidOriented(aReversed, out bool isOriented);
        Assert.True(isOriented);
        Assert.True(aResult.IsSame(aBox));
        Assert.Equal(aBox.Orientation(), aResult.Orientation());
        Assert.True(BRepLib.ClosedSolidOriented(aBox, out isOriented).IsEqual(aBox));
        Assert.True(isOriented);
    }

    [Fact]
    public void Twins_of_a_null_shape_return_it()
    {
        TopoDS_Shape aNull = new();
        Assert.True(BRepLib.SameParametered(aNull).IsNull());
        Assert.True(BRepLib.TolerancesUpdated(aNull).IsNull());
        Assert.True(BRepLib.InnerTolerancesUpdated(aNull).IsNull());
        Assert.True(BRepLib.Curves3dBuilt(aNull, out _).IsNull());
        Assert.True(BRepLib.RegularityEncoded(aNull).IsNull());
        Assert.True(BRepLib.DeflectionUpdated(aNull).IsNull());
        Assert.True(BRepLib.NormalConsistencyEnsured(aNull, out _).IsNull());
    }
}
