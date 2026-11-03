// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Text.RegularExpressions;
using OcctNet.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

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
    private static string Text(in Shape theShape)
    {
        var aWriter = new StringWriter();
        BrepTools.Write(theShape, aWriter);
        return Regex.Replace(aWriter.ToString(), "(?m)^[01]{7}$", "flags");
    }

    private static void AssertTwin(in Shape theInput, Func<Shape, Shape> theTwin, Action<Shape> theStatic)
    {
        ShapeSnapshot aSnap = ShapeSnapshot.Of(theInput);
        Shape aResult = theTwin(theInput);
        aSnap.AssertUnchanged(theInput);
        Assert.True(aResult.TShape()!.IsFrozen);

        Shape aCopy = BrepTools.ExactCopyForWrite(theInput, out _);
        theStatic(aCopy);
        Assert.Equal(Text(aCopy), Text(aResult));
    }

    private static int CountShared(in Shape theA, in Shape theB, ShapeType theType)
    {
        var aMapA = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        var aMapB = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theA, theType, aMapA);
        Exploration.MapShapes(theB, theType, aMapB);
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
    private static Face SquareWithAPcurveOnlyEdge()
    {
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, Conf);
        Point3D[] aP = [new(0, 0, 0), new(2, 0, 0), new(2, 2, 0), new(0, 2, 0)];
        Vertex[] aV = aP.Select(p => TestShapes.Vertex(p)).ToArray();
        Wire aW = new();
        aB.MakeWire(ref aW);
        for (int i = 0; i < 3; i++)
        {
            aB.Add(aW, TestShapes.Edge(aV[i], aV[i + 1]));
        }
        Edge aLast = new();
        aB.MakeEdge(ref aLast);
        aB.UpdateEdge(aLast, new LineCurve2D(new Point2D(0, 2), new Direction2D(0, -1)), aF, Conf);
        aB.Range(aLast, 0, 2);
        aB.Add(aLast, aV[3].Oriented(Orientation.Forward));
        aB.Add(aLast, aV[0].Oriented(Orientation.Reversed));
        aB.Add(aW, aLast);
        aB.Add(aF, aW);
        return TestShapes.Frozen(aF);
    }

    [Fact]
    public void Curves3dBuilt_builds_the_missing_curve_and_keeps_the_other_edges()
    {
        Face aF = SquareWithAPcurveOnlyEdge();
        AssertTwin(aF, s => BrepLibrary.Curves3dBuilt(s, 1e-5, out _), s => BrepLibrary.BuildCurves3d(s, 1e-5));

        Shape aResult = BrepLibrary.Curves3dBuilt(aF, 1e-5, out bool isDone, out History aH);
        Assert.True(isDone);
        Assert.Equal(3, CountShared(aF, aResult, ShapeType.Edge));
        for (var anExp = new Explorer(aResult, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.NotNull(Tool.Curve(ShapeCasts.Edge(anExp.Current()), out _, out _));
        }
        Assert.True(aH.HasModified);
        Assert.True(aH.IsRemoved(aF) == false);
    }

    [Fact]
    public void A_twin_that_changes_nothing_returns_its_input()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        Shape aResult = BrepLibrary.Curves3dBuilt(aBox, 1e-5, out bool isDone, out History aH);
        Assert.True(isDone);
        Assert.True(aResult.IsEqual(aBox));
        Assert.False(aH.HasModified);

        Shape anEncoded = BrepLibrary.RegularityEncoded(aBox);
        Shape aTwice = BrepLibrary.RegularityEncoded(anEncoded);
        Assert.True(aTwice.IsEqual(anEncoded));
    }

    [Fact]
    public void RegularityEncoded_codes_the_edges_of_a_frozen_box()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        AssertTwin(aBox, s => BrepLibrary.RegularityEncoded(s), s => BrepLibrary.EncodeRegularity(s));
        Shape aResult = BrepLibrary.RegularityEncoded(aBox, out History aH);
        var anEF = new NCollection_IndexedDataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aResult, ShapeType.Edge, ShapeType.Face, anEF);
        for (int i = 1; i <= anEF.Extent(); i++)
        {
            Edge anE = ShapeCasts.Edge(anEF.FindKey(i));
            Assert.Equal(Continuity.C0,
                         Tool.Continuity(anE, ShapeCasts.Face(anEF[i].First()), ShapeCasts.Face(anEF[i].Last())));
        }
        // every edge changed, the vertices didn't
        Assert.Equal(0, CountShared(aBox, aResult, ShapeType.Edge));
        Assert.Equal(8, CountShared(aBox, aResult, ShapeType.Vertex));
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Assert.Single(aH.Modified(anExp.Current()));
        }
    }

    [Fact]
    public void RegularityEncoded_of_listed_edges_codes_only_those()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Shape anEdge = new Explorer(aBox, ShapeType.Edge).Current();
        Shape aResult = BrepLibrary.RegularityEncoded(aBox, [anEdge], out History aH);
        Assert.Single(aH.Modified(anEdge));
        Assert.Equal(11, CountShared(aBox, aResult, ShapeType.Edge));
    }

    [Fact]
    public void SameParametered_makes_the_pcurve_follow_the_3d_curve()
    {
        var aB = new BrepBuilder();
        var aPlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Face aF = new();
        aB.MakeFace(ref aF, aPlane, Conf);
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        var aBent = new BSplineCurve2D(NCollection_Array1<Point2D>.From([new Point2D(0, 0), new Point2D(2, 0), new Point2D(10, 0)]),
                                            NCollection_Array1<double>.From([0.0, 10.0]), NCollection_Array1<int>.From([3, 3]), 2);
        aB.UpdateEdge(anE, aBent, aF, Conf);
        aB.SameParameter(anE, false);
        Wire aW = new();
        aB.MakeWire(ref aW);
        aB.Add(aW, anE);
        aB.Add(aF, aW);
        TestShapes.Frozen(aF);

        AssertTwin(aF, s => BrepLibrary.SameParametered(s, 1e-5), s => BrepLibrary.SameParameter(s, 1e-5));
        Shape aResult = BrepLibrary.SameParametered(aF, out History aH, 1e-5);
        Edge aNewE = ShapeCasts.Edge(new Explorer(aResult, ShapeType.Edge).Current());
        Assert.True(Tool.SameParameter(aNewE));
        Assert.False(Tool.SameParameter(anE));
        Assert.Single(aH.Modified(anE));
        Point2D aUV = Tool.CurveOnSurface(aNewE, ShapeCasts.Face(aResult), out _, out _)!.Value(5);
        Assert.Equal(5, aUV.X(), 1e-5);
    }

    [Fact]
    public void SameParametered_of_a_same_parameter_box_only_moves_its_tolerances()
    {
        // OCCT's UpdateTolerances (which SameParameter ends with) adds its rounding guard to every vertex tolerance, so
        // the vertices change by two units in the last place, the edges holding them and the faces with them; the
        // curves and surfaces are the box's own
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 2, 3));
        AssertTwin(aBox, s => BrepLibrary.SameParametered(s), s => BrepLibrary.SameParameter(s));
        Shape aResult = BrepLibrary.SameParametered(aBox, out History aH);
        Assert.Equal(0, CountShared(aBox, aResult, ShapeType.Vertex));
        for (var anExp = new Explorer(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anE = ShapeCasts.Edge(anExp.Current());
            Edge aNewE = ShapeCasts.Edge(Assert.Single(aH.Modified(anE)));
            Assert.Same(Tool.Curve(anE, out _, out _), Tool.Curve(aNewE, out _, out _));
            Assert.Equal(Tool.Tolerance(anE), Tool.Tolerance(aNewE));
        }
        for (var anExp = new Explorer(aResult, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Assert.Equal(Conf + 2 * Epsilon(Conf), Tool.Tolerance(ShapeCasts.Vertex(anExp.Current())));
        }
    }

    [Fact]
    public void TolerancesUpdated_raises_the_vertices_to_their_edges()
    {
        var aB = new BrepBuilder();
        Edge anE = TestShapes.Edge(new Point3D(0, 0, 0), new Point3D(1, 0, 0));
        aB.UpdateEdge(anE, 1e-3);
        TestShapes.Frozen(anE);
        AssertTwin(anE, s => BrepLibrary.TolerancesUpdated(s), s => BrepLibrary.UpdateTolerances(s));
        Shape aResult = BrepLibrary.TolerancesUpdated(anE, out History aH);
        for (var anExp = new Explorer(aResult, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Assert.True(Tool.Tolerance(ShapeCasts.Vertex(anExp.Current())) >= 1e-3);
        }
        for (var anExp = new Explorer(anE, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            Assert.Equal(Conf, Tool.Tolerance(ShapeCasts.Vertex(anExp.Current())));
            Assert.Single(aH.Modified(anExp.Current()));
        }
    }

    [Fact]
    public void InnerTolerancesUpdated_covers_a_vertex_off_the_curve()
    {
        var aB = new BrepBuilder();
        Vertex aV1 = TestShapes.Vertex(new Point3D(0, 0, 0));
        Vertex aV2 = new();
        aB.MakeVertex(ref aV2, new Point3D(1, 0.01, 0), Conf);
        Edge anE = TestShapes.Edge(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 1, aV1, aV2);
        TestShapes.Frozen(anE);
        AssertTwin(anE, s => BrepLibrary.InnerTolerancesUpdated(s), s => BrepLibrary.UpdateInnerTolerances(s));
        Shape aResult = BrepLibrary.InnerTolerancesUpdated(anE);
        Exploration.Vertices(ShapeCasts.Edge(aResult), out _, out Vertex aNewV2);
        Assert.True(Tool.Tolerance(aNewV2) >= 0.01);

        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        AssertTwin(aBox, s => BrepLibrary.InnerTolerancesUpdated(s), s => BrepLibrary.UpdateInnerTolerances(s));
    }

    [Fact]
    public void ClosedSolidOriented_reverses_a_reversed_box_without_copying()
    {
        Solid aBox = TestShapes.Frozen(TestShapes.Box(1, 1, 1));
        Solid aReversed = ShapeCasts.Solid(aBox.Reversed());
        Solid aResult = BrepLibrary.ClosedSolidOriented(aReversed, out bool isOriented);
        Assert.True(isOriented);
        Assert.True(aResult.IsSame(aBox));
        Assert.Equal(aBox.Orientation(), aResult.Orientation());
        Assert.True(BrepLibrary.ClosedSolidOriented(aBox, out isOriented).IsEqual(aBox));
        Assert.True(isOriented);
    }

    [Fact]
    public void Twins_of_a_null_shape_return_it()
    {
        Shape aNull = new();
        Assert.True(BrepLibrary.SameParametered(aNull).IsNull);
        Assert.True(BrepLibrary.TolerancesUpdated(aNull).IsNull);
        Assert.True(BrepLibrary.InnerTolerancesUpdated(aNull).IsNull);
        Assert.True(BrepLibrary.Curves3dBuilt(aNull, out _).IsNull);
        Assert.True(BrepLibrary.RegularityEncoded(aNull).IsNull);
        Assert.True(BrepLibrary.DeflectionUpdated(aNull).IsNull);
        Assert.True(BrepLibrary.NormalConsistencyEnsured(aNull, out _).IsNull);
    }
}
