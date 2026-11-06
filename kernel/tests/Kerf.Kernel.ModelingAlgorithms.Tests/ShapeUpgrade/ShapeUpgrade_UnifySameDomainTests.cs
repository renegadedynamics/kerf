// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Reflection;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of ShapeUpgrade_UnifySameDomain on frozen inputs (unit 6U, plan 2.7): the caller's shape is never
/// written whatever the safe input mode, the result is frozen, a shape needing nothing comes back as itself, the
/// unchanged sub-shapes are the caller's own, the history answers for the caller's sub-shapes, and an unfrozen input
/// (ported code's) is worked on in place, as in OCCT.
/// </summary>
public class ShapeUpgrade_UnifySameDomainTests
{
    private static string Metadata(string theKey) =>
        typeof(ShapeUpgrade_UnifySameDomainTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    // A shell of two coplanar rectangles [0, 5] x [0, 4] and [5, 10] x [0, 4] sharing the edge x = 5, built as a
    // caller would get it: the faces are planar faces of their own (two Geom_Plane objects).
    private static Shell TwoCoplanarFaces(bool theToFreeze, out Edge theSharedEdge)
    {
        var B = new BrepBuilder();
        Point3D[] aP = [new(0, 0, 0), new(5, 0, 0), new(10, 0, 0), new(10, 4, 0), new(5, 4, 0), new(0, 4, 0)];
        var aV = new Vertex[6];
        for (int i = 0; i < 6; i++)
        {
            B.MakeVertex(ref aV[i], aP[i], global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        Edge E(int i, int j) => new MakeEdge(aV[i], aV[j]).InternalEdge();
        Edge e01 = E(0, 1), e12 = E(1, 2), e23 = E(2, 3), e34 = E(3, 4), e45 = E(4, 5), e50 = E(5, 0), e14 = E(1, 4);

        Face F(params Edge[] theEdges)
        {
            var aW = new Wire();
            B.MakeWire(ref aW);
            foreach (Edge anEdge in theEdges)
            {
                B.Add(aW, anEdge);
            }
            aW.Closed(true);
            return new MakeFace(new Plane3D(new CoordinateSystem(Constants.XOY())), aW).InternalFace();
        }
        Face aLeft = F(e01, e14, e45, e50);
        Face aRight = F(e12, e23, e34, ShapeCasts.Edge(e14.Reversed()));

        var aShell = new Shell();
        B.MakeShell(ref aShell);
        B.Add(aShell, aLeft);
        B.Add(aShell, aRight);
        if (theToFreeze)
        {
            aShell.TShape()!.FreezeGraph();
        }
        theSharedEdge = e14;
        return aShell;
    }

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static HashSet<TShape> TShapes(in Shape theShape, ShapeType theType)
    {
        var aSet = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        for (var anExp = new Explorer(theShape, theType); anExp.More(); anExp.Next())
        {
            aSet.Add(anExp.Current().TShape()!);
        }
        return aSet;
    }

    private static bool AllFrozen(in Shape theShape)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        return aMap.All(s => s.TShape()!.IsFrozen);
    }

    [Fact]
    public void A_box_cut_by_a_sphere_stays_valid_and_keeps_its_volume()
    {
        // #1541: the circle on the box's face is two arcs. Geom2dConvert's ConcatC1 prepended the second, so the merged
        // pcurve started half a turn from the 3D circle: a vertex tolerance of the circle's diameter and a missing face.
        var aSphere = new MakeSphere(new Point3D(-0.8941468682889828, -0.05608050987114277, 0.16128077950256148), 0.5).Shape();
        Shape aCut = new Cut(new MakeBox(new Point3D(-0.5, -0.5, -0.5), 1, 1, 1).Shape(), aSphere).Shape();
        var aUnifier = new UnifySameDomain(aCut, true, true, true);
        aUnifier.Build();

        Assert.True(new Analyzer(aUnifier.Shape()).IsValid());
        Properties aBefore = new();
        Properties anAfter = new();
        MassPropertiesBuilder.VolumeProperties(aCut, aBefore);
        MassPropertiesBuilder.VolumeProperties(aUnifier.Shape(), anAfter);
        Assert.Equal(aBefore.Mass(), anAfter.Mass(), 1e-6);
    }

    [Fact]
    public void FrozenShell_CoplanarFacesAreMerged_TheInputIsUnchanged_TheHistoryAnswersForTheCallersShapes()
    {
        Shell aShell = TwoCoplanarFaces(true, out Edge aShared);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);

        var aUnifier = new UnifySameDomain(aShell, true, true, true);
        aUnifier.AllowInternalEdges(false);
        aUnifier.Build();
        Shape aResult = aUnifier.Shape();

        aSnap.AssertUnchanged(aShell);
        Assert.True(AllFrozen(aResult));
        Assert.Equal(1, Count(aResult, ShapeType.Face));
        // the bottom and top edges are merged across x = 5: 4 edges, 4 vertices
        Assert.Equal(4, Count(aResult, ShapeType.Edge));
        Assert.Equal(4, Count(aResult, ShapeType.Vertex));

        // the history answers for the caller's sub-shapes
        History aHistory = aUnifier.History()!;
        Shape aFace = new Explorer(aResult, ShapeType.Face).Current();
        for (var anExp = new Explorer(aShell, ShapeType.Face); anExp.More(); anExp.Next())
        {
            Assert.Single(aHistory.Modified(anExp.Current()));
            Assert.True(aHistory.Modified(anExp.Current())[0].IsSame(aFace));
        }
        Assert.True(aHistory.IsRemoved(aShared));

        // faces only: the six boundary edges are the caller's own
        var aFacesOnly = new UnifySameDomain(aShell, false, true);
        aFacesOnly.Build();
        Shape aResult2 = aFacesOnly.Shape();
        aSnap.AssertUnchanged(aShell);
        Assert.Equal(6, TShapes(aResult2, ShapeType.Edge).Intersect(TShapes(aShell, ShapeType.Edge)).Count());
        Assert.Equal(6, Count(aResult2, ShapeType.Edge));
    }

    [Fact]
    public void NothingToUnify_GivesBackTheInputItself()
    {
        Shape aBox = new MakeBox(new Point3D(0, 0, 0), new Point3D(10, 10, 10)).Shape();
        var aUnifier = new UnifySameDomain(aBox, true, true, true);
        aUnifier.Build();
        Shape aResult = aUnifier.Shape();
        Assert.Same(aBox.TShape(), aResult.TShape());
        History aHistory = aUnifier.History()!;
        Assert.False(aHistory.HasModified);
        Assert.False(aHistory.HasRemoved);
    }

    [Fact]
    public void SafeInputModeOff_OnAFrozenInput_ActsOnACopy()
    {
        Shell aShell = TwoCoplanarFaces(true, out _);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShell);

        var aUnifier = new UnifySameDomain(aShell);
        aUnifier.SetSafeInputMode(false);
        aUnifier.Build();
        Shape aResult = aUnifier.Shape();

        aSnap.AssertUnchanged(aShell);
        Assert.Equal(1, Count(aResult, ShapeType.Face));
        Assert.Equal(4, Count(aResult, ShapeType.Edge));
    }

    [Fact]
    public void UnfrozenInput_IsWorkedOnInPlace_AndTheResultIsInternal()
    {
        Shell aShell = TwoCoplanarFaces(false, out _);
        HashSet<TShape> anEdges = TShapes(aShell, ShapeType.Edge);

        var aUnifier = new UnifySameDomain(aShell, false, true);
        aUnifier.Build();
        Shape anInternal = aUnifier.InternalShape();
        Assert.False(anInternal.TShape()!.IsFrozen);
        Assert.Equal(1, Count(anInternal, ShapeType.Face));
        // no copy: the boundary edges are the input's, as in OCCT
        Assert.Equal(6, TShapes(anInternal, ShapeType.Edge).Intersect(anEdges).Count());
        Assert.True(aUnifier.Shape().TShape()!.IsFrozen);
    }

    [Fact]
    public void KeepShape_TheCallersEdge_ForbidsTheMerge()
    {
        Shell aShell = TwoCoplanarFaces(true, out Edge aShared);
        var aUnifier = new UnifySameDomain(aShell);
        aUnifier.KeepShape(aShared);
        aUnifier.Build();
        Shape aResult = aUnifier.Shape();
        Assert.Equal(2, Count(aResult, ShapeType.Face));
        Assert.Same(aShell.TShape(), aResult.TShape());

        // a kept vertex forbids the merge of the edges through it
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(aShell, ShapeType.Vertex, aMap);
        var aUnifier2 = new UnifySameDomain(aShell);
        for (int i = 0; i < aMap.Count; i++)
        {
            aUnifier2.KeepShape(aMap[i]);
        }
        aUnifier2.Build();
        Shape aResult2 = aUnifier2.Shape();
        Assert.Equal(1, Count(aResult2, ShapeType.Face));
        Assert.Equal(6, Count(aResult2, ShapeType.Edge));
    }

    [Fact]
    public void NullHistory_CollectsNone_AndInitializeReuses()
    {
        Shell aShell = TwoCoplanarFaces(true, out _);
        var aUnifier = new UnifySameDomain();
        aUnifier.Initialize(aShell);
        aUnifier.History(null);
        aUnifier.Build();
        Assert.Null(aUnifier.History());
        Assert.Equal(1, Count(aUnifier.Shape(), ShapeType.Face));

        Shell aShell2 = TwoCoplanarFaces(true, out _);
        aUnifier.History(new History());
        aUnifier.Initialize(aShell2, true, false);
        aUnifier.Build();
        // edges only: the faces stay, nothing to merge
        Assert.Equal(2, Count(aUnifier.Shape(), ShapeType.Face));
        Assert.Same(aShell2.TShape(), aUnifier.Shape().TShape());
    }

    // heal group's split_closed_faces/F6 input: Clean (ConcatBSplines) makes a closed B-spline surface periodic, which
    // OCCT does in place on the face's own surface (decision 769)
    [Fact]
    public void PeriodicSurface_IsACopy_TheCallersSurfaceIsUnchanged()
    {
        string aRoot = Metadata("OcctTestDataDir");
        string aFile = Path.Combine(aRoot, "brep", "bowl_noremove.brep");
        if (string.IsNullOrEmpty(aRoot) || !File.Exists(aFile))
        {
            return;
        }
        Assert.True(BrepTools.Read(out Shape aShape, aFile));
        aShape.TShape()!.FreezeGraph();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);
        var aSurfaces = new List<(BSplineSurface, bool, bool)>();
        for (var anExp = new Explorer(aShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            if (Tool.Surface(ShapeCasts.Face(anExp.Current())) is BSplineSurface aBS)
            {
                aSurfaces.Add((aBS, aBS.IsUPeriodic, aBS.IsVPeriodic));
            }
        }

        var aUnifier = new UnifySameDomain(aShape, true, true, true);
        aUnifier.Build();
        Shape aResult = aUnifier.Shape();

        aSnap.AssertUnchanged(aShape);
        Assert.All(aSurfaces, s => Assert.Equal((s.Item2, s.Item3), (s.Item1.IsUPeriodic, s.Item1.IsVPeriodic)));
        bool hasPeriodic = false;
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
        {
            if (Tool.Surface(ShapeCasts.Face(anExp.Current())) is BSplineSurface aBS && (aBS.IsUPeriodic || aBS.IsVPeriodic))
            {
                hasPeriodic = true;
                Assert.DoesNotContain(aSurfaces, s => ReferenceEquals(s.Item1, aBS));
            }
        }
        Assert.True(hasPeriodic);
        Assert.True(AllFrozen(aResult));
    }

    // A cubic B-spline edge through theFrom and theTo, with the tangent at the end named made zero by repeating that pole
    private static BSplineCurve3D CubicWithZeroTangent(Point3D theFrom, Point3D theTo, bool theAtEnd)
    {
        Vector3D aStep = new Vector3D(theFrom, theTo) / 3;
        Point3D aSecond = theFrom.Translated(aStep);
        Point3D aThird = theFrom.Translated(aStep * 2);
        Point3D[] aPoles = theAtEnd ? [theFrom, aSecond, theTo, theTo] : [theFrom, theFrom, aThird, theTo];
        return new BSplineCurve3D(aPoles, [0.0, 1.0], [4, 4], 3);
    }

    private static Wire WireOf(params Edge[] theEdges)
    {
        var B = new BrepBuilder();
        var aWire = new Wire();
        B.MakeWire(ref aWire);
        foreach (Edge anEdge in theEdges)
        {
            B.Add(aWire, anEdge);
        }
        return aWire;
    }

    // Decision 1652: OCCT's Diff1.Angle(Diff2) raises here, and the whole of the unification with it
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ZeroTangentWhereTwoEdgesMeet_TheyAreLeftUnmerged(bool theOnTheFirstEdge)
    {
        Point3D aStart = new(0, 0, 0), aMiddle = new(3, 0, 0), anEnd = new(6, 0, 0);
        Vertex aV0 = new MakeVertex(aStart).Vertex(), aV1 = new MakeVertex(aMiddle).Vertex(), aV2 = new MakeVertex(anEnd).Vertex();
        Curve3D aFirst = theOnTheFirstEdge
            ? CubicWithZeroTangent(aStart, aMiddle, true)
            : new BSplineCurve3D([aStart, new(1, 0, 0), new(2, 0, 0), aMiddle], [0.0, 1.0], [4, 4], 3);
        Curve3D aSecond = theOnTheFirstEdge
            ? new BSplineCurve3D([aMiddle, new(4, 0, 0), new(5, 0, 0), anEnd], [0.0, 1.0], [4, 4], 3)
            : CubicWithZeroTangent(aMiddle, anEnd, false);
        Wire aWire = WireOf(new MakeEdge(aFirst, aV0, aV1).InternalEdge(), new MakeEdge(aSecond, aV1, aV2).InternalEdge());

        var aUnifier = new UnifySameDomain(aWire, true, false, false);
        aUnifier.Build();

        Assert.Equal(2, Count(aUnifier.Shape(), ShapeType.Edge));
        Assert.Equal(3, Count(aUnifier.Shape(), ShapeType.Vertex));
    }

    // Decision 1653: OCCT's CheckSharedVertices looks up the null vertex of an edge with none, and raises. One infinite
    // line has two such ends, as bug27333_simple.stp's has; a second gives the same with a further null
    [Theory]
    [InlineData(1)]
    [InlineData(2)]
    public void InfiniteLines_WithNoVertices_ComeThroughUnchanged(int theNbLines)
    {
        var B = new BrepBuilder();
        var aCompound = new Compound();
        B.MakeCompound(ref aCompound);
        for (int i = 0; i < theNbLines; i++)
        {
            B.Add(aCompound, new MakeEdge(new Line3D(new Point3D(i, 0, 0), Direction3D.UnitZ)).InternalEdge());
        }

        var aUnifier = new UnifySameDomain(aCompound, true, true, false);
        aUnifier.Build();

        Assert.Equal(theNbLines, Count(aUnifier.Shape(), ShapeType.Edge));
        Assert.Equal(0, Count(aUnifier.Shape(), ShapeType.Vertex));
    }
}
