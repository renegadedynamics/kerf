// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_Face's fixes on frozen faces (the caller's face unchanged, the result frozen, the caller's own sub-shapes
/// kept where nothing changed, the context answering for the caller's shapes) and on unfrozen ones (fixed in place, the
/// internal results unfrozen): missing seams on a cylinder and a sphere (through ShapeFix_ComposeShell), faces made from
/// a surface and wires, natural bounds, orientation, small-area wires, two-edge coincident wires, split faces and loops.
/// </summary>
public class ShapeFix_FaceTests
{
    private const double Conf = 1e-7;

    private static Frame3D ZAx2(double theZ = 0.0) => new(new Point3D(0, 0, theZ), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));

    private static CoordinateSystem ZAx3() => new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));

    private static Wire CircleWire(double theZ, double theR)
    {
        Edge anEdge = new MakeEdge(new CircleCurve3D(ZAx2(theZ), theR)).Edge();
        return new MakeWire(anEdge).Wire();
    }

    private static Wire PolygonWire(params Point3D[] thePoints)
    {
        var aPoly = new MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aPoly.Add(aP);
        }
        aPoly.Close();
        return aPoly.Wire();
    }

    private static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        var aList = new List<Shape>();
        for (int i = 0; i < aMap.Count; i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    private static int Count(in Shape theShape, ShapeType theType) => SubShapes(theShape, theType).Count;

    private static bool HasTShapeOf(in Shape theIn, in Shape theShape, ShapeType theType)
    {
        TShape? aTShape = theShape.TShape();
        return SubShapes(theIn, theType).Any(s => ReferenceEquals(s.TShape(), aTShape));
    }

    private static void AssertFrozen(in Shape theShape)
    {
        Assert.False(theShape.IsNull);
        Assert.True(theShape.TShape()!.IsFrozen);
        foreach (Shape anE in SubShapes(theShape, ShapeType.Edge))
        {
            Assert.True(anE.TShape()!.IsFrozen);
        }
    }

    // A face on a cylinder bounded by two 3d circles without pcurves, as build123d.NET's MakeFaceOnSurface makes it.
    private static Face CylinderBand(double theR, double theH)
    {
        var aMaker = new MakeFace(new CylindricalSurface(ZAx3(), theR), CircleWire(0.0, theR), true);
        aMaker.Add(ShapeCasts.Wire(CircleWire(theH, theR).Reversed()));
        Assert.True(aMaker.IsDone);
        return aMaker.Face();
    }

    [Fact]
    public void MissingSeam_OnACylinderBand_TheFrozenInputIsUnchanged_TheResultIsValidAndFrozen()
    {
        Face aFace = CylinderBand(5.0, 10.0);
        Assert.True(aFace.TShape()!.IsFrozen);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aContext = new ReShapeBuilder();

        var aFix = new FaceFixer(aFace);
        aFix.SetContext(aContext);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ExtendStatus.DONE3)); // the seam
        Shape aResult = aFix.Result();

        aSnap.AssertUnchanged(aFace);
        AssertFrozen(aResult);
        Assert.Equal(ShapeType.Face, aResult.Type);
        Assert.Equal(3, Count(aResult, ShapeType.Edge)); // two circles and the seam
        Assert.True(new Analyzer(aResult).IsValid());

        // the context answers for the caller's face and edges
        Assert.True(ReferenceEquals(aContext.Apply(aFace).TShape(), aResult.TShape()));
        foreach (Shape anEdge in SubShapes(aFace, ShapeType.Edge))
        {
            Assert.True(aContext.IsRecorded(anEdge));
            Assert.False(aContext.Value(anEdge).IsNull);
        }
        Assert.NotEmpty(aContext.History().Modified(aFace));
    }

    [Fact]
    public void MissingSeam_FromASurfaceAndWires_OnASphereCap()
    {
        var aSphere = new SphericalSurface(ZAx3(), 4.0);
        double v = 0.5;
        Wire aWire = CircleWire(4.0 * Math.Sin(v), 4.0 * Math.Cos(v));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);

        var aFix = new FaceFixer();
        aFix.Init(aSphere, Conf);
        aFix.Add(aWire);
        aFix.SetContext(new ReShapeBuilder());
        Assert.True(aFix.Perform());
        Shape aResult = aFix.Result();

        aSnap.AssertUnchanged(aWire);
        AssertFrozen(aResult);
        Assert.True(aFix.Status(ExtendStatus.DONE3));
        Assert.True(new Analyzer(aResult).IsValid());
        // a seam and the degenerated edge at the pole were added
        Assert.Contains(SubShapes(aResult, ShapeType.Edge), e => Tool.Degenerated(ShapeCasts.Edge(e)));
    }

    [Fact]
    public void NaturalBound_AFaceWithoutWiresOnASphere_GetsItsNaturalBounds()
    {
        var aFix = new FaceFixer();
        aFix.Init(new SphericalSurface(ZAx3(), 2.0), Conf);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ExtendStatus.DONE5));
        Face aResult = aFix.Face();
        AssertFrozen(aResult);
        Assert.Equal(1, Count(aResult, ShapeType.Wire));
        Assert.True(new Analyzer(aResult).IsValid());
    }

    [Fact]
    public void FixOrientation_AMisorientedHole_IsReversed_TheOuterWireIsTheCallers()
    {
        Wire anOuter = PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0));
        Wire aHole = PolygonWire(new Point3D(3, 3, 0), new Point3D(6, 3, 0), new Point3D(6, 6, 0), new Point3D(3, 6, 0));
        var aMaker = new MakeFace(anOuter, true);
        aMaker.Add(aHole); // counter-clockwise, as the outer wire: misoriented
        Face aFace = aMaker.Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);

        var aFix = new FaceFixer(aFace);
        Assert.True(aFix.FixOrientation());
        Face aResult = aFix.Face();

        aSnap.AssertUnchanged(aFace);
        AssertFrozen(aResult);
        Assert.True(new Analyzer(aResult).IsValid());
        // all the edges and vertices are the caller's (reversing a wire changes no edge)
        foreach (Shape anEdge in SubShapes(aFace, ShapeType.Edge))
        {
            Assert.True(HasTShapeOf(aResult, anEdge, ShapeType.Edge));
        }
        Assert.True(HasTShapeOf(aResult, anOuter, ShapeType.Wire));
    }

    [Fact]
    public void Perform_AValidFace_GivesBackTheCallersFace()
    {
        Face aFace = new MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.0, 1.0, 0.0, 1.0).Face();
        var aContext = new ReShapeBuilder();
        var aFix = new FaceFixer(aFace);
        aFix.SetContext(aContext);
        Assert.False(aFix.Perform());
        Assert.True(aFix.Status(ExtendStatus.OK));
        Assert.True(ReferenceEquals(aFix.Result().TShape(), aFace.TShape()));
        Assert.True(ReferenceEquals(aFix.Face().TShape(), aFace.TShape()));
        Assert.False(aContext.IsRecorded(aFace));
    }

    [Fact]
    public void FixSmallAreaWire_ASliverHoleIsRemoved()
    {
        Wire anOuter = PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0));
        Wire aSliver = PolygonWire(new Point3D(3, 3, 0), new Point3D(3, 7, 0), new Point3D(3 + 1e-9, 5, 0));
        var aMaker = new MakeFace(anOuter, true);
        aMaker.Add(aSliver);
        Face aFace = aMaker.Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);

        var aFix = new FaceFixer(aFace);
        aFix.SetContext(new ReShapeBuilder());
        Assert.True(aFix.FixSmallAreaWire(false));
        Face aResult = aFix.Face();

        aSnap.AssertUnchanged(aFace);
        AssertFrozen(aResult);
        Assert.Equal(1, Count(aResult, ShapeType.Wire));
        Assert.True(HasTShapeOf(aResult, anOuter, ShapeType.Wire));

        // with the face mode, a face of small-area wires only is removed through the context
        var aFace2 = new MakeFace(aSliver, true).Face();
        var aContext = new ReShapeBuilder();
        var aFix2 = new FaceFixer(aFace2);
        aFix2.SetContext(aContext);
        Assert.False(aFix2.FixSmallAreaWire(true));
        aFix2.Result(); // the public result re-keys the context to the caller's shapes
        Assert.True(aContext.Apply(aFace2).IsNull);
    }

    [Fact]
    public void FixSmallAreaWire_AForwardVertexOfTheFace_IsKeptAsOCCTsReleaseBuildDoes()
    {
        // OCCT's test lets a FORWARD vertex through to TopoDS::Wire (occt-issues.md); its release build keeps it
        var aB = new BrepBuilder();
        Face aFace = ShapeCasts.Face(new MakeFace(
            PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0)), true).Face().EmptyCopied());
        Face aSource = new MakeFace(
            PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0)), true).Face();
        for (var anIt = new Iterator(aSource); anIt.More(); anIt.Next())
        {
            aB.Add(aFace, anIt.Value());
        }
        aB.Add(aFace, TestShapes.Vertex(new Point3D(5, 5, 0))); // FORWARD
        TestShapes.Frozen(aFace);

        var aFix = new FaceFixer(aFace);
        Assert.False(aFix.FixSmallAreaWire(false));
        Assert.True(ReferenceEquals(aFix.Face().TShape(), aFace.TShape()));
    }

    [Fact]
    public void FixSplitFace_TwoOuterWires_MakeTwoFaces()
    {
        // two disjoint squares as the outer wires of one planar face
        Wire aW1 = PolygonWire(new Point3D(0, 0, 0), new Point3D(4, 0, 0), new Point3D(4, 4, 0), new Point3D(0, 4, 0));
        Wire aW2 = PolygonWire(new Point3D(6, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 4, 0), new Point3D(6, 4, 0));
        var aMaker = new MakeFace(aW1, true);
        aMaker.Add(aW2);
        Face aFace = aMaker.Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);

        var aFix = new FaceFixer(aFace);
        aFix.SetContext(new ReShapeBuilder());
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ExtendStatus.DONE8));
        Shape aResult = aFix.Result();

        aSnap.AssertUnchanged(aFace);
        AssertFrozen(aResult);
        Assert.Equal(ShapeType.Compound, aResult.Type);
        Assert.Equal(2, Count(aResult, ShapeType.Face));
        // the caller's wires, untouched, are in the new faces
        Assert.True(HasTShapeOf(aResult, aW1, ShapeType.Wire));
        Assert.True(HasTShapeOf(aResult, aW2, ShapeType.Wire));
    }

    [Fact]
    public void FixWiresTwoCoincEdges_AWireOfTwoCoincidentEdgesIsRemoved()
    {
        Wire anOuter = PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0));
        Edge anEdge = new MakeEdge(new Point3D(2, 2, 0), new Point3D(5, 5, 0)).Edge();
        var aB = new BrepBuilder();
        var aTwo = new Wire();
        aB.MakeWire(ref aTwo);
        aB.Add(aTwo, anEdge);
        aB.Add(aTwo, anEdge.Reversed());
        var aMaker = new MakeFace(anOuter, true);
        aMaker.Add(aTwo);
        Face aFace = aMaker.Face();

        var aFix = new FaceFixer(aFace);
        Assert.True(aFix.FixWiresTwoCoincEdges());
        Face aResult = aFix.Face();
        AssertFrozen(aResult);
        Assert.Equal(1, Count(aResult, ShapeType.Wire));
        Assert.True(HasTShapeOf(aResult, anOuter, ShapeType.Wire));
    }

    [Fact]
    public void UnfrozenFace_IsFixedInPlace_TheInternalResultsAreUnfrozen()
    {
        // a frozen face's sub-shapes would be copied; an unfrozen one (as ShapeFix_Shell gives) is worked in place
        var aB = new BrepBuilder();
        Wire anOuter = ShapeCasts.Wire(PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0)).EmptyCopied());
        var aFace = new Face();
        aB.MakeFace(ref aFace, new PlaneSurface(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Conf);
        Vertex[] aV =
        [
            TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(0, 10, 0)),
            TestShapes.Vertex(new Point3D(10, 10, 0)), TestShapes.Vertex(new Point3D(10, 0, 0)),
        ];
        for (int i = 0; i < 4; i++)
        {
            aB.Add(anOuter, TestShapes.Edge(aV[i], aV[(i + 1) % 4])); // clockwise: misoriented for a single wire
        }
        aB.Add(aFace, anOuter);
        Assert.False(TestCopies.HoldsFrozen(aFace));

        var aFix = new FaceFixer(aFace);
        Assert.True(aFix.FixOrientation());
        Face aResult = aFix.InternalFace();
        Assert.False(aResult.TShape()!.IsFrozen);
        // the edges are the very ones of the input (no copy was made)
        foreach (Shape anEdge in SubShapes(aFace, ShapeType.Edge))
        {
            Assert.True(HasTShapeOf(aResult, anEdge, ShapeType.Edge));
        }
        Assert.Equal(Forward, aResult.Orientation);
    }

    [Fact]
    public void AFixAfterAPublicResult_CopiesAgain()
    {
        Face aFace = CylinderBand(3.0, 4.0);
        var aFix = new FaceFixer(aFace);
        aFix.SetContext(new ReShapeBuilder());
        Assert.True(aFix.Perform());
        Face aFirst = aFix.Face();
        AssertFrozen(aFirst);
        // the published face is frozen: a further fix works on a copy of it and leaves it unchanged
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);
        aFix.FixOrientation();
        Face aSecond = aFix.Face();
        aSnap.AssertUnchanged(aFirst);
        AssertFrozen(aSecond);
    }

    [Fact]
    public void Perform_ACylinderFaceWithItsSeam_NeedsNothing()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aFix = new FaceFixer(aFace);
        aFix.Perform();
        Assert.False(aFix.Status(ExtendStatus.DONE3));
        Assert.True(new Analyzer(aFix.Result()).IsValid());
    }
}
