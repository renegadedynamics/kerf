// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

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

    private static TopoDS_Wire CircleWire(double theZ, double theR)
    {
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new Geom_Circle(ZAx2(theZ), theR)).Edge();
        return new BRepBuilderAPI_MakeWire(anEdge).Wire();
    }

    private static TopoDS_Wire PolygonWire(params Point3D[] thePoints)
    {
        var aPoly = new BRepBuilderAPI_MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aPoly.Add(aP);
        }
        aPoly.Close();
        return aPoly.Wire();
    }

    private static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        var aList = new List<TopoDS_Shape>();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    private static int Count(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType) => SubShapes(theShape, theType).Count;

    private static bool HasTShapeOf(in TopoDS_Shape theIn, in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        TopoDS_TShape? aTShape = theShape.TShape();
        return SubShapes(theIn, theType).Any(s => ReferenceEquals(s.TShape(), aTShape));
    }

    private static void AssertFrozen(in TopoDS_Shape theShape)
    {
        Assert.False(theShape.IsNull());
        Assert.True(theShape.TShape()!.IsFrozen);
        foreach (TopoDS_Shape anE in SubShapes(theShape, TopAbs_EDGE))
        {
            Assert.True(anE.TShape()!.IsFrozen);
        }
    }

    // A face on a cylinder bounded by two 3d circles without pcurves, as build123d.NET's MakeFaceOnSurface makes it.
    private static TopoDS_Face CylinderBand(double theR, double theH)
    {
        var aMaker = new BRepBuilderAPI_MakeFace(new Geom_CylindricalSurface(ZAx3(), theR), CircleWire(0.0, theR), true);
        aMaker.Add(TopoDS.Wire(CircleWire(theH, theR).Reversed()));
        Assert.True(aMaker.IsDone());
        return aMaker.Face();
    }

    [Fact]
    public void MissingSeam_OnACylinderBand_TheFrozenInputIsUnchanged_TheResultIsValidAndFrozen()
    {
        TopoDS_Face aFace = CylinderBand(5.0, 10.0);
        Assert.True(aFace.TShape()!.IsFrozen);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aContext = new ShapeBuild_ReShape();

        var aFix = new ShapeFix_Face(aFace);
        aFix.SetContext(aContext);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ShapeExtend_DONE3)); // the seam
        TopoDS_Shape aResult = aFix.Result();

        aSnap.AssertUnchanged(aFace);
        AssertFrozen(aResult);
        Assert.Equal(TopAbs_FACE, aResult.ShapeType());
        Assert.Equal(3, Count(aResult, TopAbs_EDGE)); // two circles and the seam
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());

        // the context answers for the caller's face and edges
        Assert.True(ReferenceEquals(aContext.Apply(aFace).TShape(), aResult.TShape()));
        foreach (TopoDS_Shape anEdge in SubShapes(aFace, TopAbs_EDGE))
        {
            Assert.True(aContext.IsRecorded(anEdge));
            Assert.False(aContext.Value(anEdge).IsNull());
        }
        Assert.NotEmpty(aContext.History().Modified(aFace));
    }

    [Fact]
    public void MissingSeam_FromASurfaceAndWires_OnASphereCap()
    {
        var aSphere = new Geom_SphericalSurface(ZAx3(), 4.0);
        double v = 0.5;
        TopoDS_Wire aWire = CircleWire(4.0 * Math.Sin(v), 4.0 * Math.Cos(v));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aWire);

        var aFix = new ShapeFix_Face();
        aFix.Init(aSphere, Conf);
        aFix.Add(aWire);
        aFix.SetContext(new ShapeBuild_ReShape());
        Assert.True(aFix.Perform());
        TopoDS_Shape aResult = aFix.Result();

        aSnap.AssertUnchanged(aWire);
        AssertFrozen(aResult);
        Assert.True(aFix.Status(ShapeExtend_DONE3));
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        // a seam and the degenerated edge at the pole were added
        Assert.Contains(SubShapes(aResult, TopAbs_EDGE), e => BRep_Tool.Degenerated(TopoDS.Edge(e)));
    }

    [Fact]
    public void NaturalBound_AFaceWithoutWiresOnASphere_GetsItsNaturalBounds()
    {
        var aFix = new ShapeFix_Face();
        aFix.Init(new Geom_SphericalSurface(ZAx3(), 2.0), Conf);
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ShapeExtend_DONE5));
        TopoDS_Face aResult = aFix.Face();
        AssertFrozen(aResult);
        Assert.Equal(1, Count(aResult, TopAbs_WIRE));
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void FixOrientation_AMisorientedHole_IsReversed_TheOuterWireIsTheCallers()
    {
        TopoDS_Wire anOuter = PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0));
        TopoDS_Wire aHole = PolygonWire(new Point3D(3, 3, 0), new Point3D(6, 3, 0), new Point3D(6, 6, 0), new Point3D(3, 6, 0));
        var aMaker = new BRepBuilderAPI_MakeFace(anOuter, true);
        aMaker.Add(aHole); // counter-clockwise, as the outer wire: misoriented
        TopoDS_Face aFace = aMaker.Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);

        var aFix = new ShapeFix_Face(aFace);
        Assert.True(aFix.FixOrientation());
        TopoDS_Face aResult = aFix.Face();

        aSnap.AssertUnchanged(aFace);
        AssertFrozen(aResult);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        // all the edges and vertices are the caller's (reversing a wire changes no edge)
        foreach (TopoDS_Shape anEdge in SubShapes(aFace, TopAbs_EDGE))
        {
            Assert.True(HasTShapeOf(aResult, anEdge, TopAbs_EDGE));
        }
        Assert.True(HasTShapeOf(aResult, anOuter, TopAbs_WIRE));
    }

    [Fact]
    public void Perform_AValidFace_GivesBackTheCallersFace()
    {
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0.0, 1.0, 0.0, 1.0).Face();
        var aContext = new ShapeBuild_ReShape();
        var aFix = new ShapeFix_Face(aFace);
        aFix.SetContext(aContext);
        Assert.False(aFix.Perform());
        Assert.True(aFix.Status(ShapeExtend_OK));
        Assert.True(ReferenceEquals(aFix.Result().TShape(), aFace.TShape()));
        Assert.True(ReferenceEquals(aFix.Face().TShape(), aFace.TShape()));
        Assert.False(aContext.IsRecorded(aFace));
    }

    [Fact]
    public void FixSmallAreaWire_ASliverHoleIsRemoved()
    {
        TopoDS_Wire anOuter = PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0));
        TopoDS_Wire aSliver = PolygonWire(new Point3D(3, 3, 0), new Point3D(3, 7, 0), new Point3D(3 + 1e-9, 5, 0));
        var aMaker = new BRepBuilderAPI_MakeFace(anOuter, true);
        aMaker.Add(aSliver);
        TopoDS_Face aFace = aMaker.Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);

        var aFix = new ShapeFix_Face(aFace);
        aFix.SetContext(new ShapeBuild_ReShape());
        Assert.True(aFix.FixSmallAreaWire(false));
        TopoDS_Face aResult = aFix.Face();

        aSnap.AssertUnchanged(aFace);
        AssertFrozen(aResult);
        Assert.Equal(1, Count(aResult, TopAbs_WIRE));
        Assert.True(HasTShapeOf(aResult, anOuter, TopAbs_WIRE));

        // with the face mode, a face of small-area wires only is removed through the context
        var aFace2 = new BRepBuilderAPI_MakeFace(aSliver, true).Face();
        var aContext = new ShapeBuild_ReShape();
        var aFix2 = new ShapeFix_Face(aFace2);
        aFix2.SetContext(aContext);
        Assert.False(aFix2.FixSmallAreaWire(true));
        aFix2.Result(); // the public result re-keys the context to the caller's shapes
        Assert.True(aContext.Apply(aFace2).IsNull());
    }

    [Fact]
    public void FixSmallAreaWire_AForwardVertexOfTheFace_IsKeptAsOCCTsReleaseBuildDoes()
    {
        // OCCT's test lets a FORWARD vertex through to TopoDS::Wire (occt-issues.md); its release build keeps it
        var aB = new BRep_Builder();
        TopoDS_Face aFace = TopoDS.Face(new BRepBuilderAPI_MakeFace(
            PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0)), true).Face().EmptyCopied());
        TopoDS_Face aSource = new BRepBuilderAPI_MakeFace(
            PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0)), true).Face();
        for (var anIt = new TopoDS_Iterator(aSource); anIt.More(); anIt.Next())
        {
            aB.Add(aFace, anIt.Value());
        }
        aB.Add(aFace, TestShapes.Vertex(new Point3D(5, 5, 0))); // FORWARD
        TestShapes.Frozen(aFace);

        var aFix = new ShapeFix_Face(aFace);
        Assert.False(aFix.FixSmallAreaWire(false));
        Assert.True(ReferenceEquals(aFix.Face().TShape(), aFace.TShape()));
    }

    [Fact]
    public void FixSplitFace_TwoOuterWires_MakeTwoFaces()
    {
        // two disjoint squares as the outer wires of one planar face
        TopoDS_Wire aW1 = PolygonWire(new Point3D(0, 0, 0), new Point3D(4, 0, 0), new Point3D(4, 4, 0), new Point3D(0, 4, 0));
        TopoDS_Wire aW2 = PolygonWire(new Point3D(6, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 4, 0), new Point3D(6, 4, 0));
        var aMaker = new BRepBuilderAPI_MakeFace(aW1, true);
        aMaker.Add(aW2);
        TopoDS_Face aFace = aMaker.Face();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);

        var aFix = new ShapeFix_Face(aFace);
        aFix.SetContext(new ShapeBuild_ReShape());
        Assert.True(aFix.Perform());
        Assert.True(aFix.Status(ShapeExtend_DONE8));
        TopoDS_Shape aResult = aFix.Result();

        aSnap.AssertUnchanged(aFace);
        AssertFrozen(aResult);
        Assert.Equal(TopAbs_COMPOUND, aResult.ShapeType());
        Assert.Equal(2, Count(aResult, TopAbs_FACE));
        // the caller's wires, untouched, are in the new faces
        Assert.True(HasTShapeOf(aResult, aW1, TopAbs_WIRE));
        Assert.True(HasTShapeOf(aResult, aW2, TopAbs_WIRE));
    }

    [Fact]
    public void FixWiresTwoCoincEdges_AWireOfTwoCoincidentEdgesIsRemoved()
    {
        TopoDS_Wire anOuter = PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0));
        TopoDS_Edge anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(2, 2, 0), new Point3D(5, 5, 0)).Edge();
        var aB = new BRep_Builder();
        var aTwo = new TopoDS_Wire();
        aB.MakeWire(ref aTwo);
        aB.Add(aTwo, anEdge);
        aB.Add(aTwo, anEdge.Reversed());
        var aMaker = new BRepBuilderAPI_MakeFace(anOuter, true);
        aMaker.Add(aTwo);
        TopoDS_Face aFace = aMaker.Face();

        var aFix = new ShapeFix_Face(aFace);
        Assert.True(aFix.FixWiresTwoCoincEdges());
        TopoDS_Face aResult = aFix.Face();
        AssertFrozen(aResult);
        Assert.Equal(1, Count(aResult, TopAbs_WIRE));
        Assert.True(HasTShapeOf(aResult, anOuter, TopAbs_WIRE));
    }

    [Fact]
    public void UnfrozenFace_IsFixedInPlace_TheInternalResultsAreUnfrozen()
    {
        // a frozen face's sub-shapes would be copied; an unfrozen one (as ShapeFix_Shell gives) is worked in place
        var aB = new BRep_Builder();
        TopoDS_Wire anOuter = TopoDS.Wire(PolygonWire(new Point3D(0, 0, 0), new Point3D(10, 0, 0), new Point3D(10, 10, 0), new Point3D(0, 10, 0)).EmptyCopied());
        var aFace = new TopoDS_Face();
        aB.MakeFace(ref aFace, new Geom_Plane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), Conf);
        TopoDS_Vertex[] aV =
        [
            TestShapes.Vertex(new Point3D(0, 0, 0)), TestShapes.Vertex(new Point3D(0, 10, 0)),
            TestShapes.Vertex(new Point3D(10, 10, 0)), TestShapes.Vertex(new Point3D(10, 0, 0)),
        ];
        for (int i = 0; i < 4; i++)
        {
            aB.Add(anOuter, TestShapes.Edge(aV[i], aV[(i + 1) % 4])); // clockwise: misoriented for a single wire
        }
        aB.Add(aFace, anOuter);
        Assert.False(BRepTools.HoldsFrozen(aFace));

        var aFix = new ShapeFix_Face(aFace);
        Assert.True(aFix.FixOrientation());
        TopoDS_Face aResult = aFix.InternalFace();
        Assert.False(aResult.TShape()!.IsFrozen);
        // the edges are the very ones of the input (no copy was made)
        foreach (TopoDS_Shape anEdge in SubShapes(aFace, TopAbs_EDGE))
        {
            Assert.True(HasTShapeOf(aResult, anEdge, TopAbs_EDGE));
        }
        Assert.Equal(TopAbs_FORWARD, aResult.Orientation());
    }

    [Fact]
    public void AFixAfterAPublicResult_CopiesAgain()
    {
        TopoDS_Face aFace = CylinderBand(3.0, 4.0);
        var aFix = new ShapeFix_Face(aFace);
        aFix.SetContext(new ShapeBuild_ReShape());
        Assert.True(aFix.Perform());
        TopoDS_Face aFirst = aFix.Face();
        AssertFrozen(aFirst);
        // the published face is frozen: a further fix works on a copy of it and leaves it unchanged
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);
        aFix.FixOrientation();
        TopoDS_Face aSecond = aFix.Face();
        aSnap.AssertUnchanged(aFirst);
        AssertFrozen(aSecond);
    }

    [Fact]
    public void Perform_ACylinderFaceWithItsSeam_NeedsNothing()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 3.0));
        var aFix = new ShapeFix_Face(aFace);
        aFix.Perform();
        Assert.False(aFix.Status(ShapeExtend_DONE3));
        Assert.True(new BRepCheck_Analyzer(aFix.Result()).IsValid());
    }
}
