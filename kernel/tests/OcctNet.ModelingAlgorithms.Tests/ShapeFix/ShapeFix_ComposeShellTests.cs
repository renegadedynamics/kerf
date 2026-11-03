// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_ComposeShell: a face split by the joint lines of a grid of surfaces, on a frozen face (copied at the entry,
/// the result frozen, the context answering for the caller's edges) and on an unfrozen one (in place); SplitEdges.
/// </summary>
public class ShapeFix_ComposeShellTests
{
    private const double Conf = 1e-7;

    // the plane z = 0, as two patches [0, 1] x [0, 1] and [1, 2] x [0, 1]
    private static ShapeExtend_CompositeSurface TwoPatches(out PlaneSurface thePlane)
    {
        thePlane = new PlaneSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        var aGrid = new NCollection_HArray2<Surface>(1, 2, 1, 1);
        aGrid.SetValue(1, 1, new TrimmedSurface(thePlane, 0.0, 1.0, 0.0, 1.0));
        aGrid.SetValue(2, 1, new TrimmedSurface(thePlane, 1.0, 2.0, 0.0, 1.0));
        return new ShapeExtend_CompositeSurface(aGrid);
    }

    // the face [0.2, 1.8] x [0.2, 0.8] on the plane, with pcurves
    private static Face Rectangle(PlaneSurface thePlane)
    {
        Vertex[] aV =
        [
            TestShapes.Vertex(new Point3D(0.2, 0.2, 0)), TestShapes.Vertex(new Point3D(1.8, 0.2, 0)),
            TestShapes.Vertex(new Point3D(1.8, 0.8, 0)), TestShapes.Vertex(new Point3D(0.2, 0.8, 0)),
        ];
        return TestShapes.PolygonFace(aV, new Direction3D(0, 0, 1), new Dictionary<(int, int), Edge>(), [0, 1, 2, 3], thePlane);
    }

    private static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        var aList = new List<Shape>();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    [Fact]
    public void Perform_AFrozenFaceAcrossTwoPatches_MakesTwoFaces_TheInputUnchanged()
    {
        ShapeExtend_CompositeSurface aGrid = TwoPatches(out PlaneSurface aPlane);
        Face aFace = TestShapes.Frozen(Rectangle(aPlane));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aContext = new ShapeBuild_ReShape();

        var aTool = new ShapeFix_ComposeShell();
        aTool.Init(aGrid, new Placement(), aFace, Conf);
        aTool.SetContext(aContext);
        Assert.True(aTool.Perform());
        Assert.True(aTool.Status(ShapeExtend_DONE1));
        Shape aResult = aTool.Result();

        aSnap.AssertUnchanged(aFace);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Shell, aResult.ShapeType());
        List<Shape> aFaces = SubShapes(aResult, ShapeType.Face);
        Assert.Equal(2, aFaces.Count);
        // each new face is on its own patch
        Assert.NotSame(Tool.Surface(ShapeCasts.Face(aFaces[0])), Tool.Surface(ShapeCasts.Face(aFaces[1])));
        // 4 sides split into 6 edges, and the joint line's edge
        Assert.Equal(7, SubShapes(aResult, ShapeType.Edge).Count);

        // the bottom and top edges, which cross u = 1, are recorded as split into wires, keyed by the caller's edges
        int aNbSplit = 0;
        foreach (Shape anEdge in SubShapes(aFace, ShapeType.Edge))
        {
            if (aContext.IsRecorded(anEdge) && aContext.Value(anEdge).ShapeType() == ShapeType.Wire)
            {
                aNbSplit++;
            }
        }
        Assert.Equal(2, aNbSplit);
    }

    [Fact]
    public void SplitEdges_RecordsTheSplitsOnly()
    {
        ShapeExtend_CompositeSurface aGrid = TwoPatches(out PlaneSurface aPlane);
        Face aFace = Rectangle(aPlane); // unfrozen: worked in place
        var aContext = new ShapeBuild_ReShape();
        var aTool = new ShapeFix_ComposeShell();
        aTool.Init(aGrid, new Placement(), aFace, Conf);
        aTool.SetContext(aContext);
        aTool.SplitEdges();
        Assert.True(aTool.InternalResult().IsNull);
        int aNbSplit = 0;
        foreach (Shape anEdge in SubShapes(aFace, ShapeType.Edge))
        {
            if (!aContext.IsRecorded(anEdge))
            {
                continue;
            }
            // the bottom and top edges are split into wires of two edges; the sides, whose vertices the split
            // replaced, are rebuilt with the new vertices (ShapeBuild_ReShape records the rebuilt edge)
            Shape aValue = aContext.InternalValue(anEdge);
            if (aValue.ShapeType() == ShapeType.Wire)
            {
                Assert.Equal(2, SubShapes(aValue, ShapeType.Edge).Count);
                aNbSplit++;
            }
            else
            {
                Assert.Equal(ShapeType.Edge, aValue.ShapeType());
            }
        }
        Assert.Equal(2, aNbSplit);
    }

    [Fact]
    public void Perform_AnUnfrozenFace_KeepsTheUnsplitEdges()
    {
        ShapeExtend_CompositeSurface aGrid = TwoPatches(out PlaneSurface aPlane);
        Face aFace = Rectangle(aPlane);
        var aTool = new ShapeFix_ComposeShell();
        aTool.Init(aGrid, new Placement(), aFace, Conf);
        aTool.SetContext(new ShapeBuild_ReShape());
        Assert.True(aTool.Perform());
        Shape aResult = aTool.InternalResult();
        Assert.False(aResult.TShape()!.IsFrozen);
        Assert.Equal(2, SubShapes(aResult, ShapeType.Face).Count);
        Assert.True(ShapeFix_FaceTestsAccess.IsValid(aResult));
    }

    private static class ShapeFix_FaceTestsAccess
    {
        public static bool IsValid(in Shape theShape) => new BRepCheck_Analyzer(theShape).IsValid();
    }
}
