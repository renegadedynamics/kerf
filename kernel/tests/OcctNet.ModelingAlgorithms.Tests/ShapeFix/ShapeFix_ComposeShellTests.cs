// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeFix_ComposeShell: a face split by the joint lines of a grid of surfaces, on a frozen face (copied at the entry,
/// the result frozen, the context answering for the caller's edges) and on an unfrozen one (in place); SplitEdges.
/// </summary>
public class ShapeFix_ComposeShellTests
{
    private const double Conf = 1e-7;

    // the plane z = 0, as two patches [0, 1] x [0, 1] and [1, 2] x [0, 1]
    private static ShapeExtend_CompositeSurface TwoPatches(out Geom_Plane thePlane)
    {
        thePlane = new Geom_Plane(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        var aGrid = new NCollection_HArray2<Geom_Surface>(1, 2, 1, 1);
        aGrid.SetValue(1, 1, new Geom_RectangularTrimmedSurface(thePlane, 0.0, 1.0, 0.0, 1.0));
        aGrid.SetValue(2, 1, new Geom_RectangularTrimmedSurface(thePlane, 1.0, 2.0, 0.0, 1.0));
        return new ShapeExtend_CompositeSurface(aGrid);
    }

    // the face [0.2, 1.8] x [0.2, 0.8] on the plane, with pcurves
    private static TopoDS_Face Rectangle(Geom_Plane thePlane)
    {
        TopoDS_Vertex[] aV =
        [
            TestShapes.Vertex(new Point3D(0.2, 0.2, 0)), TestShapes.Vertex(new Point3D(1.8, 0.2, 0)),
            TestShapes.Vertex(new Point3D(1.8, 0.8, 0)), TestShapes.Vertex(new Point3D(0.2, 0.8, 0)),
        ];
        return TestShapes.PolygonFace(aV, new Direction3D(0, 0, 1), new Dictionary<(int, int), TopoDS_Edge>(), [0, 1, 2, 3], thePlane);
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

    [Fact]
    public void Perform_AFrozenFaceAcrossTwoPatches_MakesTwoFaces_TheInputUnchanged()
    {
        ShapeExtend_CompositeSurface aGrid = TwoPatches(out Geom_Plane aPlane);
        TopoDS_Face aFace = TestShapes.Frozen(Rectangle(aPlane));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFace);
        var aContext = new ShapeBuild_ReShape();

        var aTool = new ShapeFix_ComposeShell();
        aTool.Init(aGrid, new Placement(), aFace, Conf);
        aTool.SetContext(aContext);
        Assert.True(aTool.Perform());
        Assert.True(aTool.Status(ShapeExtend_DONE1));
        TopoDS_Shape aResult = aTool.Result();

        aSnap.AssertUnchanged(aFace);
        Assert.True(aResult.TShape()!.IsFrozen);
        Assert.Equal(TopAbs_SHELL, aResult.ShapeType());
        List<TopoDS_Shape> aFaces = SubShapes(aResult, TopAbs_FACE);
        Assert.Equal(2, aFaces.Count);
        // each new face is on its own patch
        Assert.NotSame(BRep_Tool.Surface(TopoDS.Face(aFaces[0])), BRep_Tool.Surface(TopoDS.Face(aFaces[1])));
        // 4 sides split into 6 edges, and the joint line's edge
        Assert.Equal(7, SubShapes(aResult, TopAbs_EDGE).Count);

        // the bottom and top edges, which cross u = 1, are recorded as split into wires, keyed by the caller's edges
        int aNbSplit = 0;
        foreach (TopoDS_Shape anEdge in SubShapes(aFace, TopAbs_EDGE))
        {
            if (aContext.IsRecorded(anEdge) && aContext.Value(anEdge).ShapeType() == TopAbs_WIRE)
            {
                aNbSplit++;
            }
        }
        Assert.Equal(2, aNbSplit);
    }

    [Fact]
    public void SplitEdges_RecordsTheSplitsOnly()
    {
        ShapeExtend_CompositeSurface aGrid = TwoPatches(out Geom_Plane aPlane);
        TopoDS_Face aFace = Rectangle(aPlane); // unfrozen: worked in place
        var aContext = new ShapeBuild_ReShape();
        var aTool = new ShapeFix_ComposeShell();
        aTool.Init(aGrid, new Placement(), aFace, Conf);
        aTool.SetContext(aContext);
        aTool.SplitEdges();
        Assert.True(aTool.InternalResult().IsNull());
        int aNbSplit = 0;
        foreach (TopoDS_Shape anEdge in SubShapes(aFace, TopAbs_EDGE))
        {
            if (!aContext.IsRecorded(anEdge))
            {
                continue;
            }
            // the bottom and top edges are split into wires of two edges; the sides, whose vertices the split
            // replaced, are rebuilt with the new vertices (ShapeBuild_ReShape records the rebuilt edge)
            TopoDS_Shape aValue = aContext.InternalValue(anEdge);
            if (aValue.ShapeType() == TopAbs_WIRE)
            {
                Assert.Equal(2, SubShapes(aValue, TopAbs_EDGE).Count);
                aNbSplit++;
            }
            else
            {
                Assert.Equal(TopAbs_EDGE, aValue.ShapeType());
            }
        }
        Assert.Equal(2, aNbSplit);
    }

    [Fact]
    public void Perform_AnUnfrozenFace_KeepsTheUnsplitEdges()
    {
        ShapeExtend_CompositeSurface aGrid = TwoPatches(out Geom_Plane aPlane);
        TopoDS_Face aFace = Rectangle(aPlane);
        var aTool = new ShapeFix_ComposeShell();
        aTool.Init(aGrid, new Placement(), aFace, Conf);
        aTool.SetContext(new ShapeBuild_ReShape());
        Assert.True(aTool.Perform());
        TopoDS_Shape aResult = aTool.InternalResult();
        Assert.False(aResult.TShape()!.IsFrozen);
        Assert.Equal(2, SubShapes(aResult, TopAbs_FACE).Count);
        Assert.True(ShapeFix_FaceTestsAccess.IsValid(aResult));
    }

    private static class ShapeFix_FaceTestsAccess
    {
        public static bool IsValid(in TopoDS_Shape theShape) => new BRepCheck_Analyzer(theShape).IsValid();
    }
}
