// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Smoke tests of BRepOffset_Inter2d's entries that don't go through BRepOffset_Tool or BRepOffset_Offset (unit 7E).</summary>
public class BRepOffset_Inter2dTests
{
    private static List<TopoDS_Shape> Items(NCollection_List<TopoDS_Shape> theList)
    {
        List<TopoDS_Shape> anItems = [];
        for (NCollection_List<TopoDS_Shape>.Iterator it = new(theList); it.More(); it.Next())
        {
            anItems.Add(it.Value());
        }
        return anItems;
    }

    // A working (unfrozen) edge on the plane along the 2D line from theP, in direction theD, on [0, theL]
    private static TopoDS_Edge EdgeOnPlane(Geom_Plane theS, Point2D theP, Direction2D theD, double theL) =>
        new BRepLib_MakeEdge(new Geom2d_Line(theP, theD), theS, 0.0, theL).InternalEdge();

    [Fact]
    public void Compute_IntersectsCrossingNewEdgesOnAFace()
    {
        Geom_Plane aS = new(new CoordinateSystem());
        TopoDS_Face aF = new BRepLib_MakeFace(aS, -10, 10, -10, 10, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
        TopoDS_Edge aE1 = EdgeOnPlane(aS, new Point2D(-5, 0), new Direction2D(1, 0), 10);
        TopoDS_Edge aE2 = EdgeOnPlane(aS, new Point2D(1, -5), new Direction2D(0, 1), 10);

        BRepAlgo_AsDes anAsDes = new();
        anAsDes.Add(aF, aE1);
        anAsDes.Add(aF, aE2);
        NCollection_IndexedMap<TopoDS_Shape> aNewEdges = new(TopTools_ShapeMapHasher.Instance);
        aNewEdges.Add(aE1);
        aNewEdges.Add(aE2);
        NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEdgeIntEdges = new(TopTools_ShapeMapHasher.Instance);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVV = new(TopTools_ShapeMapHasher.Instance);

        BRepOffset_Inter2d.Compute(anAsDes, aF, aNewEdges, 1.0e-7, anEdgeIntEdges, aDMVV);

        List<TopoDS_Shape> aLV1 = Items(anAsDes.Descendant(aE1));
        List<TopoDS_Shape> aLV2 = Items(anAsDes.Descendant(aE2));
        Assert.Single(aLV1);
        Assert.Single(aLV2);
        Assert.True(aLV1[0].IsSame(aLV2[0]));
        TopoDS_Vertex aV = TopoDS.Vertex(aLV1[0]);
        Assert.True(BRep_Tool.Pnt(aV).IsEqual(new Point3D(1, 0, 0), 1.0e-9));
        Assert.Equal(6.0, BRep_Tool.Parameter(TopoDS.Vertex(aV.Oriented(TopAbs_Orientation.TopAbs_INTERNAL)), aE1), 1.0e-9);
        Assert.Equal(5.0, BRep_Tool.Parameter(TopoDS.Vertex(aV.Oriented(TopAbs_Orientation.TopAbs_INTERNAL)), aE2), 1.0e-9);
        Assert.Equal(0, aDMVV.Extent());
    }

    [Fact]
    public void FuseVertices_ReplacesAChainByOneVertex()
    {
        Geom_Plane aS = new(new CoordinateSystem());
        TopoDS_Edge aE = EdgeOnPlane(aS, new Point2D(0, 0), new Direction2D(1, 0), 10);
        BRep_Builder aBB = new();
        TopoDS_Vertex aV1 = new BRepLib_MakeVertex(new Point3D(4, 0, 0)).InternalVertex();
        TopoDS_Vertex aV2 = new BRepLib_MakeVertex(new Point3D(4.0001, 0, 0)).InternalVertex();
        aBB.UpdateVertex(TopoDS.Vertex(aV1.Oriented(TopAbs_Orientation.TopAbs_INTERNAL)), 4.0, aE, 1.0e-3);
        aBB.UpdateVertex(TopoDS.Vertex(aV2.Oriented(TopAbs_Orientation.TopAbs_INTERNAL)), 4.0001, aE, 1.0e-3);

        BRepAlgo_AsDes anAsDes = new();
        anAsDes.Add(aE, aV1);
        anAsDes.Add(aE, aV2);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVV = new(TopTools_ShapeMapHasher.Instance);
        aDMVV.Add(aV1, new NCollection_List<TopoDS_Shape>([aV2]));
        BRepAlgo_Image anImageVV = new();

        Assert.True(BRepOffset_Inter2d.FuseVertices(aDMVV, anAsDes, anImageVV));

        List<TopoDS_Shape> aLV = Items(anAsDes.Descendant(aE));
        Assert.Single(aLV); // AsDes.Replace doesn't duplicate the new vertex
        Assert.False(aLV[0].IsSame(aV1));
        Assert.False(aLV[0].IsSame(aV2));
        TopoDS_Vertex aVNew = TopoDS.Vertex(aLV[0]);
        Assert.InRange(BRep_Tool.Pnt(aVNew).X(), 4.0 - 1.0e-3, 4.0001 + 1.0e-3);
        Assert.True(BRep_Tool.Parameter(TopoDS.Vertex(aVNew.Oriented(TopAbs_Orientation.TopAbs_INTERNAL)), aE, out double aT));
        Assert.InRange(aT, 4.0, 4.0001);
    }

    [Fact]
    public void ExtentEdge_ProlongsABoundedCurveWithoutPCurves()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 0, 0);
        aPoles[3] = new Point3D(2, 1, 0);
        TopoDS_Edge anE = new BRepBuilderAPI_MakeEdge(new Geom_BezierCurve(aPoles)).Edge();

        Assert.True(BRepOffset_Inter2d.ExtentEdge(anE, out TopoDS_Edge aNE, 1.0));

        Assert.False(aNE.IsSame(anE));
        BRep_Tool.Range(anE, out double anF, out double anL);
        Assert.Equal(0.0, anF);
        Assert.Equal(1.0, anL);
        Geom_Curve? aC = BRep_Tool.Curve(aNE, out double aF, out double aL);
        Assert.IsType<Geom_BSplineCurve>(aC);
        // segments of length 2 (twice the offset) are added along the tangents at both ends
        Assert.True(aC!.Value(aF).IsEqual(new Point3D(-2, 0, 0), 1.0e-7));
        Assert.True(aC.Value(aL).IsEqual(new Point3D(2 + Math.Sqrt(2.0), 1 + Math.Sqrt(2.0), 0), 1.0e-7));
    }
}
