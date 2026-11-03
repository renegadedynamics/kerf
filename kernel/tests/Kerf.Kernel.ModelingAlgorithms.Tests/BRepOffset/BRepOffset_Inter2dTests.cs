// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Algo;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Offsets;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Smoke tests of BRepOffset_Inter2d's entries that don't go through BRepOffset_Tool or BRepOffset_Offset (unit 7E).</summary>
public class BRepOffset_Inter2dTests
{
    private static List<Shape> Items(NCollection_List<Shape> theList)
    {
        List<Shape> anItems = [];
        for (NCollection_List<Shape>.Iterator it = new(theList); it.More(); it.Next())
        {
            anItems.Add(it.Value());
        }
        return anItems;
    }

    // A working (unfrozen) edge on the plane along the 2D line from theP, in direction theD, on [0, theL]
    private static Edge EdgeOnPlane(PlaneSurface theS, Point2D theP, Direction2D theD, double theL) =>
        new LibraryMakeEdge(new LineCurve2D(theP, theD), theS, 0.0, theL).InternalEdge();

    [Fact]
    public void Compute_IntersectsCrossingNewEdgesOnAFace()
    {
        PlaneSurface aS = new(new CoordinateSystem());
        Face aF = new LibraryMakeFace(aS, -10, 10, -10, 10, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalFace();
        Edge aE1 = EdgeOnPlane(aS, new Point2D(-5, 0), new Direction2D(1, 0), 10);
        Edge aE2 = EdgeOnPlane(aS, new Point2D(1, -5), new Direction2D(0, 1), 10);

        AsDes anAsDes = new();
        anAsDes.Add(aF, aE1);
        anAsDes.Add(aF, aE2);
        NCollection_IndexedMap<Shape> aNewEdges = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        aNewEdges.Add(aE1);
        aNewEdges.Add(aE2);
        NCollection_DataMap<Shape, NCollection_List<Shape>> anEdgeIntEdges = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVV = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);

        Inter2d.Compute(anAsDes, aF, aNewEdges, 1.0e-7, anEdgeIntEdges, aDMVV);

        List<Shape> aLV1 = Items(anAsDes.Descendant(aE1));
        List<Shape> aLV2 = Items(anAsDes.Descendant(aE2));
        Assert.Single(aLV1);
        Assert.Single(aLV2);
        Assert.True(aLV1[0].IsSame(aLV2[0]));
        Vertex aV = global::Kerf.Engine.Topology.ShapeCasts.Vertex(aLV1[0]);
        Assert.True(Tool.Pnt(aV).IsEqual(new Point3D(1, 0, 0), 1.0e-9));
        Assert.Equal(6.0, Tool.Parameter(global::Kerf.Engine.Topology.ShapeCasts.Vertex(aV.Oriented(Orientation.Internal)), aE1), 1.0e-9);
        Assert.Equal(5.0, Tool.Parameter(global::Kerf.Engine.Topology.ShapeCasts.Vertex(aV.Oriented(Orientation.Internal)), aE2), 1.0e-9);
        Assert.Equal(0, aDMVV.Extent());
    }

    [Fact]
    public void FuseVertices_ReplacesAChainByOneVertex()
    {
        PlaneSurface aS = new(new CoordinateSystem());
        Edge aE = EdgeOnPlane(aS, new Point2D(0, 0), new Direction2D(1, 0), 10);
        BrepBuilder aBB = new();
        Vertex aV1 = new LibraryMakeVertex(new Point3D(4, 0, 0)).InternalVertex();
        Vertex aV2 = new LibraryMakeVertex(new Point3D(4.0001, 0, 0)).InternalVertex();
        aBB.UpdateVertex(global::Kerf.Engine.Topology.ShapeCasts.Vertex(aV1.Oriented(Orientation.Internal)), 4.0, aE, 1.0e-3);
        aBB.UpdateVertex(global::Kerf.Engine.Topology.ShapeCasts.Vertex(aV2.Oriented(Orientation.Internal)), 4.0001, aE, 1.0e-3);

        AsDes anAsDes = new();
        anAsDes.Add(aE, aV1);
        anAsDes.Add(aE, aV2);
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVV = new(global::Kerf.Engine.Topology.ShapeHasher.Instance);
        aDMVV.Add(aV1, new NCollection_List<Shape>([aV2]));
        ShapeImage anImageVV = new();

        Assert.True(Inter2d.FuseVertices(aDMVV, anAsDes, anImageVV));

        List<Shape> aLV = Items(anAsDes.Descendant(aE));
        Assert.Single(aLV); // AsDes.Replace doesn't duplicate the new vertex
        Assert.False(aLV[0].IsSame(aV1));
        Assert.False(aLV[0].IsSame(aV2));
        Vertex aVNew = global::Kerf.Engine.Topology.ShapeCasts.Vertex(aLV[0]);
        Assert.InRange(Tool.Pnt(aVNew).X(), 4.0 - 1.0e-3, 4.0001 + 1.0e-3);
        Assert.True(Tool.Parameter(global::Kerf.Engine.Topology.ShapeCasts.Vertex(aVNew.Oriented(Orientation.Internal)), aE, out double aT));
        Assert.InRange(aT, 4.0, 4.0001);
    }

    [Fact]
    public void ExtentEdge_ProlongsABoundedCurveWithoutPCurves()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 0, 0);
        aPoles[3] = new Point3D(2, 1, 0);
        Edge anE = new MakeEdge(new BezierCurve3D(aPoles)).Edge();

        Assert.True(Inter2d.ExtentEdge(anE, out Edge aNE, 1.0));

        Assert.False(aNE.IsSame(anE));
        Tool.Range(anE, out double anF, out double anL);
        Assert.Equal(0.0, anF);
        Assert.Equal(1.0, anL);
        Curve3D? aC = Tool.Curve(aNE, out double aF, out double aL);
        Assert.IsType<BSplineCurve3D>(aC);
        // segments of length 2 (twice the offset) are added along the tangents at both ends
        Assert.True(aC!.Value(aF).IsEqual(new Point3D(-2, 0, 0), 1.0e-7));
        Assert.True(aC.Value(aL).IsEqual(new Point3D(2 + Math.Sqrt(2.0), 1 + Math.Sqrt(2.0), 0), 1.0e-7));
    }
}
