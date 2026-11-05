// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Filling;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.TestHarness;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The BRepFill package statics (unit 5S1): ruled faces and shells, Axe, the reduced curvilinear abscissae and
/// SearchOrigin; frozen inputs copied before the pcurves are written (decision 897). Native parity is in
/// Kerf.Kernel.Oracle.Tests' BRepFillOracleTests.
/// </summary>
public class BRepFillTests
{
    private static Edge Segment(Point3D theP1, Point3D theP2) => new MakeEdge(theP1, theP2).Edge();

    private static Wire Polygon(bool theClosed, params Point3D[] thePoints)
    {
        MakePolygon aP = new();
        foreach (Point3D aQ in thePoints)
        {
            aP.Add(aQ);
        }
        if (theClosed)
        {
            aP.Close();
        }
        return aP.Wire();
    }

    private static Wire Square(double theSide, double theZ) =>
        Polygon(true, new Point3D(0, 0, theZ), new Point3D(theSide, 0, theZ), new Point3D(theSide, theSide, theZ),
                new Point3D(0, theSide, theZ));

    private static int Count(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static bool Holds(in Shape theShape, TShape theTShape)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            if (ReferenceEquals(anExp.Current().TShape(), theTShape))
            {
                return true;
            }
        }
        for (var anExp = new Explorer(theShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (ReferenceEquals(anExp.Current().TShape(), theTShape))
            {
                return true;
            }
        }
        return false;
    }

    private static double Area(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void Face_ParallelSegments_IsPlanarValidAndFrozen_InputsUnchanged()
    {
        Edge aE1 = Segment(new Point3D(0, 0, 0), new Point3D(10, 0, 0));
        Edge aE2 = Segment(new Point3D(0, 5, 0), new Point3D(10, 5, 0));
        string aBefore1 = BRepTextComparison.Write(aE1), aBefore2 = BRepTextComparison.Write(aE2);

        Face aF = Fill.Face(aE1, aE2);

        Assert.True(aF.TShape()!.IsFrozen);
        Assert.True(new Analyzer(aF).IsValid());
        Assert.Equal(50.0, Area(aF), 1e-9);
        Assert.Equal(4, Count(aF, ShapeType.Edge));
        Surface aS = Tool.Surface(aF)!;
        for (double u = 0; u <= 1; u += 0.25)
        {
            for (double v = 0; v <= 1; v += 0.25)
            {
                Assert.Equal(0.0, aS.Value(u, v).Z, 1e-12);
            }
        }

        // the caller's edges are not written: the face holds copies of them (and of their vertices, which
        // BRepLib::SameParameter rebuilds), so its BRep text is native's byte for byte (decision 897)
        Assert.Equal(aBefore1, BRepTextComparison.Write(aE1));
        Assert.Equal(aBefore2, BRepTextComparison.Write(aE2));
        Assert.False(Holds(aF, aE1.TShape()!));
        Assert.Equal(4, Count(aF, ShapeType.Vertex));
    }

    [Fact]
    public void Face_SharedVertex_GivesDegeneratedEdge_AndKeepsSharing()
    {
        Vertex aA = new MakeVertex(new Point3D(0, 0, 0)).Vertex();
        Vertex aB = new MakeVertex(new Point3D(10, 0, 0)).Vertex();
        Vertex aC = new MakeVertex(new Point3D(10, 6, 2)).Vertex();
        Edge aE1 = new MakeEdge(aA, aB).Edge();
        Edge aE2 = new MakeEdge(aA, aC).Edge();

        Face aF = Fill.Face(aE1, aE2);

        Assert.True(new Analyzer(aF).IsValid());
        int aNbDegenerated = 0;
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (Tool.Degenerated(ShapeCasts.Edge(anExp.Current())))
            {
                aNbDegenerated++;
            }
        }
        Assert.Equal(1, aNbDegenerated);
        // the shared vertex is one vertex in the face: three vertices in all
        Assert.Equal(3, Count(aF, ShapeType.Vertex));
    }

    [Fact]
    public void Face_ClosedEdges_GiveSeam()
    {
        Edge aE1 = new MakeEdge(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 5)).Edge();
        Edge aE2 = new MakeEdge(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 8), new Direction3D(0, 0, 1)), 3)).Edge();

        Face aF = Fill.Face(aE1, aE2);

        Assert.True(new Analyzer(aF).IsValid());
        Assert.Equal(3, Count(aF, ShapeType.Edge));
        int aNbSeams = 0;
        for (var anExp = new Explorer(aF, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            if (Tool.IsClosed(ShapeCasts.Edge(anExp.Current()), aF))
            {
                aNbSeams++;
            }
        }
        Assert.Equal(2, aNbSeams); // the seam appears twice, with both orientations
        // the lateral area of a frustum of radii 5 and 3 and height 8
        Assert.Equal(Math.PI * (5 + 3) * Math.Sqrt(8 * 8 + 2 * 2), Area(aF), 1e-6);
    }

    [Fact]
    public void Shell_TwoSquares_GivesFourFacesTube()
    {
        Wire aW1 = Square(10, 0), aW2 = Square(10, 7);
        string aBefore1 = BRepTextComparison.Write(aW1), aBefore2 = BRepTextComparison.Write(aW2);

        Shell aS = Fill.Shell(aW1, aW2);

        Assert.True(aS.TShape()!.IsFrozen);
        Assert.True(new Analyzer(aS).IsValid());
        Assert.Equal(4, Count(aS, ShapeType.Face));
        Assert.Equal(12, Count(aS, ShapeType.Edge));
        Assert.Equal(8, Count(aS, ShapeType.Vertex));
        Assert.False(aS.ClosedFlag);
        Assert.Equal(4 * 10 * 7, Area(aS), 1e-9);
        Assert.Equal(aBefore1, BRepTextComparison.Write(aW1));
        Assert.Equal(aBefore2, BRepTextComparison.Write(aW2));
    }

    [Fact]
    public void Axe_OnSquareSpine()
    {
        Wire aSpine = Square(10, 0);

        // a profile starting at the middle of the edge y = 0, standing up
        Wire aProfile = Polygon(false, new Point3D(4, 0, 0), new Point3D(4, 0, 3), new Point3D(6, 0, 3));
        Fill.Axe(aSpine, aProfile, out CoordinateSystem anAx, out bool isOn, 1e-7);
        Assert.True(isOn);
        Assert.True(anAx.Location.IsEqual(new Point3D(4, 0, 0), 1e-12));
        Assert.Equal(1.0, Math.Abs(anAx.Direction.Z), 1e-12);
        Assert.Equal(1.0, Math.Abs(anAx.XDirection.X), 1e-12);

        // a profile away from the spine
        Wire aFar = Polygon(false, new Point3D(12, 5, 2), new Point3D(12, 5, 6));
        Fill.Axe(aSpine, aFar, out anAx, out isOn, 1e-7);
        Assert.False(isOn);
        Assert.True(anAx.Location.IsEqual(new Point3D(10, 5, 0), 1e-9));
        Assert.Equal(1.0, Math.Abs(anAx.XDirection.Y), 1e-12);

        // a profile on a spine vertex
        Wire aOnVertex = Polygon(false, new Point3D(10, 10, 0), new Point3D(10, 10, 4), new Point3D(11, 10, 4));
        Fill.Axe(aSpine, aOnVertex, out anAx, out isOn, 1e-7);
        Assert.True(isOn);
        Assert.True(anAx.Location.IsEqual(new Point3D(10, 10, 0), 1e-12));
    }

    [Fact]
    public void Axe_NonPlanarFace_Throws()
    {
        Face aCyl = new MakeFace(new CylindricalSurface(new CoordinateSystem(), 5), 0, 2, 0, 3, 1e-7).Face();
        Wire aProfile = Polygon(false, new Point3D(5, 0, 0), new Point3D(6, 0, 0));
        EngineAssert.ThrowsAny<NoSuchObjectException>(() => Fill.Axe(aCyl, aProfile, out _, out _, 1e-7));
    }

    private static Wire Polyline345() =>
        Polygon(false, new Point3D(0, 0, 0), new Point3D(3, 0, 0), new Point3D(3, 4, 0), new Point3D(3, 4, 5));

    [Fact]
    public void ComputeACR_Polyline()
    {
        var aACR = new NCollection_Array1<double>(0, 3);
        Fill.ComputeACR(Polyline345(), aACR);
        Assert.Equal(12.0, aACR[0], 1e-12);
        Assert.Equal(3.0 / 12, aACR[1], 1e-12);
        Assert.Equal(7.0 / 12, aACR[2], 1e-12);
        Assert.Equal(1.0, aACR[3], 1e-12);
    }

    [Fact]
    public void InsertACR_CutsTheEdges()
    {
        Wire aW = Polyline345();
        string aBefore = BRepTextComparison.Write(aW);
        var aCuts = new NCollection_Array1<double>(1, 4);
        aCuts[1] = 0.1;          // on the first edge
        aCuts[2] = 0.25;         // the first edge's end: no cut
        aCuts[3] = 0.5;          // on the second edge
        aCuts[4] = 0.9;          // on the third edge

        Wire aRes = Fill.InsertACR(aW, aCuts, 1e-6);

        Assert.True(aRes.TShape()!.IsFrozen);
        Assert.Equal(6, Count(aRes, ShapeType.Edge));
        var aACR = new NCollection_Array1<double>(0, 6);
        Fill.ComputeACR(aRes, aACR);
        Assert.Equal(12.0, aACR[0], 1e-9);
        Assert.Equal(0.1, aACR[1], 1e-9);
        Assert.Equal(0.25, aACR[2], 1e-9);
        Assert.Equal(0.5, aACR[3], 1e-9);
        Assert.Equal(7.0 / 12, aACR[4], 1e-9);
        Assert.Equal(0.9, aACR[5], 1e-9);
        Assert.Equal(aBefore, BRepTextComparison.Write(aW));

        // with a precision larger than the distance to the ends, nothing is cut
        var aNear = new NCollection_Array1<double>(1, 1);
        aNear[1] = 0.26;
        Assert.Equal(3, Count(Fill.InsertACR(aW, aNear, 0.05), ShapeType.Edge));
    }

    [Fact]
    public void SearchOrigin_CutsAndOrients()
    {
        Wire aW = Square(10, 0);
        string aBefore = BRepTextComparison.Write(aW);

        // a point in the middle of the edge y = 0: the edge is cut there and the wire starts at the new vertex
        Wire aCut = aW;
        Fill.SearchOrigin(ref aCut, new Point3D(5, 0, 0), new Vector3D(1, 0, 0), 1e-7);
        Assert.Equal(5, Count(aCut, ShapeType.Edge));
        Assert.True(aCut.ClosedFlag);
        var anExp = new WireExplorer(aCut);
        Assert.True(Tool.Pnt(anExp.CurrentVertex()).IsEqual(new Point3D(5, 0, 0), 1e-9));
        Vertex aV1 = Exploration.LastVertex(anExp.Current(), true);
        Assert.True(Tool.Pnt(aV1).IsEqual(new Point3D(10, 0, 0), 1e-9)); // along +X

        // a point on a vertex, against the wire's direction: the same four edges, reversed, nothing cut (the
        // reversed wire is rebuilt from the edge starting at the vertex, so it starts at that edge's other end)
        Wire aRev = aW;
        Fill.SearchOrigin(ref aRev, new Point3D(10, 10, 0), new Vector3D(1, 0, 0), 1e-7);
        Assert.Equal(4, Count(aRev, ShapeType.Edge));
        Assert.Equal(4, Count(aRev, ShapeType.Vertex));
        Assert.True(aRev.ClosedFlag);

        Assert.Equal(aBefore, BRepTextComparison.Write(aW));
        EngineAssert.Throws<NoSuchObjectException>(() =>
        {
            Wire anOpen = Polyline345();
            Fill.SearchOrigin(ref anOpen, new Point3D(0, 0, 0), new Vector3D(1, 0, 0), 1e-7);
        });
    }

    [Fact]
    public void InternalFace_UnfrozenInputs_WrittenInPlace()
    {
        // ported code's own unfrozen edges are used and written in place, as in OCCT
        Edge aE1 = new LibraryMakeEdge(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).InternalEdge();
        Edge aE2 = new LibraryMakeEdge(new Point3D(0, 5, 3), new Point3D(10, 5, 3)).InternalEdge();
        Assert.False(aE1.TShape()!.IsFrozen);

        Face aF = Fill.InternalFace(aE1, aE2);

        Assert.False(aF.TShape()!.IsFrozen);
        Assert.True(Holds(aF, aE1.TShape()!));
        Assert.True(Holds(aF, aE2.TShape()!));
        Assert.NotNull(Tool.CurveOnSurface(aE1, aF, out _, out _));

        Shell aS = Fill.InternalShell(
            ShapeCasts.Wire(new LibraryMakePolygon(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(1, 1, 0)).InternalWire()),
            ShapeCasts.Wire(new LibraryMakePolygon(new Point3D(0, 0, 1), new Point3D(1, 0, 1), new Point3D(1, 1, 1)).InternalWire()));
        Assert.False(aS.TShape()!.IsFrozen);
        Assert.Equal(2, Count(aS, ShapeType.Face));
    }
}
