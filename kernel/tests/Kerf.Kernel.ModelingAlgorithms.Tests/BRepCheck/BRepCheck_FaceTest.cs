// Port of: src/ModelingAlgorithms/TKTopAlgo/GTests/BRepCheck_Face_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Healing.Checks.StatusCheck;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's BRepCheck_Face GTests. BRepBuilderAPI_MakeEdge, MakeWire and MakeFace (unit 3K) are replaced by
/// BRep_Builder shapes with the same topology: segments sharing their vertices, circles closed on one vertex, an
/// infinite line without vertices, a plane face without pcurves (planes compute them). The faces are frozen before
/// they are checked, as the makers' results will be.
/// </summary>
public class BRepCheck_FaceTest
{
    private static Wire MakeWire(params Edge[] theEdges)
    {
        var aB = new BrepBuilder();
        Wire aW = new();
        aB.MakeWire(ref aW);
        foreach (Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }

        return aW;
    }

    private static Wire makeRectangle(double theXMin, double theYMin, double theXMax, double theYMax)
    {
        Vertex aP1 = TestShapes.Vertex(new Point3D(theXMin, theYMin, 0.0));
        Vertex aP2 = TestShapes.Vertex(new Point3D(theXMax, theYMin, 0.0));
        Vertex aP3 = TestShapes.Vertex(new Point3D(theXMax, theYMax, 0.0));
        Vertex aP4 = TestShapes.Vertex(new Point3D(theXMin, theYMax, 0.0));
        Wire aW = MakeWire(TestShapes.Edge(aP1, aP2), TestShapes.Edge(aP2, aP3), TestShapes.Edge(aP3, aP4),
                                  TestShapes.Edge(aP4, aP1));
        aW.Closed(true);
        return aW;
    }

    private static Face makePlanarFace()
    {
        Wire anOuterWire = makeRectangle(0.0, 0.0, 100.0, 100.0);
        var aB = new BrepBuilder();
        Face aF = new();
        aB.MakeFace(ref aF, new PlaneSurface(new Plane3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0))),
                    global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.Add(aF, anOuterWire);
        return aF;
    }

    // BRepBuilderAPI_MakeEdge(gp_Circ): a closed edge on one vertex, over [0, 2 Pi]
    private static Edge CircleEdge(in Circle3D theCirc)
    {
        var aC = new CircleCurve3D(theCirc);
        return TestShapes.Edge(aC, 0.0, 2.0 * Math.PI);
    }

    private static bool addCircularHole(in Face theFace, in Point3D theCenter, double theRadius)
    {
        Edge anEdge = CircleEdge(new Circle3D(new Frame3D(theCenter, new Direction3D(0.0, 0.0, 1.0)), theRadius));
        Wire aHole = MakeWire(anEdge);
        aHole.Closed(true);
        aHole.Reverse();
        new BrepBuilder().Add(theFace, aHole);
        return true;
    }

    private static StatusCheck classifyWires(in Face theFace)
    {
        var aChecker = new FaceCheck(theFace);
        return aChecker.ClassifyWires();
    }

    [Fact]
    public void ClassifyWires_DisjointHolesAreValid()
    {
        Face aFace = makePlanarFace();
        Assert.False(aFace.IsNull);

        for (int aRow = 0; aRow < 5; ++aRow)
        {
            for (int aColumn = 0; aColumn < 8; ++aColumn)
            {
                var aCenter = new Point3D(10.0 + 11.0 * aColumn, 10.0 + 11.0 * aRow, 0.0);
                Assert.True(addCircularHole(aFace, aCenter, 2.0));
            }
        }

        aFace = TestShapes.Frozen(aFace);
        Assert.Equal(StatusCheck.NoError, classifyWires(aFace));
        Assert.True(new Analyzer(aFace).IsValid());
    }

    [Fact]
    public void ClassifyWires_OffsetPCurve()
    {
        Face aFace = makePlanarFace();
        Assert.False(aFace.IsNull);

        var aCenter2d = new Point2D(50.0, 50.0);
        var aBasis = new CircleCurve2D(new Axis2D(aCenter2d, new Direction2D(1.0, 0.0)), 4.0);
        var anOffset = new OffsetCurve2D(aBasis, 1.0);
        double aRadius = anOffset.Value(0.0).DistanceTo(aCenter2d);

        Edge anEdge = CircleEdge(new Circle3D(new Frame3D(new Point3D(50.0, 50.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), aRadius));

        new BrepBuilder().UpdateEdge(anEdge, anOffset, aFace, global::Kerf.Engine.Foundation.Precision.Confusion());
        Wire aHole = MakeWire(anEdge);
        aHole.Closed(true);
        aHole.Reverse();
        new BrepBuilder().Add(aFace, aHole);

        aFace = TestShapes.Frozen(aFace);
        Assert.Equal(StatusCheck.NoError, classifyWires(aFace));
    }

    [Fact]
    public void ClassifyWires_OpenWireFallbackIsOrientationIndependent()
    {
        Face aFace = makePlanarFace();
        Assert.False(aFace.IsNull);

        Edge anEdge = TestShapes.Edge(new Point3D(120.0, 10.0, 0.0), new Point3D(130.0, 10.0, 0.0));
        new BrepBuilder().Add(aFace, MakeWire(anEdge));

        aFace = TestShapes.Frozen(aFace);
        StatusCheck aForwardStatus = classifyWires(aFace);
        Face aReversedFace = ShapeCasts.Face(aFace.Reversed());
        Assert.Equal(StatusCheck.NoError, aForwardStatus);
        Assert.Equal(aForwardStatus, classifyWires(aReversedFace));
    }

    [Fact]
    public void ClassifyWires_UnboundedWireFallbackIsOrientationIndependent()
    {
        Face aFace = makePlanarFace();
        Assert.False(aFace.IsNull);

        var aLine = new LineCurve3D(new Line3D(new Point3D(0.0, 120.0, 0.0), new Direction3D(1.0, 0.0, 0.0)));
        // BRepBuilderAPI_MakeEdge(line): the whole line, without vertices
        var aB = new BrepBuilder();
        Edge anEdge = new();
        aB.MakeEdge(ref anEdge, aLine, global::Kerf.Engine.Foundation.Precision.Confusion());
        aB.Range(anEdge, aLine.FirstParameter, aLine.LastParameter);
        aB.Add(aFace, MakeWire(anEdge));

        aFace = TestShapes.Frozen(aFace);
        StatusCheck aForwardStatus = classifyWires(aFace);
        Face aReversedFace = ShapeCasts.Face(aFace.Reversed());
        Assert.Equal(StatusCheck.NoError, aForwardStatus);
        Assert.Equal(aForwardStatus, classifyWires(aReversedFace));
        Assert.Equal(Reversed, aReversedFace.Orientation);
    }
}
