// Port of: src/ModelingData/TKBRep/GTests/BRepAdaptor_CompCurve_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's BRepAdaptor_CompCurve GTests, with BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire and GC_MakeArcOfCircle,
/// and further tests of the composite curve (on <see cref="TestShapes"/>).
/// </summary>
public class BRepAdaptor_CompCurveTests
{
    /// <summary>A frozen wire of the edges, in this order (as BRepBuilderAPI_MakeWire of one edge gives).</summary>
    internal static Wire MakeWire(params Edge[] theEdges)
    {
        var aB = new BrepBuilder();
        Wire aW = new Wire();
        aB.MakeWire(ref aW);
        foreach (Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }

        return TestShapes.Frozen(aW);
    }

    /// <summary>The outer wire of the box's first face (a closed wire of 4 segments, from a frozen box).</summary>
    internal static Wire BoxFaceWire(out Face theFace)
    {
        Shape aBox = TestShapes.Frozen(TestShapes.Box(1.0, 2.0, 3.0));
        theFace = ShapeCasts.Face(new Explorer(aBox, ShapeType.Face).Current());
        return ShapeCasts.Wire(new Explorer(theFace, ShapeType.Wire).Current());
    }

    // Test OCC5696: BRepAdaptor_CompCurve::Edge() method
    [Fact]
    public void OCC5696_EdgeMethod()
    {
        // Create a simple edge from two points
        Edge anEdge = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(2, 0, 0)).Edge();

        // Create a wire from the edge
        Wire aWire = new BRepBuilderAPI_MakeWire(anEdge).Wire();

        // Create a composite curve adaptor
        var aCurve = new WireCurveAdaptor(aWire);

        // Get curve parameters
        double aFirst = aCurve.FirstParameter();
        double aLast = aCurve.LastParameter();
        double aPar = (aFirst + aLast) / 2.0;

        // Test the Edge() method
        double aParEdge = 0.0;
        Edge anEdgeFound = new Edge();

        // The original test was checking that this method doesn't throw an exception and returns valid parameter
        Exception? anException = Record.Exception(() => aCurve.Edge(aPar, out anEdgeFound, out aParEdge));
        Assert.Null(anException);

        // Verify that the returned edge is valid
        Assert.False(anEdgeFound.IsNull);

        // Verify that the parameter is within valid range [0, edge length]
        Assert.True(aParEdge >= 0.0);
        Assert.True(aParEdge <= 2.0);

        // The parameter should be approximately half of the edge length
        Assert.Equal(1.0, aParEdge, 0.01);
    }

    // Test OCC29430: BRepAdaptor_CompCurve::Value() at boundary parameters matches wire vertices.
    [Fact]
    public void OCC29430_ArcBoundaryPoints()
    {
        double r45 = Math.PI / 4.0, r225 = 3.0 * Math.PI / 4.0;

        var arcMaker = new MakeArcOfCircle(
            new Circle3D(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1.0),
            r45,
            r225,
            true);
        var edgeMaker = new BRepBuilderAPI_MakeEdge(arcMaker.Value());
        var wireMaker = new BRepBuilderAPI_MakeWire(edgeMaker.Edge());
        Wire aWire = wireMaker.Wire();

        var aCurve = new WireCurveAdaptor(aWire);
        Point3D aStartPt = aCurve.Value(aCurve.FirstParameter());
        Point3D anEndPt = aCurve.Value(aCurve.LastParameter());

        // Collect wire vertices
        var aVertices = new List<Point3D>();
        for (var anExp = new Explorer(aWire, ShapeType.Vertex); anExp.More(); anExp.Next())
        {
            aVertices.Add(Tool.Pnt(ShapeCasts.Vertex(anExp.Current())));
        }
        Assert.True(aVertices.Count >= 1);

        bool aStartMatchesAnyVertex = false;
        bool anEndMatchesAnyVertex = false;
        foreach (Point3D aV in aVertices)
        {
            if (aStartPt.Distance(aV) < 1.0e-7)
            {
                aStartMatchesAnyVertex = true;
            }
            if (anEndPt.Distance(aV) < 1.0e-7)
            {
                anEndMatchesAnyVertex = true;
            }
        }
        Assert.True(aStartMatchesAnyVertex, "Start point does not match any wire vertex");
        Assert.True(anEndMatchesAnyVertex, "End point does not match any wire vertex");
        Assert.True(aStartPt.Distance(anEndPt) > 1.0e-7, "Start and end points should be different");
    }

    // Test OCC30869: BRepAdaptor_CompCurve D1 at boundary parameters of a wire with reversed edge.
    [Fact]
    public void OCC30869_ReversedEdgeBoundaryPoints()
    {
        // Build a circle: center(1,0,0), Z-axis(0,-1,0), X-axis(0,0,-1), radius=1
        var anAx2 = new Frame3D(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, -1.0, 0.0), new Direction3D(0.0, 0.0, -1.0));
        var aCircle = new CircleCurve3D(anAx2, 1.0);

        double t1 = Math.PI / 2.0;
        double t2 = 3.0 * Math.PI / 2.0;

        var aTrimmed = new TrimmedCurve3D(aCircle, t1, t2);
        Edge anEdge = new BRepBuilderAPI_MakeEdge(aTrimmed).Edge();

        // Reverse the edge, then wrap it in a wire
        anEdge = ShapeCasts.Edge(anEdge.Oriented(Reversed));
        Wire aWire = new BRepBuilderAPI_MakeWire(anEdge).Wire();

        var aBACC = new WireCurveAdaptor(aWire);
        double aFirst = aBACC.FirstParameter();
        double aLast = aBACC.LastParameter();

        aBACC.D1(aFirst, out Point3D aPFirst, out Vector3D aVFirst);
        aBACC.D1(aLast, out Point3D aPLast, out Vector3D aVLast);

        if (aVFirst.SquareMagnitude() > Constants.Resolution())
        {
            aVFirst.Normalize();
        }
        if (aVLast.SquareMagnitude() > Constants.Resolution())
        {
            aVLast.Normalize();
        }

        // Reference: inverse circle (normal = (0,1,0)), evaluated at the same parameters
        var anAx2Ref = new Frame3D(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0), new Direction3D(0.0, 0.0, -1.0));
        var aCircleRef = new CircleCurve3D(anAx2Ref, 1.0);

        aCircleRef.D1(t1, out Point3D aRefP1, out Vector3D aRefV1);
        aCircleRef.D1(t2, out Point3D aRefP2, out Vector3D aRefV2);
        if (aRefV1.SquareMagnitude() > Constants.Resolution())
        {
            aRefV1.Normalize();
        }
        if (aRefV2.SquareMagnitude() > Constants.Resolution())
        {
            aRefV2.Normalize();
        }

        const double aTol = 1.0e-7;
        Assert.Equal(aRefP1.X(), aPFirst.X(), aTol);
        Assert.Equal(aRefP1.Y(), aPFirst.Y(), aTol);
        Assert.Equal(aRefP1.Z(), aPFirst.Z(), aTol);

        Assert.Equal(aRefV1.X(), aVFirst.X(), aTol);
        Assert.Equal(aRefV1.Y(), aVFirst.Y(), aTol);
        Assert.Equal(aRefV1.Z(), aVFirst.Z(), aTol);

        Assert.Equal(aRefP2.X(), aPLast.X(), aTol);
        Assert.Equal(aRefP2.Y(), aPLast.Y(), aTol);
        Assert.Equal(aRefP2.Z(), aPLast.Z(), aTol);

        Assert.Equal(aRefV2.X(), aVLast.X(), aTol);
        Assert.Equal(aRefV2.Y(), aVLast.Y(), aTol);
        Assert.Equal(aRefV2.Z(), aVLast.Z(), aTol);
    }

    // Beyond the GTests

    [Fact]
    public void ClosedWire_UnitParametrisation()
    {
        Wire aWire = BoxFaceWire(out _);
        var aC = new WireCurveAdaptor(aWire);
        Assert.Equal(0.0, aC.FirstParameter());
        Assert.Equal(4.0, aC.LastParameter());
        Assert.True(aC.IsClosed);
        Assert.False(aC.IsPeriodic);
        Assert.Equal(4.0, aC.Period());
        Assert.Equal(C0, aC.Continuity());
        Assert.Equal(OtherCurve, aC.GetType());
        Assert.True(aC.Wire().IsSame(aWire));

        // the knots are the vertices, met in the explorer's order; the curve closes on itself
        var anExp = new WireExplorer(aWire);
        for (int i = 0; i < 4; i++, anExp.Next())
        {
            Point3D aV = Tool.Pnt(anExp.CurrentVertex());
            Assert.True(aC.Value(i).Distance(aV) < 1e-12, $"knot {i}");
        }
        Assert.True(aC.Value(0.0).Distance(aC.Value(4.0)) < 1e-12);

        // the intervals are the knots
        Assert.Equal(4, aC.NbIntervals(C1));
        var aT = new NCollection_Array1<double>(1, 5);
        aC.Intervals(aT, C1);
        for (int i = 1; i <= 5; i++)
        {
            Assert.Equal(i - 1.0, aT[i], 1e-14);
        }
    }

    [Fact]
    public void ClosedWire_CurvilinearAbscissa()
    {
        Wire aWire = BoxFaceWire(out _);
        var aC = new WireCurveAdaptor(aWire, true);
        // the first face of the box 1 x 2 x 3 is the bottom: a 1 x 2 rectangle
        Assert.Equal(6.0, aC.LastParameter(), 1e-12);

        // unit speed along every edge
        for (double u = 0.1; u < 6.0; u += 0.5)
        {
            aC.D1(u, out _, out Vector3D aD1);
            Assert.Equal(1.0, aD1.Magnitude(), 1e-12);
        }

        aC.Edge(2.5, out Edge anE, out double aUonE);
        Assert.False(anE.IsNull);
        Tool.Range(anE, out double f, out double l);
        Assert.InRange(aUonE, Math.Min(f, l), Math.Max(f, l));
    }

    [Fact]
    public void Trim_And_ShallowCopy()
    {
        Wire aWire = BoxFaceWire(out _);
        var aC = new WireCurveAdaptor(aWire);
        var aT = (WireCurveAdaptor)aC.Trim(0.5, 2.5, 1e-9);
        Assert.Equal(0.5, aT.FirstParameter());
        Assert.Equal(2.5, aT.LastParameter());
        foreach (double u in new[] { 0.5, 1.0, 1.7, 2.5 })
        {
            Assert.True(aT.Value(u).Distance(aC.Value(u)) < 1e-12, $"u = {u}");
        }

        var aCopy = (WireCurveAdaptor)aC.ShallowCopy();
        foreach (double u in new[] { 0.0, 0.3, 1.5, 3.9, 4.0 })
        {
            Assert.Equal(aC.Value(u), aCopy.Value(u));
            aC.D2(u, out Point3D aP, out Vector3D aV1, out Vector3D aV2);
            aCopy.D2(u, out Point3D aPc, out Vector3D aV1c, out Vector3D aV2c);
            Assert.Equal(aP, aPc);
            Assert.Equal(aV1, aV1c);
            Assert.Equal(aV2, aV2c);
        }
    }

    [Fact]
    public void SinglePeriodicEdge_IsNotPeriodic()
    {
        Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 1.0));
        Edge aBottom = ShapeCasts.Edge(new Explorer(aFace, ShapeType.Edge).Current());
        var aC = new WireCurveAdaptor(MakeWire(aBottom));
        Assert.False(aC.IsPeriodic);
        Assert.Equal(1.0, aC.LastParameter());
        Assert.Equal(CN, aC.Continuity());
        Assert.Equal(Circle, new EdgeCurveAdaptor(aBottom).GetType());
        Assert.Equal(2.0, aC.Circle().Radius());
        Assert.True(aC.Value(0.25).Distance(new Point3D(0, 2, 0)) < 1e-12);
    }

    [Fact]
    public void DegeneratedEdgesAreSkipped()
    {
        var aB = new BrepBuilder();
        Vertex aV0 = TestShapes.Vertex(new Point3D(0, 0, 0));
        Vertex aV1 = TestShapes.Vertex(new Point3D(1, 0, 0));
        Edge anE = TestShapes.Edge(aV0, aV1);
        Edge aDeg = new Edge();
        aB.MakeEdge(ref aDeg);
        aB.Add(aDeg, aV1.Oriented(Forward));
        aB.Add(aDeg, aV1.Oriented(Reversed));
        aB.Degenerated(aDeg, true);
        var aC = new WireCurveAdaptor(MakeWire(anE, aDeg));
        Assert.Equal(1.0, aC.LastParameter());
        Assert.Equal(1, aC.NbIntervals(C0));
    }
}
