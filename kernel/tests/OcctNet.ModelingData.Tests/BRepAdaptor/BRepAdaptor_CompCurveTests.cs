// Port of: src/ModelingData/TKBRep/GTests/BRepAdaptor_CompCurve_Test.cxx
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT's BRepAdaptor_CompCurve GTests, on <see cref="TestShapes"/> in place of BRepBuilderAPI_MakeEdge,
/// BRepBuilderAPI_MakeWire and GC_MakeArcOfCircle (unit 3K), and further tests of the composite curve.
/// </summary>
public class BRepAdaptor_CompCurveTests
{
    /// <summary>A frozen wire of the edges, in this order (as BRepBuilderAPI_MakeWire of one edge gives).</summary>
    internal static TopoDS_Wire MakeWire(params TopoDS_Edge[] theEdges)
    {
        var aB = new BRep_Builder();
        TopoDS_Wire aW = new TopoDS_Wire();
        aB.MakeWire(ref aW);
        foreach (TopoDS_Edge anE in theEdges)
        {
            aB.Add(aW, anE);
        }

        return TestShapes.Frozen(aW);
    }

    /// <summary>The outer wire of the box's first face (a closed wire of 4 segments, from a frozen box).</summary>
    internal static TopoDS_Wire BoxFaceWire(out TopoDS_Face theFace)
    {
        TopoDS_Shape aBox = TestShapes.Frozen(TestShapes.Box(1.0, 2.0, 3.0));
        theFace = TopoDS.Face(new TopExp_Explorer(aBox, TopAbs_FACE).Current());
        return TopoDS.Wire(new TopExp_Explorer(theFace, TopAbs_WIRE).Current());
    }

    // Test OCC5696: BRepAdaptor_CompCurve::Edge() method
    [Fact]
    public void OCC5696_EdgeMethod()
    {
        // Create a simple edge from two points
        TopoDS_Edge anEdge = TestShapes.Edge(new gp_Pnt(0, 0, 0), new gp_Pnt(2, 0, 0));

        // Create a wire from the edge
        TopoDS_Wire aWire = MakeWire(anEdge);

        // Create a composite curve adaptor
        var aCurve = new BRepAdaptor_CompCurve(aWire);

        // Get curve parameters
        double aFirst = aCurve.FirstParameter();
        double aLast = aCurve.LastParameter();
        double aPar = (aFirst + aLast) / 2.0;

        // Test the Edge() method
        double aParEdge = 0.0;
        TopoDS_Edge anEdgeFound = new TopoDS_Edge();

        // The original test was checking that this method doesn't throw an exception and returns valid parameter
        Exception? anException = Record.Exception(() => aCurve.Edge(aPar, out anEdgeFound, out aParEdge));
        Assert.Null(anException);

        // Verify that the returned edge is valid
        Assert.False(anEdgeFound.IsNull());

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

        var aCircle = new Geom_Circle(new gp_Circ(new gp_Ax2(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 1.0));
        // GC_MakeArcOfCircle(circ, r45, r225, true) is a trimmed circle; BRepBuilderAPI_MakeEdge keeps its basis and range
        TopoDS_Wire aWire = MakeWire(TestShapes.Edge(aCircle, r45, r225));

        var aCurve = new BRepAdaptor_CompCurve(aWire);
        gp_Pnt aStartPt = aCurve.Value(aCurve.FirstParameter());
        gp_Pnt anEndPt = aCurve.Value(aCurve.LastParameter());

        // Collect wire vertices
        var aVertices = new List<gp_Pnt>();
        for (var anExp = new TopExp_Explorer(aWire, TopAbs_VERTEX); anExp.More(); anExp.Next())
        {
            aVertices.Add(BRep_Tool.Pnt(TopoDS.Vertex(anExp.Current())));
        }
        Assert.True(aVertices.Count >= 1);

        bool aStartMatchesAnyVertex = false;
        bool anEndMatchesAnyVertex = false;
        foreach (gp_Pnt aV in aVertices)
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
        var anAx2 = new gp_Ax2(new gp_Pnt(1.0, 0.0, 0.0), new gp_Dir(0.0, -1.0, 0.0), new gp_Dir(0.0, 0.0, -1.0));
        var aCircle = new Geom_Circle(anAx2, 1.0);

        double t1 = Math.PI / 2.0;
        double t2 = 3.0 * Math.PI / 2.0;

        // BRepBuilderAPI_MakeEdge of the trimmed circle: the circle over [t1, t2]
        TopoDS_Edge anEdge = TestShapes.Edge(aCircle, t1, t2);

        // Reverse the edge, then wrap it in a wire
        anEdge = TopoDS.Edge(anEdge.Oriented(TopAbs_REVERSED));
        TopoDS_Wire aWire = MakeWire(anEdge);

        var aBACC = new BRepAdaptor_CompCurve(aWire);
        double aFirst = aBACC.FirstParameter();
        double aLast = aBACC.LastParameter();

        aBACC.D1(aFirst, out gp_Pnt aPFirst, out gp_Vec aVFirst);
        aBACC.D1(aLast, out gp_Pnt aPLast, out gp_Vec aVLast);

        if (aVFirst.SquareMagnitude() > gp.Resolution())
        {
            aVFirst.Normalize();
        }
        if (aVLast.SquareMagnitude() > gp.Resolution())
        {
            aVLast.Normalize();
        }

        // Reference: inverse circle (normal = (0,1,0)), evaluated at the same parameters
        var anAx2Ref = new gp_Ax2(new gp_Pnt(1.0, 0.0, 0.0), new gp_Dir(0.0, 1.0, 0.0), new gp_Dir(0.0, 0.0, -1.0));
        var aCircleRef = new Geom_Circle(anAx2Ref, 1.0);

        aCircleRef.D1(t1, out gp_Pnt aRefP1, out gp_Vec aRefV1);
        aCircleRef.D1(t2, out gp_Pnt aRefP2, out gp_Vec aRefV2);
        if (aRefV1.SquareMagnitude() > gp.Resolution())
        {
            aRefV1.Normalize();
        }
        if (aRefV2.SquareMagnitude() > gp.Resolution())
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
        TopoDS_Wire aWire = BoxFaceWire(out _);
        var aC = new BRepAdaptor_CompCurve(aWire);
        Assert.Equal(0.0, aC.FirstParameter());
        Assert.Equal(4.0, aC.LastParameter());
        Assert.True(aC.IsClosed());
        Assert.False(aC.IsPeriodic());
        Assert.Equal(4.0, aC.Period());
        Assert.Equal(GeomAbs_C0, aC.Continuity());
        Assert.Equal(GeomAbs_OtherCurve, aC.GetType());
        Assert.True(aC.Wire().IsSame(aWire));

        // the knots are the vertices, met in the explorer's order; the curve closes on itself
        var anExp = new BRepTools_WireExplorer(aWire);
        for (int i = 0; i < 4; i++, anExp.Next())
        {
            gp_Pnt aV = BRep_Tool.Pnt(anExp.CurrentVertex());
            Assert.True(aC.Value(i).Distance(aV) < 1e-12, $"knot {i}");
        }
        Assert.True(aC.Value(0.0).Distance(aC.Value(4.0)) < 1e-12);

        // the intervals are the knots
        Assert.Equal(4, aC.NbIntervals(GeomAbs_C1));
        var aT = new NCollection_Array1<double>(1, 5);
        aC.Intervals(aT, GeomAbs_C1);
        for (int i = 1; i <= 5; i++)
        {
            Assert.Equal(i - 1.0, aT[i], 1e-14);
        }
    }

    [Fact]
    public void ClosedWire_CurvilinearAbscissa()
    {
        TopoDS_Wire aWire = BoxFaceWire(out _);
        var aC = new BRepAdaptor_CompCurve(aWire, true);
        // the first face of the box 1 x 2 x 3 is the bottom: a 1 x 2 rectangle
        Assert.Equal(6.0, aC.LastParameter(), 1e-12);

        // unit speed along every edge
        for (double u = 0.1; u < 6.0; u += 0.5)
        {
            aC.D1(u, out _, out gp_Vec aD1);
            Assert.Equal(1.0, aD1.Magnitude(), 1e-12);
        }

        aC.Edge(2.5, out TopoDS_Edge anE, out double aUonE);
        Assert.False(anE.IsNull());
        BRep_Tool.Range(anE, out double f, out double l);
        Assert.InRange(aUonE, Math.Min(f, l), Math.Max(f, l));
    }

    [Fact]
    public void Trim_And_ShallowCopy()
    {
        TopoDS_Wire aWire = BoxFaceWire(out _);
        var aC = new BRepAdaptor_CompCurve(aWire);
        var aT = (BRepAdaptor_CompCurve)aC.Trim(0.5, 2.5, 1e-9);
        Assert.Equal(0.5, aT.FirstParameter());
        Assert.Equal(2.5, aT.LastParameter());
        foreach (double u in new[] { 0.5, 1.0, 1.7, 2.5 })
        {
            Assert.True(aT.Value(u).Distance(aC.Value(u)) < 1e-12, $"u = {u}");
        }

        var aCopy = (BRepAdaptor_CompCurve)aC.ShallowCopy();
        foreach (double u in new[] { 0.0, 0.3, 1.5, 3.9, 4.0 })
        {
            Assert.Equal(aC.Value(u), aCopy.Value(u));
            aC.D2(u, out gp_Pnt aP, out gp_Vec aV1, out gp_Vec aV2);
            aCopy.D2(u, out gp_Pnt aPc, out gp_Vec aV1c, out gp_Vec aV2c);
            Assert.Equal(aP, aPc);
            Assert.Equal(aV1, aV1c);
            Assert.Equal(aV2, aV2c);
        }
    }

    [Fact]
    public void SinglePeriodicEdge_IsNotPeriodic()
    {
        TopoDS_Face aFace = TestShapes.Frozen(TestShapes.CylinderFace(2.0, 1.0));
        TopoDS_Edge aBottom = TopoDS.Edge(new TopExp_Explorer(aFace, TopAbs_EDGE).Current());
        var aC = new BRepAdaptor_CompCurve(MakeWire(aBottom));
        Assert.False(aC.IsPeriodic());
        Assert.Equal(1.0, aC.LastParameter());
        Assert.Equal(GeomAbs_CN, aC.Continuity());
        Assert.Equal(GeomAbs_Circle, new BRepAdaptor_Curve(aBottom).GetType());
        Assert.Equal(2.0, aC.Circle().Radius());
        Assert.True(aC.Value(0.25).Distance(new gp_Pnt(0, 2, 0)) < 1e-12);
    }

    [Fact]
    public void DegeneratedEdgesAreSkipped()
    {
        var aB = new BRep_Builder();
        TopoDS_Vertex aV0 = TestShapes.Vertex(new gp_Pnt(0, 0, 0));
        TopoDS_Vertex aV1 = TestShapes.Vertex(new gp_Pnt(1, 0, 0));
        TopoDS_Edge anE = TestShapes.Edge(aV0, aV1);
        TopoDS_Edge aDeg = new TopoDS_Edge();
        aB.MakeEdge(ref aDeg);
        aB.Add(aDeg, aV1.Oriented(TopAbs_FORWARD));
        aB.Add(aDeg, aV1.Oriented(TopAbs_REVERSED));
        aB.Degenerated(aDeg, true);
        var aC = new BRepAdaptor_CompCurve(MakeWire(anE, aDeg));
        Assert.Equal(1.0, aC.LastParameter());
        Assert.Equal(1, aC.NbIntervals(GeomAbs_C0));
    }
}
