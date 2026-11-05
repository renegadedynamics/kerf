// Ported from build123d tests/test_direct_api/test_constrained_arcs.py and test_constrained_lines.py. Python's
// center_on keyword is MakeConstrainedArcsCenteredOn (ConstrainedArcs.CenteredOn) when there are two tangencies, and
// the ValueError, RuntimeError and TypeError of build123d are ArgumentException, InvalidOperationException and
// ArgumentException.
// Omitted:
// - test_param_in_trim (None arguments), most of test_constrained_arcs_arg_processing (a Solid, a qualified Vector,
//   unknown keywords, no tangency, and radius with center together) and of test_make_constrained_lines_raises
//   (an unexpected keyword, one tangency, a ("three", 0) tangency): argument-type checks the overloads rule out.
// - test_higher_level_constrained_arcs/lines, the BuildLine parts: builder mode (phase 5); the selectors are
//   checked in algebra form instead.
using Kerf.Geometry;

namespace Build123d.Tests.Objects;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class ConstrainedArcsTests
{
    /// <summary>Whether every edge has a 3D curve (Python's <c>BRep_Tool.Curve_s(e.wrapped, ...)</c> check).</summary>
    internal static bool Have3DCurves(IEnumerable<Edge> edges) => edges.All(e =>
    {
        return e.TryGetCurve(out _, out _);
    });

    [Fact]
    public void EdgeToQualified2D()
    {
        var e = new Line(new Vector(0, 0), new Vector(1, 0)).Moved(new Location(new Vector(1, 1, 1)));
        var arg = ConstrainedSolvers.EdgeToQualified2D(e.Edge, Tangency.Unqualified);
        Assert.True(arg.First < arg.Last);
    }

    [Fact]
    public void TwoArcEdgesFromParams()
    {
        var p = new Point2D(0, 0);
        var d = new Direction2D(1.0, 0.0);
        var ax = new Axis2D(p, d);
        var circle = new Circle2D(ax, 1);
        Assert.Empty(ConstrainedSolvers.TwoArcEdgesFromParams(circle, 0, Constants.Tolerance / 10));
    }

    [Fact]
    public void AsGccArgEmptyEdge()
    {
        var e = Assert.Throws<ArgumentException>(() => Edge.MakeConstrainedArcs(default(Edge), new Vector(1, 0), radius: 0.5));
        Assert.Contains("Can't create a qualified curve from empty edge", e.Message);
    }

    [Fact]
    public void ArgProcessing()
    {
        Assert.Throws<ArgumentException>(() => Edge.MakeConstrainedArcs(new Vector(0, 0), new Vector(0, 0.5), radius: -0.5));
    }

    [Fact]
    public void Tan2RadArcs1()
    {
        // 2 edges & radius
        var e1 = new Line(new Vector(-2, 0), new Vector(2, 0));
        var e2 = new Line(new Vector(0, -2), new Vector(0, 2));
        Assert.Equal(8, Edge.MakeConstrainedArcs(e1, e2, radius: 0.5, sagitta: Sagitta.Both).Count);
        Assert.Equal(4, Edge.MakeConstrainedArcs(e1, e2, radius: 0.5).Count);
        var edges = Edge.MakeConstrainedArcs(new Tangent(e1, Tangency.Unqualified), new Tangent(e2, Tangency.Unqualified), radius: 0.5);
        Assert.Equal(4, edges.Count);
        Assert.True(Have3DCurves(edges));
    }

    [Fact]
    public void Tan2RadArcs2()
    {
        var e1 = new CenterArc(new Vector(0, 0), 1, 0, 90);
        var e2 = new Line(new Vector(1, 0), new Vector(2, 0));
        var edges = Edge.MakeConstrainedArcs(e1, e2, radius: 0.5);
        Assert.Single(edges);
        Assert.True(Have3DCurves(edges));
    }

    [Fact]
    public void Tan2RadArcs3()
    {
        // 2 points & radius
        Assert.Equal(2, Edge.MakeConstrainedArcs(new Vector(0, 0), new Vector(0, 0.5), radius: 0.5).Count);
        Assert.Equal(2, Edge.MakeConstrainedArcs(Vertex.Make(0, 0, 0), Vertex.Make(0, 0.5, 0), radius: 0.5).Count);
        var edges = Edge.MakeConstrainedArcs(new Vector(0, 0), new Vector(0, 0.5), radius: 0.5);
        Assert.Equal(2, edges.Count);
        Assert.True(Have3DCurves(edges));
    }

    [Fact]
    public void Tan2RadArcs4()
    {
        // An edge, a point and a radius: the point is moved after the edge
        var e1 = new Line(new Vector(0, 0), new Vector(1, 0));
        var edges = Edge.MakeConstrainedArcs(new Vector(0, 0.5), e1, radius: 0.5);
        Assert.Single(edges);
        Assert.True(Have3DCurves(edges));
    }

    [Fact]
    public void Tan2RadArcs5()
    {
        // No solution
        var e = Assert.Throws<InvalidOperationException>(() => Edge.MakeConstrainedArcs(new Vector(0, 0), new Vector(10, 0), radius: 2));
        Assert.Contains("Unable to find a tangent arc", e.Message);
    }

    [Fact]
    public void Tan2CenterOn1()
    {
        var c1 = new PolarLine(new Vector(0, 0), 4, -20, lengthMode: LengthMode.Horizontal);
        var c2 = new Line(new Vector(4, -2), new Vector(4, 2));
        var c3CenterOn = new Line(new Vector(3, -2), new Vector(3, 2));
        var edges = Edge.MakeConstrainedArcsCenteredOn(new Tangent(c1, Tangency.Unqualified), new Tangent(c2, Tangency.Unqualified), c3CenterOn);
        Assert.Single(edges);
        Assert.True(Have3DCurves(edges));
    }

    [Fact]
    public void Tan2CenterOn2()
    {
        var edges = Edge.MakeConstrainedArcsCenteredOn(new Vector(0, 3), new Vector(5, 0), new Line(new Vector(0, -5), new Vector(0, 5)));
        Assert.Single(edges);
        Assert.True(Have3DCurves(edges));
    }

    [Fact]
    public void Tan2CenterOn3()
    {
        var edges = Edge.MakeConstrainedArcsCenteredOn(new Line(new Vector(-5, 3), new Vector(5, 3)), new Vector(5, 0), new Line(new Vector(0, -5), new Vector(0, 5)));
        Assert.Single(edges);
        Assert.True(Have3DCurves(edges));
    }

    [Fact]
    public void Tan2CenterOn4()
    {
        var edges = Edge.MakeConstrainedArcsCenteredOn(new Line(new Vector(-5, 3), new Vector(5, 3)), new Vector(5, 0), Axis.Y);
        Assert.Single(edges);
        Assert.True(Have3DCurves(edges));
    }

    [Fact]
    public void Tan2CenterOn5()
    {
        var e = Assert.Throws<InvalidOperationException>(() => Edge.MakeConstrainedArcsCenteredOn(
            new Line(new Vector(-5, 3), new Vector(5, 3)), new Line(new Vector(-5, 0), new Vector(5, 0)), new Line(new Vector(-5, -1), new Vector(5, -1))));
        Assert.Contains("Unable to find a tangent arc with center_on constraint", e.Message);
    }

    [Fact]
    public void Tan2CenterOn6()
    {
        var l1 = new Line(new Vector(0, 0), new Vector(5, 0));
        var l2 = new Line(new Vector(0, 0), new Vector(0, 5));
        var l3 = new Line(new Vector(20, 20), new Vector(22, 22));
        var e = Assert.Throws<InvalidOperationException>(() => Edge.MakeConstrainedArcsCenteredOn(l1, l2, l3));
        Assert.Contains("Unable to find a tangent arc with center_on constraint", e.Message);
    }

    [Fact]
    public void Tan2CenterOnSagittaBothReturnsTwoArcs()
    {
        // Two lines, centred on a line that crosses both angle bisectors: several circles, two arcs each with Both
        var c1 = new Line(new Vector(-10, 0), new Vector(10, 0));
        var c2 = new Line(new Vector(0, -10), new Vector(0, 10));
        var centerOn = new Line(new Vector(-10, 1), new Vector(10, 1));
        var arcs = Edge.MakeConstrainedArcsCenteredOn(new Tangent(c1, Tangency.Unqualified), new Tangent(c2, Tangency.Unqualified), centerOn, Sagitta.Both);
        Assert.True(arcs.Count >= 2);
        Assert.True(Have3DCurves(arcs));
    }

    [Fact]
    public void Tan2CenterOnSagittaLongIsLongerThanShort()
    {
        var c1 = new Line(new Vector(-10, 0), new Vector(10, 0));
        var c2 = new Line(new Vector(0, -10), new Vector(0, 10));
        var centerOn = new Line(new Vector(3, -10), new Vector(3, 10));
        var shortArc = Edge.MakeConstrainedArcsCenteredOn(new Tangent(c1, Tangency.Unqualified), new Tangent(c2, Tangency.Unqualified), centerOn, Sagitta.Short);
        var longArc = Edge.MakeConstrainedArcsCenteredOn(new Tangent(c1, Tangency.Unqualified), new Tangent(c2, Tangency.Unqualified), centerOn, Sagitta.Long);
        Assert.Equal(2, shortArc.Count);
        Assert.Equal(2, longArc.Count);
        Assert.True(longArc[0].Length > shortArc[0].Length);
        Assert.True(Have3DCurves(shortArc.Concat(longArc)));
    }

    [Fact]
    public void Tan2CenterOnFiltersOutsideFirstTangentSegment()
    {
        // The tangency on the very short first segment would be near x = 5, outside it
        var tinyFirst = new Line(new Vector(0.0, 0.0), new Vector(0.01, 0.0));
        var c2 = new Line(new Vector(10.0, -10.0), new Vector(10.0, 10.0));
        var centerOn = new Line(new Vector(5.0, -10.0), new Vector(5.0, 10.0));
        Assert.Empty(Edge.MakeConstrainedArcsCenteredOn(new Tangent(tinyFirst, Tangency.Unqualified), new Tangent(c2, Tangency.Unqualified), centerOn, Sagitta.Short));
    }

    [Fact]
    public void Tan2CenterOnFiltersOutsideSecondTangentSegment()
    {
        // Through (0, 2), tangent to x = 10 and centred on y = 2: the tangency at y = 2 is outside the tiny segment
        var tinySecond = new Line(new Vector(10.0, -0.005), new Vector(10.0, 0.005));
        var centerOn = new Line(new Vector(-10.0, 2.0), new Vector(10.0, 2.0));
        Assert.Empty(Edge.MakeConstrainedArcsCenteredOn(new Vector(0.0, 2.0), new Tangent(tinySecond, Tangency.Unqualified), centerOn, Sagitta.Short));
    }

    [Fact]
    public void Tan2CenterOnMultipleSolutionsBothCounts()
    {
        var c1 = new Line(new Vector(-20, 0), new Vector(20, 0));
        var c2 = new Line(new Vector(0, -20), new Vector(0, 20));
        var centerOn = new Line(new Vector(-20, -2), new Vector(20, -2));
        var arcs = Edge.MakeConstrainedArcsCenteredOn(new Tangent(c1, Tangency.Unqualified), new Tangent(c2, Tangency.Unqualified), centerOn, Sagitta.Both);
        Assert.True(arcs.Count >= 2);
        Assert.True(Have3DCurves(arcs));
    }

    [Fact]
    public void TanCenterOn1()
    {
        var c5 = new PolarLine(new Vector(0, 0), 4, 60);
        var tanCenter = Edge.MakeConstrainedArcs(new Tangent(c5, Tangency.Unqualified), center: new Vector(2, 1));
        Assert.Single(tanCenter);
        Assert.True(tanCenter[0].CheckClosed());
        Assert.True(Have3DCurves(tanCenter));
    }

    [Fact]
    public void TanCenterOn2()
    {
        var tanCenter = Edge.MakeConstrainedArcs(Axis.X, center: new Vector(2, 1, 5));
        Assert.Single(tanCenter);
        Assert.True(tanCenter[0].CheckClosed());
        Assert.True(Have3DCurves(tanCenter));
    }

    [Fact]
    public void TanCenterOn3()
    {
        var l1 = new CenterArc(new Vector(0, 0), 1, 180, 5);
        var tanCenter = Edge.MakeConstrainedArcs(l1, center: new Vector(2, 0));
        Assert.Single(tanCenter);
        Assert.True(tanCenter[0].CheckClosed());
        Assert.True(Have3DCurves(tanCenter));
    }

    [Fact]
    public void PntCenter1()
    {
        var pntCenter = Edge.MakeConstrainedArcs(new Vector(-2.5, 1.5), center: new Vector(-2, 1));
        Assert.Single(pntCenter);
        Assert.True(pntCenter[0].CheckClosed());
        Assert.True(Have3DCurves(pntCenter));

        pntCenter = Edge.MakeConstrainedArcs(new Vector(-2.5, 1.5), center: Vertex.Make(-2, 1, 0).ToVector());
        Assert.Single(pntCenter);
        Assert.True(pntCenter[0].CheckClosed());
    }

    [Fact]
    public void TanCenArcsCenterEqualsPointReturnsEmpty()
    {
        // The centre on the point: a zero radius, so no circle
        Assert.Empty(Edge.MakeConstrainedArcs(new Vector(0, 0), center: new Vector(0, 0)));
    }

    [Fact]
    public void TanRadCenterOn1()
    {
        var c1 = new PolarLine(new Vector(0, 0), 4, -20, lengthMode: LengthMode.Horizontal);
        var c3CenterOn = new Line(new Vector(3, -2), new Vector(3, 2));
        var tanRadOn = Edge.MakeConstrainedArcs(new Tangent(c1, Tangency.Unqualified), radius: 1, centerOn: c3CenterOn);
        Assert.Single(tanRadOn);
        Assert.True(tanRadOn[0].CheckClosed());
        Assert.True(Have3DCurves(tanRadOn));
    }

    [Fact]
    public void TanRadCenterOn2()
    {
        var c1 = new PolarLine(new Vector(0, 0), 4, -20, lengthMode: LengthMode.Horizontal);
        var tanRadOn = Edge.MakeConstrainedArcs(c1, radius: 1, centerOn: Axis.X);
        Assert.Single(tanRadOn);
        Assert.True(tanRadOn[0].CheckClosed());
        Assert.True(Have3DCurves(tanRadOn));
    }

    [Fact]
    public void TanRadCenterOn3()
    {
        // The centre on a point rather than a curve (Python passes a Face, a TypeError)
        var c1 = new PolarLine(new Vector(0, 0), 4, -20, lengthMode: LengthMode.Horizontal);
        Assert.Throws<ArgumentException>(() => Edge.MakeConstrainedArcs(c1, radius: 1, centerOn: new Vector(0, 0)));
    }

    [Fact]
    public void TanRadCenterOn4()
    {
        var c1 = new Line(new Vector(0, 10), new Vector(10, 10));
        Assert.Throws<InvalidOperationException>(() => Edge.MakeConstrainedArcs(c1, radius: 1, centerOn: Axis.X));
    }

    [Fact]
    public void Tan31()
    {
        // 3 tangents
        var c5 = new PolarLine(new Vector(0, 0), 4, 60);
        var c6 = new PolarLine(new Vector(0, 0), 4, 40);
        var c7 = new CenterArc(new Vector(0, 0), 4, 0, 90);
        var tan3 = Edge.MakeConstrainedArcs(new Tangent(c5, Tangency.Unqualified), new Tangent(c6, Tangency.Unqualified), new Tangent(c7, Tangency.Unqualified));
        Assert.Single(tan3);
        Assert.False(tan3[0].CheckClosed());

        var tan3b = Edge.MakeConstrainedArcs(c5, c6, c7, sagitta: Sagitta.Both);
        Assert.Equal(2, tan3b.Count);
        Assert.True(Have3DCurves(tan3b));
    }

    [Fact]
    public void Tan32()
    {
        try
        {
            Assert.Empty(Edge.MakeConstrainedArcs(new Line(new Vector(0, 0), new Vector(0, 1)), new Line(new Vector(0, 0), new Vector(1, 0)), new Line(new Vector(0, 0), new Vector(0, -1))));
        }
        catch (InvalidOperationException e)
        {
            Assert.Contains("Unable to find a circle tangent to all three objects", e.Message);
        }
    }

    [Fact]
    public void Tan33()
    {
        var l1 = new Line(new Vector(0, 0), new Vector(10, 0));
        var l2 = new Line(new Vector(0, 2), new Vector(10, 2));
        var l3 = new Line(new Vector(0, 5), new Vector(10, 5));
        var e = Assert.Throws<InvalidOperationException>(() => Edge.MakeConstrainedArcs(l1, l2, l3));
        Assert.Contains("Unable to find a circle tangent to all three objects", e.Message);
    }

    [Fact]
    public void Tan34()
    {
        var l1 = new Line(new Vector(-1, 0), new Vector(-1, 2));
        var l2 = new Line(new Vector(1, 0), new Vector(1, 2));
        var l3 = new Line(new Vector(-1, 0), new Vector(-0.75, 0));
        Assert.Empty(Edge.MakeConstrainedArcs(l1, l2, l3));
    }

    [Fact]
    public void MakeConstrainedArcs3Tan()
    {
        // Correct trimming in _make_3tan_arcs
        var c1 = new CenterArc(new Vector(0, 20), 20, -90, 90);
        var c2 = new CenterArc(new Vector(5, 25), 10, -90, 90);
        var ln1 = new Line(c1.PositionAt(1), c2.PositionAt(1));
        var c3 = new RadiusArc(c1.PositionAt(1), c2.PositionAt(1), 10);
        foreach (Edge el in new Edge[] { ln1, c3 })
        {
            var tan3 = Edge.MakeConstrainedArcs(new Tangent(c1, Tangency.Unqualified), new Tangent(c2, Tangency.Unqualified), new Tangent(el, Tangency.Unqualified), Sagitta.Short);
            Assert.NotEmpty(el.Intersect(tan3[0]));
        }
    }

    [Fact]
    public void ConstrainedArcs2TanCenterOn()
    {
        // Correct trimming in _make_2tan_on_arcs
        var c1 = new CenterArc(new Vector(0, 20), 20, -90, 90);
        var ln2 = new Line(c1.PositionAt(1), new Vector(10, 20));
        var ln3 = new Line(new Vector(10, 10), ln2.PositionAt(1));
        var ln4 = ConstrainedArcs.CenteredOn(tangencyOne: c1, tangencyTwo: ln2, centerOn: ln3);
        Assert.NotEmpty(ln4.Vertices().OrderBy(Axis.Y)[0].Intersect(c1));
    }

    [Fact]
    public void Eggplant()
    {
        // A complex set of 4 arcs
        double rLeft = 0.75, rRight = 1.0, rBottom = 6, rTop = 8;
        var conCircleLeft = new CenterArc(new Vector(-2, 0), rLeft, 0, 360);
        var conCircleRight = new CenterArc(new Vector(2, 0), rRight, 0, 360);
        var eggBottom = Edge.MakeConstrainedArcs(new Tangent(conCircleRight, Tangency.Outside), new Tangent(conCircleLeft, Tangency.Outside), radius: rBottom)
            .OrderBy(Axis.Y)[0];
        var eggTop = Edge.MakeConstrainedArcs(new Tangent(conCircleRight, Tangency.Enclosing), new Tangent(conCircleLeft, Tangency.Enclosing), radius: rTop)
            .OrderBy(Axis.Y)[^1];
        var eggRight = new ThreePointArc(eggBottom.SelectVertices().OrderBy(Axis.X)[^1].ToVector(), conCircleRight.PositionAt(0),
                                         eggTop.SelectVertices().OrderBy(Axis.X)[^1].ToVector());
        var eggLeft = new ThreePointArc(eggBottom.SelectVertices().OrderBy(Axis.X)[0].ToVector(), conCircleLeft.PositionAt(0.5),
                                        eggTop.SelectVertices().OrderBy(Axis.X)[0].ToVector());

        var eggPlant = Wire.Make([eggLeft, eggTop, eggRight, eggBottom]);
        Assert.True(eggPlant.CheckClosed());
        var edges = eggPlant.SelectEdges().OrderBy(eggPlant);
        Assert.Equal(4, Enumerable.Range(0, 4).Count(i => !Vertex.CommonVertex(edges[i], edges[(i + 1) % 4]).IsEmpty));

        // C1 continuity
        Assert.All(Enumerable.Range(0, 4), i => Assert.True(((edges[i]).TangentAt(1) - (edges[(i + 1) % 4]).TangentAt(0)).Length < Constants.Tolerance));
        Assert.True(Have3DCurves([eggBottom, eggTop, eggRight, eggLeft]));
    }

    [Fact]
    public void HigherLevelConstrainedArcs()
    {
        var unitCircle = Edge.MakeCircle(1.0, Plane.XY);
        var lines = new ConstrainedArcs(unitCircle, Axis.Y, radius: 1);
        Assert.Equal(4, lines.Edges().Count);

        var axisCentered = new ConstrainedArcs(new Line(new Vector(0, 0), new Vector(4, 0)), radius: 1, centerOn: Axis.Y);
        Assert.Equal(2, axisCentered.Edges().Count);

        var selected = new ConstrainedArcs(unitCircle, Axis.Y, radius: 1, selector: a => a.GroupBy(Axis.Y)[^1]);
        Assert.Equal(2, selected.Edges().Count);

        Assert.Throws<ArgumentException>(() => new ConstrainedArcs(unitCircle, Axis.Y, radius: 1, selector: l => l.Where(GeomType.Line)));
    }
}

public class ConstrainedLinesTests
{
    private static Edge UnitCircle() => Edge.MakeCircle(1.0, Plane.XY);

    [Fact]
    public void EdgeFromLine()
    {
        var p0 = new Point2D(0, 0);
        var p1 = new Point2D(1, 0);
        Assert.Equal(1, ConstrainedSolvers.EdgeFromLine(p0, p1).Length);

        var e = Assert.Throws<InvalidOperationException>(() => ConstrainedSolvers.EdgeFromLine(p0, p0));
        Assert.Contains("Failed to build edge from line contacts", e.Message);
    }

    [Fact]
    public void TwoCirclesTangents()
    {
        // Tangent lines between two separated circles
        var c1 = UnitCircle();
        var c2 = UnitCircle().Translate(new Vector(3, 0, 0));
        var lines = ConstrainedSolvers.TwoTanLines(c1, c2);
        Assert.Contains(lines.Count, new[] { 4, 2 });
        // Tangent lines don't cut the circle
        Assert.All(lines, ln => Assert.True(c1.DistanceTo(ln) >= -1e-6));
    }

    [Fact]
    public void TwoConstrainedCirclesTangents1()
    {
        var c1 = UnitCircle();
        var c2 = UnitCircle().Translate(new Vector(3, 0, 0));
        var lines = ConstrainedSolvers.TwoTanLines(new Tangent(c1, Tangency.Enclosing), c2);
        Assert.Equal(2, lines.Count);
        Assert.All(lines, ln => Assert.True(c1.DistanceTo(ln) >= -1e-6));
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void TwoConstrainedCirclesTangents2()
    {
        var c1 = UnitCircle();
        var c2 = UnitCircle().Translate(new Vector(3, 0, 0));
        var lines = ConstrainedSolvers.TwoTanLines(new Tangent(c1, Tangency.Enclosing), new Tangent(c2, Tangency.Enclosing));
        Assert.Single(lines);
        Assert.All(lines, ln => Assert.True(c1.DistanceTo(ln) >= -1e-6));
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void CurveAndPointTangent()
    {
        // Lines tangent to a circle through a point
        var unitCircle = UnitCircle();
        var pt = new Vector(2.0, 0.0);
        var lines = ConstrainedSolvers.TwoTanLines(unitCircle, pt);
        Assert.Equal(2, lines.Count);
        foreach (var ln in lines)
        {
            Assert.True(Math.Abs(ln.DistanceTo(pt)) <= 1e-6);
            Assert.True(Math.Abs(unitCircle.DistanceTo(ln)) <= Constants.Tolerance);
        }
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void InvalidTangentRaises()
    {
        // Degenerate input: two identical circles have no common tangent the solver gives (OCCT returned degenerate
        // lines, which were skipped; Kerf decision 1544)
        var e0 = Assert.Throws<InvalidOperationException>(() => ConstrainedSolvers.TwoTanLines(UnitCircle(), UnitCircle()));
        Assert.Contains("Unable to find common tangent line(s)", e0.Message);
        var e = Assert.Throws<InvalidOperationException>(() => ConstrainedSolvers.TwoTanLines(UnitCircle(), new Vector(0, 0)));
        Assert.Contains("Unable to find common tangent line(s)", e.Message);
    }

    [Theory]
    [InlineData(30.0)]
    [InlineData(-30.0)]
    public void OrientedTangentsWithXAxis(double angleDeg)
    {
        // Lines tangent to a circle at ±30° from the X axis
        var lines = ConstrainedSolvers.TanOrientedLines(UnitCircle(), Axis.X, angleDeg * Constants.DegToRad);
        Assert.All(lines, ln => Assert.True(Math.Abs(ln.PositionAt(0.5).Z) < 1e-9));

        Assert.Empty(ConstrainedSolvers.TanOrientedLines(UnitCircle(), Axis.X, 0));
        Assert.Empty(ConstrainedSolvers.TanOrientedLines(UnitCircle(), new Axis(new Vector(0, -2), new Vector(1, 0)), 0));
    }

    [Fact]
    public void OrientedTangentsWithYAxis()
    {
        var unitCircle = UnitCircle();
        var lines = ConstrainedSolvers.TanOrientedLines(unitCircle, Axis.Y, 30 * Constants.DegToRad);
        Assert.NotEmpty(lines);
        Assert.All(lines, ln => Assert.True(unitCircle.DistanceTo(ln) < 1e-6));
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void OrientedConstrainedTangentsWithYAxis()
    {
        var unitCircle = UnitCircle();
        var lines = ConstrainedSolvers.TanOrientedLines(new Tangent(unitCircle, Tangency.Enclosing), Axis.Y, 30 * Constants.DegToRad);
        Assert.Single(lines);
        Assert.All(lines, ln => Assert.True(unitCircle.DistanceTo(ln) < 1e-6));
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void InvalidOrientedTangentRaises()
    {
        var e = Assert.Throws<ArgumentException>(() => ConstrainedSolvers.TanOrientedLines(UnitCircle(), Axis.Z, 1));
        Assert.Contains("reference Axis can't be perpendicular to Plane.XY", e.Message);
        e = Assert.Throws<ArgumentException>(() => ConstrainedSolvers.TanOrientedLines(UnitCircle(), new Axis(new Vector(1, 2, 3), new Vector(0, 0, -1)), 1));
        Assert.Contains("reference Axis can't be perpendicular to Plane.XY", e.Message);
    }

    [Fact]
    public void InvalidOrientedTangent()
    {
        Assert.Empty(ConstrainedSolvers.TanOrientedLines(UnitCircle(), new Axis(new Vector(1, 0), new Vector(0, 1)), 0));
        Assert.Empty(ConstrainedSolvers.TanOrientedLines(UnitCircle().Translate(new Vector(0, 1 + 1e-7)), Axis.X, 0));
    }

    [Fact]
    public void MakeConstrainedLines0()
    {
        var unitCircle = UnitCircle();
        var lines = Edge.MakeConstrainedLines(unitCircle, unitCircle.Translate(new Vector(3, 0, 0)));
        Assert.Equal(4, lines.Count);
        Assert.All(lines, ln => Assert.True(unitCircle.DistanceTo(ln) < 1e-6));
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void MakeConstrainedLines1()
    {
        var unitCircle = UnitCircle();
        var lines = Edge.MakeConstrainedLines(unitCircle, new Vector(3, 0));
        Assert.Equal(2, lines.Count);
        Assert.All(lines, ln => Assert.True(unitCircle.DistanceTo(ln) < 1e-6));
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void MakeConstrainedLines3()
    {
        var unitCircle = UnitCircle();
        var lines = Edge.MakeConstrainedLines(unitCircle, Axis.X, angle: 30);
        Assert.Equal(2, lines.Count);
        foreach (var ln in lines)
        {
            Assert.True(unitCircle.DistanceTo(ln) < 1e-6);
            Assert.True(Math.Abs(ln.PositionAt(1).Y) < 1e-6);
        }
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void MakeConstrainedLines4()
    {
        var unitCircle = UnitCircle();
        var lines = Edge.MakeConstrainedLines(unitCircle, Axis.Y, angle: 30);
        Assert.Equal(2, lines.Count);
        foreach (var ln in lines)
        {
            Assert.True(unitCircle.DistanceTo(ln) < 1e-6);
            Assert.True(Math.Abs(ln.PositionAt(1).X) < 1e-6);
        }
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void MakeConstrainedLines5()
    {
        var unitCircle = UnitCircle();
        var lines = Edge.MakeConstrainedLines(new Tangent(unitCircle, Tangency.Enclosing), Axis.Y, angle: 30);
        Assert.Single(lines);
        Assert.All(lines, ln => Assert.True(unitCircle.DistanceTo(ln) < 1e-6));
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void MakeConstrainedLines6()
    {
        var unitCircle = UnitCircle();
        var lines = Edge.MakeConstrainedLines(new Tangent(unitCircle, Tangency.Enclosing), Axis.Y, direction: new Vector(1, 1));
        Assert.Single(lines);
        Assert.All(lines, ln => Assert.True(unitCircle.DistanceTo(ln) < 1e-6));
        Assert.True(ConstrainedArcsTests.Have3DCurves(lines));
    }

    [Fact]
    public void MakeConstrainedLinesRaises()
    {
        var e = Assert.Throws<InvalidOperationException>(() => Edge.MakeConstrainedLines(Axis.X, Axis.Y));
        Assert.Contains("Unable to find common tangent line(s)", e.Message);
    }

    [Fact]
    public void HigherLevelConstrainedLines()
    {
        var unitCircle = UnitCircle();
        var lines = new ConstrainedLines(unitCircle, Axis.Y, direction: new Vector(1, 1));
        Assert.Equal(2, lines.Edges().Count);

        var selected = new ConstrainedLines(unitCircle, Axis.Y, direction: new Vector(1, 1), selector: l => [l.OrderBy(Axis.Y)[^1]]);
        Assert.Single(selected.Edges());

        Assert.Throws<ArgumentException>(() => new ConstrainedLines(unitCircle, Axis.Y, direction: new Vector(1, 1), selector: l => l.Where(GeomType.Circle)));
    }
}
