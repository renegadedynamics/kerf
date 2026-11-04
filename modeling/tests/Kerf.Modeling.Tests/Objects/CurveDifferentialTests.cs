// Checks against values computed by Python build123d (and scipy 1.18) for what its tests don't pin down: the length
// and the points at 0, 0.37 and 1 of each curve, the counts of edges and solutions, and areas. Python's values are
// printed to 12 significant figures.

namespace Build123d.Tests.Objects;

// The tests keep build123d.NET's namespaces (see the project file); the library is Kerf.Modeling
using Kerf.Modeling;

public class CurveDifferentialTests
{
    /// <summary>The curve's length and its points at 0, 1 and 0.37, as Python gives them.</summary>
    private static void Same(Shape1D curve, double length, Vector start, Vector end, Vector at37, int edgeCount = 1)
    {
        Check.AlmostEqual(length, curve.Length, 7);
        Check.AlmostEqual(start, curve.PositionAt(0), 7);
        Check.AlmostEqual(end, curve.PositionAt(1), 7);
        Check.AlmostEqual(at37, curve.PositionAt(0.37), 7);
        Assert.Equal(edgeCount, curve.Edges().Count);
    }

    [Fact]
    public void DoubleTangentArc()
    {
        Same(new DoubleTangentArc(new Vector(0, 5), new Vector(1, 0), new Line(new Vector(10, 0), new Vector(30, 20))),
             28.4417832494, new Vector(0, 5, 0), new Vector(25.6066015625, 15.6066015625, 0), new Vector(10.3759718394, 6.51831459795, 0));
        Same(new DoubleTangentArc(new Vector(0, 0), new Vector(1, 0), new Line(new Vector(10, 0), new Vector(20, -10))),
             18.9611890631, new Vector(0, 0, 0), new Vector(17.0710678711, -7.07106787109, 0), new Vector(6.91731464118, -1.01220975637, 0));
        var l5 = new Polyline(new Vector(20, -10), new Vector(10, 0), new Vector(20, 10));
        Same(new DoubleTangentArc(new Vector(0, 0), new Vector(1, 0), l5, keep: Keep.Bottom),
             18.9611890631, new Vector(0, 0, 0), new Vector(17.0710678711, -7.07106787109, 0), new Vector(6.91731464118, -1.01220975637, 0));
        Same(new DoubleTangentArc(new Vector(0, 0), new Vector(1, 0), l5, keep: Keep.Top),
             18.9611890631, new Vector(0, 0, 0), new Vector(17.0710678711, 7.07106787109, 0), new Vector(6.91731464118, 1.01220975637, 0));
        // Which of the arcs is Top depends on the orientation of the common plane, so on Python's set order
        var spline = new Spline([new Vector(15, 5), new Vector(5, 0), new Vector(15, -5)], tangents: [new Vector(-1, 0), new Vector(1, 0)]);
        Same(new DoubleTangentArc(new Vector(0, 0, 0), new Vector(1, 0, 0), spline, keep: Keep.Top),
             6.70960537467, new Vector(0, 0, 0), new Vector(5.94660970005, 2.65828420822, 0), new Vector(2.44270770447, 0.383031616692, 0));
        Same(new DoubleTangentArc(new Vector(0, 0, 0), new Vector(1, 0, 0), spline, keep: Keep.Bottom),
             6.70960537467, new Vector(0, 0, 0), new Vector(5.94660970005, -2.65828420822, 0), new Vector(2.44270770447, -0.383031616692, 0));
        var ellipse = new EllipticalCenterArc(new Vector(15, 0), 10, 5, startAngle: 90, arcSize: 180);
        Same(new DoubleTangentArc(new Vector(0, 0, 0), new Vector(1, 0, 0), ellipse, keep: Keep.Top),
             6.84287606213, new Vector(0, 0, 0), new Vector(6.23926954104, 2.41089204577, 0), new Vector(2.50056305469, 0.343287304678, 0));
        Same(new DoubleTangentArc(new Vector(0, 0, 0), new Vector(1, 0, 0), ellipse, keep: Keep.Bottom),
             6.84287606213, new Vector(0, 0, 0), new Vector(6.23926954104, -2.41089204577, 0), new Vector(2.50056305469, -0.343287304678, 0));
        Same(new DoubleTangentArc(new Vector(0, 0, 0), new Vector(1, 1, 0), new CenterArc(new Vector(10, 3), 3, 0, 360)),
             8.96926996571, new Vector(0, 0, 0), new Vector(8.2068007011, 0.594914497483, 0), new Vector(2.84410653632, 1.63438194631, 0));
        Same(new DoubleTangentArc(new Vector(2, 1, 0), new Vector(0, 1, 0), new Line(new Vector(5, -10), new Vector(6, 10))),
             5.66146692484, new Vector(2, 1, 0), new Vector(5.545570802, 0.911416039978, 0), new Vector(3.09960608105, 2.64074109724, 0));
    }

    [Fact]
    public void MinimizeNelderMead()
    {
        // scipy's minimize(method="Nelder-Mead", bounds=[(0, upper)], tol=TOLERANCE) of DoubleTangentArc's function: the
        // same floating-point result
        var line = new Line(new Vector(10, 0), new Vector(30, 20));
        double Func(double r) => Math.Abs(line.DistanceTo(new Vector(0, 5) + new Vector(0, -1) * r) - r);
        Assert.Equal(12.500000000000014, Numerics.MinimizeNelderMead(Func, 0.0, 0, 300, Constants.Tolerance));
        Assert.Equal(0.001, Numerics.MinimizeNelderMead(Func, 0.0, 0, 1e-3, Constants.Tolerance));
        Assert.Equal(12.5, Numerics.MinimizeNelderMead(Func, 5.0, 0, 300, Constants.Tolerance));
        Assert.Equal(12.49999985033217, Numerics.MinimizeNelderMead(Func, 299.99, 0, 300, Constants.Tolerance));
    }

    [Fact]
    public void PythonSetOrder()
    {
        Assert.Equal(0, PointOrder.HashDouble(-0.0));
        Assert.Equal(-2, PointOrder.HashDouble(-1.0));
        Assert.Equal(1152921504606846976, PointOrder.HashDouble(0.5));
        Assert.Equal(-326484311674566659, PointOrder.HashDouble(-3.14159));
        Assert.Equal(52920977297143526, PointOrder.HashDouble(2.5e-300));
        Assert.Equal(529344067295497451, PointOrder.HashTuple(1.0, 2.0, 3.0));
        Assert.Equal(3250134588623674580, PointOrder.HashTuple(0.0, -0.5, 1e-6));

        // list(set(points)) of 40 points and a repeated one, through two resizes of the table
        var points = Enumerable.Range(0, 40).Select(i => new Vector(Math.Cos(i) * 10, Math.Sin(i * 1.3) * 7, i % 3 * 0.1))
            .Concat(Enumerable.Repeat(new Vector(1, 2, 3), 3));
        var ordered = PointOrder.Distinct(points);
        Assert.Equal(41, ordered.Count);
        // Python printed X and Y only
        Vector Xy(int i) => new Vector(ordered[i].X, ordered[i].Y);
        Check.AlmostEqual(new Vector(5.403023, 6.744907), Xy(0), 6);
        Check.AlmostEqual(new Vector(9.147424, 3.603181), Xy(1), 6);
        Check.AlmostEqual(new Vector(10, 0), Xy(30), 6);
        Check.AlmostEqual(new Vector(1, 2), Xy(32), 6);
        Check.AlmostEqual(new Vector(-1.455, -5.794785), Xy(40), 6);

        // The common plane that follows from it
        var plane = Edge.MakeLine(new Vector(0, 0), new Vector(1, 0)).CommonPlane(new Spline([new Vector(15, 5), new Vector(5, 0), new Vector(15, -5)], tangents: [new Vector(-1, 0), new Vector(1, 0)]))!;
        Check.AlmostEqual(new Vector(0, 0, -1), plane.ZDir);
        Check.AlmostEqual(new Vector(-1, 0, 0), plane.XDir);
        Check.AlmostEqual(new Vector(2.75, 0, 0), plane.Origin);
    }

    [Fact]
    public void Airfoil()
    {
        (string Code, int N, bool FiniteTE, double Length, int Edges, Vector At25, double Camber, double M, double P, double T)[] cases =
        [
            ("2412", 50, false, 2.04185953094, 1, new Vector(0.49512385478, 0.0727048379178, 0), 1.00110982263, 0.02, 0.4, 0.12),
            ("0012", 30, true, 2.04178730735, 2, new Vector(0.492341500268, 0.0534171804189, 0), 1, 0.0, 0.0, 0.12),
            ("2213.323", 40, false, 2.05024901803, 1, new Vector(0.493486391268, 0.0765380823839, 0), 1.0016586637, 0.02, 0.2, 0.13323000000000002),
            ("4415", 9, true, 2.06759474466, 2, new Vector(0.494811614659, 0.105744898545, 0), 1.00442443685, 0.04, 0.4, 0.15),
        ];
        foreach (var c in cases)
        {
            var a = new Airfoil(c.Code, c.N, c.FiniteTE);
            Check.AlmostEqual(c.Length, a.Length, 7);
            Assert.Equal(c.Edges, a.Edges().Count);
            Check.AlmostEqual(c.At25, a.PositionAt(0.25), 7);
            Check.AlmostEqual(c.Camber, a.CamberLine.Length, 7);
            Assert.Equal((c.M, c.P, c.T), (a.MaxCamber, a.CamberPos, a.Thickness));
        }
    }

    [Fact]
    public void BlendCurve()
    {
        var c0 = new CenterArc(new Vector(0, 0), 2, 0, 90);
        var c1 = new Spline(new Vector(-3, 3), new Vector(-4, 5), new Vector(-2, 7));
        Same(new BlendCurve(c0, c1, ContinuityLevel.C0), 3.16227766017, new Vector(0, 2, 0), new Vector(-3, 3, 0), new Vector(-1.11, 2.37, 0));
        Same(new BlendCurve(c0, c1, ContinuityLevel.C1), 3.19009718784, new Vector(0, 2, 0), new Vector(-3, 3, 0), new Vector(-1.15482669186, 2.22958150515, 0));
        Same(new BlendCurve(c0, c1, ContinuityLevel.C2), 3.21583263285, new Vector(0, 2, 0), new Vector(-3, 3, 0), new Vector(-1.17314166977, 2.16238740454, 0));
        Same(new BlendCurve(c0, c1, endPoints: (c0.PositionAt(0), c1.PositionAt(1)), tangentScalars: (2, 0.5)),
             8.16983700827, new Vector(2, 0, 0), new Vector(-2, 7, 0), new Vector(0.775471220097, 2.72210821189, 0));
    }

    [Fact]
    public void FilletPolyline()
    {
        void Same(Wire p, double length, int edges, Vector start, Vector mid, Vector end)
        {
            Check.AlmostEqual(length, p.Length, 7);
            Assert.Equal(edges, p.Edges().Count);
            Check.AlmostEqual(start, p.PositionAt(0), 7);
            Check.AlmostEqual(mid, p.PositionAt(0.5), 7);
            Check.AlmostEqual(end, p.PositionAt(1), 7);
        }
        Same(new FilletPolyline([new Vector(0, 0, 0), new Vector(0, 10, 2), new Vector(0, 10, 10), new Vector(5, 20, 10)], radius: 2),
             27.9875574913, 5, new Vector(0, 0, 0), new Vector(0, 10, 6.32815379544), new Vector(5, 20, 10));
        Same(new FilletPolyline([new Vector(0, 0), new Vector(10, 0), new Vector(10, 10), new Vector(0, 10)], radii: [1, 2, 3, 0], close: true),
             37.4247779608, 7, new Vector(0, 10, 0), new Vector(9.08060461174, 0.317058030384, 0), new Vector(0, 10, 0));
        Same(new FilletPolyline([new Vector(0, 0), new Vector(10, 0), new Vector(10, 10), new Vector(20, 10)], radii: [0, 1]),
             29.5707963268, 4, new Vector(0, 0, 0), new Vector(10, 4.7853981634, 0), new Vector(20, 10, 0));
        Same(new FilletPolyline([new Vector(0, 0), new Vector(2, 6), new Vector(0, 5), new Vector(-2, 6), new Vector(0, 0)], radii: [0, 0.1, 0, 0]),
             16.8740233322, 5, new Vector(-2, 6, 0), new Vector(0.668017351032, 2.00405205309, 0), new Vector(-2, 6, 0));
    }

    [Fact]
    public void Arcs()
    {
        Same(new EllipticalStartArc(new Vector(1, 1), new Vector(0, 1), 3, 1, 90, majorAxisDir: new Vector(1, 1)),
             2.75177211679, new Vector(1, 1, 0), new Vector(-1.2360679775, 0.5527864045, 0), new Vector(0.260602611497, 1.40527282616, 0));
        Same(new EllipticalStartArc(new Vector(1, 1), new Vector(0, 1), 3, 1, -45, startAngle: 45),
             1.19127006326, new Vector(1, 1, 0), new Vector(0.607042511551, -0.0571955121517, 0), new Vector(0.968581147629, 0.560809449021, 0));
        Same(new JernArc(new Vector(1, 2), new Vector(1, 1), 3, -120), 6.28318530718, new Vector(1, 2, 0), new Vector(6.01909782243, 0.655136791748, 0),
             new Vector(3.08990499121, 2.87851516454, 0));
        Same(new JernArc(new Vector(1, 0), new Vector(0, 1), 1, new Line(new Vector(1, 0), new Vector(0, 1))), 1.57079632679, new Vector(1, 0, 0), new Vector(0, 1, 0),
             new Vector(0.835807361368, 0.549022817998, 0));
        Same(new JernArc(new Vector(0, 0, 0), new Vector(1, 0, 1), 1, 90), 1.57079632679, new Vector(0, 0, 0), new Vector(0, 0, 1.41421356237),
             new Vector(0.272116029435, 0, 0.50431948583));
        Same(new CenterArc(new Vector(1, 0), 2, 30, new Axis(new Vector(0, 0), new Vector(1, 1))), 1.24633302341, new Vector(2.73205080757, 1, 0),
             new Vector(1.82287565553, 1.82287565553, 0), new Vector(2.45767963661, 1.3693684957, 0));
    }

    [Fact]
    public void LinesSplinesHelix()
    {
        Same(new PolarLine(new Vector(1, 1), 3, direction: new Vector(-1, -2), lengthMode: LengthMode.Vertical), 3.35410196625, new Vector(1, 1, 0), new Vector(-0.5, -2, 0),
             new Vector(0.445, -0.11, 0));
        Same(new Spline([new Vector(0, 0), new Vector(1, 1), new Vector(2, 0), new Vector(3, 2)], tangents: [new Vector(1, 0), new Vector(0, 1)]), 6.57927674248, new Vector(0, 0, 0), new Vector(3, 2, 0),
             new Vector(1.63014392221, 0.477365157051, 0));
        Same(new Spline([new Vector(0, 0), new Vector(1, 1), new Vector(2, 0)], periodic: true), 5.7514266401, new Vector(0, 0, 0), new Vector(0, 0, 0), new Vector(1.55860351964, 0.77599713828, 0));
        Same(new Helix(2, 5, 1, coneAngle: 10, lefthand: true), 23.5527575399, new Vector(1, 0, 0), new Vector(-1.82666055869, 0.451509815563, 5),
             new Vector(1.00418732011, -0.960337530539, 2.20882161079));
    }

    [Fact]
    public void ConstrainedArcs()
    {
        var arcs = Edge.MakeConstrainedArcs(new Line(new Vector(-2, 0), new Vector(2, 0)), new Line(new Vector(0, -2), new Vector(0, 2)), radius: 0.5, sagitta: Sagitta.Both);
        Assert.Equal(8, arcs.Count);
        Same(arcs[0], 0.785398163397, new Vector(-0.5, 0, 0), new Vector(0, 0.5, 0), new Vector(-0.225488591001, 0.0820963193159, 0));
        Same(arcs[1], 2.35619449019, new Vector(0, 0.5, 0), new Vector(-0.5, 0, 0), new Vector(-0.58596455014, 0.992554663077, 0));
        Same(arcs[2], 2.35619449019, new Vector(0.5, 0, 0), new Vector(0, 0.5, 0), new Vector(0.992554663077, 0.58596455014, 0));
        Same(arcs[7], 2.35619449019, new Vector(0, -0.5, 0), new Vector(0.5, 0, 0), new Vector(0.58596455014, -0.992554663077, 0));

        arcs = Edge.MakeConstrainedArcs(new PolarLine(new Vector(0, 0), 4, 60), new PolarLine(new Vector(0, 0), 4, 40), new CenterArc(new Vector(0, 0), 4, 0, 90),
                                        sagitta: Sagitta.Both);
        Assert.Equal(2, arcs.Count);
        Same(arcs[0], 1.65268331579, new Vector(1.67819926235, 2.90672638762, 0), new Vector(2.57115043875, 2.15745138485, 0), new Vector(1.6741176481, 2.32208771016, 0));
        Same(arcs[1], 2.06585414474, new Vector(2.57115043875, 2.15745138485, 0), new Vector(1.67819926235, 2.90672638762, 0), new Vector(2.7313913279, 2.85153093031, 0));

        arcs = Edge.MakeConstrainedArcsCenteredOn(new Line(new Vector(-10, 0), new Vector(10, 0)), new Line(new Vector(0, -10), new Vector(0, 10)), new Line(new Vector(-10, 1), new Vector(10, 1)),
                                                  Sagitta.Both);
        Assert.Equal(4, arcs.Count);
        Same(arcs[0], 4.71238898038, new Vector(1, 0, 0), new Vector(0, 1, 0), new Vector(1.98510932615, 1.17192910028, 0));
        Same(arcs[1], 1.57079632679, new Vector(0, 1, 0), new Vector(1, 0, 0), new Vector(0.164192638632, 0.450977182002, 0));
        Same(arcs[2], 1.57079632679, new Vector(-1, 0, 0), new Vector(0, 1, 0), new Vector(-0.450977182002, 0.164192638632, 0));
        Same(arcs[3], 4.71238898038, new Vector(0, 1, 0), new Vector(-1, 0, 0), new Vector(-1.17192910028, 1.98510932615, 0));

        // An ellipse, whose solver parameters are corrected
        arcs = Edge.MakeConstrainedArcs(new EllipticalCenterArc(new Vector(0, 0), 4, 2, 0, arcSize: 360), new Line(new Vector(5, -5), new Vector(5, 5)), radius: 1,
                                        sagitta: Sagitta.Both);
        Assert.Equal(4, arcs.Count);
        Same(arcs[0], 4.04371028003, new Vector(3.38005021802, -1.06949293168, 0), new Vector(5, -1.85413442168, 0), new Vector(3.17132228941, -2.41386047251, 0));
        Same(arcs[1], 2.23947502715, new Vector(5, -1.85413442168, 0), new Vector(3.38005021802, -1.06949293168, 0), new Vector(4.67590395719, -1.11714470624, 0));
        Same(arcs[2], 2.23947502715, new Vector(3.38005021802, 1.06949293168, 0), new Vector(5, 1.85413442168, 0), new Vector(4.15924619752, 0.866895520208, 0));
        Same(arcs[3], 4.04371028003, new Vector(5, 1.85413442168, 0), new Vector(3.38005021802, 1.06949293168, 0), new Vector(4.0745542835, 2.85135137842, 0));
    }

    [Fact]
    public void ConstrainedLines()
    {
        var lines = Edge.MakeConstrainedLines(new CenterArc(new Vector(-5, 0), 4, 0, 360), new CenterArc(new Vector(5, 0), 3, 0, 360));
        Assert.Equal(4, lines.Count);
        Same(lines[0], 9.94987437107, new Vector(-4.6, -3.97994974843, 0), new Vector(5.3, -2.98496231132, 0), new Vector(-0.937, -3.6118043967, 0));
        Same(lines[1], 7.14142842854, new Vector(-2.2, 2.85657137142, 0), new Vector(2.9, -2.14242852856, 0), new Vector(-0.313, 1.00694140842, 0));
        Same(lines[2], 9.94987437107, new Vector(-4.6, 3.97994974843, 0), new Vector(5.3, 2.98496231132, 0), new Vector(-0.937, 3.6118043967, 0));
        Same(lines[3], 7.14142842854, new Vector(-2.2, -2.85657137142, 0), new Vector(2.9, 2.14242852856, 0), new Vector(-0.313, -1.00694140842, 0));

        lines = Edge.MakeConstrainedLines(new CenterArc(new Vector(-5, 0), 4, 0, 360), Axis.Y, angle: 30);
        Assert.Equal(2, lines.Count);
        Same(lines[0], 3.07179676972, new Vector(-1.53589838486, 2, 0), new Vector(0, -0.660254037844, 0), new Vector(-0.967615982463, 1.015706006, 0));
        Same(lines[1], 16.9282032303, new Vector(-8.46410161514, -2, 0), new Vector(0, -16.6602540378, 0), new Vector(-5.33238401754, -7.424293994, 0));
    }

    [Fact]
    public void Fillets()
    {
        var r = new Rectangle(6, 6) - new Pos(1, 1) * new Circle(2) - new Pos(3, 3) * new Rectangle(4, 4);
        var filleted = r.SingleFace().Fillet2D(0.2, r.Vertices());
        Check.AlmostEqual(22.3180459479, filleted.Area, 7);
        Assert.Equal(10, filleted.Edges().Count);

        var cross = (new Rectangle(5.3, 5.3 / 6) + new Rectangle(5.3 / 6, 5.3)).SingleFace();
        var recess = cross.Fillet2D(5.3 / 3, new ShapeList<Vertex>(cross.Vertices()).OrderByDistanceTo(new Vector(0, 0, 0)).Take(4));
        Check.AlmostEqual(11.2622402623, recess.Area, 7);
        Assert.Equal(16, recess.Edges().Count);

        var sketch = new Sketch() + new Rectangle(10, 20) + new Ellipse(20, 5);
        var mixed = sketch.Faces()[0].Fillet2D(1.0, [sketch.Vertices().GroupBy(Axis.X)[0].OrderBy(Axis.Y)[0]]);
        Check.AlmostEqual(414.996320666, mixed.Area, 6);
        Assert.Equal(10, mixed.Edges().Count);
        mixed = sketch.Faces()[0].Fillet2D(1.0, [sketch.Vertices().GroupBy(Axis.X)[1].OrderBy(Axis.Y)[1]]);
        Check.AlmostEqual(415.391549244, mixed.Area, 6);
        Assert.Equal(10, mixed.Edges().Count);

        var rounded = new RectangleRounded(20, 10, 1);
        Check.AlmostEqual(199.141592654, rounded.Area, 7);
        Assert.Equal(8, rounded.Edges().Count);

        var polygon = Wire.MakePolygon([new Vector(0, 0), new Vector(3, 0), new Vector(4, 2), new Vector(1, 5), new Vector(-1, 2)], close: true);
        var filletedPolygon = polygon.Fillet2D(0.7, polygon.Vertices());
        Check.AlmostEqual(14.4588225426, filletedPolygon.Length, 7);
        Assert.Equal(10, filletedPolygon.Edges().Count);
        Check.AlmostEqual(new Vector(0.432623792125, 0, 0), filletedPolygon.PositionAt(0), 7);
        Check.AlmostEqual(new Vector(3.82112355044, 1.66874161429, 0), filletedPolygon.PositionAt(0.3), 7);
    }
}
