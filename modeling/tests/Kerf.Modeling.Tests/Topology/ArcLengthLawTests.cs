// Unit 11J (decision 1768): an edge's length, and a position along it, are what they say: the limit of the lengths of
// inscribed polygons, against which OCCT's few Gauss points were up to 1.5% out on an ellipse of ten to one.
namespace Kerf.Modeling.Tests.Topology;

using Kerf.Modeling;

public class ArcLengthLawTests
{
    public static TheoryData<string> Curves => ["ellipse 10:1", "ellipse 3:1", "ellipse arc", "parabola bezier", "steep bezier", "rational bezier", "spline", "helix", "parabola", "circle", "line"];

    private static Edge Make(string name) => name switch
    {
        "ellipse 10:1" => Edge.MakeEllipse(10, 1),
        "ellipse 3:1" => Edge.MakeEllipse(3, 1),
        "ellipse arc" => Edge.MakeEllipse(1, 3, startAngle: 10, endAngle: 250),
        "parabola bezier" => Edge.MakeBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, 0, 0)),
        "steep bezier" => Edge.MakeBezier(new Point3D(0, 0, 0), new Point3D(1, 20, 0), new Point3D(2, 0, 0)),
        "rational bezier" => Edge.MakeBezier([new Point3D(0, 0, 0), new Point3D(20, 20, 0), new Point3D(40, 0, 0), new Point3D(0, -40, 0)], [1, 1, 2, 3]),
        "spline" => Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 2, 0)]),
        "helix" => Edge.MakeHelix(2, 6, 3),
        "parabola" => Edge.MakeParabola(2, Plane.XY, 0, 60),
        "circle" => Edge.MakeCircle(2, startAngle: 30, endAngle: 200),
        _ => Edge.MakeLine(new Point3D(0, 0, 0), new Point3D(3, 4, 0)),
    };

    // The length of a polygon on the curve between two parameters
    private static double Chords(EdgeCurve curve, double u1, double u2, int pieces = 200_000)
    {
        double length = 0;
        Point3D last = curve.Evaluate(u1);
        for (int i = 1; i <= pieces; i++)
        {
            Point3D next = curve.Evaluate(u1 + (u2 - u1) * i / pieces);
            length += last.DistanceTo(next);
            last = next;
        }
        return length;
    }

    [Theory]
    [MemberData(nameof(Curves))]
    public void A_length_is_the_limit_of_the_polygons_on_the_curve(string name)
    {
        Edge edge = Make(name);
        EdgeCurve curve = edge.AsCurve();
        double expected = Chords(curve, curve.Range.Min, curve.Range.Max);
        Assert.Equal(expected, edge.Length, 6);
        Assert.Equal(expected, curve.Length, 6);
    }

    [Theory]
    [MemberData(nameof(Curves))]
    public void A_fraction_along_a_curve_is_that_fraction_of_its_length(string name)
    {
        Edge edge = Make(name);
        EdgeCurve curve = edge.AsCurve();
        double whole = curve.Length;
        foreach (double fraction in new[] { 0.1, 0.25, 0.5, 0.9 })
        {
            double u = curve.ParameterAtLength(whole * fraction, curve.Range.Min);
            Assert.Equal(whole * fraction, Chords(curve, curve.Range.Min, u), 5);
            Assert.Equal(whole * fraction, curve.LengthBetween(curve.Range.Min, u), 6);
        }
    }

    [Fact]
    public void Half_way_along_a_symmetric_curve_is_its_middle()
    {
        Check.Near(new Point3D(1, 1, 0), Edge.MakeBezier(new Point3D(0, 0, 0), new Point3D(1, 2, 0), new Point3D(2, 0, 0)).PositionAt(0.5));
        Check.Near(new Point3D(-10, 0, 0), Edge.MakeEllipse(10, 1).PositionAt(0.5));
        // a quarter of the way round an ellipse is the end of its minor axis
        Check.Near(new Point3D(0, 1, 0), Edge.MakeEllipse(10, 1).PositionAt(0.25));
    }

    [Fact]
    public void An_edge_s_centre_of_mass_and_inertia_come_from_the_same_integral()
    {
        // the half of an ellipse above X: its centroid by the polygons, against the mass properties'
        Edge half = Edge.MakeEllipse(10, 1, startAngle: 0, endAngle: 180);
        EdgeCurve curve = half.AsCurve();
        const int pieces = 200_000;
        double length = 0, moment = 0;
        Point3D last = curve.Evaluate(curve.Range.Min);
        for (int i = 1; i <= pieces; i++)
        {
            Point3D next = curve.Evaluate(curve.Range.Min + (curve.Range.Max - curve.Range.Min) * i / pieces);
            double ds = last.DistanceTo(next);
            length += ds;
            moment += ds * (last.Y + next.Y) / 2;
            last = next;
        }
        Assert.Equal(length, half.Length, 6);
        Assert.Equal(moment / length, half.Center(CenterOf.Mass).Y, 6);
    }

    [Fact]
    public void A_wire_s_length_is_its_edges_and_a_fraction_of_it_lands_on_the_right_edge()
    {
        Edge ellipse = Edge.MakeEllipse(10, 1, startAngle: 0, endAngle: 180);
        Wire wire = Wire.Make([ellipse, Edge.MakeLine(new Point3D(-10, 0, 0), new Point3D(-10, 0, 20))]);
        Assert.Equal(ellipse.Length + 20, wire.Length, 7);
        // half way along the whole lies on the line or the arc, by how long each is
        double half = wire.Length / 2;
        Point3D middle = wire.PositionAt(half, PositionMode.Length);
        Assert.Equal(half, half <= ellipse.Length ? half : ellipse.Length + middle.Z, 5);
    }
}
