namespace Kerf.Modeling.Tests.Topology;

using Kerf.Measurement;
using Kerf.Modeling;

/// <summary>A centre of mass is on the precise path, as the volume beside it is: known values, from the curve alone.</summary>
public class CentreOfMassTests
{
    // A wavy spline closed below it, as the fixed integration gets wrong
    private static Wire Outline(out Edge spline)
    {
        spline = Edge.MakeSpline([new Point3D(0, 0, 0), new Point3D(10, 40, 0), new Point3D(20, -30, 0), new Point3D(30, 50, 0), new Point3D(40, -20, 0), new Point3D(50, 10, 0)]);
        return Wire.Make([spline, Edge.MakeLine(new Point3D(50, 10, 0), new Point3D(50, -60, 0)),
                          Edge.MakeLine(new Point3D(50, -60, 0), new Point3D(0, -60, 0)), Edge.MakeLine(new Point3D(0, -60, 0), new Point3D(0, 0, 0))]);
    }

    // The centroid of the region inside the outline, from a fine polygon of it
    private static (double X, double Y, double Area) PolygonCentroid(Edge spline)
    {
        const int samples = 200000;
        var curve = spline.ToCurve();
        var points = Enumerable.Range(0, samples + 1).Select(i => curve.Evaluate(curve.Range.Min + (curve.Range.Max - curve.Range.Min) * i / samples)).ToList();
        points.AddRange([new Point3D(50, -60, 0), new Point3D(0, -60, 0)]);
        double area = 0, x = 0, y = 0;
        for (int i = 0; i < points.Count; i++)
        {
            var (p, q) = (points[i], points[(i + 1) % points.Count]);
            double cross = p.X * q.Y - q.X * p.Y;
            area += cross;
            x += (p.X + q.X) * cross;
            y += (p.Y + q.Y) * cross;
        }
        return (x / (3 * area), y / (3 * area), Math.Abs(area) / 2);
    }

    [Fact]
    public void A_prism_of_a_spline_has_its_section_s_centroid()
    {
        var outline = Outline(out var spline);
        var known = PolygonCentroid(spline);
        var prism = Solid.Extrude(Face.Make(outline), new Vector3D(0, 0, 10));
        var centre = prism.Center(CenterOf.Mass);
        // Within 2e-3 on a part 100 across, where the default was 8e-3 out: on so wild a boundary, refining stops short of
        // the precision asked for
        Assert.Equal(known.X, centre.X, 2e-3);
        Assert.Equal(known.Y, centre.Y, 2e-3);
        Assert.Equal(5, centre.Z, 1e-6);
        Assert.Equal(known.Area * 10, prism.Volume, known.Area * 10 * 1e-5);
    }

    [Fact]
    public void The_default_integration_was_further_out()
    {
        var outline = Outline(out var spline);
        var known = PolygonCentroid(spline);
        Shape prism = Solid.Extrude(Face.Make(outline), new Vector3D(0, 0, 10));
        var old = prism.MassProperties(Dimension.Volume).CentreOfMass;
        var centre = prism.Center(CenterOf.Mass);
        Assert.True(Math.Abs(centre.X - known.X) + Math.Abs(centre.Y - known.Y) <= Math.Abs(old.X - known.X) + Math.Abs(old.Y - known.Y));
    }

    [Fact]
    public void A_flat_face_has_its_centroid()
    {
        var outline = Outline(out var spline);
        var known = PolygonCentroid(spline);
        var centre = Face.Make(outline).Center(CenterOf.Mass);
        Assert.Equal(known.X, centre.X, 2e-3);
        Assert.Equal(known.Y, centre.Y, 2e-3);
        Assert.Equal(known.Area, Face.Make(outline).Area, known.Area * 1e-5);
    }

    [Fact]
    public void A_swept_wall_has_the_mean_of_its_curve()
    {
        // Extruded square to its curve, a wall's area element is the length along it: the centre is the curve's mean point
        Outline(out var spline);
        const int samples = 4000;
        var points = Enumerable.Range(0, samples).Select(i => spline.PositionAt((i + 0.5) / samples)).ToList();
        var wall = Face.Extrude(spline, new Vector3D(0, 0, 10));
        var centre = wall.Center(CenterOf.Mass);
        Assert.Equal(points.Average(p => p.X), centre.X, 1e-4);
        Assert.Equal(points.Average(p => p.Y), centre.Y, 1e-4);
        Assert.Equal(5, centre.Z, 1e-6);
        // 10.97 by the default, for a true 9.96: a whole unit on a part 100 across
        Assert.True(Math.Abs(((Shape)wall).MassProperties(Dimension.Area).CentreOfMass.Y - centre.Y) > 0.5);
    }

    [Fact]
    public void Symmetric_parts_stay_on_their_planes_of_symmetry()
    {
        // At the volume's own 1e-6, this part's centre is 1.6e-6 off Y = 0
        var centre = Examples.IntroductoryExamplesTests.Build(36).Center(CenterOf.Mass);
        Assert.Equal(0, centre.Y, 1e-8);
        Assert.Equal(0, centre.X, 1e-8);
    }

    [Fact]
    public void Plain_parts_keep_their_exact_centres()
    {
        var box = new Box(80, 60, 10).Center(CenterOf.Mass);
        Assert.Equal(0, box.Z, 1e-9);
        var cylinder = new Pos(1, 2, 3) * new Cylinder(2, 5);
        Assert.Equal(3, cylinder.Center(CenterOf.Mass).Z, 1e-9);
    }
}
