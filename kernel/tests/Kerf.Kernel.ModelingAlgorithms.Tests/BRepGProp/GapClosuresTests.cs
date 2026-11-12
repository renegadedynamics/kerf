// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A face whose boundary is open in its parameters inside a vertex's tolerance is closed there, straight, where the public
/// measurements read it (<see cref="GapClosures"/>, decision 31801); OCCT's own reading, which the corpus and the oracle
/// compare, is left as it is.
/// </summary>
public class GapClosuresTests
{
    // A cylinder of radius 500 less an equal one across it, whose axis lies in the first's end plane 3e-7 short of crossing its
    // axis: the end circle crosses the tool 0.0173 either side of its seam vertex, which grows to cover both crossings
    private const double Radius = 500, Offset = -3e-7;

    private static Shape Cut()
    {
        Shape cylinder = Solid.Cylinder(Radius, 2 * Radius).Value.Shape.Transformed(Transform3D.Translation(new Vector3D(0, 0, -Radius))).Value.Shape;
        // Turned so that the tool's face inside the cylinder ends at its seam, half a turn from where its integrals start
        var turn = Transform3D.Rotation(new Axis3D(new Point3D(0, 0, 0), Direction3D.UnitX), Math.PI / 2);
        Shape tool = Solid.Cylinder(Radius, 4 * Radius).Value.Shape.Transformed(turn).Value.Shape
            .Transformed(Transform3D.Translation(new Vector3D(Offset, 2 * Radius, Radius))).Value.Shape;
        return cylinder.Cut(tool, new BooleanOptions { CheckResult = false }).Value.Shape;
    }

    // The cylinder less half of two equal cylinders' common part, 16/3 r^3, which the offset changes by less than 1e-12 of it
    private static readonly double Expected = Math.PI * Radius * Radius * 2 * Radius - 8.0 / 3 * Radius * Radius * Radius;

    [Fact]
    public void A_face_with_no_grown_vertex_has_no_closing_arcs()
    {
        foreach (Shape shape in new[] { TestShapes.Box(3, 5, 7), Solid.Cylinder(2, 5).Value.Shape, Solid.Sphere(3).Value.Shape })
        {
            for (var faces = new Explorer(shape, ShapeType.Face); faces.More(); faces.Next())
            {
                Assert.Null(GapClosures.Of(ShapeCasts.Face(faces.Current())));
            }
        }
    }

    [Fact]
    public void The_gaps_at_a_grown_vertex_are_closed_by_arcs_no_longer_than_it_is_wide()
    {
        Shape cut = Cut();
        int arcs = 0;
        double widest = 0;
        for (var vertices = new Explorer(cut, ShapeType.Vertex); vertices.More(); vertices.Next())
        {
            widest = Math.Max(widest, Tool.Tolerance(ShapeCasts.Vertex(vertices.Current())));
        }

        // sqrt(2 R d), where the end circle crosses the tool
        Assert.Equal(Math.Sqrt(2 * Radius * -Offset), widest, 1e-4);
        for (var faces = new Explorer(cut, ShapeType.Face); faces.More(); faces.Next())
        {
            Face face = ShapeCasts.Face(faces.Current());
            Surface surface = Tool.Surface(face, out Placement placement)!;
            foreach (Edge arc in GapClosures.Of(face) ?? [])
            {
                arcs++;
                Curve2D line = Tool.CurveOnSurface(arc, surface, placement, out double first, out double last)!;
                Assert.IsType<LineCurve2D>(line);
                Point2D from = line.Value(first), to = line.Value(last);
                double length = surface.Value(from.X, from.Y).DistanceTo(surface.Value(to.X, to.Y));
                Assert.InRange(length, 1e-7, 2 * widest * 1.001);
            }
        }

        // Two on the cylinder's face, either side of its seam, and one on the tool's, along its seam
        Assert.Equal(3, arcs);
    }

    [Fact]
    public void The_public_volume_closes_the_gaps_and_OCCTs_reading_doesnt()
    {
        Shape cut = Cut();
        var asOcct = new Properties();
        MassPropertiesBuilder.VolumeProperties(cut, asOcct, 1e-9, massOnly: true);
        // What the gap on the tool's face leaves out: its length, 0.0346, by the face's half turn and the radius, 54 of area,
        // and 1.05e-5 of the volume
        Assert.InRange((asOcct.Mass() - Expected) / Expected, 1.0e-5, 1.1e-5);

        var closed = new Properties();
        MassPropertiesBuilder.VolumeProperties(cut, closed, 1e-9, massOnly: true, closeGaps: true);
        Assert.Equal(Expected, closed.Mass(), 1e-9 * Expected);
        Assert.Equal(Expected, cut.IntegrateVolume(new MassOptions { Tolerance = 1e-9 }), 1e-9 * Expected);
        // The fixed Gauss points and the whole properties take the arcs too; on these curves they are 6e-4 out with or without
        Assert.Equal(Expected, cut.IntegrateVolume(new MassOptions { Tolerance = 0 }), 1e-3 * Expected);
        Assert.Equal(Expected, cut.MassProperties(Dimension.Volume).Mass, 1e-3 * Expected);
    }

    // The Booleans' test of faces that close (unit 15AH) reads them closed through their vertices too, so a gap a vertex covers
    // isn't one, and a shape that's open any other way still is: its volume depends on where it's taken from
    [Fact]
    public void A_volume_about_two_points_is_the_same_through_a_vertex_and_not_through_a_missing_face()
    {
        Shape cut = Cut();
        var low = new Point3D(-Radius, -Radius, -Radius);
        var high = new Point3D(Radius, Radius, Radius);
        Assert.Equal(MassPropertiesBuilder.VolumeAbout(cut, low, 1e-9), MassPropertiesBuilder.VolumeAbout(cut, high, 1e-9), 1e-8 * Expected);

        var open = new List<Shape>();
        for (var faces = new Explorer(TestShapes.Box(3, 5, 7), ShapeType.Face); faces.More(); faces.Next())
        {
            open.Add(faces.Current());
        }

        open.RemoveAt(0);
        Shape five = Kerf.Topology.Compound.Create([.. open]);
        double fromLow = MassPropertiesBuilder.VolumeAbout(five, new Point3D(0, 0, 0), 1e-9), fromHigh = MassPropertiesBuilder.VolumeAbout(five, new Point3D(3, 5, 7), 1e-9);
        Assert.True(Math.Abs(fromLow - fromHigh) > 1, $"{fromLow} and {fromHigh}");
    }

    [Fact]
    public void The_public_area_closes_the_gaps()
    {
        Shape cut = Cut();
        // The cylinder's side less the tool's hole in it, 4 r^2; the tool's side inside the cylinder, 4 r^2; and the far end
        double expected = 2 * Math.PI * Radius * 2 * Radius + Math.PI * Radius * Radius;
        Assert.Equal(expected, cut.IntegrateArea(new MassOptions { Tolerance = 1e-9 }), 1e-9 * expected);
        var asOcct = new Properties();
        MassPropertiesBuilder.SurfaceProperties(cut, asOcct, 1e-9, massOnly: true);
        Assert.InRange(expected - asOcct.Mass(), 50, 60);
    }
}
