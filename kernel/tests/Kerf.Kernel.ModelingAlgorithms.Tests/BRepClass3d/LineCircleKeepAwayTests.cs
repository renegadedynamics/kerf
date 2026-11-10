// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Classification;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;
using Xunit.Abstractions;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A solid's classifier doesn't look for the extrema of its point, or line, with a line or circle edge that its geometry keeps
/// further than the edge's tolerance (unit 15C, <c>SplineHull.KeepsAway</c>): a line edge measured exactly as its segment, a
/// circle as the whole circle. The states are the geometry's, and an edge said to keep away is further than its tolerance.
/// </summary>
public class LineCircleKeepAwayTests(ITestOutputHelper output)
{
    private static Shape Moved(Shape shape, double x, double y, double z) =>
        shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(x, y, z))));

    // A plate 40 by 30 by 6 with four holes of radius 2 through it, and a boss of radius 5 and 6 high on it
    private static Shape Part()
    {
        Shape plate = Solid.Box(40, 30, 6).Value.Shape;
        for (int i = 0; i < 4; i++)
        {
            plate = plate.Cut(Moved(Solid.Cylinder(2, 10).Value.Shape, 6 + 9 * i, 8, -2)).Value.Shape;
        }
        return plate.Fuse(Moved(Solid.Cylinder(5, 6).Value.Shape, 20, 20, 6)).Value.Shape;
    }

    // Inside by the geometry, or null within a hundredth of a surface
    private static bool? Inside(in Point3D p)
    {
        double plate = Math.Min(Math.Min(p.X, 40 - p.X), Math.Min(Math.Min(p.Y, 30 - p.Y), Math.Min(p.Z, 6 - p.Z)));
        for (int i = 0; i < 4; i++)
        {
            double dx = p.X - (6 + (9 * i)), dy = p.Y - 8;
            plate = Math.Min(plate, Math.Sqrt((dx * dx) + (dy * dy)) - 2);
        }
        double bx = p.X - 20, by = p.Y - 20;
        double boss = Math.Min(5 - Math.Sqrt((bx * bx) + (by * by)), Math.Min(p.Z - 6, 12 - p.Z));
        return Math.Abs(plate) < 0.01 || Math.Abs(boss) < 0.01 ? null : Math.Max(plate, boss) > 0;
    }

    [Fact]
    public void A_classifier_skips_the_line_and_circle_edges_its_point_or_line_keeps_away_from()
    {
        Shape part = Part();
        var classifier = new SolidClassifier(part);
        SplineHull.s_analyticKeptAway = 0;
        int points = 0, inside = 0;
        for (int i = 0; i <= 20; i++)
        {
            for (int j = 0; j <= 15; j++)
            {
                for (int k = 0; k <= 7; k++)
                {
                    var point = new Point3D(-1 + (42 * i / 20.0), -1 + (32 * j / 15.0), -1 + (14 * k / 7.0));
                    if (Inside(point) is not { } expected)
                    {
                        continue;
                    }
                    classifier.Perform(point, 1e-7);
                    Assert.Equal(expected ? Containment.In : Containment.Out, classifier.State());
                    points++;
                    inside += expected ? 1 : 0;
                }
            }
        }
        output.WriteLine($"{points} points, {inside} inside; line and circle edges kept away {SplineHull.s_analyticKeptAway}");
        Assert.True(inside > 300);
        Assert.True(SplineHull.s_analyticKeptAway > 200, $"{SplineHull.s_analyticKeptAway} kept away");
    }

    [Fact]
    public void An_edge_kept_away_from_a_point_is_further_than_its_tolerance()
    {
        Shape part = Part();
        var random = new Random(15);
        int kept = 0, near = 0;
        foreach (Edge edge in part.Explore<Edge>())
        {
            double tolerance = Engine.Topology.Brep.Tool.Tolerance(edge);
            BoundingBox box = edge.BoundingBox();
            for (int n = 0; n < 40; n++)
            {
                // Points about the edge's box, some within a few tolerances of the edge
                Point3D centre = box.Min + ((box.Max - box.Min) * random.NextDouble());
                double spread = n % 2 == 0 ? 3 * tolerance : 1;
                var point = new Point3D(centre.X + (spread * (random.NextDouble() - 0.5)), centre.Y + (spread * (random.NextDouble() - 0.5)),
                                        centre.Z + (spread * (random.NextDouble() - 0.5)));
                double distance = edge.Distance(Vertex.At(point)).Value.Value;
                if (SplineHull.KeepsAway(edge, tolerance, point, null))
                {
                    Assert.True(distance > tolerance, $"{distance} from an edge of tolerance {tolerance}");
                    kept++;
                }
                else
                {
                    near++;
                }
            }
        }
        Assert.True(kept > 100);
        Assert.True(near > 10);
    }
}
