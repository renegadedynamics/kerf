// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Classification;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A frozen shape's classifier is kept with its <c>TShape</c> for the next point asked of it (unit 15C, <c>KeptClassifiers</c>):
/// each point gets what a new classifier gives it, on one thread or several, and nothing is kept inside a working overlay, at
/// or at another placement.
/// </summary>
public class KeptClassifiersTests
{
    private static Shape Moved(Shape shape, double x, double y, double z) =>
        shape.Moved(new Placement(new Transform3D().WithTranslation(new Vector3D(x, y, z))));

    // A plate with holes and a boss: planes, cylinders and edges a ray can pass through
    private static Shape Part()
    {
        Shape plate = Solid.Box(40, 30, 6).Value.Shape;
        for (int i = 0; i < 4; i++)
        {
            plate = plate.Cut(Moved(Solid.Cylinder(2, 10).Value.Shape, 6 + 9 * i, 8, -2)).Value.Shape;
        }
        return plate.Fuse(Moved(Solid.Cylinder(5, 6).Value.Shape, 20, 20, 6)).Value.Shape;
    }

    // A grid through the part's box, with points on the faces' planes and the holes' axes, where the ray meets edges
    private static List<Point3D> Points()
    {
        var points = new List<Point3D>();
        for (int i = 0; i <= 16; i++)
        {
            for (int j = 0; j <= 12; j++)
            {
                for (int k = 0; k <= 6; k++)
                {
                    points.Add(new Point3D(-1 + 42 * i / 16.0, -1 + 32 * j / 12.0, -1 + 14 * k / 6.0));
                }
            }
        }
        return points;
    }

    private static Containment Fresh(in Shape shape, in Point3D point)
    {
        var classifier = new SolidClassifier(shape);
        classifier.Perform(point, 1e-7);
        return classifier.State();
    }

    [Fact]
    public void Points_asked_one_at_a_time_share_a_classifier_and_get_a_new_ones_states()
    {
        Shape part = Part();
        KeptClassifiers.s_made = 0;
        KeptClassifiers.s_lent = 0;
        int inside = 0, on = 0;
        foreach (Point3D point in Points())
        {
            Containment state = part.ClassifyPoint(point, 1e-7);
            Assert.Equal(Fresh(part, point), state);
            Assert.Equal(state == Containment.On, part.IsOnFace(point, 1e-7));
            inside += state == Containment.In ? 1 : 0;
            on += state == Containment.On ? 1 : 0;
        }
        Assert.True(inside > 100);
        Assert.True(on > 10);
        Assert.Equal(1, KeptClassifiers.s_made);
        Assert.True(KeptClassifiers.s_lent > 2000, $"{KeptClassifiers.s_lent} lent");
    }

    [Fact]
    public void Two_threads_asking_one_shape_get_a_new_ones_states()
    {
        Shape part = Part();
        List<Point3D> points = Points();
        Containment[] expected = [.. points.Select(p => Fresh(part, p))];
        var states = new Containment[points.Count];
        Parallel.For(0, points.Count, new ParallelOptions { MaxDegreeOfParallelism = 2 }, i => states[i] = part.ClassifyPoint(points[i], 1e-7));
        Assert.Equal(expected, states);
    }

    [Fact]
    public void Nothing_is_kept_in_an_overlay_and_another_placement_has_its_own()
    {
        Shape part = Part();
        var point = new Point3D(3, 3, 3);
        Assert.Equal(Containment.In, part.ClassifyPoint(point));
        KeptClassifiers.s_made = 0;
        KeptClassifiers.s_lent = 0;
        using (WorkingOverlay.Begin(part))
        {
            Assert.Equal(Containment.In, part.ClassifyPoint(point));
        }
        Assert.Equal((1, 0), (KeptClassifiers.s_made, KeptClassifiers.s_lent));

        // Moved off the point: a classifier of its own, kept for the next point there
        Shape moved = Moved(part, 100, 0, 0);
        Assert.Equal(Containment.Out, moved.ClassifyPoint(point));
        Assert.Equal(Containment.In, moved.ClassifyPoint(new Point3D(103, 3, 3)));
        Assert.Equal((2, 1), (KeptClassifiers.s_made, KeptClassifiers.s_lent));
    }
}
