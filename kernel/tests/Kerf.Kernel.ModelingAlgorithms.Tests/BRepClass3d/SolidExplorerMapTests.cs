// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Engine.Classification;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A solid explorer keeps the faces of each edge of a frozen shape for the points it classifies (unit 12Y): the states must
/// be those of a classifier that maps them afresh for each point, and nothing is kept inside a working overlay.
/// </summary>
public class SolidExplorerMapTests
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

    [Fact]
    public void A_kept_map_classifies_as_a_fresh_one()
    {
        Shape part = Part();
        var kept = new SolidClassifier(part);
        int inside = 0, on = 0;
        // A grid through the part's box, with points on the faces' planes and the holes' axes, where the ray meets edges
        for (int i = 0; i <= 16; i++)
        {
            for (int j = 0; j <= 12; j++)
            {
                for (int k = 0; k <= 6; k++)
                {
                    var point = new Point3D(-1 + 42 * i / 16.0, -1 + 32 * j / 12.0, -1 + 14 * k / 6.0);
                    kept.Perform(point, 1e-7);
                    var fresh = new SolidClassifier(part);
                    fresh.Perform(point, 1e-7);
                    Assert.Equal(fresh.State(), kept.State());
                    inside += kept.State() == Containment.In ? 1 : 0;
                    on += kept.State() == Containment.On ? 1 : 0;
                }
            }
        }
        Assert.True(inside > 100);
        Assert.True(on > 10);
    }

    [Fact]
    public void A_frozen_shape_keeps_its_map_and_an_overlay_makes_its_own()
    {
        Shape part = Part();
        var explorer = new SolidExplorer(part);
        var first = explorer.GetMapEF();
        Assert.Same(first, explorer.GetMapEF());
        using (WorkingOverlay.Begin(part))
        {
            Assert.NotSame(first, explorer.GetMapEF());
        }
        Assert.Same(first, explorer.GetMapEF());
        // Loading the shape again forgets it
        explorer.InitShape(part);
        Assert.NotSame(first, explorer.GetMapEF());
    }
}
