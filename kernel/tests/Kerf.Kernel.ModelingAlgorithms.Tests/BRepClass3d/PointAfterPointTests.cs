// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A solid classifier asked one point after another gives each what a new classifier gives it (unit 15C2): OCCT's carries
/// each face intersector's flag of a line found parallel over to the next point, and a stale flag called a point ON.
/// </summary>
public class PointAfterPointTests
{
    private static Shape? Dataset(string file)
    {
        string path = Path.Combine(typeof(PointAfterPointTests).Assembly.GetCustomAttributes(typeof(System.Reflection.AssemblyMetadataAttribute), false)
                                       .Cast<System.Reflection.AssemblyMetadataAttribute>().Single(a => a.Key == "OcctTestDataDir").Value!, "brep", file);
        if (!File.Exists(path))
        {
            return null;
        }
        return BrepTools.Read(out Shape shape, path) ? shape : null;
    }

    [Fact]
    public void A_point_after_another_gets_a_new_classifiers_state()
    {
        // One of the dataset's 40 points of decision 2111: within 1e-7 of a face of OCC330.brep's solid, which a new classifier
        // calls IN; after the point before it in the grid, whose line ran parallel to a face, OCCT's classifier called it ON
        if (Dataset("OCC330.brep") is not { } shape)
        {
            return;
        }
        Shape solid = shape.Explore<Solid>().First();
        var before = new Point3D(527886.85, 341093.0166666667, 128.13333333333333);
        var point = new Point3D(527902.0999998761, 341111.8499998463, 133.699999968279);

        var fresh = new SolidClassifier(solid);
        fresh.Perform(point, 1e-7);
        Assert.Equal(Containment.In, fresh.State());

        var shared = new SolidClassifier(solid);
        shared.Perform(before, 1e-7);
        Assert.Equal(Containment.Out, shared.State());
        shared.Perform(point, 1e-7);
        Assert.Equal(Containment.In, shared.State());
    }
}
