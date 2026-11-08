// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// <see cref="SolidClassifier.InfinitePointState"/> answers as a classifier made, asked and dropped does, and an
/// explorer that leaves its tree of edges and vertices until asked fills the very map and tree an eager one has.
/// </summary>
public class InfinitePointStateTests
{
    private static IEnumerable<Shape> Shapes()
    {
        Shape box = TestShapes.Frozen(TestShapes.Box(10, 10, 10));
        yield return box;
        yield return box.Reversed();
        yield return TestShapes.Frozen(BRepGPropTestShapes.CylinderSolid(2.0, 3.0));
        yield return TestShapes.Frozen(BRepGPropTestShapes.SphereSolid(10.0));
        yield return TestShapes.Frozen(BRepGPropTestShapes.SphereSolid(10.0)).Reversed();
        var builder = new BrepBuilder();
        Solid empty = new();
        builder.MakeSolid(ref empty);
        yield return TestShapes.Frozen(empty);
    }

    [Fact]
    public void The_state_is_a_classifiers()
    {
        int holes = 0, solids = 0;
        foreach (Shape shape in Shapes())
        {
            var classifier = new SolidClassifier(shape);
            classifier.PerformInfinitePoint(1e-7);
            Containment state = SolidClassifier.InfinitePointState(shape, 1e-7);
            Assert.Equal(classifier.State(), state);
            holes += state == In ? 1 : 0;
            solids += state == Out ? 1 : 0;
        }

        // Both answers come up: a finite solid, and a hole in space
        Assert.True(holes >= 2 && solids >= 2, $"{holes} holes, {solids} solids");
    }

    [Fact]
    public void A_tree_left_until_asked_is_the_eager_one()
    {
        foreach (Shape shape in Shapes())
        {
            var eager = new SolidExplorer(shape);
            var deferred = new SolidExplorer(shape, fillTree: false);
            Assert.Equal(eager.GetMapEV().Count, deferred.GetMapEV().Count);
            for (int i = 0; i < eager.GetMapEV().Count; i++)
            {
                Assert.True(eager.GetMapEV()[i].IsEqual(deferred.GetMapEV()[i]));
            }

            // A point on an edge or a vertex is found near it through either tree
            foreach (Point3D point in new[] { new Point3D(10, 10, 5), new Point3D(0, 0, 0), new Point3D(5, 5, 5), new Point3D(0, 10, 0) })
            {
                var near = new SolidClassifier(shape, point, 1e-7);
                var classifier = new SolidClassifierOn(deferred);
                Assert.Equal(near.State(), classifier.Classify(point));
            }
        }
    }

    // A classifier over a given explorer, for the deferred one
    private sealed class SolidClassifierOn(SolidExplorer explorer) : SClassifier
    {
        public Containment Classify(in Point3D point)
        {
            Perform(explorer, point, 1e-7);
            return State();
        }
    }
}
