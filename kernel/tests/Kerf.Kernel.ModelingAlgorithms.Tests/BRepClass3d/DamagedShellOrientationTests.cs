// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// A damaged solid's orientation is the same without the explorer's tree of edges, and healing it gives a valid shape
/// or a failure, never an invalid shape as a success (decisions 1742 and 1973).
/// </summary>
public class DamagedShellOrientationTests
{
    [Theory]
    [InlineData("no curve", true)]
    [InlineData("vertex at NaN", false)]
    [InlineData("range of NaN", false)]
    [InlineData("vertex far away", true)]
    public void A_damaged_solid_is_oriented_as_before_and_healed_or_refused(string damage, bool heals)
    {
        Shape box = TestShapes.Box(10, 10, 10);
        var builder = new BrepBuilder();
        Edge edge = ShapeCasts.Edge(new Explorer(box, ShapeType.Edge).Current());
        switch (damage)
        {
            case "no curve": builder.UpdateEdge(edge, (Curve3D?)null, 1e-7); break;
            case "vertex at NaN": builder.UpdateVertex(Exploration.FirstVertex(edge), new Point3D(double.NaN, 0, 0), 1e-7); break;
            case "range of NaN": builder.Range(edge, double.NaN, double.NaN); break;
            case "vertex far away": builder.UpdateVertex(Exploration.FirstVertex(edge), new Point3D(1e300, 0, 0), 1e-7); break;
        }

        Shape damaged = TestShapes.Frozen(box);
        var classifier = new SolidClassifier(damaged);
        classifier.PerformInfinitePoint(1e-7);
        Assert.Equal(classifier.State(), SolidClassifier.InfinitePointState(damaged, 1e-7));
        Assert.Equal(Containment.Out, classifier.State());

        var healed = damaged.Fix();
        Assert.Equal(heals, healed.IsSuccess);
        if (healed.IsSuccess)
        {
            Assert.True(healed.Value.Shape.Validate().IsValid);
        }
        else
        {
            Assert.Equal(DiagnosticCode.InvalidResult, healed.Error!.Code);
        }
    }
}
