// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.ShapeUpgrade;

/// <summary>
/// Loops that a damaged file or an unlucky number could keep going for ever (decision 1913). Each runs on a task with a
/// time limit, so that one coming back fails its test and doesn't hang the suite.
/// </summary>
public class LoopBoundTests
{
    private static T Soon<T>(Func<T> work)
    {
        Task<T> task = Task.Run(work);
        Assert.True(task.Wait(TimeSpan.FromSeconds(60)), "It didn't return");
        return task.Result;
    }

    private static BSplineCurve3D PeriodicCurve()
    {
        var poles = new NCollection_Array1<Point3D>(1, 4);
        poles[1] = new Point3D(0, 0, 0);
        poles[2] = new Point3D(1, 0, 0);
        poles[3] = new Point3D(1, 1, 0);
        poles[4] = new Point3D(0, 1, 0);
        var knots = new NCollection_Array1<double>(1, 5);
        var mults = new NCollection_Array1<int>(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            knots[i] = i - 1;
            mults[i] = 1;
        }

        return new BSplineCurve3D(poles, knots, mults, 2, true);
    }

    [Fact]
    public void StepLimit_counts_and_stops()
    {
        int steps = 0;
        for (int i = 0; i < 1000; i++)
        {
            StepLimit.Next(ref steps);
        }

        Assert.Equal(1000, steps);
        steps = 10_000_000;
        Assert.Throws<ConstructionErrorException>(() => StepLimit.Next(ref steps));
    }

    [Fact]
    public void StepLimit_refuses_a_step_that_does_not_move()
    {
        Assert.Equal(1.5, StepLimit.Moved(1, 1.5));
        Assert.Throws<ConstructionErrorException>(() => StepLimit.Moved(1e17, 1e17 + 1));
    }

    [Theory]
    [InlineData(1e17)]
    [InlineData(-1e17)]
    [InlineData(1e300)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public void A_periodic_curve_far_outside_its_period_ends(double parameter)
    {
        BSplineCurve3D curve = PeriodicCurve();
        // A closed form brings it back, or the limit stops it: only going on is wrong
        Soon(() =>
        {
            try
            {
                curve.Value(parameter);
            }
            catch (Exception e) when (Refusal.IsFailure(e))
            {
            }

            return true;
        });
    }

    [Fact]
    public void A_periodic_curve_a_thousand_periods_out_is_the_same_as_at_home()
    {
        BSplineCurve3D curve = PeriodicCurve();
        Assert.True(curve.Value(1.25).DistanceTo(curve.Value(1.25 + 4 * 1000)) < 1e-9);
    }

    [Fact]
    public void An_edge_with_no_vertices_stops_relocating_pcurves_as_a_failure()
    {
        Face face = Solid.Box(10, 10, 10).Value.Shape.AllFaces()[0];
        var edge = new Edge();
        var builder = new BrepBuilder();
        builder.MakeEdge(ref edge, new LineCurve3D(new Line3D(new Axis3D(new Point3D(1, 1, 0), new Direction3D(1, 0, 0)))), 1e-7);
        builder.Range(edge, 0, 1);
        var edges = new NCollection_Sequence<Shape>();
        edges.Append(edge);

        Assert.False(Soon(() => UnifySameDomain.RelocatePCurvesToNewUorigin(
            edges,
            edge,
            face,
            1e-7,
            1,
            10,
            new IndexedMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance),
            new Dictionary<Shape, Curve2D>(ShapeHasher.Instance),
            new HashSet<Shape>(ShapeHasher.Instance))));
    }

    [Theory]
    [InlineData(0)]
    [InlineData(-1)]
    public void A_restriction_to_no_segments_is_refused(int segments)
    {
        Assert.Throws<ArgumentException>(() => new BSplineRestriction(
            true, true, true, 1e-3, 1e-5, Continuity.C1, Continuity.C2, 6, segments, true, false));
    }
}
