// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>OCCT #1630: a circle along the issue's cubic makes no pipe with the corrected Frenet frame, the default.</summary>
public class SweepFramesTests
{
    private static (Wire Section, Wire Spine) Issue1630()
    {
        Curve3D cubic = new BezierCurve3D([new Point3D(0, 0, 0), new Point3D(0, 0, 15), new Point3D(15, 15, 15), new Point3D(15, 15, 0)]);
        Wire spine = Wire.FromEdges([Edge.FromCurve(cubic).Value]).Value;
        var circle = new Circle3D(new Frame3D(new Point3D(0, 0, 0), Direction3D.UnitZ), 1);
        Wire section = Wire.FromEdges([Edge.Circle(circle).Value]).Value;
        return (section, spine);
    }

    [Fact]
    public void The_default_falls_back_on_another_frame_says_so_and_gives_a_valid_result()
    {
        var (section, spine) = Issue1630();
        Result<Built<Shape>> swept = Sweeps.Sweep(section, spine);
        Assert.True(swept.IsSuccess, swept.ToString());
        Diagnostic warning = Assert.Single(swept.Diagnostics, d => d.Code == Sweeps.SweptWithFrenet);
        Assert.False(warning.IsError);
        Assert.Contains("the Frenet frame", warning.Message);
        Assert.True(swept.Value.Shape.Validate().IsValid);
    }

    [Fact]
    public void It_matches_asking_for_that_frame_and_the_issue_s_area()
    {
        var (section, spine) = Issue1630();
        Shape frenet = Sweeps.Sweep(section, spine, new SweepOptions { Frame = SweepFrame.Frenet }).Value.Shape;
        Shape swept = Sweeps.Sweep(section, spine).Value.Shape;
        var precise = new MassOptions { Tolerance = 1e-7 };
        Assert.Equal(frenet.IntegrateVolume(precise), swept.IntegrateVolume(precise), 9);
        // Pipe of the circle: its side, 217.81, as discrete gives too, and the two ends
        double ends = 2 * Math.PI;
        Assert.Equal(217.81 + ends, swept.IntegrateArea(precise), 1);
        Shape discrete = Sweeps.Sweep(section, spine, new SweepOptions { Frame = SweepFrame.Discrete }).Value.Shape;
        Assert.Equal(217.81 + ends, discrete.IntegrateArea(precise), 1);
    }

    [Fact]
    public void Without_the_fallback_it_fails()
    {
        var (section, spine) = Issue1630();
        Assert.True(Sweeps.Sweep(section, spine, new SweepOptions { TryOtherFrames = false }).IsFailure);
    }

    [Fact]
    public void A_frame_asked_for_is_not_replaced_and_says_nothing()
    {
        var (section, spine) = Issue1630();
        Result<Built<Shape>> plain = Sweeps.Sweep(section, spine, new SweepOptions { Frame = SweepFrame.Frenet });
        Assert.True(plain.IsSuccess);
        Assert.DoesNotContain(plain.Diagnostics, d => d.Code == Sweeps.SweptWithFrenet || d.Code == Sweeps.SweptWithDiscrete);
    }

    [Fact]
    public void A_sweep_that_works_has_no_warning()
    {
        Wire spine = Wire.FromEdges([Edge.Line(new Point3D(0, 0, 0), new Point3D(10, 0, 0)).Value]).Value;
        var circle = new Circle3D(new Frame3D(new Point3D(0, 0, 0), Direction3D.UnitX), 1);
        Wire section = Wire.FromEdges([Edge.Circle(circle).Value]).Value;
        Assert.Empty(Sweeps.Sweep(section, spine).Diagnostics);
    }
}
