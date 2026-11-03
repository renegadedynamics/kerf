// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// BndLib's boxes of parabola and hyperbola arcs turned in their plane (occt-issues.md, unit 4A's BndLib row; Kerf unit
/// 10K1, decision 1429). OCCT added only the arc's ends and vertex, so a box missed the coordinate extrema between
/// them. Each box is checked against points sampled on the arc and the extremum found by calculus.
/// </summary>
public class BndLibTurnedConicTests
{
    private const int Samples = 400;

    [Fact]
    public void Parabola_TurnedInItsPlane_TheBoxContainsTheArc()
    {
        // axes (1,1,0)/sqrt2 and (-1,1,0)/sqrt2, focal 2: x(u) = (u^2/8 - u)/sqrt2, least at u = 4
        Parabola3D aParab = new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 1.0, 0.0)), 2.0);
        Box aBox = new();
        BndLib.Add(aParab, -6.0, 6.0, 0.0, ref aBox);

        Assert.Equal(-Math.Sqrt(2.0), aBox.GetXMin(), 1e-12);
        for (int i = 0; i <= Samples; i++)
        {
            Assert.False(aBox.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(-6.0 + 12.0 * i / Samples, aParab)));
        }
    }

    [Fact]
    public void Parabola2d_TurnedInItsPlane_TheBoxContainsTheArc()
    {
        // mirror axis (1,1)/sqrt2, focal 2: x(u) = (u^2/8 - u)/sqrt2, least at u = 4
        Parabola2D aParab = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 1.0)), 2.0);
        Box2d aBox = new();
        BndLib.Add(aParab, -6.0, 6.0, 0.0, ref aBox);

        aBox.Get(out double aXmin, out _, out _, out _);
        Assert.Equal(-Math.Sqrt(2.0), aXmin, 1e-12);
        for (int i = 0; i <= Samples; i++)
        {
            Assert.False(aBox.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(-6.0 + 12.0 * i / Samples, aParab)));
        }
    }

    [Fact]
    public void Hyperbola2d_TurnedInItsPlane_TheBoxContainsTheArc()
    {
        // radii 3 and 2, turned 45 degrees: x(u) = (3 cosh u - 2 sinh u)/sqrt2, least where tanh u = 2/3, sqrt5/sqrt2
        Hyperbola2D aHypr = new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 1.0)), 3.0, 2.0);
        Box2d aBox = new();
        BndLib.Add(aHypr, -1.0, 1.0, 0.0, ref aBox);

        aBox.Get(out double aXmin, out _, out _, out _);
        Assert.Equal(Math.Sqrt(5.0 / 2.0), aXmin, 1e-12);
        for (int i = 0; i <= Samples; i++)
        {
            Assert.False(aBox.IsOut(global::Kerf.Engine.Geometry.ElCLib.Value(-1.0 + 2.0 * i / Samples, aHypr)));
        }
    }
}
