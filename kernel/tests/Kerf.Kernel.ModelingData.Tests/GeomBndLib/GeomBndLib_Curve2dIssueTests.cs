// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The 2D curve bounding boxes that OCCT V8_0_1 made too small (occt-issues.md, unit 4B), fixed in Kerf unit 10K1
/// (decision 1429): each box must contain the arc, checked against points sampled on it and the analytic extremum.
/// </summary>
public class GeomBndLib_Curve2dIssueTest
{
    [Fact]
    public void Parabola2d_TurnedArc_ContainsTheCoordinateExtremum()
    {
        // Mirror axis (1,1)/sqrt2, focal 2: x(u) = (u^2/8 - u)/sqrt2, minimal at u = 4 (-sqrt2).
        double aS = 1.0 / Math.Sqrt(2.0);
        ParabolaCurve2D aParab = new(new Frame2D(new Point2D(0.0, 0.0), new Direction2D(aS, aS), new Direction2D(-aS, aS)), 2.0);
        Box2d aBox = new Curve2dBounds(aParab).Box(-6.0, 6.0, 0.0);

        var (aXmin, _, _, _) = aBox.Get();
        Assert.Equal(-Math.Sqrt(2.0), aParab.Value(4.0).X(), 1e-12);
        Assert.Equal(-Math.Sqrt(2.0), aXmin, 1e-12);
        for (int i = 0; i <= 120; i++)
        {
            Assert.False(aBox.IsOut(aParab.Value(-6.0 + i * 0.1)));
        }
    }
}
