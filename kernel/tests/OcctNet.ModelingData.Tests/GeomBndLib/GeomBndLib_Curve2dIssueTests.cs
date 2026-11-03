// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Pins OCCT V8_0_1's behaviour where its 2D curve bounding boxes do not contain the curve (occt-issues.md, unit 4B).
/// The port reproduces OCCT; these tests fail once the behaviour is fixed, as a reminder to update the issue list.
/// </summary>
public class GeomBndLib_Curve2dIssueTest
{
    [Fact]
    public void Parabola2d_TurnedArc_MissesCoordinateExtremum()
    {
        // Mirror axis (1,1)/sqrt2, focal 2: x(u) = (u^2/8 - u)/sqrt2, minimal at u = 4 (-sqrt2).
        double aS = 1.0 / Math.Sqrt(2.0);
        ParabolaCurve2D aParab = new(new Frame2D(new Point2D(0.0, 0.0), new Direction2D(aS, aS), new Direction2D(-aS, aS)), 2.0);
        Box2d aBox = new Curve2dBounds(aParab).Box(-6.0, 6.0, 0.0);

        var (aXmin, _, _, _) = aBox.Get();
        Assert.Equal(-1.5 / Math.Sqrt(2.0), aXmin, 1e-12);
        Assert.Equal(-Math.Sqrt(2.0), aParab.Value(4.0).X(), 1e-12);
        Assert.True(aBox.IsOut(aParab.Value(4.0)), "OCCT's box misses the parabola's X extremum");
    }
}
