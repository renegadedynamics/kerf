// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// The 3D curve bounding boxes that OCCT V8_0_1 made too small (occt-issues.md, unit 4B), fixed in Kerf unit 10K1
/// (decision 16502): each box must contain the arc.
/// </summary>
public class GeomBndLib_CurveIssueTests
{
    /// <summary>
    /// A parabola turned in its plane, axes (1,1,0)/sqrt2 and (-1,1,0)/sqrt2, focal 2: x(u) = (u^2/8 - u)/sqrt2,
    /// minimum -sqrt2 at u = 4. OCCT added only the arc's ends and vertex, so its box's Xmin on [-6, 6] was the end
    /// u = 6's, (4.5 - 6)/sqrt2 = -1.0607. The same box reaches BndLib_Add3dCurve.
    /// </summary>
    [Fact]
    public void TurnedParabola_BoxContainsTheCoordinateExtremum()
    {
        Frame3D anAx = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 1.0, 0.0));
        ParabolaCurve3D aParab = new(anAx, 2.0);

        Box aBox = new CurveBounds(aParab).Box(-6.0, 6.0, 0.0);
        Box anAdd = new();
        Add3dCurve.Add(new CurveAdaptor(aParab), -6.0, 6.0, 0.0, ref anAdd);

        double aTrueXmin = aParab.Value(4.0).X();
        Assert.Equal(-Math.Sqrt(2.0), aTrueXmin, 1e-12);
        Assert.Equal(aTrueXmin, aBox.GetXMin(), 1e-12);
        Assert.Equal(aBox.GetXMin(), anAdd.GetXMin());
        for (int i = 0; i <= 120; i++)
        {
            Assert.False(aBox.IsOut(aParab.Value(-6.0 + i * 0.1)));
        }
    }
}
