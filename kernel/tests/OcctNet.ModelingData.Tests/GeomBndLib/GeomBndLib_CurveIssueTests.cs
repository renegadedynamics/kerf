// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>Pins OCCT bugs of the 3D curve bounding boxes (docs/occt-issues.md, unit 4B).</summary>
public class GeomBndLib_CurveIssueTests
{
    /// <summary>
    /// GeomBndLib_Parabola::Box adds only the arc ends and the vertex. A parabola turned in its plane, axes
    /// (1,1,0)/sqrt2 and (-1,1,0)/sqrt2, focal 2: x(u) = (u^2/8 - u)/sqrt2, minimum -sqrt2 at u = 4. On [-6, 6] the
    /// box's Xmin is that of the end u = 6, (4.5 - 6)/sqrt2 = -1.0607. The same box reaches BndLib_Add3dCurve.
    /// </summary>
    [Fact]
    public void TurnedParabola_BoxMissesCoordinateExtremum()
    {
        gp_Ax2 anAx = new(new gp_Pnt(0.0, 0.0, 0.0), new gp_Dir(0.0, 0.0, 1.0), new gp_Dir(1.0, 1.0, 0.0));
        Geom_Parabola aParab = new(anAx, 2.0);

        Bnd_Box aBox = new GeomBndLib_Curve(aParab).Box(-6.0, 6.0, 0.0);
        Bnd_Box anAdd = new();
        BndLib_Add3dCurve.Add(new GeomAdaptor_Curve(aParab), -6.0, 6.0, 0.0, ref anAdd);

        double aTrueXmin = aParab.Value(4.0).X();
        Assert.Equal(-Math.Sqrt(2.0), aTrueXmin, 1e-12);
        Assert.Equal(-1.5 / Math.Sqrt(2.0), aBox.GetXMin(), 1e-12);
        Assert.Equal(aBox.GetXMin(), anAdd.GetXMin());
        Assert.True(aBox.GetXMin() > aTrueXmin + 0.3, "the box misses the curve's X minimum");
    }
}
