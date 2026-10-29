// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Pins OCCT V8_0_1's behaviour where its surface bounding boxes do not contain the surface (occt-issues.md, unit 4B).
/// The port reproduces OCCT; these tests fail once the behaviour is fixed, as a reminder to update the issue list.
/// </summary>
public class GeomBndLib_SurfaceIssueTest
{
    [Fact]
    public void Cone_InfiniteV_RadialDirectionsStayClosed()
    {
        // Cone about Z, V in [0, +inf): OCCT opens only Zmax, but the radius grows with V.
        Geom_ConicalSurface aCone = new(new gp_Ax3(gp.XOY()), M_PI / 4.0, 2.0);
        Bnd_Box aBox = new GeomBndLib_Surface(aCone).Box(0.0, 2.0 * M_PI, 0.0, Precision.Infinite(), 0.0);

        Assert.True(aBox.IsOpenZmax());
        Assert.False(aBox.IsOpenXmax());
        gp_Pnt aFar = aCone.Value(0.0, 100.0);
        Assert.True(aBox.IsOut(aFar), "OCCT's box misses the cone far from its V=0 circle");
    }

    [Fact]
    public void OffsetSurface_Fallback_ToleranceNotAddedToOffset()
    {
        // Offset Bezier surface (no analytic equivalent): basis box enlarged by max(|offset|, Tol), not |offset| + Tol.
        NCollection_Array2<gp_Pnt> aPoles = new(1, 2, 1, 2);
        aPoles[1, 1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles[1, 2] = new gp_Pnt(10.0, 0.0, 0.0);
        aPoles[2, 1] = new gp_Pnt(0.0, 10.0, 0.0);
        aPoles[2, 2] = new gp_Pnt(10.0, 10.0, 5.0);
        Geom_OffsetSurface anOff = new(new Geom_BezierSurface(aPoles), 1.0);
        Assert.Null(anOff.Surface());
        Bnd_Box aBox = new GeomBndLib_Surface(anOff).Box(0.0, 1.0, 0.0, 1.0, 0.5);

        Assert.Equal(1.0, aBox.GetGap());
    }

    [Fact]
    public void BezierSurface_TrimmedPatch_SampledWithoutDeflection()
    {
        // z(u, v) = 20 u (1 - u): maximum 5 at u = 0.5. The trimmed-patch path samples 6 U values on
        // [0.1, 0.65], none at 0.5, and does not enlarge the box by a deflection estimate.
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 2);
        aPoles[1, 1] = new gp_Pnt(0.0, 0.0, 0.0);
        aPoles[2, 1] = new gp_Pnt(5.0, 0.0, 10.0);
        aPoles[3, 1] = new gp_Pnt(10.0, 0.0, 0.0);
        aPoles[1, 2] = new gp_Pnt(0.0, 10.0, 0.0);
        aPoles[2, 2] = new gp_Pnt(5.0, 10.0, 10.0);
        aPoles[3, 2] = new gp_Pnt(10.0, 10.0, 0.0);
        Geom_BezierSurface aBezier = new(aPoles);

        Bnd_Box aBox = new GeomBndLib_Surface(aBezier).Box(0.1, 0.65, 0.0, 1.0, Precision.Confusion());

        Assert.Equal(5.0, aBezier.Value(0.5, 0.5).Z(), 1e-12);
        Assert.True(aBox.GetZMax() < 4.98, $"Zmax {aBox.GetZMax()}");
    }
}
