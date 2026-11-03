// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

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
        ConicalSurface aCone = new(new CoordinateSystem(Constants.XOY()), M_PI / 4.0, 2.0);
        Box aBox = new SurfaceBounds(aCone).Box(0.0, 2.0 * M_PI, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite(), 0.0);

        Assert.True(aBox.IsOpenZmax);
        Assert.False(aBox.IsOpenXmax);
        Point3D aFar = aCone.Value(0.0, 100.0);
        Assert.True(aBox.IsOut(aFar), "OCCT's box misses the cone far from its V=0 circle");
    }

    [Fact]
    public void OffsetSurface_Fallback_ToleranceNotAddedToOffset()
    {
        // Offset Bezier surface (no analytic equivalent): basis box enlarged by max(|offset|, Tol), not |offset| + Tol.
        NCollection_Array2<Point3D> aPoles = new(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(10.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 10.0, 0.0);
        aPoles[2, 2] = new Point3D(10.0, 10.0, 5.0);
        OffsetSurface anOff = new(new BezierSurface(aPoles), 1.0);
        Assert.Null(anOff.Surface());
        Box aBox = new SurfaceBounds(anOff).Box(0.0, 1.0, 0.0, 1.0, 0.5);

        Assert.Equal(1.0, aBox.GetGap());
    }

    [Fact]
    public void BezierSurface_TrimmedPatch_SampledWithoutDeflection()
    {
        // z(u, v) = 20 u (1 - u): maximum 5 at u = 0.5. The trimmed-patch path samples 6 U values on
        // [0.1, 0.65], none at 0.5, and does not enlarge the box by a deflection estimate.
        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(5.0, 0.0, 10.0);
        aPoles[3, 1] = new Point3D(10.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(0.0, 10.0, 0.0);
        aPoles[2, 2] = new Point3D(5.0, 10.0, 10.0);
        aPoles[3, 2] = new Point3D(10.0, 10.0, 0.0);
        BezierSurface aBezier = new(aPoles);

        Box aBox = new SurfaceBounds(aBezier).Box(0.1, 0.65, 0.0, 1.0, global::Kerf.Engine.Foundation.Precision.Confusion());

        Assert.Equal(5.0, aBezier.Value(0.5, 0.5).Z, 1e-12);
        Assert.True(aBox.GetZMax() < 4.98, $"Zmax {aBox.GetZMax()}");
    }
}
