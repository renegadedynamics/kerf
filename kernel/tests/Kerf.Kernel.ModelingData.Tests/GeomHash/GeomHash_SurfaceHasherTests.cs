// Port of: src/ModelingData/TKG3d/GTests/GeomHash_SurfaceHasher_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Hashing;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomHash_SurfaceHasherTest
{
    private readonly SurfaceHasher myHasher = new();

    // ============================================================================
    // Plane Tests
    // ============================================================================

    [Fact]
    public void Plane_CopiedPlanes_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface aPlane1 = new PlaneSurface(anAxis);
        PlaneSurface aPlane2 = (PlaneSurface)aPlane1.Copy();

        Assert.Equal(myHasher.GetHashCode(aPlane1), myHasher.GetHashCode(aPlane2));
        Assert.True(myHasher.Equals(aPlane1, aPlane2));
    }

    [Fact]
    public void Plane_DifferentPlanes_DifferentHash()
    {
        PlaneSurface aPlane1 =
            new PlaneSurface(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)));
        PlaneSurface aPlane2 =
            new PlaneSurface(new CoordinateSystem(new Point3D(0.0, 0.0, 1.0), new Direction3D(0.0, 0.0, 1.0)));

        Assert.NotEqual(myHasher.GetHashCode(aPlane1), myHasher.GetHashCode(aPlane2));
        Assert.False(myHasher.Equals(aPlane1, aPlane2));
    }

    // ============================================================================
    // Cylindrical Surface Tests
    // ============================================================================

    [Fact]
    public void CylindricalSurface_CopiedSurfaces_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CylindricalSurface aCyl1 = new CylindricalSurface(anAxis, 5.0);
        CylindricalSurface aCyl2 =
            (CylindricalSurface)aCyl1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCyl1), myHasher.GetHashCode(aCyl2));
        Assert.True(myHasher.Equals(aCyl1, aCyl2));
    }

    [Fact]
    public void CylindricalSurface_DifferentRadius_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CylindricalSurface aCyl1 = new CylindricalSurface(anAxis, 5.0);
        CylindricalSurface aCyl2 = new CylindricalSurface(anAxis, 10.0);

        Assert.NotEqual(myHasher.GetHashCode(aCyl1), myHasher.GetHashCode(aCyl2));
        Assert.False(myHasher.Equals(aCyl1, aCyl2));
    }

    // ============================================================================
    // Conical Surface Tests
    // ============================================================================

    [Fact]
    public void ConicalSurface_CopiedSurfaces_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        ConicalSurface aCone1 = new ConicalSurface(anAxis, M_PI / 6.0, 5.0);
        ConicalSurface aCone2 = (ConicalSurface)aCone1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCone1), myHasher.GetHashCode(aCone2));
        Assert.True(myHasher.Equals(aCone1, aCone2));
    }

    [Fact]
    public void ConicalSurface_DifferentAngle_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        ConicalSurface aCone1 = new ConicalSurface(anAxis, M_PI / 6.0, 5.0);
        ConicalSurface aCone2 = new ConicalSurface(anAxis, M_PI / 4.0, 5.0);

        Assert.NotEqual(myHasher.GetHashCode(aCone1), myHasher.GetHashCode(aCone2));
        Assert.False(myHasher.Equals(aCone1, aCone2));
    }

    // ============================================================================
    // Spherical Surface Tests
    // ============================================================================

    [Fact]
    public void SphericalSurface_CopiedSurfaces_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        SphericalSurface aSphere1 = new SphericalSurface(anAxis, 5.0);
        SphericalSurface aSphere2 =
            (SphericalSurface)aSphere1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSphere1), myHasher.GetHashCode(aSphere2));
        Assert.True(myHasher.Equals(aSphere1, aSphere2));
    }

    [Fact]
    public void SphericalSurface_DifferentRadius_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        SphericalSurface aSphere1 = new SphericalSurface(anAxis, 5.0);
        SphericalSurface aSphere2 = new SphericalSurface(anAxis, 10.0);

        Assert.NotEqual(myHasher.GetHashCode(aSphere1), myHasher.GetHashCode(aSphere2));
        Assert.False(myHasher.Equals(aSphere1, aSphere2));
    }

    // ============================================================================
    // Toroidal Surface Tests
    // ============================================================================

    [Fact]
    public void ToroidalSurface_CopiedSurfaces_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        ToroidalSurface aTorus1 = new ToroidalSurface(anAxis, 10.0, 3.0);
        ToroidalSurface aTorus2 = (ToroidalSurface)aTorus1.Copy();

        Assert.Equal(myHasher.GetHashCode(aTorus1), myHasher.GetHashCode(aTorus2));
        Assert.True(myHasher.Equals(aTorus1, aTorus2));
    }

    [Fact]
    public void ToroidalSurface_DifferentRadii_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        ToroidalSurface aTorus1 = new ToroidalSurface(anAxis, 10.0, 3.0);
        ToroidalSurface aTorus2 = new ToroidalSurface(anAxis, 10.0, 5.0);

        Assert.NotEqual(myHasher.GetHashCode(aTorus1), myHasher.GetHashCode(aTorus2));
        Assert.False(myHasher.Equals(aTorus1, aTorus2));
    }

    // ============================================================================
    // BezierSurface Tests
    // ============================================================================

    [Fact]
    public void BezierSurface_CopiedSurfaces_SameHash()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 1.0);

        BezierSurface aSurf1 = new BezierSurface(aPoles);
        BezierSurface aSurf2 = (BezierSurface)aSurf1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSurf1), myHasher.GetHashCode(aSurf2));
        Assert.True(myHasher.Equals(aSurf1, aSurf2));
    }

    [Fact]
    public void BezierSurface_DifferentPoles_DifferentComparison()
    {
        var aPoles1 = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles1[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles1[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles1[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles1[2, 2] = new Point3D(1.0, 1.0, 1.0);

        var aPoles2 = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles2[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles2[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles2[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles2[2, 2] = new Point3D(1.0, 1.0, 2.0); // Different

        BezierSurface aSurf1 = new BezierSurface(aPoles1);
        BezierSurface aSurf2 = new BezierSurface(aPoles2);

        // Hash may be same (metadata only), but comparison should differ
        Assert.False(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // BSplineSurface Tests
    // ============================================================================

    [Fact]
    public void BSplineSurface_CopiedSurfaces_SameHash()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 1.0);

        var aUKnots = new NCollection_Array1<double>(1, 2);
        var aVKnots = new NCollection_Array1<double>(1, 2);
        aUKnots[1] = 0.0;
        aUKnots[2] = 1.0;
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;

        var aUMults = new NCollection_Array1<int>(1, 2);
        var aVMults = new NCollection_Array1<int>(1, 2);
        aUMults[1] = 2;
        aUMults[2] = 2;
        aVMults[1] = 2;
        aVMults[2] = 2;

        BSplineSurface aSurf1 =
            new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 1, 1);
        BSplineSurface aSurf2 = (BSplineSurface)aSurf1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSurf1), myHasher.GetHashCode(aSurf2));
        Assert.True(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // SurfaceOfRevolution Tests
    // ============================================================================

    [Fact]
    public void SurfaceOfRevolution_CopiedSurfaces_SameHash()
    {
        LineCurve3D aLine = new LineCurve3D(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        RevolutionSurface aRev1 = new RevolutionSurface(aLine, anAxis);
        RevolutionSurface aRev2 =
            (RevolutionSurface)aRev1.Copy();

        Assert.Equal(myHasher.GetHashCode(aRev1), myHasher.GetHashCode(aRev2));
        Assert.True(myHasher.Equals(aRev1, aRev2));
    }

    // ============================================================================
    // SurfaceOfLinearExtrusion Tests
    // ============================================================================

    [Fact]
    public void SurfaceOfLinearExtrusion_CopiedSurfaces_SameHash()
    {
        LineCurve3D aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Direction3D aDir = new(0.0, 0.0, 1.0);
        ExtrusionSurface anExt1 =
            new ExtrusionSurface(aLine, aDir);
        ExtrusionSurface anExt2 =
            (ExtrusionSurface)anExt1.Copy();

        Assert.Equal(myHasher.GetHashCode(anExt1), myHasher.GetHashCode(anExt2));
        Assert.True(myHasher.Equals(anExt1, anExt2));
    }

    // ============================================================================
    // RectangularTrimmedSurface Tests
    // ============================================================================

    [Fact]
    public void RectangularTrimmedSurface_CopiedSurfaces_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface                     aPlane = new PlaneSurface(anAxis);
        TrimmedSurface aTrimmed1 =
            new TrimmedSurface(aPlane, 0.0, 10.0, 0.0, 10.0);
        TrimmedSurface aTrimmed2 =
            (TrimmedSurface)aTrimmed1.Copy();

        Assert.Equal(myHasher.GetHashCode(aTrimmed1), myHasher.GetHashCode(aTrimmed2));
        Assert.True(myHasher.Equals(aTrimmed1, aTrimmed2));
    }

    [Fact]
    public void RectangularTrimmedSurface_DifferentBounds_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface                     aPlane = new PlaneSurface(anAxis);
        TrimmedSurface aTrimmed1 =
            new TrimmedSurface(aPlane, 0.0, 10.0, 0.0, 10.0);
        TrimmedSurface aTrimmed2 =
            new TrimmedSurface(aPlane, 0.0, 20.0, 0.0, 10.0);

        Assert.NotEqual(myHasher.GetHashCode(aTrimmed1), myHasher.GetHashCode(aTrimmed2));
        Assert.False(myHasher.Equals(aTrimmed1, aTrimmed2));
    }

    // ============================================================================
    // OffsetSurface Tests
    // ============================================================================

    [Fact]
    public void OffsetSurface_CopiedSurfaces_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface         aPlane    = new PlaneSurface(anAxis);
        OffsetSurface anOffset1 = new OffsetSurface(aPlane, 5.0);
        OffsetSurface anOffset2 = (OffsetSurface)anOffset1.Copy();

        Assert.Equal(myHasher.GetHashCode(anOffset1), myHasher.GetHashCode(anOffset2));
        Assert.True(myHasher.Equals(anOffset1, anOffset2));
    }

    [Fact]
    public void OffsetSurface_DifferentOffset_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface         aPlane    = new PlaneSurface(anAxis);
        OffsetSurface anOffset1 = new OffsetSurface(aPlane, 5.0);
        OffsetSurface anOffset2 = new OffsetSurface(aPlane, 10.0);

        Assert.NotEqual(myHasher.GetHashCode(anOffset1), myHasher.GetHashCode(anOffset2));
        Assert.False(myHasher.Equals(anOffset1, anOffset2));
    }

    // ============================================================================
    // Cross-type Tests
    // ============================================================================

    [Fact]
    public void DifferentTypes_DifferentComparison()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface            aPlane  = new PlaneSurface(anAxis);
        SphericalSurface aSphere = new SphericalSurface(anAxis, 5.0);

        Assert.False(myHasher.Equals(aPlane, aSphere));
    }

    [Fact]
    public void NullSurfaces_HandledCorrectly()
    {
        Surface? aNullSurface = null;
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface   aPlane = new PlaneSurface(anAxis);

        Assert.Equal(0, myHasher.GetHashCode(aNullSurface));
        Assert.True(myHasher.Equals(aNullSurface, aNullSurface));
        Assert.False(myHasher.Equals(aPlane, aNullSurface));
    }

    [Fact]
    public void SameObject_Equal()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface aPlane = new PlaneSurface(anAxis);
        Assert.True(myHasher.Equals(aPlane, aPlane));
    }

    // ============================================================================
    // Weighted BSpline Surface Tests
    // ============================================================================

    [Fact]
    public void BSplineSurface_Weighted_CopiedSurfaces_SameHash()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 1.0);

        var aWeights = new NCollection_Array2<double>(1, 2, 1, 2);
        aWeights[1, 1] = 1.0;
        aWeights[1, 2] = 1.0;
        aWeights[2, 1] = 1.0;
        aWeights[2, 2] = 2.0;

        var aUKnots = new NCollection_Array1<double>(1, 2);
        var aVKnots = new NCollection_Array1<double>(1, 2);
        aUKnots[1] = 0.0;
        aUKnots[2] = 1.0;
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;

        var aUMults = new NCollection_Array1<int>(1, 2);
        var aVMults = new NCollection_Array1<int>(1, 2);
        aUMults[1] = 2;
        aUMults[2] = 2;
        aVMults[1] = 2;
        aVMults[2] = 2;

        BSplineSurface aSurf1 =
            new BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, 1, 1);
        BSplineSurface aSurf2 = (BSplineSurface)aSurf1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSurf1), myHasher.GetHashCode(aSurf2));
        Assert.True(myHasher.Equals(aSurf1, aSurf2));
    }

    [Fact]
    public void BSplineSurface_DifferentWeights_DifferentComparison()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 1.0);

        var aWeights1 = new NCollection_Array2<double>(1, 2, 1, 2);
        aWeights1[1, 1] = 1.0;
        aWeights1[1, 2] = 1.0;
        aWeights1[2, 1] = 1.0;
        aWeights1[2, 2] = 2.0;

        var aWeights2 = new NCollection_Array2<double>(1, 2, 1, 2);
        aWeights2[1, 1] = 1.0;
        aWeights2[1, 2] = 1.0;
        aWeights2[2, 1] = 1.0;
        aWeights2[2, 2] = 3.0; // Different

        var aUKnots = new NCollection_Array1<double>(1, 2);
        var aVKnots = new NCollection_Array1<double>(1, 2);
        aUKnots[1] = 0.0;
        aUKnots[2] = 1.0;
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;

        var aUMults = new NCollection_Array1<int>(1, 2);
        var aVMults = new NCollection_Array1<int>(1, 2);
        aUMults[1] = 2;
        aUMults[2] = 2;
        aVMults[1] = 2;
        aVMults[2] = 2;

        BSplineSurface aSurf1 =
            new BSplineSurface(aPoles, aWeights1, aUKnots, aVKnots, aUMults, aVMults, 1, 1);
        BSplineSurface aSurf2 =
            new BSplineSurface(aPoles, aWeights2, aUKnots, aVKnots, aUMults, aVMults, 1, 1);

        Assert.False(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // Weighted Bezier Surface Tests
    // ============================================================================

    [Fact]
    public void BezierSurface_Weighted_CopiedSurfaces_SameHash()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 1.0);

        var aWeights = new NCollection_Array2<double>(1, 2, 1, 2);
        aWeights[1, 1] = 1.0;
        aWeights[1, 2] = 1.0;
        aWeights[2, 1] = 1.0;
        aWeights[2, 2] = 2.0;

        BezierSurface aSurf1 = new BezierSurface(aPoles, aWeights);
        BezierSurface aSurf2 = (BezierSurface)aSurf1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSurf1), myHasher.GetHashCode(aSurf2));
        Assert.True(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // Axis Orientation Tests
    // ============================================================================

    [Fact]
    public void Plane_DifferentNormal_DifferentHash()
    {
        CoordinateSystem anAxis1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CoordinateSystem anAxis2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));
        PlaneSurface aPlane1 = new PlaneSurface(anAxis1);
        PlaneSurface aPlane2 = new PlaneSurface(anAxis2);

        Assert.NotEqual(myHasher.GetHashCode(aPlane1), myHasher.GetHashCode(aPlane2));
        Assert.False(myHasher.Equals(aPlane1, aPlane2));
    }

    [Fact]
    public void CylindricalSurface_DifferentAxis_DifferentHash()
    {
        CoordinateSystem anAxis1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CoordinateSystem anAxis2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        CylindricalSurface aCyl1 = new CylindricalSurface(anAxis1, 5.0);
        CylindricalSurface aCyl2 = new CylindricalSurface(anAxis2, 5.0);

        Assert.NotEqual(myHasher.GetHashCode(aCyl1), myHasher.GetHashCode(aCyl2));
        Assert.False(myHasher.Equals(aCyl1, aCyl2));
    }

    // ============================================================================
    // Revolution with Different Basis Curve Tests
    // ============================================================================

    [Fact]
    public void SurfaceOfRevolution_DifferentBasisCurve_DifferentComparison()
    {
        LineCurve3D aLine1 = new LineCurve3D(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        LineCurve3D aLine2 = new LineCurve3D(new Point3D(2.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        RevolutionSurface aRev1 = new RevolutionSurface(aLine1, anAxis);
        RevolutionSurface aRev2 = new RevolutionSurface(aLine2, anAxis);

        Assert.False(myHasher.Equals(aRev1, aRev2));
    }

    // ============================================================================
    // Extrusion with Different Direction Tests
    // ============================================================================

    [Fact]
    public void SurfaceOfLinearExtrusion_DifferentDirection_DifferentHash()
    {
        LineCurve3D aLine = new LineCurve3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Direction3D aDir1 = new(0.0, 0.0, 1.0);
        Direction3D aDir2 = new(0.0, 1.0, 0.0);
        ExtrusionSurface anExt1 =
            new ExtrusionSurface(aLine, aDir1);
        ExtrusionSurface anExt2 =
            new ExtrusionSurface(aLine, aDir2);

        Assert.NotEqual(myHasher.GetHashCode(anExt1), myHasher.GetHashCode(anExt2));
        Assert.False(myHasher.Equals(anExt1, anExt2));
    }

    // ============================================================================
    // BSpline Surface with Different Knots Tests
    // ============================================================================

    [Fact]
    public void BSplineSurface_DifferentKnots_DifferentComparison()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 1.0);

        var aUKnots1 = new NCollection_Array1<double>(1, 2);
        var aUKnots2 = new NCollection_Array1<double>(1, 2);
        var aVKnots = new NCollection_Array1<double>(1, 2);
        aUKnots1[1] = 0.0;
        aUKnots1[2] = 1.0;
        aUKnots2[1] = 0.0;
        aUKnots2[2] = 2.0; // Different
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;

        var aUMults = new NCollection_Array1<int>(1, 2);
        var aVMults = new NCollection_Array1<int>(1, 2);
        aUMults[1] = 2;
        aUMults[2] = 2;
        aVMults[1] = 2;
        aVMults[2] = 2;

        BSplineSurface aSurf1 =
            new BSplineSurface(aPoles, aUKnots1, aVKnots, aUMults, aVMults, 1, 1);
        BSplineSurface aSurf2 =
            new BSplineSurface(aPoles, aUKnots2, aVKnots, aUMults, aVMults, 1, 1);

        Assert.False(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // Transformed Surface Tests
    // ============================================================================

    [Fact]
    public void Plane_Translated_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface aPlane1 = new PlaneSurface(anAxis);
        PlaneSurface aPlane2 = (PlaneSurface)aPlane1.Copy();
        aPlane2.Translate(new Vector3D(0.0, 0.0, 1.0));

        Assert.NotEqual(myHasher.GetHashCode(aPlane1), myHasher.GetHashCode(aPlane2));
        Assert.False(myHasher.Equals(aPlane1, aPlane2));
    }

    [Fact]
    public void Sphere_Scaled_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        SphericalSurface aSphere1 = new SphericalSurface(anAxis, 5.0);
        SphericalSurface aSphere2 =
            (SphericalSurface)aSphere1.Copy();
        aSphere2.Scale(new Point3D(0.0, 0.0, 0.0), 2.0);

        Assert.NotEqual(myHasher.GetHashCode(aSphere1), myHasher.GetHashCode(aSphere2));
        Assert.False(myHasher.Equals(aSphere1, aSphere2));
    }

    // ============================================================================
    // UReversed/VReversed Surface Tests
    // ============================================================================

    [Fact]
    public void BezierSurface_UReversed_DifferentComparison()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 1.0);

        BezierSurface aSurf1 = new BezierSurface(aPoles);
        BezierSurface aSurf2 = (BezierSurface)aSurf1.UReversed();

        Assert.False(myHasher.Equals(aSurf1, aSurf2));
    }

    [Fact]
    public void BezierSurface_VReversed_DifferentComparison()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 2, 1, 2);
        aPoles[1, 1] = new Point3D(0.0, 0.0, 0.0);
        aPoles[1, 2] = new Point3D(1.0, 0.0, 0.0);
        aPoles[2, 1] = new Point3D(0.0, 1.0, 0.0);
        aPoles[2, 2] = new Point3D(1.0, 1.0, 1.0);

        BezierSurface aSurf1 = new BezierSurface(aPoles);
        BezierSurface aSurf2 = (BezierSurface)aSurf1.VReversed();

        Assert.False(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // Higher Degree BSpline Surface Tests
    // ============================================================================

    [Fact]
    public void BSplineSurface_HigherDegree_CopiedSurfaces_SameHash()
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; ++i)
        {
            for (int j = 1; j <= 4; ++j)
            {
                aPoles[i, j] = new Point3D((i - 1) * 1.0, (j - 1) * 1.0, (i + j) * 0.1);
            }
        }

        var aUKnots = new NCollection_Array1<double>(1, 2);
        var aVKnots = new NCollection_Array1<double>(1, 2);
        aUKnots[1] = 0.0;
        aUKnots[2] = 1.0;
        aVKnots[1] = 0.0;
        aVKnots[2] = 1.0;

        var aUMults = new NCollection_Array1<int>(1, 2);
        var aVMults = new NCollection_Array1<int>(1, 2);
        aUMults[1] = 4;
        aUMults[2] = 4;
        aVMults[1] = 4;
        aVMults[2] = 4;

        BSplineSurface aSurf1 =
            new BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 3);
        BSplineSurface aSurf2 = (BSplineSurface)aSurf1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSurf1), myHasher.GetHashCode(aSurf2));
        Assert.True(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // Cross-type Elementary Surface Tests
    // ============================================================================

    [Fact]
    public void Cylinder_vs_Cone_DifferentComparison()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CylindricalSurface aCylinder = new CylindricalSurface(anAxis, 5.0);
        ConicalSurface     aCone     = new ConicalSurface(anAxis, M_PI / 6.0, 5.0);

        Assert.False(myHasher.Equals(aCylinder, aCone));
    }

    [Fact]
    public void Sphere_vs_Torus_DifferentComparison()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        SphericalSurface aSphere = new SphericalSurface(anAxis, 5.0);
        ToroidalSurface  aTorus  = new ToroidalSurface(anAxis, 5.0, 1.0);

        Assert.False(myHasher.Equals(aSphere, aTorus));
    }

    // ============================================================================
    // Trimmed vs Base Surface Tests
    // ============================================================================

    [Fact]
    public void RectangularTrimmedSurface_vs_BaseSurface_DifferentComparison()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface                     aPlane = new PlaneSurface(anAxis);
        TrimmedSurface aTrimmed =
            new TrimmedSurface(aPlane, 0.0, 10.0, 0.0, 10.0);

        Assert.False(myHasher.Equals(aPlane, aTrimmed));
    }

    // ============================================================================
    // Edge Cases - Degenerate/Special Values
    // ============================================================================

    [Fact]
    public void Sphere_VerySmallRadius_CopiedSpheres_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        SphericalSurface aSphere1 = new SphericalSurface(anAxis, 1e-10);
        SphericalSurface aSphere2 =
            (SphericalSurface)aSphere1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSphere1), myHasher.GetHashCode(aSphere2));
        Assert.True(myHasher.Equals(aSphere1, aSphere2));
    }

    [Fact]
    public void Sphere_VeryLargeRadius_CopiedSpheres_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        SphericalSurface aSphere1 = new SphericalSurface(anAxis, 1e10);
        SphericalSurface aSphere2 =
            (SphericalSurface)aSphere1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSphere1), myHasher.GetHashCode(aSphere2));
        Assert.True(myHasher.Equals(aSphere1, aSphere2));
    }

    [Fact]
    public void Plane_AtOrigin_vs_FarFromOrigin_DifferentHash()
    {
        CoordinateSystem anAxis1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CoordinateSystem anAxis2 = new(new Point3D(1e10, 1e10, 1e10), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface aPlane1 = new PlaneSurface(anAxis1);
        PlaneSurface aPlane2 = new PlaneSurface(anAxis2);

        Assert.NotEqual(myHasher.GetHashCode(aPlane1), myHasher.GetHashCode(aPlane2));
        Assert.False(myHasher.Equals(aPlane1, aPlane2));
    }

    // ============================================================================
    // Revolution with Circle Basis (forms Torus-like)
    // ============================================================================

    [Fact]
    public void SurfaceOfRevolution_CircleBasis_CopiedSurfaces_SameHash()
    {
        Frame3D aCircleAxis = new(new Point3D(5.0, 0.0, 0.0), new Direction3D(0.0, 1.0, 0.0));
        CircleCurve3D              aCircle = new CircleCurve3D(aCircleAxis, 1.0);
        Axis3D aRevAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        RevolutionSurface aRev1 = new RevolutionSurface(aCircle, aRevAxis);
        RevolutionSurface aRev2 =
            (RevolutionSurface)aRev1.Copy();

        Assert.Equal(myHasher.GetHashCode(aRev1), myHasher.GetHashCode(aRev2));
        Assert.True(myHasher.Equals(aRev1, aRev2));
    }

    // ============================================================================
    // Offset Surface with Different Base
    // ============================================================================

    [Fact]
    public void OffsetSurface_DifferentBaseSurface_DifferentComparison()
    {
        CoordinateSystem anAxis1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CoordinateSystem anAxis2 = new(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        PlaneSurface         aPlane1   = new PlaneSurface(anAxis1);
        PlaneSurface         aPlane2   = new PlaneSurface(anAxis2);
        OffsetSurface anOffset1 = new OffsetSurface(aPlane1, 5.0);
        OffsetSurface anOffset2 = new OffsetSurface(aPlane2, 5.0);

        Assert.False(myHasher.Equals(anOffset1, anOffset2));
    }
}
