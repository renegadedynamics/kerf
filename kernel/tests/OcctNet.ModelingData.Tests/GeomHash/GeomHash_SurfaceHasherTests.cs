// Port of: src/ModelingData/TKG3d/GTests/GeomHash_SurfaceHasher_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class GeomHash_SurfaceHasherTest
{
    private readonly GeomHash_SurfaceHasher myHasher = new();

    // ============================================================================
    // Plane Tests
    // ============================================================================

    [Fact]
    public void Plane_CopiedPlanes_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_Plane aPlane1 = new Geom_Plane(anAxis);
        Geom_Plane aPlane2 = (Geom_Plane)aPlane1.Copy();

        Assert.Equal(myHasher.GetHashCode(aPlane1), myHasher.GetHashCode(aPlane2));
        Assert.True(myHasher.Equals(aPlane1, aPlane2));
    }

    [Fact]
    public void Plane_DifferentPlanes_DifferentHash()
    {
        Geom_Plane aPlane1 =
            new Geom_Plane(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)));
        Geom_Plane aPlane2 =
            new Geom_Plane(new CoordinateSystem(new Point3D(0.0, 0.0, 1.0), new Direction3D(0.0, 0.0, 1.0)));

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
        Geom_CylindricalSurface aCyl1 = new Geom_CylindricalSurface(anAxis, 5.0);
        Geom_CylindricalSurface aCyl2 =
            (Geom_CylindricalSurface)aCyl1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCyl1), myHasher.GetHashCode(aCyl2));
        Assert.True(myHasher.Equals(aCyl1, aCyl2));
    }

    [Fact]
    public void CylindricalSurface_DifferentRadius_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_CylindricalSurface aCyl1 = new Geom_CylindricalSurface(anAxis, 5.0);
        Geom_CylindricalSurface aCyl2 = new Geom_CylindricalSurface(anAxis, 10.0);

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
        Geom_ConicalSurface aCone1 = new Geom_ConicalSurface(anAxis, M_PI / 6.0, 5.0);
        Geom_ConicalSurface aCone2 = (Geom_ConicalSurface)aCone1.Copy();

        Assert.Equal(myHasher.GetHashCode(aCone1), myHasher.GetHashCode(aCone2));
        Assert.True(myHasher.Equals(aCone1, aCone2));
    }

    [Fact]
    public void ConicalSurface_DifferentAngle_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_ConicalSurface aCone1 = new Geom_ConicalSurface(anAxis, M_PI / 6.0, 5.0);
        Geom_ConicalSurface aCone2 = new Geom_ConicalSurface(anAxis, M_PI / 4.0, 5.0);

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
        Geom_SphericalSurface aSphere1 = new Geom_SphericalSurface(anAxis, 5.0);
        Geom_SphericalSurface aSphere2 =
            (Geom_SphericalSurface)aSphere1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSphere1), myHasher.GetHashCode(aSphere2));
        Assert.True(myHasher.Equals(aSphere1, aSphere2));
    }

    [Fact]
    public void SphericalSurface_DifferentRadius_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SphericalSurface aSphere1 = new Geom_SphericalSurface(anAxis, 5.0);
        Geom_SphericalSurface aSphere2 = new Geom_SphericalSurface(anAxis, 10.0);

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
        Geom_ToroidalSurface aTorus1 = new Geom_ToroidalSurface(anAxis, 10.0, 3.0);
        Geom_ToroidalSurface aTorus2 = (Geom_ToroidalSurface)aTorus1.Copy();

        Assert.Equal(myHasher.GetHashCode(aTorus1), myHasher.GetHashCode(aTorus2));
        Assert.True(myHasher.Equals(aTorus1, aTorus2));
    }

    [Fact]
    public void ToroidalSurface_DifferentRadii_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_ToroidalSurface aTorus1 = new Geom_ToroidalSurface(anAxis, 10.0, 3.0);
        Geom_ToroidalSurface aTorus2 = new Geom_ToroidalSurface(anAxis, 10.0, 5.0);

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

        Geom_BezierSurface aSurf1 = new Geom_BezierSurface(aPoles);
        Geom_BezierSurface aSurf2 = (Geom_BezierSurface)aSurf1.Copy();

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

        Geom_BezierSurface aSurf1 = new Geom_BezierSurface(aPoles1);
        Geom_BezierSurface aSurf2 = new Geom_BezierSurface(aPoles2);

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

        Geom_BSplineSurface aSurf1 =
            new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 1, 1);
        Geom_BSplineSurface aSurf2 = (Geom_BSplineSurface)aSurf1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSurf1), myHasher.GetHashCode(aSurf2));
        Assert.True(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // SurfaceOfRevolution Tests
    // ============================================================================

    [Fact]
    public void SurfaceOfRevolution_CopiedSurfaces_SameHash()
    {
        Geom_Line aLine = new Geom_Line(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SurfaceOfRevolution aRev1 = new Geom_SurfaceOfRevolution(aLine, anAxis);
        Geom_SurfaceOfRevolution aRev2 =
            (Geom_SurfaceOfRevolution)aRev1.Copy();

        Assert.Equal(myHasher.GetHashCode(aRev1), myHasher.GetHashCode(aRev2));
        Assert.True(myHasher.Equals(aRev1, aRev2));
    }

    // ============================================================================
    // SurfaceOfLinearExtrusion Tests
    // ============================================================================

    [Fact]
    public void SurfaceOfLinearExtrusion_CopiedSurfaces_SameHash()
    {
        Geom_Line aLine = new Geom_Line(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Direction3D aDir = new(0.0, 0.0, 1.0);
        Geom_SurfaceOfLinearExtrusion anExt1 =
            new Geom_SurfaceOfLinearExtrusion(aLine, aDir);
        Geom_SurfaceOfLinearExtrusion anExt2 =
            (Geom_SurfaceOfLinearExtrusion)anExt1.Copy();

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
        Geom_Plane                     aPlane = new Geom_Plane(anAxis);
        Geom_RectangularTrimmedSurface aTrimmed1 =
            new Geom_RectangularTrimmedSurface(aPlane, 0.0, 10.0, 0.0, 10.0);
        Geom_RectangularTrimmedSurface aTrimmed2 =
            (Geom_RectangularTrimmedSurface)aTrimmed1.Copy();

        Assert.Equal(myHasher.GetHashCode(aTrimmed1), myHasher.GetHashCode(aTrimmed2));
        Assert.True(myHasher.Equals(aTrimmed1, aTrimmed2));
    }

    [Fact]
    public void RectangularTrimmedSurface_DifferentBounds_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_Plane                     aPlane = new Geom_Plane(anAxis);
        Geom_RectangularTrimmedSurface aTrimmed1 =
            new Geom_RectangularTrimmedSurface(aPlane, 0.0, 10.0, 0.0, 10.0);
        Geom_RectangularTrimmedSurface aTrimmed2 =
            new Geom_RectangularTrimmedSurface(aPlane, 0.0, 20.0, 0.0, 10.0);

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
        Geom_Plane         aPlane    = new Geom_Plane(anAxis);
        Geom_OffsetSurface anOffset1 = new Geom_OffsetSurface(aPlane, 5.0);
        Geom_OffsetSurface anOffset2 = (Geom_OffsetSurface)anOffset1.Copy();

        Assert.Equal(myHasher.GetHashCode(anOffset1), myHasher.GetHashCode(anOffset2));
        Assert.True(myHasher.Equals(anOffset1, anOffset2));
    }

    [Fact]
    public void OffsetSurface_DifferentOffset_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_Plane         aPlane    = new Geom_Plane(anAxis);
        Geom_OffsetSurface anOffset1 = new Geom_OffsetSurface(aPlane, 5.0);
        Geom_OffsetSurface anOffset2 = new Geom_OffsetSurface(aPlane, 10.0);

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
        Geom_Plane            aPlane  = new Geom_Plane(anAxis);
        Geom_SphericalSurface aSphere = new Geom_SphericalSurface(anAxis, 5.0);

        Assert.False(myHasher.Equals(aPlane, aSphere));
    }

    [Fact]
    public void NullSurfaces_HandledCorrectly()
    {
        Geom_Surface? aNullSurface = null;
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_Plane   aPlane = new Geom_Plane(anAxis);

        Assert.Equal(0, myHasher.GetHashCode(aNullSurface));
        Assert.True(myHasher.Equals(aNullSurface, aNullSurface));
        Assert.False(myHasher.Equals(aPlane, aNullSurface));
    }

    [Fact]
    public void SameObject_Equal()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_Plane aPlane = new Geom_Plane(anAxis);
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

        Geom_BSplineSurface aSurf1 =
            new Geom_BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, 1, 1);
        Geom_BSplineSurface aSurf2 = (Geom_BSplineSurface)aSurf1.Copy();

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

        Geom_BSplineSurface aSurf1 =
            new Geom_BSplineSurface(aPoles, aWeights1, aUKnots, aVKnots, aUMults, aVMults, 1, 1);
        Geom_BSplineSurface aSurf2 =
            new Geom_BSplineSurface(aPoles, aWeights2, aUKnots, aVKnots, aUMults, aVMults, 1, 1);

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

        Geom_BezierSurface aSurf1 = new Geom_BezierSurface(aPoles, aWeights);
        Geom_BezierSurface aSurf2 = (Geom_BezierSurface)aSurf1.Copy();

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
        Geom_Plane aPlane1 = new Geom_Plane(anAxis1);
        Geom_Plane aPlane2 = new Geom_Plane(anAxis2);

        Assert.NotEqual(myHasher.GetHashCode(aPlane1), myHasher.GetHashCode(aPlane2));
        Assert.False(myHasher.Equals(aPlane1, aPlane2));
    }

    [Fact]
    public void CylindricalSurface_DifferentAxis_DifferentHash()
    {
        CoordinateSystem anAxis1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CoordinateSystem anAxis2 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Geom_CylindricalSurface aCyl1 = new Geom_CylindricalSurface(anAxis1, 5.0);
        Geom_CylindricalSurface aCyl2 = new Geom_CylindricalSurface(anAxis2, 5.0);

        Assert.NotEqual(myHasher.GetHashCode(aCyl1), myHasher.GetHashCode(aCyl2));
        Assert.False(myHasher.Equals(aCyl1, aCyl2));
    }

    // ============================================================================
    // Revolution with Different Basis Curve Tests
    // ============================================================================

    [Fact]
    public void SurfaceOfRevolution_DifferentBasisCurve_DifferentComparison()
    {
        Geom_Line aLine1 = new Geom_Line(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_Line aLine2 = new Geom_Line(new Point3D(2.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Axis3D anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SurfaceOfRevolution aRev1 = new Geom_SurfaceOfRevolution(aLine1, anAxis);
        Geom_SurfaceOfRevolution aRev2 = new Geom_SurfaceOfRevolution(aLine2, anAxis);

        Assert.False(myHasher.Equals(aRev1, aRev2));
    }

    // ============================================================================
    // Extrusion with Different Direction Tests
    // ============================================================================

    [Fact]
    public void SurfaceOfLinearExtrusion_DifferentDirection_DifferentHash()
    {
        Geom_Line aLine = new Geom_Line(new Point3D(0.0, 0.0, 0.0), new Direction3D(1.0, 0.0, 0.0));
        Direction3D aDir1 = new(0.0, 0.0, 1.0);
        Direction3D aDir2 = new(0.0, 1.0, 0.0);
        Geom_SurfaceOfLinearExtrusion anExt1 =
            new Geom_SurfaceOfLinearExtrusion(aLine, aDir1);
        Geom_SurfaceOfLinearExtrusion anExt2 =
            new Geom_SurfaceOfLinearExtrusion(aLine, aDir2);

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

        Geom_BSplineSurface aSurf1 =
            new Geom_BSplineSurface(aPoles, aUKnots1, aVKnots, aUMults, aVMults, 1, 1);
        Geom_BSplineSurface aSurf2 =
            new Geom_BSplineSurface(aPoles, aUKnots2, aVKnots, aUMults, aVMults, 1, 1);

        Assert.False(myHasher.Equals(aSurf1, aSurf2));
    }

    // ============================================================================
    // Transformed Surface Tests
    // ============================================================================

    [Fact]
    public void Plane_Translated_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_Plane aPlane1 = new Geom_Plane(anAxis);
        Geom_Plane aPlane2 = (Geom_Plane)aPlane1.Copy();
        aPlane2.Translate(new Vector3D(0.0, 0.0, 1.0));

        Assert.NotEqual(myHasher.GetHashCode(aPlane1), myHasher.GetHashCode(aPlane2));
        Assert.False(myHasher.Equals(aPlane1, aPlane2));
    }

    [Fact]
    public void Sphere_Scaled_DifferentHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SphericalSurface aSphere1 = new Geom_SphericalSurface(anAxis, 5.0);
        Geom_SphericalSurface aSphere2 =
            (Geom_SphericalSurface)aSphere1.Copy();
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

        Geom_BezierSurface aSurf1 = new Geom_BezierSurface(aPoles);
        Geom_BezierSurface aSurf2 = (Geom_BezierSurface)aSurf1.UReversed();

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

        Geom_BezierSurface aSurf1 = new Geom_BezierSurface(aPoles);
        Geom_BezierSurface aSurf2 = (Geom_BezierSurface)aSurf1.VReversed();

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

        Geom_BSplineSurface aSurf1 =
            new Geom_BSplineSurface(aPoles, aUKnots, aVKnots, aUMults, aVMults, 3, 3);
        Geom_BSplineSurface aSurf2 = (Geom_BSplineSurface)aSurf1.Copy();

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
        Geom_CylindricalSurface aCylinder = new Geom_CylindricalSurface(anAxis, 5.0);
        Geom_ConicalSurface     aCone     = new Geom_ConicalSurface(anAxis, M_PI / 6.0, 5.0);

        Assert.False(myHasher.Equals(aCylinder, aCone));
    }

    [Fact]
    public void Sphere_vs_Torus_DifferentComparison()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SphericalSurface aSphere = new Geom_SphericalSurface(anAxis, 5.0);
        Geom_ToroidalSurface  aTorus  = new Geom_ToroidalSurface(anAxis, 5.0, 1.0);

        Assert.False(myHasher.Equals(aSphere, aTorus));
    }

    // ============================================================================
    // Trimmed vs Base Surface Tests
    // ============================================================================

    [Fact]
    public void RectangularTrimmedSurface_vs_BaseSurface_DifferentComparison()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_Plane                     aPlane = new Geom_Plane(anAxis);
        Geom_RectangularTrimmedSurface aTrimmed =
            new Geom_RectangularTrimmedSurface(aPlane, 0.0, 10.0, 0.0, 10.0);

        Assert.False(myHasher.Equals(aPlane, aTrimmed));
    }

    // ============================================================================
    // Edge Cases - Degenerate/Special Values
    // ============================================================================

    [Fact]
    public void Sphere_VerySmallRadius_CopiedSpheres_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SphericalSurface aSphere1 = new Geom_SphericalSurface(anAxis, 1e-10);
        Geom_SphericalSurface aSphere2 =
            (Geom_SphericalSurface)aSphere1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSphere1), myHasher.GetHashCode(aSphere2));
        Assert.True(myHasher.Equals(aSphere1, aSphere2));
    }

    [Fact]
    public void Sphere_VeryLargeRadius_CopiedSpheres_SameHash()
    {
        CoordinateSystem anAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SphericalSurface aSphere1 = new Geom_SphericalSurface(anAxis, 1e10);
        Geom_SphericalSurface aSphere2 =
            (Geom_SphericalSurface)aSphere1.Copy();

        Assert.Equal(myHasher.GetHashCode(aSphere1), myHasher.GetHashCode(aSphere2));
        Assert.True(myHasher.Equals(aSphere1, aSphere2));
    }

    [Fact]
    public void Plane_AtOrigin_vs_FarFromOrigin_DifferentHash()
    {
        CoordinateSystem anAxis1 = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        CoordinateSystem anAxis2 = new(new Point3D(1e10, 1e10, 1e10), new Direction3D(0.0, 0.0, 1.0));
        Geom_Plane aPlane1 = new Geom_Plane(anAxis1);
        Geom_Plane aPlane2 = new Geom_Plane(anAxis2);

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
        Geom_Circle              aCircle = new Geom_Circle(aCircleAxis, 1.0);
        Axis3D aRevAxis = new(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0));
        Geom_SurfaceOfRevolution aRev1 = new Geom_SurfaceOfRevolution(aCircle, aRevAxis);
        Geom_SurfaceOfRevolution aRev2 =
            (Geom_SurfaceOfRevolution)aRev1.Copy();

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
        Geom_Plane         aPlane1   = new Geom_Plane(anAxis1);
        Geom_Plane         aPlane2   = new Geom_Plane(anAxis2);
        Geom_OffsetSurface anOffset1 = new Geom_OffsetSurface(aPlane1, 5.0);
        Geom_OffsetSurface anOffset2 = new Geom_OffsetSurface(aPlane2, 5.0);

        Assert.False(myHasher.Equals(anOffset1, anOffset2));
    }
}
