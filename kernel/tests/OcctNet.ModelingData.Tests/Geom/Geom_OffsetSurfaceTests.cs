// Port of OCCT's src/ModelingData/TKG3d/GTests/Geom_OffsetSurface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>Geom_OffsetSurface: copies (the fixture suite of the GTest file).</summary>
public class Geom_OffsetSurface_Test
{
    private readonly Geom_OffsetSurface myOriginalSurface;

    public Geom_OffsetSurface_Test()
    {
        // Create a plane as basis surface
        gp_Pln aPlane = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));
        Geom_Plane aBasisSurface = new(aPlane);

        // Create offset surface
        double anOffsetValue = 3.0;

        myOriginalSurface = new Geom_OffsetSurface(aBasisSurface, anOffsetValue);
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        // Test copy constructor
        Geom_OffsetSurface aCopiedSurface = new(myOriginalSurface);

        // Verify basic properties are identical
        Assert.Equal(myOriginalSurface.Offset(), aCopiedSurface.Offset());
        Assert.Equal(myOriginalSurface.IsUPeriodic(), aCopiedSurface.IsUPeriodic());
        Assert.Equal(myOriginalSurface.IsVPeriodic(), aCopiedSurface.IsVPeriodic());
        Assert.Equal(myOriginalSurface.IsUClosed(), aCopiedSurface.IsUClosed());
        Assert.Equal(myOriginalSurface.IsVClosed(), aCopiedSurface.IsVClosed());
    }

    [Fact]
    public void CopyConstructorBasisSurface()
    {
        Geom_OffsetSurface aCopiedSurface = new(myOriginalSurface);

        // Verify basis surfaces are equivalent but independent
        Geom_Surface anOrigBasis = myOriginalSurface.BasisSurface();
        Geom_Surface aCopyBasis = aCopiedSurface.BasisSurface();

        // They should be different objects
        Assert.NotSame(anOrigBasis, aCopyBasis);

        // But functionally equivalent
        anOrigBasis.Bounds(out double anUFirst, out double anULast, out double aVFirst, out double aVLast);
        aCopyBasis.Bounds(out double anUFirstCopy, out double anULastCopy, out double aVFirstCopy, out double aVLastCopy);

        Assert.Equal(anUFirst, anUFirstCopy);
        Assert.Equal(anULast, anULastCopy);
        Assert.Equal(aVFirst, aVFirstCopy);
        Assert.Equal(aVLast, aVLastCopy);
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        // Test that Copy() method uses the optimized copy constructor
        Geom_Geometry aCopiedGeom = myOriginalSurface.Copy();
        Geom_OffsetSurface? aCopiedSurface = aCopiedGeom as Geom_OffsetSurface;

        Assert.NotNull(aCopiedSurface);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalSurface.Offset(), aCopiedSurface.Offset());

        // Test evaluation at several points
        for (double u = -5.0; u <= 5.0; u += 2.5)
        {
            for (double v = -5.0; v <= 5.0; v += 2.5)
            {
                gp_Pnt anOrigPnt = myOriginalSurface.Value(u, v);
                gp_Pnt aCopyPnt = aCopiedSurface.Value(u, v);
                Assert.True(anOrigPnt.IsEqual(aCopyPnt, 1e-10));
            }
        }
    }

    [Fact]
    public void CopyIndependence()
    {
        Geom_OffsetSurface aCopiedSurface = new(myOriginalSurface);

        // Store original offset value
        double anOrigOffset = aCopiedSurface.Offset();

        // Modify the original surface
        myOriginalSurface.SetOffsetValue(15.0);

        // Verify the copied surface is not affected
        Assert.Equal(anOrigOffset, aCopiedSurface.Offset());
        Assert.NotEqual(myOriginalSurface.Offset(), aCopiedSurface.Offset());
    }
}

/// <summary>Tests for Surface() method - equivalent surface computation for canonical surfaces.</summary>
public class Geom_OffsetSurface_EquivalentSurface
{
    [Fact]
    public void Plane_ReturnsTranslatedPlane()
    {
        // Create a plane at origin with Z-normal.
        gp_Ax3 anAxis = new(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0, 0, 1));
        Geom_Plane aPlane = new(anAxis);

        double anOffset = 5.0;
        Geom_OffsetSurface anOffsetSurf = new(aPlane, anOffset);

        // Get equivalent surface.
        Geom_Surface? anEquiv = anOffsetSurf.Surface();
        Assert.NotNull(anEquiv);

        // Should be a plane.
        Geom_Plane? anEquivPlane = anEquiv as Geom_Plane;
        Assert.NotNull(anEquivPlane);

        // Verify the plane is translated by offset along normal.
        gp_Pnt anOrigLoc = aPlane.Location();
        gp_Pnt anEquivLoc = anEquivPlane.Location();
        Assert.Equal(anOffset, anEquivLoc.Z() - anOrigLoc.Z(), Precision.Confusion());
        Assert.Equal(anOrigLoc.X(), anEquivLoc.X(), Precision.Confusion());
        Assert.Equal(anOrigLoc.Y(), anEquivLoc.Y(), Precision.Confusion());
    }

    [Fact]
    public void Cylinder_PositiveOffset_ReturnsLargerCylinder()
    {
        // Create a cylinder with radius 10.
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);

        Geom_OffsetSurface anOffsetSurf = new(aCyl, 3.0);

        Geom_CylindricalSurface? anEquivCyl = anOffsetSurf.Surface() as Geom_CylindricalSurface;
        Assert.NotNull(anEquivCyl);

        // Radius should increase by offset.
        Assert.Equal(13.0, anEquivCyl.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Cylinder_NegativeOffset_ReturnsSmallerCylinder()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);

        Geom_OffsetSurface anOffsetSurf = new(aCyl, -3.0);

        Geom_CylindricalSurface? anEquivCyl = anOffsetSurf.Surface() as Geom_CylindricalSurface;
        Assert.NotNull(anEquivCyl);

        Assert.Equal(7.0, anEquivCyl.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Cylinder_NegativeRadiusFlip_ReturnsFlippedCylinder()
    {
        // Offset that makes radius negative should flip the surface.
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 5.0);

        Geom_OffsetSurface anOffsetSurf = new(aCyl, -8.0); // Results in radius = -3

        Geom_CylindricalSurface? anEquivCyl = anOffsetSurf.Surface() as Geom_CylindricalSurface;
        Assert.NotNull(anEquivCyl);

        // Radius should be absolute value.
        Assert.Equal(3.0, anEquivCyl.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Sphere_PositiveOffset_ReturnsLargerSphere()
    {
        Geom_SphericalSurface aSphere = new(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 10.0);

        Geom_OffsetSurface anOffsetSurf = new(aSphere, 5.0);

        Geom_SphericalSurface? anEquivSphere = anOffsetSurf.Surface() as Geom_SphericalSurface;
        Assert.NotNull(anEquivSphere);

        Assert.Equal(15.0, anEquivSphere.Radius(), Precision.Confusion());
        // Center should remain the same.
        Assert.True(anEquivSphere.Location().IsEqual(aSphere.Location(), Precision.Confusion()));
    }

    [Fact]
    public void Sphere_NegativeRadiusFlip_ReturnsFlippedSphere()
    {
        Geom_SphericalSurface aSphere = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 5.0);

        Geom_OffsetSurface anOffsetSurf = new(aSphere, -8.0); // Results in radius = -3

        Geom_SphericalSurface? anEquivSphere = anOffsetSurf.Surface() as Geom_SphericalSurface;
        Assert.NotNull(anEquivSphere);

        Assert.Equal(3.0, anEquivSphere.Radius(), Precision.Confusion());
    }

    [Fact]
    public void Cone_PositiveOffset_ReturnsOffsetCone()
    {
        double aSemiAngle = M_PI / 6.0; // 30 degrees
        double aRefRadius = 10.0;
        Geom_ConicalSurface aCone = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), aSemiAngle, aRefRadius);

        double anOffset = 2.0;
        Geom_OffsetSurface anOffsetSurf = new(aCone, anOffset);

        Geom_ConicalSurface? anEquivCone = anOffsetSurf.Surface() as Geom_ConicalSurface;
        Assert.NotNull(anEquivCone);

        // Semi-angle should remain the same.
        Assert.Equal(aSemiAngle, anEquivCone.SemiAngle(), Precision.Confusion());

        // Reference radius should change by offset * cos(alpha).
        double anExpectedRadius = aRefRadius + anOffset * Math.Cos(aSemiAngle);
        Assert.Equal(anExpectedRadius, anEquivCone.RefRadius(), Precision.Confusion());
    }

    [Fact]
    public void Torus_PositiveOffset_ReturnsLargerTorus()
    {
        double aMajorRadius = 20.0;
        double aMinorRadius = 5.0;
        Geom_ToroidalSurface aTorus = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), aMajorRadius, aMinorRadius);

        double anOffset = 2.0;
        Geom_OffsetSurface anOffsetSurf = new(aTorus, anOffset);

        Geom_ToroidalSurface? anEquivTorus = anOffsetSurf.Surface() as Geom_ToroidalSurface;
        Assert.NotNull(anEquivTorus);

        // Major radius should remain the same.
        Assert.Equal(aMajorRadius, anEquivTorus.MajorRadius(), Precision.Confusion());
        // Minor radius should increase by offset.
        Assert.Equal(aMinorRadius + anOffset, anEquivTorus.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void Torus_NegativeOffset_ReturnsSmallerTorus()
    {
        double aMajorRadius = 20.0;
        double aMinorRadius = 5.0;
        Geom_ToroidalSurface aTorus = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), aMajorRadius, aMinorRadius);

        Geom_OffsetSurface anOffsetSurf = new(aTorus, -2.0);

        Geom_ToroidalSurface? anEquivTorus = anOffsetSurf.Surface() as Geom_ToroidalSurface;
        Assert.NotNull(anEquivTorus);

        Assert.Equal(aMajorRadius, anEquivTorus.MajorRadius(), Precision.Confusion());
        Assert.Equal(aMinorRadius - 2.0, anEquivTorus.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void Torus_NegativeMinorRadiusFlip_ReturnsFlippedTorus()
    {
        double aMajorRadius = 20.0;
        double aMinorRadius = 3.0;
        Geom_ToroidalSurface aTorus = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), aMajorRadius, aMinorRadius);

        Geom_OffsetSurface anOffsetSurf = new(aTorus, -5.0); // Results in minor radius = -2

        Geom_ToroidalSurface? anEquivTorus = anOffsetSurf.Surface() as Geom_ToroidalSurface;
        Assert.NotNull(anEquivTorus);

        Assert.Equal(aMajorRadius, anEquivTorus.MajorRadius(), Precision.Confusion());
        Assert.Equal(2.0, anEquivTorus.MinorRadius(), Precision.Confusion());
    }

    [Fact]
    public void ZeroOffset_ReturnsBasisSurface()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);

        Geom_OffsetSurface anOffsetSurf = new(aCyl, 0.0);

        Geom_Surface? anEquiv = anOffsetSurf.Surface();
        Assert.NotNull(anEquiv);

        // Should return the internal basis surface (which is a copy of the original).
        Assert.Same(anOffsetSurf.BasisSurface(), anEquiv);

        // The equivalent should be geometrically identical to the original.
        Geom_CylindricalSurface? anEquivCyl = anEquiv as Geom_CylindricalSurface;
        Assert.NotNull(anEquivCyl);
        Assert.Equal(10.0, anEquivCyl.Radius(), Precision.Confusion());
    }

    [Fact]
    public void TrimmedPlane_ReturnsTrimmedTranslatedPlane()
    {
        Geom_Plane aPlane = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));

        // Create trimmed surface.
        Geom_RectangularTrimmedSurface aTrimmed = new(aPlane, 0.0, 10.0, 0.0, 5.0);

        Geom_OffsetSurface anOffsetSurf = new(aTrimmed, 3.0);

        // Should be a trimmed surface.
        Geom_RectangularTrimmedSurface? anEquivTrimmed = anOffsetSurf.Surface() as Geom_RectangularTrimmedSurface;
        Assert.NotNull(anEquivTrimmed);

        // Verify bounds are preserved.
        anEquivTrimmed.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal(0.0, U1, Precision.Confusion());
        Assert.Equal(10.0, U2, Precision.Confusion());
        Assert.Equal(0.0, V1, Precision.Confusion());
        Assert.Equal(5.0, V2, Precision.Confusion());
    }

    [Fact]
    public void DegenerateCylinder_ReturnsNull()
    {
        // Offset that makes radius too small (degenerate case).
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1.0);

        // Offset close to -1 should result in degenerate surface.
        double anOffset = -1.0 + 0.5 * Precision.Confusion();
        Geom_OffsetSurface anOffsetSurf = new(aCyl, anOffset);

        // Degenerate surface should return null.
        Assert.Null(anOffsetSurf.Surface());
    }

    [Fact]
    public void EquivalentSurface_EvaluationConsistency()
    {
        // Verify that evaluation on offset surface matches evaluation on equivalent surface.
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);

        Geom_OffsetSurface anOffsetSurf = new(aCyl, 3.0);

        Geom_Surface? anEquiv = anOffsetSurf.Surface();
        Assert.NotNull(anEquiv);

        // Test evaluation at several points.
        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = -5.0; v <= 5.0; v += 2.5)
            {
                gp_Pnt anOffsetPnt = anOffsetSurf.Value(u, v);
                gp_Pnt anEquivPnt = anEquiv.Value(u, v);
                Assert.True(anOffsetPnt.IsEqual(anEquivPnt, Precision.Confusion()), $"Mismatch at u={u}, v={v}");
            }
        }
    }
}

/// <summary>Shared checks of the derivative suites.</summary>
internal static class OffsetSurfaceTestUtils
{
    //! Helper to compare vectors with tolerance.
    public static bool VectorsEqual(in gp_Vec theV1, in gp_Vec theV2, double theTol) => (theV1 - theV2).Magnitude() < theTol;

    /// <summary>The offset surface's adaptor and the adaptor of its equivalent surface.</summary>
    public static (GeomAdaptor_Surface Offset, GeomAdaptor_Surface Equiv) Adaptors(Geom_OffsetSurface theSurf)
    {
        GeomAdaptor_Surface anOffsetAdaptor = new(theSurf);
        Geom_Surface? anEquiv = theSurf.Surface();
        Assert.NotNull(anEquiv);
        return (anOffsetAdaptor, new GeomAdaptor_Surface(anEquiv));
    }

    public static void CheckD0(Adaptor3d_Surface theA, Adaptor3d_Surface theB, double u, double v, double aTol)
    {
        theA.D0(u, v, out gp_Pnt aP1);
        theB.D0(u, v, out gp_Pnt aP2);
        Assert.True(aP1.IsEqual(aP2, aTol), $"D0 mismatch at u={u}, v={v}");
    }

    public static void CheckD1(Adaptor3d_Surface theA, Adaptor3d_Surface theB, double u, double v, double aTol)
    {
        theA.D1(u, v, out gp_Pnt aP1D1, out gp_Vec aD1U1, out gp_Vec aD1V1);
        theB.D1(u, v, out gp_Pnt aP2D1, out gp_Vec aD1U2, out gp_Vec aD1V2);
        Assert.True(aP1D1.IsEqual(aP2D1, aTol), $"D1 point mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD1U1, aD1U2, aTol), $"D1U mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD1V1, aD1V2, aTol), $"D1V mismatch at u={u}, v={v}");
    }

    public static void CheckD2(Adaptor3d_Surface theA, Adaptor3d_Surface theB, double u, double v, double aTol)
    {
        theA.D2(u, v, out gp_Pnt aP1D2, out _, out _, out gp_Vec aD2U1, out gp_Vec aD2V1, out gp_Vec aD2UV1);
        theB.D2(u, v, out gp_Pnt aP2D2, out _, out _, out gp_Vec aD2U2, out gp_Vec aD2V2, out gp_Vec aD2UV2);
        Assert.True(aP1D2.IsEqual(aP2D2, aTol), $"D2 point mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD2U1, aD2U2, aTol), $"D2U mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD2V1, aD2V2, aTol), $"D2V mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD2UV1, aD2UV2, aTol), $"D2UV mismatch at u={u}, v={v}");
    }

    public static void CheckD3(Adaptor3d_Surface theA, Adaptor3d_Surface theB, double u, double v, double aTol)
    {
        theA.D3(u, v, out gp_Pnt aP1D3, out _, out _, out _, out _, out _, out gp_Vec aD3U1, out gp_Vec aD3V1,
                out gp_Vec aD3UUV1, out gp_Vec aD3UVV1);
        theB.D3(u, v, out gp_Pnt aP2D3, out _, out _, out _, out _, out _, out gp_Vec aD3U2, out gp_Vec aD3V2,
                out gp_Vec aD3UUV2, out gp_Vec aD3UVV2);
        Assert.True(aP1D3.IsEqual(aP2D3, aTol), $"D3 point mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD3U1, aD3U2, aTol), $"D3U mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD3V1, aD3V2, aTol), $"D3V mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD3UUV1, aD3UUV2, aTol), $"D3UUV mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aD3UVV1, aD3UVV2, aTol), $"D3UVV mismatch at u={u}, v={v}");
    }

    /// <summary>Direct evaluation on the Geom surface against both adaptors, up to theOrder (0 to 2).</summary>
    public static void CheckThreeWay(Geom_Surface theDirect, Adaptor3d_Surface theOffsetAdaptor,
                                     Adaptor3d_Surface theEquivAdaptor, double u, double v, double aTol, int theOrder)
    {
        // D0.
        gp_Pnt aDirectPnt = theDirect.Value(u, v);
        theOffsetAdaptor.D0(u, v, out gp_Pnt aOffsetAdaptorPnt);
        theEquivAdaptor.D0(u, v, out gp_Pnt aEquivAdaptorPnt);
        Assert.True(aDirectPnt.IsEqual(aOffsetAdaptorPnt, aTol), $"Direct vs OffsetAdaptor D0 mismatch at u={u}, v={v}");
        Assert.True(aDirectPnt.IsEqual(aEquivAdaptorPnt, aTol), $"Direct vs EquivAdaptor D0 mismatch at u={u}, v={v}");
        Assert.True(aOffsetAdaptorPnt.IsEqual(aEquivAdaptorPnt, aTol), $"OffsetAdaptor vs EquivAdaptor D0 mismatch at u={u}, v={v}");
        if (theOrder < 1)
        {
            return;
        }

        // D1.
        theDirect.D1(u, v, out gp_Pnt aDirectD1Pnt, out gp_Vec aDirectD1U, out gp_Vec aDirectD1V);
        theOffsetAdaptor.D1(u, v, out gp_Pnt aOffsetD1Pnt, out gp_Vec aOffsetD1U, out gp_Vec aOffsetD1V);
        theEquivAdaptor.D1(u, v, out gp_Pnt aEquivD1Pnt, out gp_Vec aEquivD1U, out gp_Vec aEquivD1V);
        Assert.True(aDirectD1Pnt.IsEqual(aOffsetD1Pnt, aTol), $"Direct vs OffsetAdaptor D1 point mismatch at u={u}, v={v}");
        Assert.True(aDirectD1Pnt.IsEqual(aEquivD1Pnt, aTol), $"Direct vs EquivAdaptor D1 point mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD1U, aOffsetD1U, aTol), $"Direct vs OffsetAdaptor D1U mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD1U, aEquivD1U, aTol), $"Direct vs EquivAdaptor D1U mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD1V, aOffsetD1V, aTol), $"Direct vs OffsetAdaptor D1V mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD1V, aEquivD1V, aTol), $"Direct vs EquivAdaptor D1V mismatch at u={u}, v={v}");
        if (theOrder < 2)
        {
            return;
        }

        // D2.
        theDirect.D2(u, v, out _, out _, out _, out gp_Vec aDirectD2U, out gp_Vec aDirectD2V, out gp_Vec aDirectD2UV);
        theOffsetAdaptor.D2(u, v, out _, out _, out _, out gp_Vec aOffsetD2U, out gp_Vec aOffsetD2V, out gp_Vec aOffsetD2UV);
        theEquivAdaptor.D2(u, v, out _, out _, out _, out gp_Vec aEquivD2U, out gp_Vec aEquivD2V, out gp_Vec aEquivD2UV);
        Assert.True(VectorsEqual(aDirectD2U, aOffsetD2U, aTol), $"Direct vs OffsetAdaptor D2U mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD2U, aEquivD2U, aTol), $"Direct vs EquivAdaptor D2U mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD2V, aOffsetD2V, aTol), $"Direct vs OffsetAdaptor D2V mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD2V, aEquivD2V, aTol), $"Direct vs EquivAdaptor D2V mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD2UV, aOffsetD2UV, aTol), $"Direct vs OffsetAdaptor D2UV mismatch at u={u}, v={v}");
        Assert.True(VectorsEqual(aDirectD2UV, aEquivD2UV, aTol), $"Direct vs EquivAdaptor D2UV mismatch at u={u}, v={v}");
    }
}

/// <summary>Tests for D0, D1, D2, D3 consistency between offset surface and equivalent surface via adaptor.</summary>
public class Geom_OffsetSurface_Derivatives
{
    private const double aTol = 1e-10;

    [Fact]
    public void Plane_D0D1D2D3_Consistency()
    {
        Geom_Plane aPlane = new(new gp_Ax3(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0, 0, 1)));
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(new Geom_OffsetSurface(aPlane, 5.0));

        // Test at several points.
        for (double u = -5.0; u <= 5.0; u += 2.5)
        {
            for (double v = -5.0; v <= 5.0; v += 2.5)
            {
                OffsetSurfaceTestUtils.CheckD0(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD1(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD2(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
            }
        }
    }

    [Fact]
    public void Cylinder_D0D1D2D3_Consistency()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(new Geom_OffsetSurface(aCyl, 3.0));

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = -5.0; v <= 5.0; v += 2.5)
            {
                OffsetSurfaceTestUtils.CheckD0(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD1(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD2(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD3(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
            }
        }
    }

    [Fact]
    public void Sphere_D0D1D2D3_Consistency()
    {
        Geom_SphericalSurface aSphere = new(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 10.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(new Geom_OffsetSurface(aSphere, 5.0));

        // Avoid poles at v = +/- PI/2.
        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = -M_PI / 3.0; v <= M_PI / 3.0; v += M_PI / 6.0)
            {
                OffsetSurfaceTestUtils.CheckD0(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD1(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD2(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
            }
        }
    }

    [Fact]
    public void Cone_D0D1D2_Consistency()
    {
        Geom_ConicalSurface aCone = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), M_PI / 6.0, 10.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(new Geom_OffsetSurface(aCone, 2.0));

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = 1.0; v <= 10.0; v += 3.0) // Avoid apex at v=0.
            {
                OffsetSurfaceTestUtils.CheckD0(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD1(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD2(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
            }
        }
    }

    [Fact]
    public void Torus_D0D1D2D3_Consistency()
    {
        Geom_ToroidalSurface aTorus = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 5.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(new Geom_OffsetSurface(aTorus, 2.0));

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = 0.0; v < 2.0 * M_PI; v += M_PI / 4.0)
            {
                OffsetSurfaceTestUtils.CheckD0(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD1(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD2(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
                OffsetSurfaceTestUtils.CheckD3(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
            }
        }
    }

    [Fact]
    public void Cylinder_DN_Consistency()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(new Geom_OffsetSurface(aCyl, 3.0));

        double u = M_PI / 3.0;
        double v = 2.5;

        // Test various derivative orders.
        for (int Nu = 0; Nu <= 3; ++Nu)
        {
            for (int Nv = 0; Nv <= 3; ++Nv)
            {
                if (Nu + Nv == 0 || Nu + Nv > 3)
                {
                    continue;
                }
                gp_Vec aDN1 = anOffsetAdaptor.DN(u, v, Nu, Nv);
                gp_Vec aDN2 = anEquivAdaptor.DN(u, v, Nu, Nv);
                Assert.True(OffsetSurfaceTestUtils.VectorsEqual(aDN1, aDN2, aTol), $"DN({Nu},{Nv}) mismatch at u={u}, v={v}");
            }
        }
    }

    [Fact]
    public void NegativeOffset_Cylinder_D0D1D2_Consistency()
    {
        // Test with negative offset (smaller cylinder).
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(new Geom_OffsetSurface(aCyl, -3.0));

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 3.0)
        {
            for (double v = -3.0; v <= 3.0; v += 2.0)
            {
                OffsetSurfaceTestUtils.CheckD1(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
            }
        }
    }

    [Fact]
    public void FlippedCylinder_D0D1_Consistency()
    {
        // Test with offset that causes cylinder flip (negative radius).
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 5.0);
        // Results in flipped cylinder with radius 3.
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(new Geom_OffsetSurface(aCyl, -8.0));

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 3.0)
        {
            for (double v = -3.0; v <= 3.0; v += 2.0)
            {
                OffsetSurfaceTestUtils.CheckD1(anOffsetAdaptor, anEquivAdaptor, u, v, aTol);
            }
        }
    }
}

/// <summary>Three-way consistency tests: Direct Geom_OffsetSurface vs OffsetAdaptor vs EquivalentAdaptor.</summary>
public class Geom_OffsetSurface_ThreeWay
{
    private const double aTol = 1e-10;

    [Fact]
    public void Cylinder_D0_ThreeWayConsistency()
    {
        // Verify that direct Geom_OffsetSurface, adaptor from offset surface, and adaptor
        // from equivalent surface all return the same values.
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        Geom_OffsetSurface anOffsetSurf = new(aCyl, 3.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(anOffsetSurf);

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = -5.0; v <= 5.0; v += 2.5)
            {
                OffsetSurfaceTestUtils.CheckThreeWay(anOffsetSurf, anOffsetAdaptor, anEquivAdaptor, u, v, aTol, 0);
            }
        }
    }

    [Fact]
    public void Cylinder_D1_ThreeWayConsistency()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        Geom_OffsetSurface anOffsetSurf = new(aCyl, 3.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(anOffsetSurf);

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = -5.0; v <= 5.0; v += 2.5)
            {
                OffsetSurfaceTestUtils.CheckThreeWay(anOffsetSurf, anOffsetAdaptor, anEquivAdaptor, u, v, aTol, 1);
            }
        }
    }

    [Fact]
    public void Cylinder_D2_ThreeWayConsistency()
    {
        Geom_CylindricalSurface aCyl = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 10.0);
        Geom_OffsetSurface anOffsetSurf = new(aCyl, 3.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(anOffsetSurf);

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = -5.0; v <= 5.0; v += 2.5)
            {
                OffsetSurfaceTestUtils.CheckThreeWay(anOffsetSurf, anOffsetAdaptor, anEquivAdaptor, u, v, aTol, 2);
            }
        }
    }

    [Fact]
    public void Sphere_D0D1D2_ThreeWayConsistency()
    {
        Geom_SphericalSurface aSphere = new(new gp_Ax3(new gp_Pnt(1, 2, 3), new gp_Dir(0, 0, 1)), 10.0);
        Geom_OffsetSurface anOffsetSurf = new(aSphere, 5.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(anOffsetSurf);

        // Avoid poles at v = +/- PI/2.
        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = -M_PI / 3.0; v <= M_PI / 3.0; v += M_PI / 6.0)
            {
                OffsetSurfaceTestUtils.CheckThreeWay(anOffsetSurf, anOffsetAdaptor, anEquivAdaptor, u, v, aTol, 1);
            }
        }
    }

    [Fact]
    public void Plane_D0D1D2_ThreeWayConsistency()
    {
        Geom_Plane aPlane = new(new gp_Ax3(new gp_Pnt(1.0, 2.0, 3.0), new gp_Dir(0, 0, 1)));
        Geom_OffsetSurface anOffsetSurf = new(aPlane, 5.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(anOffsetSurf);

        for (double u = -5.0; u <= 5.0; u += 2.5)
        {
            for (double v = -5.0; v <= 5.0; v += 2.5)
            {
                OffsetSurfaceTestUtils.CheckThreeWay(anOffsetSurf, anOffsetAdaptor, anEquivAdaptor, u, v, aTol, 2);
            }
        }
    }

    [Fact]
    public void Torus_D0D1D2_ThreeWayConsistency()
    {
        Geom_ToroidalSurface aTorus = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 20.0, 5.0);
        Geom_OffsetSurface anOffsetSurf = new(aTorus, 2.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(anOffsetSurf);

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = 0.0; v < 2.0 * M_PI; v += M_PI / 4.0)
            {
                OffsetSurfaceTestUtils.CheckThreeWay(anOffsetSurf, anOffsetAdaptor, anEquivAdaptor, u, v, aTol, 1);
            }
        }
    }

    [Fact]
    public void Cone_D0D1_ThreeWayConsistency()
    {
        Geom_ConicalSurface aCone = new(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), M_PI / 6.0, 10.0);
        Geom_OffsetSurface anOffsetSurf = new(aCone, 2.0);
        var (anOffsetAdaptor, anEquivAdaptor) = OffsetSurfaceTestUtils.Adaptors(anOffsetSurf);

        for (double u = 0.0; u < 2.0 * M_PI; u += M_PI / 4.0)
        {
            for (double v = 1.0; v <= 10.0; v += 3.0) // Avoid apex at v=0.
            {
                OffsetSurfaceTestUtils.CheckThreeWay(anOffsetSurf, anOffsetAdaptor, anEquivAdaptor, u, v, aTol, 1);
            }
        }
    }
}
