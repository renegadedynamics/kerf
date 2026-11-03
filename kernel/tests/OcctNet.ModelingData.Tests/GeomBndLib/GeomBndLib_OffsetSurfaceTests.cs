// Port of OCCT's src/ModelingData/TKGeomBase/GTests/GeomBndLib_OffsetSurface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;

using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class GeomBndLib_OffsetSurfaceTest
{
    private static readonly double Conf = global::Kerf.Engine.Foundation.Precision.Confusion();

    private static CoordinateSystem XOY3() => GeomBndLibSurfaceTestUtils.XOY3();

    /// <summary>Helper: verify new box is no larger than old box (tighter or equal).</summary>
    private static void ExpectNoLarger(in Box theNew, in Box theOld, double theTol)
    {
        var (aXminN, aXmaxN, aYminN, aYmaxN, aZminN, aZmaxN) = theNew.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO, aZminO, aZmaxO) = theOld.Get();
        GeomBndLibSurfaceTestUtils.ExpectGE(aXminN, aXminO - theTol, "New Xmin is smaller than old");
        GeomBndLibSurfaceTestUtils.ExpectLE(aXmaxN, aXmaxO + theTol, "New Xmax is larger than old");
        GeomBndLibSurfaceTestUtils.ExpectGE(aYminN, aYminO - theTol, "New Ymin is smaller than old");
        GeomBndLibSurfaceTestUtils.ExpectLE(aYmaxN, aYmaxO + theTol, "New Ymax is larger than old");
        GeomBndLibSurfaceTestUtils.ExpectGE(aZminN, aZminO - theTol, "New Zmin is smaller than old");
        GeomBndLibSurfaceTestUtils.ExpectLE(aZmaxN, aZmaxO + theTol, "New Zmax is larger than old");
    }

    /// <summary>Helper: verify box contains all sampled points of the offset surface.</summary>
    private static void ExpectContainsOffsetSurface(in Box theBox,
                                                    OffsetSurface theSurf,
                                                    double theUMin,
                                                    double theUMax,
                                                    double theVMin,
                                                    double theVMax,
                                                    int theNbU,
                                                    int theNbV)
    {
        theBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);
        double aUStep = (theUMax - theUMin) / theNbU;
        double aVStep = (theVMax - theVMin) / theNbV;
        for (int i = 0; i <= theNbU; ++i)
        {
            double aU = theUMin + i * aUStep;
            for (int j = 0; j <= theNbV; ++j)
            {
                double aV = theVMin + j * aVStep;
                Point3D aPnt = theSurf.Value(aU, aV);
                string aWhere = $"Point outside box at U={aU} V={aV}";
                GeomBndLibSurfaceTestUtils.ExpectGE(aPnt.X(), aXmin - Conf, aWhere);
                GeomBndLibSurfaceTestUtils.ExpectLE(aPnt.X(), aXmax + Conf, aWhere);
                GeomBndLibSurfaceTestUtils.ExpectGE(aPnt.Y(), aYmin - Conf, aWhere);
                GeomBndLibSurfaceTestUtils.ExpectLE(aPnt.Y(), aYmax + Conf, aWhere);
                GeomBndLibSurfaceTestUtils.ExpectGE(aPnt.Z(), aZmin - Conf, aWhere);
                GeomBndLibSurfaceTestUtils.ExpectLE(aPnt.Z(), aZmax + Conf, aWhere);
            }
        }
    }

    // =========================================================================
    // Offset sphere (positive)
    // =========================================================================

    [Fact]
    public void Sphere_PositiveOffset_Full_CompareWithBndLib()
    {
        SphericalSurface aSphere = new(XOY3(), 5.0);
        OffsetSurface anOff = new(aSphere, 2.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, 2.0 * M_PI, -M_PI / 2.0, M_PI / 2.0, 36, 18);
    }

    [Fact]
    public void Sphere_PositiveOffset_Patch_CompareWithBndLib()
    {
        SphericalSurface aSphere = new(XOY3(), 5.0);
        OffsetSurface anOff = new(aSphere, 2.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(0.0, M_PI, -M_PI / 4.0, M_PI / 4.0, Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, 0.0, M_PI, -M_PI / 4.0, M_PI / 4.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, M_PI, -M_PI / 4.0, M_PI / 4.0, 18, 9);
    }

    // =========================================================================
    // Offset sphere (negative)
    // =========================================================================

    [Fact]
    public void Sphere_NegativeOffset_Full_CompareWithBndLib()
    {
        SphericalSurface aSphere = new(XOY3(), 10.0);
        OffsetSurface anOff = new(aSphere, -3.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, 2.0 * M_PI, -M_PI / 2.0, M_PI / 2.0, 36, 18);
    }

    // =========================================================================
    // Offset cylinder
    // =========================================================================

    [Fact]
    public void Cylinder_Full_CompareWithBndLib()
    {
        CylindricalSurface aCyl = new(XOY3(), 4.0);
        OffsetSurface anOff = new(aCyl, 1.5);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(0.0, 2.0 * M_PI, -5.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, -5.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, 2.0 * M_PI, -5.0, 5.0, 36, 10);
    }

    [Fact]
    public void Cylinder_Patch_CompareWithBndLib()
    {
        CylindricalSurface aCyl = new(XOY3(), 4.0);
        OffsetSurface anOff = new(aCyl, 1.5);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(0.0, M_PI, 0.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, 0.0, M_PI, 0.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, M_PI, 0.0, 5.0, 18, 5);
    }

    [Fact]
    public void Cylinder_NegativeOffset_CompareWithBndLib()
    {
        CylindricalSurface aCyl = new(XOY3(), 8.0);
        OffsetSurface anOff = new(aCyl, -2.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, 2.0 * M_PI, 0.0, 10.0, 36, 10);
    }

    // =========================================================================
    // Offset plane
    // =========================================================================

    [Fact]
    public void Plane_PositiveOffset_CompareWithBndLib()
    {
        PlaneSurface aPlane = new(XOY3());
        OffsetSurface anOff = new(aPlane, 3.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(-5.0, 5.0, -5.0, 5.0, Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, -5.0, 5.0, -5.0, 5.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, -5.0, 5.0, -5.0, 5.0, 10, 10);
    }

    [Fact]
    public void Plane_PositiveOffset_Patch_IsNotArtificiallyThickenedInZ()
    {
        PlaneSurface aPlane = new(XOY3());
        OffsetSurface anOff = new(aPlane, 3.0);

        Box aBox = new();
        new SurfaceBounds(anOff).Add(-4.0, 6.0, -2.0, 7.0, Conf, ref aBox);

        var (_, _, _, _, aZmin, aZmax) = aBox.Get();
        double aTol = Conf;
        Assert.Equal(3.0 - aTol, aZmin, 1e-12);
        Assert.Equal(3.0 + aTol, aZmax, 1e-12);
    }

    [Fact]
    public void Plane_NegativeOffset_CompareWithBndLib()
    {
        PlaneSurface aPlane = new(XOY3());
        OffsetSurface anOff = new(aPlane, -5.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(-10.0, 10.0, -10.0, 10.0, Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, -10.0, 10.0, -10.0, 10.0, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, -10.0, 10.0, -10.0, 10.0, 10, 10);
    }

    // =========================================================================
    // Offset torus
    // =========================================================================

    [Fact]
    public void Torus_PositiveOffset_CompareWithBndLib()
    {
        ToroidalSurface aTorus = new(XOY3(), 10.0, 3.0);
        OffsetSurface anOff = new(aTorus, 1.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, 36, 36);
    }

    [Fact]
    public void Torus_Patch_CompareWithBndLib()
    {
        ToroidalSurface aTorus = new(XOY3(), 10.0, 3.0);
        OffsetSurface anOff = new(aTorus, 1.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(0.0, M_PI, 0.0, M_PI, Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, 0.0, M_PI, 0.0, M_PI, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, M_PI, 0.0, M_PI, 18, 18);
    }

    // =========================================================================
    // BoxOptimal - plane (analytic equivalent path)
    // =========================================================================

    [Fact]
    public void Plane_BoxOptimal_TighterThanBox()
    {
        PlaneSurface aPlane = new(XOY3());
        OffsetSurface anOff = new(aPlane, 3.0);

        Box aFastBox = new();
        new SurfaceBounds(anOff).Add(-5.0, 5.0, -5.0, 5.0, Conf, ref aFastBox);

        Box aOptBox = new();
        new SurfaceBounds(anOff).AddOptimal(-5.0, 5.0, -5.0, 5.0, Conf, ref aOptBox);

        // Offset plane is a plane - BoxOptimal should be at least as tight as Box.
        ExpectNoLarger(aOptBox, aFastBox, Conf);
        ExpectContainsOffsetSurface(aOptBox, anOff, -5.0, 5.0, -5.0, 5.0, 10, 10);
    }

    // =========================================================================
    // BoxOptimal - torus (PSO fallback path)
    // =========================================================================

    [Fact]
    public void Torus_BoxOptimal_ContainsSurface()
    {
        ToroidalSurface aTorus = new(XOY3(), 10.0, 3.0);
        OffsetSurface anOff = new(aTorus, 1.0);

        Box aFastBox = new();
        new SurfaceBounds(anOff).Add(0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, Conf, ref aFastBox);

        Box aOptBox = new();
        new SurfaceBounds(anOff).AddOptimal(0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, Conf, ref aOptBox);

        // PSO optimal should not exceed the fast box and must contain the actual surface.
        ExpectNoLarger(aOptBox, aFastBox, Conf);
        ExpectContainsOffsetSurface(aOptBox, anOff, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, 36, 36);
    }

    // =========================================================================
    // BoxOptimal - sphere (analytic equivalent: offset sphere -> sphere)
    // =========================================================================

    [Fact]
    public void Sphere_BoxOptimal_ContainsSurface()
    {
        SphericalSurface aSphere = new(XOY3(), 5.0);
        OffsetSurface anOff = new(aSphere, 2.0);

        Box aFastBox = new();
        new SurfaceBounds(anOff).Add(Conf, ref aFastBox);

        Box aOptBox = new();
        new SurfaceBounds(anOff).AddOptimal(Conf, ref aOptBox);

        // Analytic path: offset sphere -> sphere, so BoxOptimal should be at least as tight.
        ExpectNoLarger(aOptBox, aFastBox, Conf);
        ExpectContainsOffsetSurface(aOptBox, anOff, 0.0, 2.0 * M_PI, -M_PI / 2.0, M_PI / 2.0, 36, 18);
    }

    // =========================================================================
    // Large offset
    // =========================================================================

    [Fact]
    public void Sphere_LargeOffset_CompareWithBndLib()
    {
        SphericalSurface aSphere = new(XOY3(), 2.0);
        OffsetSurface anOff = new(aSphere, 20.0);
        SurfaceAdaptor anAdaptor = new(anOff);

        Box aNewBox = new();
        new SurfaceBounds(anOff).Add(Conf, ref aNewBox);

        Box anOldBox = new();
        AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        ExpectNoLarger(aNewBox, anOldBox, Conf);
        ExpectContainsOffsetSurface(aNewBox, anOff, 0.0, 2.0 * M_PI, -M_PI / 2.0, M_PI / 2.0, 36, 18);
    }
}
