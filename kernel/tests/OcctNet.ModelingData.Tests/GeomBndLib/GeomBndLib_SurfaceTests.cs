// Port of OCCT's src/ModelingData/TKGeomBase/GTests/GeomBndLib_Surface_Test.cxx (V8_0_1).
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_SurfaceType;

namespace OcctNet.ModelingData.Tests;

/// <summary>Assertions shared by the GeomBndLib surface tests (EXPECT_GE/EXPECT_LE and the GTests' CompareBoxes).</summary>
internal static class GeomBndLibSurfaceTestUtils
{
    internal static gp_Ax3 XOY3() => new(gp.XOY());

    internal static void ExpectGE(double theA, double theB, string theWhat = "")
        => Assert.True(theA >= theB, $"{theWhat}: expected {theA:R} >= {theB:R}");

    internal static void ExpectLE(double theA, double theB, string theWhat = "")
        => Assert.True(theA <= theB, $"{theWhat}: expected {theA:R} <= {theB:R}");

    /// <summary>Helper: compare two Bnd_Box results within tolerance.</summary>
    internal static void CompareBoxes(in Bnd_Box theNew, in Bnd_Box theOld, double theTol)
    {
        var (aXminN, aXmaxN, aYminN, aYmaxN, aZminN, aZmaxN) = theNew.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO, aZminO, aZmaxO) = theOld.Get();
        Assert.Equal(aXminO, aXminN, theTol);
        Assert.Equal(aXmaxO, aXmaxN, theTol);
        Assert.Equal(aYminO, aYminN, theTol);
        Assert.Equal(aYmaxO, aYmaxN, theTol);
        Assert.Equal(aZminO, aZminN, theTol);
        Assert.Equal(aZmaxO, aZmaxN, theTol);
    }

    /// <summary>Helper: verify new box is no larger than old box (tighter or equal).</summary>
    internal static void ExpectNoLarger(in Bnd_Box theNew, in Bnd_Box theOld, double theTol)
    {
        var (aXminN, aXmaxN, aYminN, aYmaxN, aZminN, aZmaxN) = theNew.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO, aZminO, aZmaxO) = theOld.Get();
        ExpectGE(aXminN, aXminO - theTol, "New Xmin is smaller than old");
        ExpectLE(aXmaxN, aXmaxO + theTol, "New Xmax is larger than old");
        ExpectGE(aYminN, aYminO - theTol, "New Ymin is smaller than old");
        ExpectLE(aYmaxN, aYmaxO + theTol, "New Ymax is larger than old");
        ExpectGE(aZminN, aZminO - theTol, "New Zmin is smaller than old");
        ExpectLE(aZmaxN, aZmaxO + theTol, "New Zmax is larger than old");
    }

    /// <summary>Helper: verify box contains all sampled points of the surface.</summary>
    internal static void ExpectContainsSurface(in Bnd_Box theBox,
                                               Geom_Surface theSurf,
                                               double theUMin,
                                               double theUMax,
                                               double theVMin,
                                               double theVMax,
                                               int theNbU,
                                               int theNbV)
    {
        double aConf = Precision.Confusion();
        theBox.Get(out double aXmin, out double aYmin, out double aZmin, out double aXmax, out double aYmax, out double aZmax);
        double aUStep = (theUMax - theUMin) / theNbU;
        double aVStep = (theVMax - theVMin) / theNbV;
        for (int i = 0; i <= theNbU; ++i)
        {
            double aU = theUMin + i * aUStep;
            for (int j = 0; j <= theNbV; ++j)
            {
                double aV = theVMin + j * aVStep;
                gp_Pnt aPnt = theSurf.Value(aU, aV);
                string aWhere = $"Point outside box at U={aU} V={aV}";
                ExpectGE(aPnt.X(), aXmin - aConf, aWhere);
                ExpectLE(aPnt.X(), aXmax + aConf, aWhere);
                ExpectGE(aPnt.Y(), aYmin - aConf, aWhere);
                ExpectLE(aPnt.Y(), aYmax + aConf, aWhere);
                ExpectGE(aPnt.Z(), aZmin - aConf, aWhere);
                ExpectLE(aPnt.Z(), aZmax + aConf, aWhere);
            }
        }
    }
}

public class GeomBndLib_SurfaceTest
{
    private static readonly double Conf = Precision.Confusion();

    private static gp_Ax3 XOY3() => GeomBndLibSurfaceTestUtils.XOY3();

    private static void CompareBoxes(in Bnd_Box theNew, in Bnd_Box theOld, double theTol)
        => GeomBndLibSurfaceTestUtils.CompareBoxes(theNew, theOld, theTol);

    private static void ExpectGE(double theA, double theB, string theWhat = "") => GeomBndLibSurfaceTestUtils.ExpectGE(theA, theB, theWhat);

    private static void ExpectLE(double theA, double theB, string theWhat = "") => GeomBndLibSurfaceTestUtils.ExpectLE(theA, theB, theWhat);

    // =========================================================================
    // Plane
    // =========================================================================

    [Fact]
    public void Plane_FinitePatch()
    {
        Geom_Plane aPlane = new(XOY3());
        GeomBndLib_Surface aSurf = new(aPlane);
        Assert.Equal(GeomAbs_Plane, aSurf.GetType());

        Bnd_Box aBox = new();
        aSurf.Add(-5.0, 5.0, -3.0, 3.0, 0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        Assert.Equal(-5.0, aXmin, Conf);
        Assert.Equal(-3.0, aYmin, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(5.0, aXmax, Conf);
        Assert.Equal(3.0, aYmax, Conf);
        Assert.Equal(0.0, aZmax, Conf);
    }

    [Fact]
    public void Plane_CompareWithBndLib()
    {
        Geom_Plane aPlane = new(XOY3());
        GeomAdaptor_Surface anAdaptor = new(aPlane);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aPlane).Add(-5.0, 5.0, -3.0, 3.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, -5.0, 5.0, -3.0, 3.0, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Cylinder
    // =========================================================================

    [Fact]
    public void Cylinder_Patch()
    {
        double aRadius = 3.0;
        Geom_CylindricalSurface aCyl = new(XOY3(), aRadius);
        GeomBndLib_Surface aSurf = new(aCyl);
        Assert.Equal(GeomAbs_Cylinder, aSurf.GetType());

        Bnd_Box aBox = new();
        aSurf.Add(0.0, 2.0 * M_PI, 0.0, 10.0, Conf, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        Assert.Equal(-aRadius, aXmin, Conf);
        Assert.Equal(-aRadius, aYmin, Conf);
        Assert.Equal(0.0, aZmin, Conf);
        Assert.Equal(aRadius, aXmax, Conf);
        Assert.Equal(aRadius, aYmax, Conf);
        Assert.Equal(10.0, aZmax, Conf);
    }

    [Fact]
    public void Cylinder_CompareWithBndLib()
    {
        Geom_CylindricalSurface aCyl = new(XOY3(), 4.0);
        GeomAdaptor_Surface anAdaptor = new(aCyl);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aCyl).Add(0.0, 2.0 * M_PI, -5.0, 5.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, -5.0, 5.0, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Cone
    // =========================================================================

    [Fact]
    public void Cone_Patch()
    {
        double aRefRadius = 2.0;
        double aSemiAngle = M_PI / 4.0;
        gp_Cone aCone = new(XOY3(), aSemiAngle, aRefRadius);
        Geom_ConicalSurface aConeSurf = new(aCone);
        GeomBndLib_Surface aSurf = new(aConeSurf);
        Assert.Equal(GeomAbs_Cone, aSurf.GetType());

        Bnd_Box aBox = new();
        aSurf.Add(0.0, 2.0 * M_PI, 0.0, 5.0, Conf, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, _, _) = aBox.Get();

        // At V=5, radius = aRefRadius + 5*sin(PI/4)
        double aMaxR = aRefRadius + 5.0 * Math.Sin(aSemiAngle);

        ExpectGE(aXmax, aMaxR - Conf);
        ExpectGE(aYmax, aMaxR - Conf);
        ExpectLE(aXmin, -aMaxR + Conf);
        ExpectLE(aYmin, -aMaxR + Conf);
    }

    [Fact]
    public void Cone_CompareWithBndLib()
    {
        gp_Cone aCone = new(XOY3(), M_PI / 6.0, 3.0);
        Geom_ConicalSurface aConeSurf = new(aCone);
        GeomAdaptor_Surface anAdaptor = new(aConeSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aConeSurf).Add(0.0, 2.0 * M_PI, 0.0, 5.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, 2.0 * M_PI, 0.0, 5.0, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Sphere
    // =========================================================================

    [Fact]
    public void Sphere_Full()
    {
        double aRadius = 5.0;
        Geom_SphericalSurface aSphere = new(XOY3(), aRadius);
        GeomBndLib_Surface aSurf = new(aSphere);
        Assert.Equal(GeomAbs_Sphere, aSurf.GetType());

        Bnd_Box aBox = new();
        aSurf.Add(0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        Assert.Equal(-aRadius, aXmin, Conf);
        Assert.Equal(-aRadius, aYmin, Conf);
        Assert.Equal(-aRadius, aZmin, Conf);
        Assert.Equal(aRadius, aXmax, Conf);
        Assert.Equal(aRadius, aYmax, Conf);
        Assert.Equal(aRadius, aZmax, Conf);
    }

    [Fact]
    public void Sphere_Patch()
    {
        double aRadius = 5.0;
        Geom_SphericalSurface aSphere = new(XOY3(), aRadius);
        GeomBndLib_Surface aSurf = new(aSphere);

        Bnd_Box aBox = new();
        // U in [0, PI/2], V in [0, PI/2] (first octant patch)
        aSurf.Add(0.0, M_PI / 2.0, 0.0, M_PI / 2.0, Conf, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        // Patch should be in the positive octant
        ExpectGE(aXmin, -Conf);
        ExpectGE(aYmin, -Conf);
        ExpectGE(aZmin, -Conf);
        ExpectLE(aXmax, aRadius + Conf);
        ExpectLE(aYmax, aRadius + Conf);
        ExpectLE(aZmax, aRadius + Conf);
    }

    [Fact]
    public void Sphere_CompareWithBndLib()
    {
        Geom_SphericalSurface aSphere = new(XOY3(), 7.0);
        GeomAdaptor_Surface anAdaptor = new(aSphere);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aSphere).Add(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void Sphere_Patch_CompareWithBndLib()
    {
        Geom_SphericalSurface aSphere = new(XOY3(), 7.0);
        GeomAdaptor_Surface anAdaptor = new(aSphere);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aSphere).Add(0.0, M_PI, -M_PI / 4.0, M_PI / 4.0, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, M_PI, -M_PI / 4.0, M_PI / 4.0, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Torus
    // =========================================================================

    [Fact]
    public void Torus_Full()
    {
        double aMajR = 10.0;
        double aMinR = 3.0;
        Geom_ToroidalSurface aTorus = new(XOY3(), aMajR, aMinR);
        GeomBndLib_Surface aSurf = new(aTorus);
        Assert.Equal(GeomAbs_Torus, aSurf.GetType());

        Bnd_Box aBox = new();
        aSurf.Add(0.0, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        double aOuterR = aMajR + aMinR;
        Assert.Equal(-aOuterR, aXmin, Conf);
        Assert.Equal(-aOuterR, aYmin, Conf);
        Assert.Equal(-aMinR, aZmin, Conf);
        Assert.Equal(aOuterR, aXmax, Conf);
        Assert.Equal(aOuterR, aYmax, Conf);
        Assert.Equal(aMinR, aZmax, Conf);
    }

    private static void ExpectNoLarger(in Bnd_Box theNew, in Bnd_Box theOld)
    {
        var (aXminN, aXmaxN, aYminN, aYmaxN, aZminN, aZmaxN) = theNew.Get();
        var (aXminO, aXmaxO, aYminO, aYmaxO, aZminO, aZmaxO) = theOld.Get();
        ExpectGE(aXminN, aXminO - Conf, "Xmin");
        ExpectLE(aXmaxN, aXmaxO + Conf, "Xmax");
        ExpectGE(aYminN, aYminO - Conf, "Ymin");
        ExpectLE(aYmaxN, aYmaxO + Conf, "Ymax");
        ExpectGE(aZminN, aZminO - Conf, "Zmin");
        ExpectLE(aZmaxN, aZmaxO + Conf, "Zmax");
    }

    [Fact]
    public void Torus_CompareWithBndLib()
    {
        Geom_ToroidalSurface aTorus = new(XOY3(), 10.0, 3.0);
        GeomAdaptor_Surface anAdaptor = new(aTorus);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aTorus).Add(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        // New implementation produces a tighter (better) box than old BndLib for full torus.
        // Verify new box is no larger than old box.
        ExpectNoLarger(aNewBox, anOldBox);
    }

    [Fact]
    public void Torus_Patch_CompareWithBndLib()
    {
        Geom_ToroidalSurface aTorus = new(XOY3(), 10.0, 3.0);
        GeomAdaptor_Surface anAdaptor = new(aTorus);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aTorus).Add(0.0, M_PI, 0.0, M_PI, Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, 0.0, M_PI, 0.0, M_PI, Conf, ref anOldBox);

        // New implementation produces a tighter (better) box than old BndLib for torus patches.
        // Verify new box is no larger than old box.
        ExpectNoLarger(aNewBox, anOldBox);
    }

    // =========================================================================
    // BezierSurface
    // =========================================================================

    private static Geom_BezierSurface SimpleBezier()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 2, 1, 2);
        aPoles.SetValue(1, 1, new gp_Pnt(0.0, 0.0, 0.0));
        aPoles.SetValue(1, 2, new gp_Pnt(10.0, 0.0, 0.0));
        aPoles.SetValue(2, 1, new gp_Pnt(0.0, 10.0, 0.0));
        aPoles.SetValue(2, 2, new gp_Pnt(10.0, 10.0, 5.0));
        return new Geom_BezierSurface(aPoles);
    }

    [Fact]
    public void BezierSurface_Simple()
    {
        Geom_BezierSurface aBezSurf = SimpleBezier();
        GeomBndLib_Surface aSurf = new(aBezSurf);
        Assert.Equal(GeomAbs_BezierSurface, aSurf.GetType());

        Bnd_Box aBox = new();
        aSurf.Add(Conf, ref aBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aBox.Get();

        ExpectLE(aXmin, Conf);
        ExpectGE(aXmax, 10.0 - Conf);
        ExpectLE(aYmin, Conf);
        ExpectGE(aYmax, 10.0 - Conf);
        ExpectLE(aZmin, Conf);
        ExpectGE(aZmax, 5.0 - Conf);
    }

    [Fact]
    public void BezierSurface_CompareWithBndLib()
    {
        Geom_BezierSurface aBezSurf = SimpleBezier();
        GeomAdaptor_Surface anAdaptor = new(aBezSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aBezSurf).Add(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // BSplineSurface
    // =========================================================================

    [Fact]
    public void BSplineSurface_CompareWithBndLib()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 3, 1, 3);
        aPoles.SetValue(1, 1, new gp_Pnt(0.0, 0.0, 0.0));
        aPoles.SetValue(1, 2, new gp_Pnt(5.0, 0.0, 2.0));
        aPoles.SetValue(1, 3, new gp_Pnt(10.0, 0.0, 0.0));
        aPoles.SetValue(2, 1, new gp_Pnt(0.0, 5.0, 1.0));
        aPoles.SetValue(2, 2, new gp_Pnt(5.0, 5.0, 5.0));
        aPoles.SetValue(2, 3, new gp_Pnt(10.0, 5.0, 1.0));
        aPoles.SetValue(3, 1, new gp_Pnt(0.0, 10.0, 0.0));
        aPoles.SetValue(3, 2, new gp_Pnt(5.0, 10.0, 2.0));
        aPoles.SetValue(3, 3, new gp_Pnt(10.0, 10.0, 0.0));

        NCollection_Array1<double> aUKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        NCollection_Array1<double> aVKnots = NCollection_Array1<double>.From([0.0, 1.0]);
        NCollection_Array1<int> aUMults = NCollection_Array1<int>.From([3, 3]);
        NCollection_Array1<int> aVMults = NCollection_Array1<int>.From([3, 3]);

        Geom_BSplineSurface aBSplSurf = new(aPoles, aUKnots, aVKnots, aUMults, aVMults, 2, 2);
        GeomAdaptor_Surface anAdaptor = new(aBSplSurf);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aBSplSurf).Add(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    // =========================================================================
    // Adaptor constructor
    // =========================================================================

    [Fact]
    public void AdaptorConstructor_Sphere()
    {
        double aRadius = 5.0;
        Geom_SphericalSurface aSphere = new(XOY3(), aRadius);

        // Construct from handle
        GeomBndLib_Surface aSurfH = new(aSphere);
        Bnd_Box aBoxH = new();
        aSurfH.Add(Conf, ref aBoxH);

        // Construct from adaptor
        GeomAdaptor_Surface anAdaptor = new(aSphere);
        GeomBndLib_Surface aSurfA = new(anAdaptor);
        Assert.Equal(GeomAbs_Sphere, aSurfA.GetType());

        Bnd_Box aBoxA = new();
        aSurfA.Add(Conf, ref aBoxA);

        CompareBoxes(aBoxH, aBoxA, Conf);
    }

    // =========================================================================
    // AddOptimal
    // =========================================================================

    [Fact]
    public void AddOptimal_Sphere()
    {
        double aRadius = 5.0;
        Geom_SphericalSurface aSphere = new(XOY3(), aRadius);
        GeomBndLib_Surface aSurf = new(aSphere);

        Bnd_Box aBox = new();
        aSurf.Add(Conf, ref aBox);

        Bnd_Box aBoxOpt = new();
        aSurf.AddOptimal(Conf, ref aBoxOpt);

        // AddOptimal should produce a box no larger than Add
        ExpectNoLarger(aBoxOpt, aBox);
    }

    [Fact]
    public void AddOptimal_CompareWithBndLib()
    {
        Geom_SphericalSurface aSphere = new(XOY3(), 7.0);
        GeomAdaptor_Surface anAdaptor = new(aSphere);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aSphere).AddOptimal(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.AddOptimal(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void AddOptimal_Torus_TiltedBetterThanFast()
    {
        // Tilted torus: axis is (1,0,1)/sqrt(2), Ra=8, Rmi=3.
        // For a tilted torus the PI/4 circle-sampling used by Box() misses the true
        // extremum, so BoxOptimal (PSO) must give a tighter result.
        // Analytic: for axis n=(1/sqrt(2), 0, 1/sqrt(2)),
        //   Xmax = Zmax = Ra/sqrt(2) + Rmi = 4*sqrt(2)+3 ~= 8.657
        //   Ymax = Ra + Rmi = 11  (ny=0, so full equatorial radius applies)
        gp_Ax3 aTiltedAx = new(gp.Origin(), new gp_Dir(1.0, 0.0, 1.0));
        Geom_ToroidalSurface aTorus = new(aTiltedAx, 8.0, 3.0);

        Bnd_Box aFastBox = new();
        new GeomBndLib_Surface(aTorus).Add(Conf, ref aFastBox);

        Bnd_Box aOptBox = new();
        new GeomBndLib_Surface(aTorus).AddOptimal(Conf, ref aOptBox);

        // Optimal must be no larger than the fast (conservative) box.
        ExpectNoLarger(aOptBox, aFastBox);

        var (aXminO, aXmaxO, aYminO, aYmaxO, aZminO, aZmaxO) = aOptBox.Get();

        // PSO must converge to the analytic tight bound.
        double aTrueXZ = 8.0 / Math.Sqrt(2.0) + 3.0; // ~= 8.6569
        double aTrueY = 8.0 + 3.0;                  // = 11.0
        double aTol = 10.0 * Conf;                  // PSO converges to ~Confusion()
        Assert.Equal(aTrueXZ, aXmaxO, aTol);
        Assert.Equal(aTrueXZ, -aXminO, aTol);
        Assert.Equal(aTrueY, aYmaxO, aTol);
        Assert.Equal(aTrueY, -aYminO, aTol);
        Assert.Equal(aTrueXZ, aZmaxO, aTol);
        Assert.Equal(aTrueXZ, -aZminO, aTol);
    }

    [Fact]
    public void AdaptorConstructor_TrimmedRangeUsedForFullAdd()
    {
        Geom_SphericalSurface aSphere = new(XOY3(), 7.0);
        GeomAdaptor_Surface anAdaptor = new(aSphere, 0.0, M_PI, -M_PI / 4.0, M_PI / 4.0);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(anAdaptor).Add(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void TrimmedSurfaceHandle_Fallback_CompareWithBndLib()
    {
        Geom_SphericalSurface aSphere = new(XOY3(), 7.0);
        Geom_RectangularTrimmedSurface aTrim = new(aSphere, 0.0, M_PI, -M_PI / 4.0, M_PI / 4.0);
        GeomAdaptor_Surface anAdaptor = new(aTrim);

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aTrim).Add(Conf, ref aNewBox);

        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, Conf, ref anOldBox);

        Assert.False(aNewBox.IsVoid());
        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void RectangularTrimmedSurface_UsesBasisSpecialization()
    {
        Geom_SphericalSurface aSphere = new(XOY3(), 7.0);
        Geom_RectangularTrimmedSurface aTrim = new(aSphere, 0.0, M_PI, -M_PI / 4.0, M_PI / 4.0);

        GeomBndLib_Surface aSurf = new(aTrim);
        Assert.Equal(GeomAbs_Sphere, aSurf.GetType());

        GeomAdaptor_Surface anAdaptor = new(aTrim);
        Bnd_Box aNewBox = new();
        aSurf.Add(Conf, ref aNewBox);
        Bnd_Box anOldBox = new();
        BndLib_AddSurface.Add(anAdaptor, Conf, ref anOldBox);
        CompareBoxes(aNewBox, anOldBox, Conf);
    }

    [Fact]
    public void Torus_NegativeVMin_NoUnderBounding()
    {
        Geom_ToroidalSurface aTorus = new(XOY3(), 10.0, 3.0);
        double aU1 = 0.0;
        double aU2 = 2.0 * M_PI;
        double aV1 = -M_PI / 8.0;
        double aV2 = M_PI / 8.0;

        Bnd_Box aNewBox = new();
        new GeomBndLib_Surface(aTorus).Add(aU1, aU2, aV1, aV2, Conf, ref aNewBox);

        var (aXmin, aXmax, aYmin, aYmax, aZmin, aZmax) = aNewBox.Get();
        for (int i = 0; i <= 72; ++i)
        {
            double aU = aU1 + (aU2 - aU1) * i / 72.0;
            for (int j = 0; j <= 16; ++j)
            {
                double aV = aV1 + (aV2 - aV1) * j / 16.0;
                gp_Pnt aP = aTorus.Value(aU, aV);
                ExpectGE(aP.X(), aXmin - Conf);
                ExpectLE(aP.X(), aXmax + Conf);
                ExpectGE(aP.Y(), aYmin - Conf);
                ExpectLE(aP.Y(), aYmax + Conf);
                ExpectGE(aP.Z(), aZmin - Conf);
                ExpectLE(aP.Z(), aZmax + Conf);
            }
        }
    }
}
