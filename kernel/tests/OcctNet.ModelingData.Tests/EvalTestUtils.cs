// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Finite-difference checks of curve and surface derivatives, for geometry with no native oracle (GeomEval,
/// Geom2dEval). Each derivative is compared with the central difference of the derivative one order below,
/// and EvalD1/D2/D3 with EvalD0 and EvalDN.
/// </summary>
internal static class EvalTestUtils
{
    /// <summary>The step of the central differences.</summary>
    private const double Step = 1.0e-5;

    /// <summary>The relative tolerance of a finite-difference comparison.</summary>
    private const double FdTol = 1.0e-5;

    /// <summary>The relative tolerance between the EvalD* results and EvalDN.</summary>
    private const double EvalTol = 1.0e-9;

    private static void AssertClose(in gp_XYZ theExpected, in gp_XYZ theActual, double theRelTol, string theWhat)
    {
        double aScale = Math.Max(1.0, theExpected.Modulus());
        double aDiff = (theActual - theExpected).Modulus();
        Assert.True(aDiff <= theRelTol * aScale,
            $"{theWhat}: expected {theExpected}, got {theActual} (difference {aDiff})");
    }

    private static void AssertClose(in gp_XY theExpected, in gp_XY theActual, double theRelTol, string theWhat)
    {
        double aScale = Math.Max(1.0, theExpected.Modulus());
        double aDiff = (theActual - theExpected).Modulus();
        Assert.True(aDiff <= theRelTol * aScale,
            $"{theWhat}: expected {theExpected}, got {theActual} (difference {aDiff})");
    }

    private static gp_XYZ CurveDeriv(Geom_Curve theCurve, double theU, int theN) =>
        theN == 0 ? theCurve.EvalD0(theU).XYZ() : theCurve.EvalDN(theU, theN).XYZ();

    /// <summary>Checks the derivatives of orders 1 to <paramref name="theMaxOrder"/> of a 3D curve at U.</summary>
    public static void CheckCurveDerivatives(Geom_Curve theCurve, double theU, int theMaxOrder = 5)
    {
        for (int aN = 1; aN <= theMaxOrder; ++aN)
        {
            gp_XYZ aFd = (CurveDeriv(theCurve, theU + Step, aN - 1) - CurveDeriv(theCurve, theU - Step, aN - 1))
                         / (2.0 * Step);
            AssertClose(aFd, CurveDeriv(theCurve, theU, aN), FdTol, $"D{aN} at U={theU}");
        }

        Geom_Curve.ResD3 aD3 = theCurve.EvalD3(theU);
        Geom_Curve.ResD2 aD2 = theCurve.EvalD2(theU);
        Geom_Curve.ResD1 aD1 = theCurve.EvalD1(theU);
        gp_XYZ aP = theCurve.EvalD0(theU).XYZ();
        AssertClose(aP, aD1.Point.XYZ(), EvalTol, "EvalD1 point");
        AssertClose(aP, aD2.Point.XYZ(), EvalTol, "EvalD2 point");
        AssertClose(aP, aD3.Point.XYZ(), EvalTol, "EvalD3 point");
        AssertClose(CurveDeriv(theCurve, theU, 1), aD1.D1.XYZ(), EvalTol, "EvalD1 D1");
        AssertClose(CurveDeriv(theCurve, theU, 1), aD2.D1.XYZ(), EvalTol, "EvalD2 D1");
        AssertClose(CurveDeriv(theCurve, theU, 2), aD2.D2.XYZ(), EvalTol, "EvalD2 D2");
        AssertClose(CurveDeriv(theCurve, theU, 1), aD3.D1.XYZ(), EvalTol, "EvalD3 D1");
        AssertClose(CurveDeriv(theCurve, theU, 2), aD3.D2.XYZ(), EvalTol, "EvalD3 D2");
        AssertClose(CurveDeriv(theCurve, theU, 3), aD3.D3.XYZ(), EvalTol, "EvalD3 D3");
    }

    private static gp_XY Curve2dDeriv(Geom2d_Curve theCurve, double theU, int theN) =>
        theN == 0 ? theCurve.EvalD0(theU).XY() : theCurve.EvalDN(theU, theN).XY();

    /// <summary>Checks the derivatives of orders 1 to <paramref name="theMaxOrder"/> of a 2D curve at U.</summary>
    public static void CheckCurveDerivatives(Geom2d_Curve theCurve, double theU, int theMaxOrder = 5)
    {
        for (int aN = 1; aN <= theMaxOrder; ++aN)
        {
            gp_XY aFd = (Curve2dDeriv(theCurve, theU + Step, aN - 1) - Curve2dDeriv(theCurve, theU - Step, aN - 1))
                        / (2.0 * Step);
            AssertClose(aFd, Curve2dDeriv(theCurve, theU, aN), FdTol, $"D{aN} at U={theU}");
        }

        Geom2d_Curve.ResD3 aD3 = theCurve.EvalD3(theU);
        Geom2d_Curve.ResD2 aD2 = theCurve.EvalD2(theU);
        Geom2d_Curve.ResD1 aD1 = theCurve.EvalD1(theU);
        gp_XY aP = theCurve.EvalD0(theU).XY();
        AssertClose(aP, aD1.Point.XY(), EvalTol, "EvalD1 point");
        AssertClose(aP, aD2.Point.XY(), EvalTol, "EvalD2 point");
        AssertClose(aP, aD3.Point.XY(), EvalTol, "EvalD3 point");
        AssertClose(Curve2dDeriv(theCurve, theU, 1), aD1.D1.XY(), EvalTol, "EvalD1 D1");
        AssertClose(Curve2dDeriv(theCurve, theU, 1), aD2.D1.XY(), EvalTol, "EvalD2 D1");
        AssertClose(Curve2dDeriv(theCurve, theU, 2), aD2.D2.XY(), EvalTol, "EvalD2 D2");
        AssertClose(Curve2dDeriv(theCurve, theU, 1), aD3.D1.XY(), EvalTol, "EvalD3 D1");
        AssertClose(Curve2dDeriv(theCurve, theU, 2), aD3.D2.XY(), EvalTol, "EvalD3 D2");
        AssertClose(Curve2dDeriv(theCurve, theU, 3), aD3.D3.XY(), EvalTol, "EvalD3 D3");
    }

    private static gp_XYZ SurfDeriv(Geom_Surface theSurf, double theU, double theV, int theNu, int theNv) =>
        theNu + theNv == 0 ? theSurf.EvalD0(theU, theV).XYZ() : theSurf.EvalDN(theU, theV, theNu, theNv).XYZ();

    /// <summary>
    /// Checks the partial derivatives of a surface at (U, V), up to total order <paramref name="theMaxOrder"/>:
    /// each against the central difference, in U (or in V when its U order is 0), of the one below.
    /// </summary>
    public static void CheckSurfaceDerivatives(Geom_Surface theSurf, double theU, double theV, int theMaxOrder = 4)
    {
        for (int aNu = 0; aNu <= theMaxOrder; ++aNu)
        {
            for (int aNv = 0; aNu + aNv <= theMaxOrder; ++aNv)
            {
                if (aNu + aNv == 0)
                {
                    continue;
                }
                gp_XYZ aFd = aNu > 0
                    ? (SurfDeriv(theSurf, theU + Step, theV, aNu - 1, aNv)
                       - SurfDeriv(theSurf, theU - Step, theV, aNu - 1, aNv)) / (2.0 * Step)
                    : (SurfDeriv(theSurf, theU, theV + Step, aNu, aNv - 1)
                       - SurfDeriv(theSurf, theU, theV - Step, aNu, aNv - 1)) / (2.0 * Step);
                AssertClose(aFd, SurfDeriv(theSurf, theU, theV, aNu, aNv), FdTol,
                    $"D(u{aNu},v{aNv}) at ({theU}, {theV})");
                if (aNu > 0 && aNv > 0)
                {
                    // The mixed derivative also against the difference in V.
                    gp_XYZ aFdV = (SurfDeriv(theSurf, theU, theV + Step, aNu, aNv - 1)
                                   - SurfDeriv(theSurf, theU, theV - Step, aNu, aNv - 1)) / (2.0 * Step);
                    AssertClose(aFdV, SurfDeriv(theSurf, theU, theV, aNu, aNv), FdTol,
                        $"D(u{aNu},v{aNv}) in V at ({theU}, {theV})");
                }
            }
        }

        Geom_Surface.ResD1 aD1 = theSurf.EvalD1(theU, theV);
        Geom_Surface.ResD2 aD2 = theSurf.EvalD2(theU, theV);
        Geom_Surface.ResD3 aD3 = theSurf.EvalD3(theU, theV);
        gp_XYZ aP = theSurf.EvalD0(theU, theV).XYZ();
        AssertClose(aP, aD1.Point.XYZ(), EvalTol, "EvalD1 point");
        AssertClose(aP, aD2.Point.XYZ(), EvalTol, "EvalD2 point");
        AssertClose(aP, aD3.Point.XYZ(), EvalTol, "EvalD3 point");
        AssertClose(SurfDeriv(theSurf, theU, theV, 1, 0), aD1.D1U.XYZ(), EvalTol, "EvalD1 D1U");
        AssertClose(SurfDeriv(theSurf, theU, theV, 0, 1), aD1.D1V.XYZ(), EvalTol, "EvalD1 D1V");
        AssertClose(SurfDeriv(theSurf, theU, theV, 1, 0), aD2.D1U.XYZ(), EvalTol, "EvalD2 D1U");
        AssertClose(SurfDeriv(theSurf, theU, theV, 0, 1), aD2.D1V.XYZ(), EvalTol, "EvalD2 D1V");
        AssertClose(SurfDeriv(theSurf, theU, theV, 2, 0), aD2.D2U.XYZ(), EvalTol, "EvalD2 D2U");
        AssertClose(SurfDeriv(theSurf, theU, theV, 0, 2), aD2.D2V.XYZ(), EvalTol, "EvalD2 D2V");
        AssertClose(SurfDeriv(theSurf, theU, theV, 1, 1), aD2.D2UV.XYZ(), EvalTol, "EvalD2 D2UV");
        AssertClose(SurfDeriv(theSurf, theU, theV, 1, 0), aD3.D1U.XYZ(), EvalTol, "EvalD3 D1U");
        AssertClose(SurfDeriv(theSurf, theU, theV, 0, 1), aD3.D1V.XYZ(), EvalTol, "EvalD3 D1V");
        AssertClose(SurfDeriv(theSurf, theU, theV, 2, 0), aD3.D2U.XYZ(), EvalTol, "EvalD3 D2U");
        AssertClose(SurfDeriv(theSurf, theU, theV, 0, 2), aD3.D2V.XYZ(), EvalTol, "EvalD3 D2V");
        AssertClose(SurfDeriv(theSurf, theU, theV, 1, 1), aD3.D2UV.XYZ(), EvalTol, "EvalD3 D2UV");
        AssertClose(SurfDeriv(theSurf, theU, theV, 3, 0), aD3.D3U.XYZ(), EvalTol, "EvalD3 D3U");
        AssertClose(SurfDeriv(theSurf, theU, theV, 0, 3), aD3.D3V.XYZ(), EvalTol, "EvalD3 D3V");
        AssertClose(SurfDeriv(theSurf, theU, theV, 2, 1), aD3.D3UUV.XYZ(), EvalTol, "EvalD3 D3UUV");
        AssertClose(SurfDeriv(theSurf, theU, theV, 1, 2), aD3.D3UVV.XYZ(), EvalTol, "EvalD3 D3UVV");
    }
}
