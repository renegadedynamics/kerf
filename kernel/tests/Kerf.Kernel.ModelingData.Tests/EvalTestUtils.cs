// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

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

    private static void AssertClose(in Xyz theExpected, in Xyz theActual, double theRelTol, string theWhat)
    {
        double aScale = Math.Max(1.0, theExpected.Modulus());
        double aDiff = (theActual - theExpected).Modulus();
        Assert.True(aDiff <= theRelTol * aScale,
            $"{theWhat}: expected {theExpected}, got {theActual} (difference {aDiff})");
    }

    private static void AssertClose(in Xy theExpected, in Xy theActual, double theRelTol, string theWhat)
    {
        double aScale = Math.Max(1.0, theExpected.Modulus());
        double aDiff = (theActual - theExpected).Modulus();
        Assert.True(aDiff <= theRelTol * aScale,
            $"{theWhat}: expected {theExpected}, got {theActual} (difference {aDiff})");
    }

    private static Xyz CurveDeriv(Curve3D theCurve, double theU, int theN) =>
        theN == 0 ? theCurve.EvalD0(theU).XYZ() : theCurve.EvalDN(theU, theN).XYZ();

    /// <summary>Checks the derivatives of orders 1 to <paramref name="theMaxOrder"/> of a 3D curve at U.</summary>
    public static void CheckCurveDerivatives(Curve3D theCurve, double theU, int theMaxOrder = 5)
    {
        for (int aN = 1; aN <= theMaxOrder; ++aN)
        {
            Xyz aFd = (CurveDeriv(theCurve, theU + Step, aN - 1) - CurveDeriv(theCurve, theU - Step, aN - 1))
                         / (2.0 * Step);
            AssertClose(aFd, CurveDeriv(theCurve, theU, aN), FdTol, $"D{aN} at U={theU}");
        }

        Curve3D.ResD3 aD3 = theCurve.EvalD3(theU);
        Curve3D.ResD2 aD2 = theCurve.EvalD2(theU);
        Curve3D.ResD1 aD1 = theCurve.EvalD1(theU);
        Xyz aP = theCurve.EvalD0(theU).XYZ();
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

    private static Xy Curve2dDeriv(Curve2D theCurve, double theU, int theN) =>
        theN == 0 ? theCurve.EvalD0(theU).XY() : theCurve.EvalDN(theU, theN).XY();

    /// <summary>Checks the derivatives of orders 1 to <paramref name="theMaxOrder"/> of a 2D curve at U.</summary>
    public static void CheckCurveDerivatives(Curve2D theCurve, double theU, int theMaxOrder = 5)
    {
        for (int aN = 1; aN <= theMaxOrder; ++aN)
        {
            Xy aFd = (Curve2dDeriv(theCurve, theU + Step, aN - 1) - Curve2dDeriv(theCurve, theU - Step, aN - 1))
                        / (2.0 * Step);
            AssertClose(aFd, Curve2dDeriv(theCurve, theU, aN), FdTol, $"D{aN} at U={theU}");
        }

        Curve2D.ResD3 aD3 = theCurve.EvalD3(theU);
        Curve2D.ResD2 aD2 = theCurve.EvalD2(theU);
        Curve2D.ResD1 aD1 = theCurve.EvalD1(theU);
        Xy aP = theCurve.EvalD0(theU).XY();
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

    private static Xyz SurfDeriv(Surface theSurf, double theU, double theV, int theNu, int theNv) =>
        theNu + theNv == 0 ? theSurf.EvalD0(theU, theV).XYZ() : theSurf.EvalDN(theU, theV, theNu, theNv).XYZ();

    /// <summary>
    /// Checks the partial derivatives of a surface at (U, V), up to total order <paramref name="theMaxOrder"/>:
    /// each against the central difference, in U (or in V when its U order is 0), of the one below.
    /// </summary>
    public static void CheckSurfaceDerivatives(Surface theSurf, double theU, double theV, int theMaxOrder = 4)
    {
        for (int aNu = 0; aNu <= theMaxOrder; ++aNu)
        {
            for (int aNv = 0; aNu + aNv <= theMaxOrder; ++aNv)
            {
                if (aNu + aNv == 0)
                {
                    continue;
                }
                Xyz aFd = aNu > 0
                    ? (SurfDeriv(theSurf, theU + Step, theV, aNu - 1, aNv)
                       - SurfDeriv(theSurf, theU - Step, theV, aNu - 1, aNv)) / (2.0 * Step)
                    : (SurfDeriv(theSurf, theU, theV + Step, aNu, aNv - 1)
                       - SurfDeriv(theSurf, theU, theV - Step, aNu, aNv - 1)) / (2.0 * Step);
                AssertClose(aFd, SurfDeriv(theSurf, theU, theV, aNu, aNv), FdTol,
                    $"D(u{aNu},v{aNv}) at ({theU}, {theV})");
                if (aNu > 0 && aNv > 0)
                {
                    // The mixed derivative also against the difference in V.
                    Xyz aFdV = (SurfDeriv(theSurf, theU, theV + Step, aNu, aNv - 1)
                                   - SurfDeriv(theSurf, theU, theV - Step, aNu, aNv - 1)) / (2.0 * Step);
                    AssertClose(aFdV, SurfDeriv(theSurf, theU, theV, aNu, aNv), FdTol,
                        $"D(u{aNu},v{aNv}) in V at ({theU}, {theV})");
                }
            }
        }

        Surface.ResD1 aD1 = theSurf.EvalD1(theU, theV);
        Surface.ResD2 aD2 = theSurf.EvalD2(theU, theV);
        Surface.ResD3 aD3 = theSurf.EvalD3(theU, theV);
        Xyz aP = theSurf.EvalD0(theU, theV).XYZ();
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
