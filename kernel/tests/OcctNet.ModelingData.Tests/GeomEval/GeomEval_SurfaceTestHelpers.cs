// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Component-wise EXPECT_NEAR on points and vectors, and stand-ins for unported Geom surfaces, for the ported
/// GeomEval surface GTests.
/// </summary>
internal static class GeomEval_SurfaceTestHelpers
{
    public static void ExpectNear(in Vector3D theActual, in Vector3D theExpected, double theTol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    public static void ExpectNear(in Point3D theActual, in Point3D theExpected, double theTol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    /// <summary>The implicit quadric equation, A1*X^2 + ... + D, at P.</summary>
    public static double Quadric(in Point3D theP, double aA1, double aA2, double aA3, double aB1, double aB2,
                                 double aB3, double aC1, double aC2, double aC3, double aD)
    {
        double aX = theP.X(), aY = theP.Y(), aZ = theP.Z();
        return aA1 * aX * aX + aA2 * aY * aY + aA3 * aZ * aZ
               + 2.0 * (aB1 * aX * aY + aB2 * aX * aZ + aB3 * aY * aZ)
               + 2.0 * (aC1 * aX + aC2 * aY + aC3 * aZ) + aD;
    }

    // Geom_SphericalSurface (unit 1C) evaluates through ElSLib; these are its EvalD0 to EvalD3.
    public static Point3D SphereEvalD0(in CoordinateSystem thePos, double theR, double theU, double theV)
    {
        global::Kerf.Engine.Geometry.ElSLib.SphereD0(theU, theV, thePos, theR, out Point3D aP);
        return aP;
    }

    public static Surface.ResD1 SphereEvalD1(in CoordinateSystem thePos, double theR, double theU, double theV)
    {
        Surface.ResD1 aR;
        global::Kerf.Engine.Geometry.ElSLib.SphereD1(theU, theV, thePos, theR, out aR.Point, out aR.D1U, out aR.D1V);
        return aR;
    }

    public static Surface.ResD2 SphereEvalD2(in CoordinateSystem thePos, double theR, double theU, double theV)
    {
        Surface.ResD2 aR;
        global::Kerf.Engine.Geometry.ElSLib.SphereD2(theU, theV, thePos, theR, out aR.Point, out aR.D1U, out aR.D1V,
                        out aR.D2U, out aR.D2V, out aR.D2UV);
        return aR;
    }

    public static Surface.ResD3 SphereEvalD3(in CoordinateSystem thePos, double theR, double theU, double theV)
    {
        Surface.ResD3 aR;
        global::Kerf.Engine.Geometry.ElSLib.SphereD3(theU, theV, thePos, theR, out aR.Point, out aR.D1U, out aR.D1V,
                        out aR.D2U, out aR.D2V, out aR.D2UV, out aR.D3U, out aR.D3V, out aR.D3UUV, out aR.D3UVV);
        return aR;
    }
}
