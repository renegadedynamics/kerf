// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>Component-wise EXPECT_NEAR on points and vectors, for the ported GeomEval surface GTests.</summary>
internal static class GeomEval_SurfaceTestHelpers
{
    public static void ExpectNear(in gp_Vec theActual, in gp_Vec theExpected, double theTol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    public static void ExpectNear(in gp_Pnt theActual, in gp_Pnt theExpected, double theTol)
    {
        Assert.Equal(theExpected.X(), theActual.X(), theTol);
        Assert.Equal(theExpected.Y(), theActual.Y(), theTol);
        Assert.Equal(theExpected.Z(), theActual.Z(), theTol);
    }

    /// <summary>The implicit quadric equation, A1*X^2 + ... + D, at P.</summary>
    public static double Quadric(in gp_Pnt theP, double aA1, double aA2, double aA3, double aB1, double aB2,
                                 double aB3, double aC1, double aC2, double aC3, double aD)
    {
        double aX = theP.X(), aY = theP.Y(), aZ = theP.Z();
        return aA1 * aX * aX + aA2 * aY * aY + aA3 * aZ * aZ
               + 2.0 * (aB1 * aX * aY + aB2 * aX * aZ + aB3 * aY * aZ)
               + 2.0 * (aC1 * aX + aC2 * aY + aC3 * aZ) + aD;
    }
}
