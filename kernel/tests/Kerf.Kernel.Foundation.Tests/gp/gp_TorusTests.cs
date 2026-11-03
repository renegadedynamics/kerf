// Port of: src/FoundationClasses/TKMath/GTests/gp_Torus_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

public class gp_TorusTest
{
    // Test OCC26746: gp_Torus::Coefficients() computes correct implicit equation coefficients.
    // Verifies that evaluating the 35-coefficient polynomial at many points on the torus surface
    // gives values close to zero within tolerance.
    // The GTest builds a Geom_ToroidalSurface, which is not ported yet: the test uses the gp_Torus it wraps and
    // evaluates points with TorusD0, a copy of ElSLib::TorusD0 (what Geom_ToroidalSurface::Value calls). OCCT's
    // NCollection_Array1 with lower bound 5 becomes a 35-element array indexed from 0.
    [Fact]
    public void OCC26746_CoefficientsCorrect()
    {
        // Torus constructed from the original Draw script:
        // torus tr 55.52514413 2.070076585 73.83409062
        //          0.37231784651136368 0.58886674834874120 0.71736697293527607
        //          0.80682335496555135 0.17666016102759910 -0.56376170618524390
        //          87.08479625 23.14682176
        CoordinateSystem anAx3 = new(new Point3D(55.52514413, 2.070076585, 73.83409062),
                           new Direction3D(0.37231784651136368, 0.58886674834874120, 0.71736697293527607),
                           new Direction3D(0.80682335496555135, 0.17666016102759910, -0.56376170618524390));

        Torus aTorus = new(anAx3, 87.08479625, 23.14682176);

        const double aTolerance = 3.0e-7;
        const int aNbPtsMax = 5;
        const int aLowIndex = 0;
        const double aStep = 2.0 * M_PI / aNbPtsMax;

        double[] aCoeffs = new double[35];
        aTorus.Coefficients(aCoeffs);

        double aUPar = 0.0;
        for (int aUind = 0; aUind <= aNbPtsMax; aUind++)
        {
            double aVPar = 0.0;
            for (int aVind = 0; aVind <= aNbPtsMax; aVind++)
            {
                Point3D aPt = TorusD0(aUPar, aVPar, aTorus.Position(), aTorus.MajorRadius(), aTorus.MinorRadius());
                double aX1 = aPt.X();
                double aX2 = aX1 * aX1;
                double aX3 = aX2 * aX1;
                double aX4 = aX2 * aX2;
                double aY1 = aPt.Y();
                double aY2 = aY1 * aY1;
                double aY3 = aY2 * aY1;
                double aY4 = aY2 * aY2;
                double aZ1 = aPt.Z();
                double aZ2 = aZ1 * aZ1;
                double aZ3 = aZ2 * aZ1;
                double aZ4 = aZ2 * aZ2;

                int i = aLowIndex;

                double aDelta = aCoeffs[i++] * aX4;
                aDelta += aCoeffs[i++] * aY4;
                aDelta += aCoeffs[i++] * aZ4;
                aDelta += aCoeffs[i++] * aX3 * aY1;
                aDelta += aCoeffs[i++] * aX3 * aZ1;
                aDelta += aCoeffs[i++] * aY3 * aX1;
                aDelta += aCoeffs[i++] * aY3 * aZ1;
                aDelta += aCoeffs[i++] * aZ3 * aX1;
                aDelta += aCoeffs[i++] * aZ3 * aY1;
                aDelta += aCoeffs[i++] * aX2 * aY2;
                aDelta += aCoeffs[i++] * aX2 * aZ2;
                aDelta += aCoeffs[i++] * aY2 * aZ2;
                aDelta += aCoeffs[i++] * aX2 * aY1 * aZ1;
                aDelta += aCoeffs[i++] * aX1 * aY2 * aZ1;
                aDelta += aCoeffs[i++] * aX1 * aY1 * aZ2;
                aDelta += aCoeffs[i++] * aX3;
                aDelta += aCoeffs[i++] * aY3;
                aDelta += aCoeffs[i++] * aZ3;
                aDelta += aCoeffs[i++] * aX2 * aY1;
                aDelta += aCoeffs[i++] * aX2 * aZ1;
                aDelta += aCoeffs[i++] * aY2 * aX1;
                aDelta += aCoeffs[i++] * aY2 * aZ1;
                aDelta += aCoeffs[i++] * aZ2 * aX1;
                aDelta += aCoeffs[i++] * aZ2 * aY1;
                aDelta += aCoeffs[i++] * aX1 * aY1 * aZ1;
                aDelta += aCoeffs[i++] * aX2;
                aDelta += aCoeffs[i++] * aY2;
                aDelta += aCoeffs[i++] * aZ2;
                aDelta += aCoeffs[i++] * aX1 * aY1;
                aDelta += aCoeffs[i++] * aX1 * aZ1;
                aDelta += aCoeffs[i++] * aY1 * aZ1;
                aDelta += aCoeffs[i++] * aX1;
                aDelta += aCoeffs[i++] * aY1;
                aDelta += aCoeffs[i++] * aZ1;
                aDelta += aCoeffs[i++];

                // Torus coefficient equation not satisfied at (u, v)
                Assert.Equal(0.0, aDelta, aTolerance);

                aVPar = (aVind == aNbPtsMax) ? 2.0 * M_PI : aVPar + aStep;
            }

            aVPar = 0.0;
            aUPar = (aUind == aNbPtsMax) ? 2.0 * M_PI : aUPar + aStep;
        }
    }

    // ElSLib::TorusD0, the evaluation behind Geom_ToroidalSurface::Value.
    private static Point3D TorusD0(double U, double V, in CoordinateSystem Pos, double MajorRadius, double MinorRadius)
    {
        Xyz XDir = Pos.XDirection().XYZ();
        Xyz YDir = Pos.YDirection().XYZ();
        Xyz ZDir = Pos.Direction().XYZ();
        Xyz PLoc = Pos.Location().XYZ();
        double R = MajorRadius + MinorRadius * Math.Cos(V);
        double A3 = MinorRadius * Math.Sin(V);
        double A1 = R * Math.Cos(U);
        double A2 = R * Math.Sin(U);
        double eps = 10.0 * (MinorRadius + MajorRadius) * RealEpsilon();

        if (Math.Abs(A1) <= eps)
        {
            A1 = 0.0;
        }

        if (Math.Abs(A2) <= eps)
        {
            A2 = 0.0;
        }

        if (Math.Abs(A3) <= eps)
        {
            A3 = 0.0;
        }

        return new Point3D(A1 * XDir.X() + A2 * YDir.X() + A3 * ZDir.X() + PLoc.X(),
                          A1 * XDir.Y() + A2 * YDir.Y() + A3 * ZDir.Y() + PLoc.Y(),
                          A1 * XDir.Z() + A2 * YDir.Z() + A3 * ZDir.Z() + PLoc.Z());
    }
}
