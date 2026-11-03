// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// GeomLib_CheckCurveOnSurface's parallel loop (moved onto OSD_Parallel, decision 487): serial, parallel and the
/// serial fallback of a parallel run give the same maximal distance and parameter, bit for bit.
/// </summary>
public class GeomLib_CheckCurveOnSurfaceParallelTest
{
    private static (double Distance, double Parameter) check(bool theIsParallel, bool theIsForcedSerial)
    {
        // A wavy cubic B-spline in the plane z = 0 with many C1 knots (one interval each), and a perturbed pcurve
        const int aNbKnots = 24;
        NCollection_Array1<double> aKnots = new(1, aNbKnots);
        NCollection_Array1<int> aMults = new(1, aNbKnots);
        for (int i = 1; i <= aNbKnots; ++i)
        {
            aKnots[i] = i - 1;
            aMults[i] = i == 1 || i == aNbKnots ? 4 : 2;
        }
        int aNbPoles = 4 + 4 + 2 * (aNbKnots - 2) - 4;
        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        NCollection_Array1<Point2D> aPoles2d = new(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; ++i)
        {
            aPoles[i] = new Point3D(i, Math.Sin(i), 0.0);
            aPoles2d[i] = new Point2D(i, Math.Sin(i) + 0.001 * Math.Cos(3.0 * i));
        }
        Geom_BSplineCurve aCurve = new(aPoles, aKnots, aMults, 3);
        Geom2d_BSplineCurve aPCurve = new(aPoles2d, aKnots, aMults, 3);
        Geom_Plane aPlane = new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)));

        GeomAdaptor_Curve aC3d = new(aCurve);
        Adaptor3d_CurveOnSurface aCoS = new(new Geom2dAdaptor_Curve(aPCurve), new GeomAdaptor_Surface(aPlane));
        GeomLib_CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, global::Kerf.Engine.Foundation.Precision.PConfusion());
        aCheck.SetParallel(theIsParallel);
        using (theIsForcedSerial ? global::Kerf.Engine.Foundation.OsdParallel.ForceSerialScope() : null)
        {
            aCheck.Perform(aCoS);
        }
        Assert.True(aCheck.IsDone());
        return (aCheck.MaxDistance(), aCheck.MaxParameter());
    }

    [Fact]
    public void SerialParallelAndFallbackAgree()
    {
        (double Distance, double Parameter) aSerial = check(false, false);
        Assert.True(aSerial.Distance > 1e-4);
        Assert.Equal(aSerial, check(true, false));
        Assert.Equal(aSerial, check(true, true));
    }
}
