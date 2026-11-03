// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Extrema_ExtCS on the paths solved analytically (Extrema_ExtElCS), with the extremity fallback, the periodic
/// adjustment of parameters and the status checks. Every extremum is checked against the geometry: its points lie
/// on the curve and the surface at its parameters and its square distance is theirs. The generic paths need unit
/// 4A's Extrema_GenExtCS and Extrema_ExtPS.
/// </summary>
public class Extrema_ExtCSTests
{
    private const double Tol = 1e-9;

    private static void CheckSolutions(ExtCS theExt, CurveAdaptorBase theC, SurfaceAdaptorBase theS)
    {
        Assert.True(theExt.IsDone);
        Assert.False(theExt.IsParallel);
        for (int i = 1; i <= theExt.ExtCount; i++)
        {
            theExt.Points(i, out POnCurv aPC, out POnSurf aPS);
            aPS.Parameter(out double aU, out double aV);
            Assert.True(theC.Value(aPC.Parameter()).DistanceTo(aPC.Value()) < Tol, $"point on curve {i}");
            Assert.True(theS.Value(aU, aV).DistanceTo(aPS.Value()) < Tol, $"point on surface {i}");
            Assert.Equal(aPC.Value().SquareDistanceTo(aPS.Value()), theExt.SquareDistance(i), Tol);
        }
    }

    private static double MinSquareDistance(ExtCS theExt)
    {
        double aMin = double.MaxValue;
        for (int i = 1; i <= theExt.ExtCount; i++)
        {
            aMin = Math.Min(aMin, theExt.SquareDistance(i));
        }
        return aMin;
    }

    private static SurfaceAdaptor Sphere(double theR)
        => new(new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), theR));

    private static SurfaceAdaptor PlaneXY()
        => new(new PlaneSurface(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));

    [Fact]
    public void LineSphere()
    {
        var aC = new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 5), new Direction3D(1, 0, 0)), -10, 10);
        var aS = Sphere(2);
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.True(anExt.ExtCount >= 1);
        Assert.Equal(9.0, MinSquareDistance(anExt), Tol);
    }

    [Fact]
    public void LineCylinder()
    {
        var aC = new CurveAdaptor(new LineCurve3D(new Point3D(0, 3, 0), new Direction3D(1, 0, 0)), -10, 10);
        var aS = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1));
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(4.0, MinSquareDistance(anExt), Tol);
    }

    /// <summary>
    /// A line whose range is shorter than Precision::Confusion against a torus: the extrema are those of the range's
    /// middle point. OCCT reported the origin as their point on the curve (a default Extrema_POnCurv's), with the right
    /// parameter and distance (occt-issues.md, unit 4A's Extrema_ExtCS row; Kerf unit 10K1, decision 1430).
    /// </summary>
    [Fact]
    public void ShortLineTorus_ThePointOnTheCurveIsTheMiddlePoint()
    {
        var aC = new CurveAdaptor(new LineCurve3D(new Point3D(10, 0, 0.5), new Direction3D(0, 0, 1)), 0, 5e-8);
        var aS = new SurfaceAdaptor(new ToroidalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 3, 1));
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        Assert.True(anExt.ExtCount >= 1);
        CheckSolutions(anExt, aC, aS);
        double aDist = Math.Sqrt(49.0 + 0.25) - 1.0;
        Assert.Equal(aDist * aDist, MinSquareDistance(anExt), 1e-7);
    }

    [Fact]
    public void LinePlane_Parallel()
    {
        var aC = new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 2), new Direction3D(1, 0, 0)), -10, 10);
        var aS = PlaneXY();
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.IsParallel);
        Assert.Equal(1, anExt.ExtCount);
        Assert.Equal(4.0, anExt.SquareDistance(1), Tol);
        // OCCT stores no points for a parallel result.
        Assert.ThrowsAny<OutOfRangeException>(() => anExt.Points(1, out _, out _));
    }

    [Fact]
    public void LinePlane_NotParallel_FindsTheCrossing()
    {
        // A line through (0, 0, 2) along (0, 1, 1) crosses the XY plane at (0, -2, 0); the generic path finds it
        var aC = new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 2), new Direction3D(0, 1, 1)), -10, 10);
        var aS = PlaneXY();
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.ExtCount >= 1);

        int aMin = 1;
        for (int i = 2; i <= anExt.ExtCount; i++)
        {
            if (anExt.SquareDistance(i) < anExt.SquareDistance(aMin))
            {
                aMin = i;
            }
        }

        Assert.Equal(0.0, anExt.SquareDistance(aMin), 1e-12);
        anExt.Points(aMin, out POnCurv aOnCurve, out _);
        Assert.True(aOnCurve.Value().IsEqual(new Point3D(0, -2, 0), 1e-6));
    }

    [Fact]
    public void CirclePlane()
    {
        // P(u) = (0, 0, 5) + 2 cos(u) Z + 2 sin(u) Y: z from 3 to 7.
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 5), new Direction3D(1, 0, 0), new Direction3D(0, 0, 1)), 2);
        var aC = new CurveAdaptor(new CircleCurve3D(aCirc));
        var aS = PlaneXY();
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(9.0, MinSquareDistance(anExt), Tol);
    }

    [Fact]
    public void CirclePlane_Parallel()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)), 2);
        var aC = new CurveAdaptor(new CircleCurve3D(aCirc));
        var anExt = new ExtCS(aC, PlaneXY(), 1e-9, 1e-9);
        Assert.True(anExt.IsDone);
        Assert.True(anExt.IsParallel);
        Assert.Equal(9.0, anExt.SquareDistance(1), Tol);
    }

    [Fact]
    public void CircleCylinder()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)), 2);
        var aC = new CurveAdaptor(new CircleCurve3D(aCirc));
        var aS = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 1));
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.True(anExt.ExtCount >= 1);
        // The lowest point of the circle, (0, +-2, 5) at distance hypot(2, 5) - 1 from the X axis, is not the closest:
        // the closest point is at x = +-2 (distance 5 - 1). Check against sampling.
        double aSampled = double.MaxValue;
        for (int i = 0; i < 3600; i++)
        {
            Point3D aP = aC.Value(2 * Math.PI * i / 3600);
            double aD = Math.Sqrt(aP.Y * aP.Y + aP.Z * aP.Z) - 1;
            aSampled = Math.Min(aSampled, aD * aD);
        }
        Assert.Equal(aSampled, MinSquareDistance(anExt), 1e-6);
    }

    [Fact]
    public void CircleSphere()
    {
        var aCirc = new Circle3D(new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1);
        var aC = new CurveAdaptor(new CircleCurve3D(aCirc));
        var aS = Sphere(2);
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(4.0, MinSquareDistance(anExt), Tol);
    }

    [Fact]
    public void HyperbolaPlane()
    {
        // P(u) = 2 cosh(u) Z + sinh(u) X, above the plane z = -1: closest at u = 0, distance 3.
        var aH = new Hyperbola3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, -1, 0), new Direction3D(0, 0, 1)), 2, 1);
        var aC = new CurveAdaptor(new HyperbolaCurve3D(aH), -3, 3);
        var aS = new SurfaceAdaptor(new PlaneSurface(new Point3D(0, 0, -1), new Direction3D(0, 0, 1)));
        var anExt = new ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(1, anExt.ExtCount);
        Assert.Equal(9.0, anExt.SquareDistance(1), Tol);
        anExt.Points(1, out POnCurv aPC, out _);
        Assert.Equal(0.0, aPC.Parameter(), Tol);
    }

    [Fact]
    public void ExtremityFallback_WhenAnalyticExtremaAreOutOfRange()
    {
        // z(u) = 5 + 2 cos(u): extrema at 0 and pi, outside [pi/4, pi/2]. The ends are tried, the closer one kept.
        var aCirc = new Circle3D(new Frame3D(new Point3D(0, 0, 5), new Direction3D(1, 0, 0), new Direction3D(0, 0, 1)), 2);
        var aC = new CurveAdaptor(new CircleCurve3D(aCirc));
        var aS = PlaneXY();
        var anExt = new ExtCS();
        anExt.Initialize(aS, 1e-9, 1e-9);
        anExt.Perform(aC, Math.PI / 4, Math.PI / 2);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(1, anExt.ExtCount);
        Assert.Equal(25.0, anExt.SquareDistance(1), Tol);
        anExt.Points(1, out POnCurv aPC, out POnSurf aPS);
        Assert.Equal(Math.PI / 2, aPC.Parameter(), Tol);
        Assert.Equal(0.0, aPS.Value().Z, Tol);
    }

    [Fact]
    public void PeriodicCurveParameter_IsPutInTheRange()
    {
        // The closest point of the circle (5 + cos u, sin u, 0) to the sphere is at u = pi, reported in [2pi, 4pi].
        var aCirc = new Circle3D(new Frame3D(new Point3D(5, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 1);
        var aC = new CurveAdaptor(new CircleCurve3D(aCirc));
        var aS = Sphere(2);
        var anExt = new ExtCS();
        anExt.Initialize(aS, 1e-9, 1e-9);
        anExt.Perform(aC, 2 * Math.PI, 4 * Math.PI);
        Assert.True(anExt.IsDone);
        int aMin = 0;
        for (int i = 1; i <= anExt.ExtCount; i++)
        {
            anExt.Points(i, out POnCurv aPCi, out _);
            Assert.InRange(aPCi.Parameter(), 2 * Math.PI - Tol, 4 * Math.PI + Tol);
            if (aMin == 0 || anExt.SquareDistance(i) < anExt.SquareDistance(aMin))
            {
                aMin = i;
            }
        }
        anExt.Points(aMin, out POnCurv aPC, out _);
        Assert.Equal(3 * Math.PI, aPC.Parameter(), Tol);
        Assert.Equal(4.0, anExt.SquareDistance(aMin), Tol);
    }

    [Fact]
    public void StatusChecks()
    {
        var anEmpty = new ExtCS();
        Assert.False(anEmpty.IsDone);
        Assert.ThrowsAny<NotDoneException>(() => anEmpty.ExtCount);
        Assert.ThrowsAny<NotDoneException>(() => anEmpty.IsParallel);
        Assert.ThrowsAny<NotDoneException>(() => anEmpty.SquareDistance(1));

        var aC = new CurveAdaptor(new LineCurve3D(new Point3D(0, 0, 5), new Direction3D(1, 0, 0)), -10, 10);
        var anExt = new ExtCS(aC, Sphere(2), 1e-9, 1e-9);
        Assert.ThrowsAny<OutOfRangeException>(() => anExt.SquareDistance(0));
        Assert.ThrowsAny<OutOfRangeException>(() => anExt.SquareDistance(anExt.ExtCount + 1));
        Assert.ThrowsAny<OutOfRangeException>(() => anExt.Points(anExt.ExtCount + 1, out _, out _));
    }
}
