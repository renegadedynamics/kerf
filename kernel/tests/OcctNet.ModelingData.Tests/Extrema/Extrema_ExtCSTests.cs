// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Extrema_ExtCS on the paths solved analytically (Extrema_ExtElCS), with the extremity fallback, the periodic
/// adjustment of parameters and the status checks. Every extremum is checked against the geometry: its points lie
/// on the curve and the surface at its parameters and its square distance is theirs. The generic paths need unit
/// 4A's Extrema_GenExtCS and Extrema_ExtPS.
/// </summary>
public class Extrema_ExtCSTests
{
    private const double Tol = 1e-9;

    private static void CheckSolutions(Extrema_ExtCS theExt, Adaptor3d_Curve theC, Adaptor3d_Surface theS)
    {
        Assert.True(theExt.IsDone());
        Assert.False(theExt.IsParallel());
        for (int i = 1; i <= theExt.NbExt(); i++)
        {
            theExt.Points(i, out Extrema_POnCurv aPC, out Extrema_POnSurf aPS);
            aPS.Parameter(out double aU, out double aV);
            Assert.True(theC.Value(aPC.Parameter()).Distance(aPC.Value()) < Tol, $"point on curve {i}");
            Assert.True(theS.Value(aU, aV).Distance(aPS.Value()) < Tol, $"point on surface {i}");
            Assert.Equal(aPC.Value().SquareDistance(aPS.Value()), theExt.SquareDistance(i), Tol);
        }
    }

    private static double MinSquareDistance(Extrema_ExtCS theExt)
    {
        double aMin = double.MaxValue;
        for (int i = 1; i <= theExt.NbExt(); i++)
        {
            aMin = Math.Min(aMin, theExt.SquareDistance(i));
        }
        return aMin;
    }

    private static GeomAdaptor_Surface Sphere(double theR)
        => new(new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), theR));

    private static GeomAdaptor_Surface PlaneXY()
        => new(new Geom_Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)));

    [Fact]
    public void LineSphere()
    {
        var aC = new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 0, 5), new gp_Dir(1, 0, 0)), -10, 10);
        var aS = Sphere(2);
        var anExt = new Extrema_ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.True(anExt.NbExt() >= 1);
        Assert.Equal(9.0, MinSquareDistance(anExt), Tol);
    }

    [Fact]
    public void LineCylinder()
    {
        var aC = new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 3, 0), new gp_Dir(1, 0, 0)), -10, 10);
        var aS = new GeomAdaptor_Surface(new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 1));
        var anExt = new Extrema_ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(4.0, MinSquareDistance(anExt), Tol);
    }

    [Fact]
    public void LinePlane_Parallel()
    {
        var aC = new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 0, 2), new gp_Dir(1, 0, 0)), -10, 10);
        var aS = PlaneXY();
        var anExt = new Extrema_ExtCS(aC, aS, 1e-9, 1e-9);
        Assert.True(anExt.IsDone());
        Assert.True(anExt.IsParallel());
        Assert.Equal(1, anExt.NbExt());
        Assert.Equal(4.0, anExt.SquareDistance(1), Tol);
        // OCCT stores no points for a parallel result.
        Assert.ThrowsAny<Standard_OutOfRange>(() => anExt.Points(1, out _, out _));
    }

    [Fact]
    public void LinePlane_NotParallel_FindsTheCrossing()
    {
        // A line through (0, 0, 2) along (0, 1, 1) crosses the XY plane at (0, -2, 0); the generic path finds it
        var aC = new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 0, 2), new gp_Dir(0, 1, 1)), -10, 10);
        var aS = PlaneXY();
        var anExt = new Extrema_ExtCS(aC, aS, 1e-9, 1e-9);
        Assert.True(anExt.IsDone());
        Assert.True(anExt.NbExt() >= 1);

        int aMin = 1;
        for (int i = 2; i <= anExt.NbExt(); i++)
        {
            if (anExt.SquareDistance(i) < anExt.SquareDistance(aMin))
            {
                aMin = i;
            }
        }

        Assert.Equal(0.0, anExt.SquareDistance(aMin), 1e-12);
        anExt.Points(aMin, out Extrema_POnCurv aOnCurve, out _);
        Assert.True(aOnCurve.Value().IsEqual(new gp_Pnt(0, -2, 0), 1e-6));
    }

    [Fact]
    public void CirclePlane()
    {
        // P(u) = (0, 0, 5) + 2 cos(u) Z + 2 sin(u) Y: z from 3 to 7.
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 5), new gp_Dir(1, 0, 0), new gp_Dir(0, 0, 1)), 2);
        var aC = new GeomAdaptor_Curve(new Geom_Circle(aCirc));
        var aS = PlaneXY();
        var anExt = new Extrema_ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(9.0, MinSquareDistance(anExt), Tol);
    }

    [Fact]
    public void CirclePlane_Parallel()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 0, 1)), 2);
        var aC = new GeomAdaptor_Curve(new Geom_Circle(aCirc));
        var anExt = new Extrema_ExtCS(aC, PlaneXY(), 1e-9, 1e-9);
        Assert.True(anExt.IsDone());
        Assert.True(anExt.IsParallel());
        Assert.Equal(9.0, anExt.SquareDistance(1), Tol);
    }

    [Fact]
    public void CircleCylinder()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 5), new gp_Dir(0, 0, 1)), 2);
        var aC = new GeomAdaptor_Curve(new Geom_Circle(aCirc));
        var aS = new GeomAdaptor_Surface(new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), 1));
        var anExt = new Extrema_ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.True(anExt.NbExt() >= 1);
        // The lowest point of the circle, (0, +-2, 5) at distance hypot(2, 5) - 1 from the X axis, is not the closest:
        // the closest point is at x = +-2 (distance 5 - 1). Check against sampling.
        double aSampled = double.MaxValue;
        for (int i = 0; i < 3600; i++)
        {
            gp_Pnt aP = aC.Value(2 * Math.PI * i / 3600);
            double aD = Math.Sqrt(aP.Y() * aP.Y() + aP.Z() * aP.Z()) - 1;
            aSampled = Math.Min(aSampled, aD * aD);
        }
        Assert.Equal(aSampled, MinSquareDistance(anExt), 1e-6);
    }

    [Fact]
    public void CircleSphere()
    {
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(5, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 1);
        var aC = new GeomAdaptor_Curve(new Geom_Circle(aCirc));
        var aS = Sphere(2);
        var anExt = new Extrema_ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(4.0, MinSquareDistance(anExt), Tol);
    }

    [Fact]
    public void HyperbolaPlane()
    {
        // P(u) = 2 cosh(u) Z + sinh(u) X, above the plane z = -1: closest at u = 0, distance 3.
        var aH = new gp_Hypr(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, -1, 0), new gp_Dir(0, 0, 1)), 2, 1);
        var aC = new GeomAdaptor_Curve(new Geom_Hyperbola(aH), -3, 3);
        var aS = new GeomAdaptor_Surface(new Geom_Plane(new gp_Pnt(0, 0, -1), new gp_Dir(0, 0, 1)));
        var anExt = new Extrema_ExtCS(aC, aS, 1e-9, 1e-9);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(1, anExt.NbExt());
        Assert.Equal(9.0, anExt.SquareDistance(1), Tol);
        anExt.Points(1, out Extrema_POnCurv aPC, out _);
        Assert.Equal(0.0, aPC.Parameter(), Tol);
    }

    [Fact]
    public void ExtremityFallback_WhenAnalyticExtremaAreOutOfRange()
    {
        // z(u) = 5 + 2 cos(u): extrema at 0 and pi, outside [pi/4, pi/2]. The ends are tried, the closer one kept.
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(0, 0, 5), new gp_Dir(1, 0, 0), new gp_Dir(0, 0, 1)), 2);
        var aC = new GeomAdaptor_Curve(new Geom_Circle(aCirc));
        var aS = PlaneXY();
        var anExt = new Extrema_ExtCS();
        anExt.Initialize(aS, 1e-9, 1e-9);
        anExt.Perform(aC, Math.PI / 4, Math.PI / 2);
        CheckSolutions(anExt, aC, aS);
        Assert.Equal(1, anExt.NbExt());
        Assert.Equal(25.0, anExt.SquareDistance(1), Tol);
        anExt.Points(1, out Extrema_POnCurv aPC, out Extrema_POnSurf aPS);
        Assert.Equal(Math.PI / 2, aPC.Parameter(), Tol);
        Assert.Equal(0.0, aPS.Value().Z(), Tol);
    }

    [Fact]
    public void PeriodicCurveParameter_IsPutInTheRange()
    {
        // The closest point of the circle (5 + cos u, sin u, 0) to the sphere is at u = pi, reported in [2pi, 4pi].
        var aCirc = new gp_Circ(new gp_Ax2(new gp_Pnt(5, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 1);
        var aC = new GeomAdaptor_Curve(new Geom_Circle(aCirc));
        var aS = Sphere(2);
        var anExt = new Extrema_ExtCS();
        anExt.Initialize(aS, 1e-9, 1e-9);
        anExt.Perform(aC, 2 * Math.PI, 4 * Math.PI);
        Assert.True(anExt.IsDone());
        int aMin = 0;
        for (int i = 1; i <= anExt.NbExt(); i++)
        {
            anExt.Points(i, out Extrema_POnCurv aPCi, out _);
            Assert.InRange(aPCi.Parameter(), 2 * Math.PI - Tol, 4 * Math.PI + Tol);
            if (aMin == 0 || anExt.SquareDistance(i) < anExt.SquareDistance(aMin))
            {
                aMin = i;
            }
        }
        anExt.Points(aMin, out Extrema_POnCurv aPC, out _);
        Assert.Equal(3 * Math.PI, aPC.Parameter(), Tol);
        Assert.Equal(4.0, anExt.SquareDistance(aMin), Tol);
    }

    [Fact]
    public void StatusChecks()
    {
        var anEmpty = new Extrema_ExtCS();
        Assert.False(anEmpty.IsDone());
        Assert.ThrowsAny<StdFail_NotDone>(() => anEmpty.NbExt());
        Assert.ThrowsAny<StdFail_NotDone>(() => anEmpty.IsParallel());
        Assert.ThrowsAny<StdFail_NotDone>(() => anEmpty.SquareDistance(1));

        var aC = new GeomAdaptor_Curve(new Geom_Line(new gp_Pnt(0, 0, 5), new gp_Dir(1, 0, 0)), -10, 10);
        var anExt = new Extrema_ExtCS(aC, Sphere(2), 1e-9, 1e-9);
        Assert.ThrowsAny<Standard_OutOfRange>(() => anExt.SquareDistance(0));
        Assert.ThrowsAny<Standard_OutOfRange>(() => anExt.SquareDistance(anExt.NbExt() + 1));
        Assert.ThrowsAny<Standard_OutOfRange>(() => anExt.Points(anExt.NbExt() + 1, out _, out _));
    }
}
