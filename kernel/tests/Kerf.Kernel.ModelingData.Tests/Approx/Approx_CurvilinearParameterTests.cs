// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

// Approx_CurvlinFunc and Approx_CurvilinearParameter on curves of known length (OCCT has no GTests for them).
public class Approx_CurvilinearParameterTests
{
    private static CurveAdaptor Ellipse(out double theLength)
    {
        EllipseCurve3D anEllipse = new(new Frame3D(new Point3D(1.0, -2.0, 0.0), new Direction3D(0.0, 0.0, 1.0)), 3.0, 1.0);
        CurveAdaptor aCurve = new(anEllipse, 0.0, 0.75 * 2.0 * Math.PI);
        theLength = AbscissaPoint.Length(new CurveAdaptor(anEllipse, 0.0, 0.75 * 2.0 * Math.PI), 1.0e-12);
        return aCurve;
    }

    private static double OnEllipse(Point3D theP)
    {
        double x = (theP.X - 1.0) / 3.0, y = theP.Y + 2.0;
        return Math.Abs(Math.Sqrt(x * x + y * y) - 1.0) + Math.Abs(theP.Z);
    }

    [Fact]
    public void CurvlinFunc_GetUParameter_InvertsGetSParameter()
    {
        CurveAdaptor aCurve = Ellipse(out double aLength);
        CurvlinFunc aFunc = new(aCurve, 1.0e-10);
        Assert.Equal(0.0, aFunc.FirstParameter());
        Assert.Equal(1.0, aFunc.LastParameter());
        Assert.Equal(aLength, aFunc.GetLength(), 1.0e-8);

        for (int i = 0; i <= 20; i++)
        {
            double s = i / 20.0;
            double u = aFunc.GetUParameter(aCurve, s, 1);
            Assert.Equal(s, aFunc.GetSParameter(u), 1.0e-8);
            Assert.Equal(s * aLength, AbscissaPoint.Length(aCurve, 0.0, u, 1.0e-12), 1.0e-7);
        }
    }

    [Fact]
    public void CurvlinFunc_EvalCase1_HasUnitSpeedTimesLength()
    {
        CurveAdaptor aCurve = Ellipse(out double aLength);
        CurvlinFunc aFunc = new(aCurve, 1.0e-10);
        NCollection_Array1<double> aRes = new(0, 2);
        for (int i = 0; i <= 10; i++)
        {
            double s = i / 10.0;
            Assert.True(aFunc.EvalCase1(s, 0, aRes));
            Assert.True(OnEllipse(new Point3D(aRes[0], aRes[1], aRes[2])) < 1.0e-12);
            Assert.True(aFunc.EvalCase1(s, 1, aRes));
            Assert.Equal(aLength, new Vector3D(aRes[0], aRes[1], aRes[2]).Magnitude, 1.0e-6);
            Assert.True(aFunc.EvalCase1(s, 2, aRes));
            // Unit-speed curves (scaled by the length) have their second derivative normal to the first.
            Vector3D aD2 = new(aRes[0], aRes[1], aRes[2]);
            Assert.True(aFunc.EvalCase1(s, 1, aRes));
            Assert.True(Math.Abs(aD2.Dot(new Vector3D(aRes[0], aRes[1], aRes[2]))) < 1.0e-6 * aLength * aLength * aLength);
        }
        Assert.False(aFunc.EvalCase1(0.5, 3, aRes));
        Assert.Equal(0.0, aRes[0]);
        Assert.Throws<ConstructionErrorException>(() => aFunc.EvalCase2(0.5, 0, new NCollection_Array1<double>(0, 4)));
        Assert.Throws<ConstructionErrorException>(() => aFunc.GetUParameter(aCurve, 1.5, 1));
        Assert.Throws<OutOfRangeException>(() => aFunc.Trim(-0.1, 0.5, 1.0e-9));
    }

    [Fact]
    public void CurvlinFunc_Trim_KeepsTheCurvilinearParameter()
    {
        CurveAdaptor aCurve = Ellipse(out _);
        CurvlinFunc aWhole = new(aCurve, 1.0e-10);
        CurvlinFunc aTrimmed = new(aCurve, 1.0e-10);
        aTrimmed.Trim(0.25, 0.75, 1.0e-9);
        Assert.Equal(0.25, aTrimmed.FirstParameter());
        Assert.Equal(0.75, aTrimmed.LastParameter());
        NCollection_Array1<double> aRes1 = new(0, 2), aRes2 = new(0, 2);
        for (int i = 0; i <= 4; i++)
        {
            double s = 0.25 + 0.125 * i;
            aWhole.EvalCase1(s, 0, aRes1);
            aTrimmed.EvalCase1(s, 0, aRes2);
            Assert.True(new Point3D(aRes1[0], aRes1[1], aRes1[2]).DistanceTo(new Point3D(aRes2[0], aRes2[1], aRes2[2])) < 1.0e-7);
        }
        // The caller's adaptor is never trimmed.
        Assert.Equal(0.0, aCurve.FirstParameter());
    }

    [Fact]
    public void Curve3d_IsParametrisedByArcLength()
    {
        CurveAdaptor aCurve = Ellipse(out double aLength);
        CurvilinearParameter anApprox = new(aCurve, 1.0e-6, C2, 14, 30);

        Assert.True(anApprox.IsDone);
        Assert.True(anApprox.HasResult);
        Assert.True(anApprox.MaxError3d() <= 1.0e-6);
        Assert.Equal(0.0, anApprox.MaxError2d1());
        Assert.Null(anApprox.Curve2d1());
        BSplineCurve3D aResult = anApprox.Curve3d()!;
        Assert.Same(aResult, anApprox.Curve3d());
        Assert.Equal(0.0, aResult.FirstParameter);
        Assert.Equal(1.0, aResult.LastParameter);
        Assert.True(aResult.Value(0.0).DistanceTo(aCurve.Value(0.0)) < 1.0e-6);
        Assert.True(aResult.Value(1.0).DistanceTo(aCurve.Value(aCurve.LastParameter())) < 1.0e-6);

        CurveAdaptor aResultAdaptor = new(aResult);
        for (int i = 1; i <= 20; i++)
        {
            double s = i / 20.0;
            Assert.True(OnEllipse(aResult.Value(s)) < 1.0e-5);
            Assert.Equal(s * aLength, AbscissaPoint.Length(aResultAdaptor, 0.0, s, 1.0e-10), 1.0e-4);
            aResult.D1(s, out _, out Vector3D aD1);
            Assert.Equal(aLength, aD1.Magnitude, 1.0e-3 * aLength);
        }
        Assert.Contains("myMaxError3d = ", anApprox.ToString());
        Assert.DoesNotContain("myMaxError2d1", anApprox.ToString());
    }

    [Fact]
    public void CurveOnSurface_HelixOnCylinder()
    {
        CylindricalSurface aCylinder = new(new CoordinateSystem(), 2.0);
        LineCurve2D aLine = new(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.5));
        CurveAdaptor2D aC2d = new(aLine, 0.0, 6.0);
        SurfaceAdaptor aSurf = new(aCylinder);
        double aLength = AbscissaPoint.Length(new CurveOnSurface(aC2d, aSurf), 1.0e-12);

        CurvilinearParameter anApprox = new(aC2d, aSurf, 1.0e-6, C1, 14, 30);
        Assert.True(anApprox.HasResult);
        Assert.True(anApprox.MaxError3d() <= 1.0e-6);
        Assert.True(anApprox.MaxError2d1() <= 1.0e-6);
        BSplineCurve3D a3d = anApprox.Curve3d()!;
        BSplineCurve2D a2d = anApprox.Curve2d1()!;
        Assert.Null(anApprox.Curve2d2());
        for (int i = 0; i <= 20; i++)
        {
            double s = i / 20.0;
            Point2D aUV = a2d.Value(s);
            Assert.True(aCylinder.Value(aUV.X, aUV.Y).DistanceTo(a3d.Value(s)) < 1.0e-5);
            // The helix has a constant speed, so the arc length is a linear reparametrisation.
            Assert.True(aUV.DistanceTo(aLine.Value(6.0 * s)) < 1.0e-5);
            a3d.D1(s, out _, out Vector3D aD1);
            Assert.Equal(aLength, aD1.Magnitude, 1.0e-3 * aLength);
        }
        Assert.Contains("myMaxError2d1 = ", anApprox.ToString());
    }

    [Fact]
    public void CurveOnTwoSurfaces_SameCircleOnShiftedPlanes()
    {
        PlaneSurface aPlane1 = new(new CoordinateSystem());
        PlaneSurface aPlane2 = new(new CoordinateSystem(new Point3D(1.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0)));
        // An ellipse's arc on plane 1 (non-uniform speed), and the same arc on plane 2, where u is shifted by -1.
        EllipseCurve2D anE1 = new(new Frame2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0), new Direction2D(0.0, 1.0)), 2.0, 1.0);
        EllipseCurve2D anE2 = new(new Frame2D(new Point2D(-1.0, 0.0), new Direction2D(1.0, 0.0), new Direction2D(0.0, 1.0)), 2.0, 1.0);
        CurveAdaptor2D aC1 = new(anE1, 0.0, 3.0);
        CurveAdaptor2D aC2 = new(anE2, 0.0, 3.0);
        SurfaceAdaptor aS1 = new(aPlane1), aS2 = new(aPlane2);
        double aLength = AbscissaPoint.Length(new CurveOnSurface(aC1, aS1), 1.0e-12);

        CurvilinearParameter anApprox = new(aC1, aS1, aC2, aS2, 1.0e-6, C1, 14, 30);
        Assert.True(anApprox.HasResult);
        BSplineCurve3D a3d = anApprox.Curve3d()!;
        BSplineCurve2D a2d1 = anApprox.Curve2d1()!, a2d2 = anApprox.Curve2d2()!;
        Assert.True(anApprox.MaxError2d2() <= 1.0e-6);
        for (int i = 0; i <= 20; i++)
        {
            double s = i / 20.0;
            Point2D aUV1 = a2d1.Value(s), aUV2 = a2d2.Value(s);
            Assert.True(aPlane1.Value(aUV1.X, aUV1.Y).DistanceTo(a3d.Value(s)) < 1.0e-5);
            Assert.True(aPlane2.Value(aUV2.X, aUV2.Y).DistanceTo(a3d.Value(s)) < 1.0e-5);
            Assert.Equal(aUV1.X - 1.0, aUV2.X, 1.0e-5);
            a3d.D1(s, out _, out Vector3D aD1);
            Assert.Equal(aLength, aD1.Magnitude, 1.0e-3 * aLength);
        }
        Assert.Contains("myMaxError2d2 = ", anApprox.ToString());
    }

    [Fact]
    public void CurvlinFunc_TwoCurves_ReusesTheFirstCurvesIntervals_OCCT()
    {
        // Two B-spline 2d curves with a C0 knot at different parameters. Case 3 means to fuse their C3 intervals,
        // but it reloads one Adaptor3d_CurveOnSurface with the second curve, and Adaptor3d_CurveOnSurface::Load
        // keeps the intervals it cached for the first curve (occt-issues.md): the second curve's knot is lost.
        NCollection_Array1<Point2D> aPoles = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aPoles[i] = new Point2D(i, (i % 2) * 0.5);
        }
        NCollection_Array1<int> aMults = new(1, 3);
        aMults[1] = 3; aMults[2] = 2; aMults[3] = 3;
        NCollection_Array1<double> aKnots1 = new(1, 3), aKnots2 = new(1, 3);
        aKnots1[1] = 0.0; aKnots1[2] = 0.3; aKnots1[3] = 1.0;
        aKnots2[1] = 0.0; aKnots2[2] = 0.6; aKnots2[3] = 1.0;
        BSplineCurve2D aB1 = new(aPoles, aKnots1, aMults, 2), aB2 = new(aPoles, aKnots2, aMults, 2);
        SurfaceAdaptor aPlane = new(new PlaneSurface(new CoordinateSystem()));
        Assert.Equal(2, new CurveOnSurface(new CurveAdaptor2D(aB2), aPlane).NbIntervals(C3));

        CurvlinFunc aFunc = new(new CurveAdaptor2D(aB1), new CurveAdaptor2D(aB2), aPlane, aPlane, 1.0e-9);
        Assert.Equal(2, aFunc.NbIntervals(C3)); // 3 with the second curve's knot
        NCollection_Array1<double> aT = new(1, 3);
        aFunc.Intervals(aT, C3);
        Assert.Equal(0.0, aT[1], 1.0e-12);
        Assert.Equal(aFunc.GetSParameter(0.3), aT[2]);
        Assert.Equal(1.0, aT[3], 1.0e-9);
        NCollection_Array1<double> aRes = new(0, 6);
        Assert.True(aFunc.EvalCase3(0.5, 0, aRes));
        Assert.Throws<ConstructionErrorException>(() => aFunc.EvalCase1(0.5, 0, aRes));
    }
}
