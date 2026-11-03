// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of GeomLib's statics (GeomLib.cxx), GeomLib_LogSample, GeomLib_PolyFunc,
/// GeomLib_DenominatorMultiplier and GeomLib_MakeCurvefromApprox. BuildCurve3d, To3d and SameRange are also compared
/// with native OCCT (the oracle tests); these tests check known answers and that the caller's geometry is never
/// changed.
/// </summary>
public class GeomLibTests
{
    private static NCollection_Array1<double> Reals(params double[] theValues)
    {
        NCollection_Array1<double> anArray = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; i++)
        {
            anArray[i + 1] = theValues[i];
        }
        return anArray;
    }

    private static BSplineCurve3D Wavy3d()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 6);
        for (int i = 1; i <= 6; i++)
        {
            aPoles[i] = new Point3D(i, Math.Sin(i), 0.3 * Math.Cos(2.0 * i));
        }
        NCollection_Array1<int> aMults = new(1, 4);
        aMults.Init(1);
        aMults[1] = 4;
        aMults[4] = 4;
        return new BSplineCurve3D(aPoles, Reals(0, 1, 2, 3), aMults, 3);
    }

    private static BSplineSurface Sheet(bool theRational)
    {
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 4);
        NCollection_Array2<double> aWeights = new(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = new Point3D(i, j, 0.2 * Math.Sin(i + 2.0 * j));
                aWeights[i, j] = theRational ? 1.0 + 0.1 * ((i + j) % 3) : 1.0;
            }
        }
        NCollection_Array1<int> aMults = new(1, 2);
        aMults.Init(4);
        return theRational
            ? new BSplineSurface(aPoles, aWeights, Reals(0, 1), Reals(0, 1), aMults, aMults, 3, 3)
            : new BSplineSurface(aPoles, Reals(0, 1), Reals(0, 1), aMults, aMults, 3, 3);
    }

    // ---------------------------------------------------------------------------------------------------------
    // Arrays and intervals

    [Fact]
    public void FuseIntervals_MergesCloseValues()
    {
        NCollection_Sequence<double> aSeq = new();
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(Reals(0, 1, 2, 3), Reals(0, 1.5, 2 + 1e-10, 4), aSeq);
        Assert.Equal(6, aSeq.Length());
        Assert.Equal([0.0, 1.0, 1.5, 2 + 0.5e-10, 3.0, 4.0], Enumerable.Range(1, 6).Select(i => aSeq.Value(i)));

        NCollection_Sequence<double> aFirst = new();
        global::Kerf.Engine.Geometry.Library.GeomLib.FuseIntervals(Reals(0, 2), Reals(1e-4, 2), aFirst, 1e-3, true);
        Assert.Equal(2, aFirst.Length());
        Assert.Equal(0.0, aFirst.Value(1));
    }

    [Fact]
    public void RemovePointsFromArray_KeepsEndsAndAtMostOnePerSegment()
    {
        NCollection_Array1<double> anIn = new(1, 101);
        for (int i = 1; i <= 101; i++)
        {
            anIn[i] = (i - 1) * 0.01;
        }
        global::Kerf.Engine.Geometry.Library.GeomLib.RemovePointsFromArray(10, anIn, out NCollection_HArray1<double> anOut);
        Assert.Equal(0.0, anOut[1]);
        Assert.Equal(1.0, anOut[anOut.Upper()]);
        Assert.True(anOut.Length() <= 11);
        for (int i = 2; i <= anOut.Upper(); i++)
        {
            Assert.True(anOut[i] > anOut[i - 1]);
        }
    }

    [Fact]
    public void DensifyArray1OfReal_AddsUniformPoints()
    {
        global::Kerf.Engine.Geometry.Library.GeomLib.DensifyArray1OfReal(5, Reals(0, 4), out NCollection_HArray1<double> anOut);
        Assert.Equal([0.0, 4.0 / 4, 2.0, 3.0, 4.0], anOut.AsSpan().ToArray().Select(x => Math.Round(x, 12)));

        // Not in order: copied
        global::Kerf.Engine.Geometry.Library.GeomLib.DensifyArray1OfReal(5, Reals(3, 1), out NCollection_HArray1<double> aCopy);
        Assert.Equal([3.0, 1.0], aCopy.AsSpan().ToArray());

        // Enough points: copied
        global::Kerf.Engine.Geometry.Library.GeomLib.DensifyArray1OfReal(2, Reals(0, 1, 2), out NCollection_HArray1<double> aSame);
        Assert.Equal([0.0, 1.0, 2.0], aSame.AsSpan().ToArray());
    }

    // ---------------------------------------------------------------------------------------------------------
    // Inertia

    [Fact]
    public void Inertia_PointsOnATiltedRectangle()
    {
        // A 4 x 1 grid of points in the plane z = x
        NCollection_Array1<Point3D> aPoints = new(1, 50);
        int k = 1;
        for (int i = 0; i < 10; i++)
        {
            for (int j = 0; j < 5; j++)
            {
                double x = 4.0 * i / 9.0;
                aPoints[k++] = new Point3D(x / Math.Sqrt(2.0), j / 4.0, x / Math.Sqrt(2.0));
            }
        }
        global::Kerf.Engine.Geometry.Library.GeomLib.Inertia(aPoints, out Point3D aBary, out Direction3D aX, out Direction3D aY, out double aXGap, out double aYGap,
                        out double aZGap);
        Assert.Equal(2.0 / Math.Sqrt(2.0), aBary.X(), 1e-12);
        Assert.Equal(0.5, aBary.Y(), 1e-12);
        Assert.True(aX.IsParallel(new Direction3D(1, 0, 1), 1e-9));
        Assert.True(aY.IsParallel(new Direction3D(0, 1, 0), 1e-9));
        Assert.True(aXGap > aYGap);
        Assert.Equal(0.0, aZGap, 1e-7);

        global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(aPoints, out Frame3D anAxe, out bool anIsSingular);
        Assert.False(anIsSingular);
        Assert.True(anAxe.Direction().IsParallel(new Direction3D(-1, 0, 1), 1e-9));

        // Points on a line are singular
        NCollection_Array1<Point3D> aLine = new(1, 5);
        for (int i = 1; i <= 5; i++)
        {
            aLine[i] = new Point3D(i, 2.0 * i, 0.0);
        }
        global::Kerf.Engine.Geometry.Library.GeomLib.AxeOfInertia(aLine, out Frame3D aLineAxe, out bool aLineSingular);
        Assert.True(aLineSingular);
        Assert.True(aLineAxe.XDirection().IsParallel(new Direction3D(1, 2, 0), 1e-9));
    }

    // ---------------------------------------------------------------------------------------------------------
    // Distances

    [Fact]
    public void EvalMaxDistances_BetweenAShiftedCurve()
    {
        BSplineCurve3D aCurve = Wavy3d();
        BSplineCurve3D aShifted = (BSplineCurve3D)aCurve.Translated(new Vector3D(0, 0, 0.01));
        CurveAdaptor aC1 = new(aCurve), aC2 = new(aShifted);
        NCollection_Array1<double> aParams = Reals(0, 0.5, 1, 1.5, 2, 2.5, 3);
        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxParametricDistance(aC1, aC2, 1e-7, aParams, out double aMax);
        Assert.Equal(0.01, aMax, 1e-12);
        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxDistanceAlongParameter(aC1, aC2, 1e-7, aParams, out double aMaxAlong);
        Assert.InRange(aMaxAlong, 0.005, 0.01 + 1e-12); // projected: at most the shift
        global::Kerf.Engine.Geometry.Library.GeomLib.EvalMaxDistanceAlongParameter(aC1, aC1, 1e-7, aParams, out double aZero);
        Assert.Equal(1e-7, aZero);
    }

    // ---------------------------------------------------------------------------------------------------------
    // To3d, GTransform, SameRange

    [Fact]
    public void To3d_EveryCurveKind()
    {
        Frame3D aPos = new(new Point3D(1, 2, 3), new Direction3D(0, 1, 1), new Direction3D(1, 0, 0));
        PlaneSurface aPlane = new(new CoordinateSystem(aPos));
        Curve2D[] aCurves =
        [
            new LineCurve2D(new Point2D(1, 1), new Direction2D(1, 2)),
            new CircleCurve2D(new Axis2D(new Point2D(0.5, -1), new Direction2D(1, 1)), 2.0),
            new EllipseCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(0, 1)), 3.0, 1.0),
            new ParabolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 0.5),
            new HyperbolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0, 1.0),
            new TrimmedCurve2D(new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0), 0.5, 2.0),
            new OffsetCurve2D(new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0), 0.25),
            new BezierCurve2D(NCollection_Array1<Point2D>.From([new(0, 0), new(1, 2), new(3, 1)])),
            global::Kerf.Engine.Geometry.Conversion.Geom2dConvert2D.CurveToBSplineCurve(new TrimmedCurve2D(
                new CircleCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0), 0.0, 2.0)),
        ];
        Type[] aTypes =
        [
            typeof(LineCurve3D), typeof(CircleCurve3D), typeof(EllipseCurve3D), typeof(ParabolaCurve3D), typeof(HyperbolaCurve3D),
            typeof(TrimmedCurve3D), typeof(OffsetCurve3D), typeof(BezierCurve3D), typeof(BSplineCurve3D),
        ];
        for (int k = 0; k < aCurves.Length; k++)
        {
            Curve3D a3d = global::Kerf.Engine.Geometry.Library.GeomLib.To3d(aPos, aCurves[k]);
            Assert.IsType(aTypes[k], a3d);
            double f = Math.Max(aCurves[k].FirstParameter(), -2.0), l = Math.Min(aCurves[k].LastParameter(), 2.0);
            for (int i = 0; i <= 8; i++)
            {
                double t = f + (l - f) * i / 8.0;
                Point2D aP2d = aCurves[k].Value(t);
                Point3D anExpected = aPlane.Value(aP2d.X(), aP2d.Y());
                Assert.True(a3d.Value(t).Distance(anExpected) < 1e-12, $"{aTypes[k].Name} at {t}");
            }
        }
    }

    [Fact]
    public void GTransform_TrsfAndAffinity()
    {
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(1, 0), new Direction2D(1, 0)), 2.0);
        GeneralTransform2D aRotation = new(new Transform2D().WithRotation(new Point2D(0, 0), 0.3));
        Assert.IsType<CircleCurve2D>(global::Kerf.Engine.Geometry.Library.GeomLib.GTransform(aCircle, aRotation));

        GeneralTransform2D anAffinity = GeneralTransform2D.Affinity(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 0.5);
        Curve2D? anEllipse = global::Kerf.Engine.Geometry.Library.GeomLib.GTransform(aCircle, anAffinity);
        Assert.IsType<BSplineCurve2D>(anEllipse);
        for (double t = 0; t < 6; t += 0.5)
        {
            // the transformed curve passes through the transformed points of the circle (not at the same parameter)
            Point2D aP = aCircle.Value(t);
            Xy anImage = anAffinity.Transformed(aP.XY());
            Geom2dAPI_Distance(anEllipse!, new Point2D(anImage), 1e-9);
        }

        LineCurve2D aLine = new(new Point2D(0, 1), new Direction2D(1, 1));
        Curve2D? aLineImage = global::Kerf.Engine.Geometry.Library.GeomLib.GTransform(aLine, anAffinity);
        Assert.IsType<LineCurve2D>(aLineImage);
        Assert.Equal(1.0, aLine.Location().Y());
        Assert.Equal(0.5, ((LineCurve2D)aLineImage!).Location().Y(), 1e-15);

        Assert.Null(global::Kerf.Engine.Geometry.Library.GeomLib.GTransform(new OffsetCurve2D(aCircle, 0.1), anAffinity));
        Assert.Null(global::Kerf.Engine.Geometry.Library.GeomLib.GTransform(new ParabolaCurve2D(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 1.0),
                                       anAffinity));
    }

    // The distance from the point to the curve is within the tolerance (dense sampling, then Newton-free bisection).
    private static void Geom2dAPI_Distance(Curve2D theC, Point2D theP, double theTol)
    {
        double f = theC.FirstParameter(), l = theC.LastParameter();
        double aBest = double.MaxValue, aBestT = f;
        for (int i = 0; i <= 2000; i++)
        {
            double t = f + (l - f) * i / 2000.0;
            double d = theC.Value(t).Distance(theP);
            if (d < aBest)
            {
                aBest = d;
                aBestT = t;
            }
        }
        double aStep = (l - f) / 2000.0;
        for (int it = 0; it < 60; it++)
        {
            double t1 = Math.Max(f, aBestT - aStep), t2 = Math.Min(l, aBestT + aStep);
            double d1 = theC.Value(t1).Distance(theP), d2 = theC.Value(t2).Distance(theP);
            if (d1 < aBest)
            {
                aBest = d1;
                aBestT = t1;
            }
            else if (d2 < aBest)
            {
                aBest = d2;
                aBestT = t2;
            }
            else
            {
                aStep *= 0.5;
            }
        }
        Assert.True(aBest < theTol, $"distance {aBest}");
    }

    [Fact]
    public void SameRange_EveryCase_LeavesTheCallersCurve()
    {
        // Within the tolerance: the curve itself
        LineCurve2D aLine = new(new Point2D(0, 0), new Direction2D(1, 0));
        Curve2D? aRes = null;
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(1e-7, aLine, 0, 1, 0, 1 + 1e-9, ref aRes);
        Assert.Same(aLine, aRes);

        // A line moved along itself
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(1e-7, aLine, 2, 5, 0, 3, ref aRes);
        Assert.IsType<LineCurve2D>(aRes);
        Assert.NotSame(aLine, aRes);
        Assert.True(aRes!.Value(0).IsEqual(aLine.Value(2), 1e-15));
        Assert.Equal(0.0, aLine.Location().X());

        // A circle turned (direct and indirect)
        foreach (bool aSense in new[] { true, false })
        {
            CircleCurve2D aCircle = new(new Axis2D(new Point2D(1, 1), new Direction2D(1, 0)), 2.0, aSense);
            aCircle.Freeze();
            global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(1e-7, aCircle, 1, 2, 3, 4, ref aRes);
            Assert.IsType<CircleCurve2D>(aRes);
            Assert.True(aRes!.Value(3.5).IsEqual(aCircle.Value(1.5), 1e-12));
        }

        // A trimmed curve: its basis moved, trimmed again
        TrimmedCurve2D aTrimmed = new(new LineCurve2D(new Point2D(0, 0), new Direction2D(0, 1)), 1, 3);
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(1e-7, aTrimmed, 1, 3, 10, 12, ref aRes);
        Assert.IsType<TrimmedCurve2D>(aRes);
        Assert.Equal(10.0, aRes!.FirstParameter());
        Assert.True(aRes.Value(11).IsEqual(aTrimmed.Value(2), 1e-15));

        // Another curve with the same length: a reparametrised B-spline
        EllipseCurve2D anEllipse = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 3, 1);
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(1e-7, anEllipse, 0.5, 1.5, 7, 8, ref aRes);
        BSplineCurve2D aBS = Assert.IsType<BSplineCurve2D>(aRes);
        Assert.Equal(7.0, aBS.FirstParameter());
        Assert.Equal(8.0, aBS.LastParameter());
        Assert.True(aBS.Value(7).IsEqual(anEllipse.Value(0.5), 1e-12));
        Assert.True(aBS.Value(8).IsEqual(anEllipse.Value(1.5), 1e-12));

        // Another length: a segment of the curve, reparametrised
        BSplineCurve2D aCurve = global::Kerf.Engine.Geometry.Conversion.Geom2dConvert2D.CurveToBSplineCurve(new TrimmedCurve2D(anEllipse, 0, 3));
        aCurve.Freeze();
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(1e-7, aCurve, 1, 2, 0, 10, ref aRes);
        aBS = Assert.IsType<BSplineCurve2D>(aRes);
        Assert.Equal(0.0, aBS.FirstParameter());
        Assert.Equal(10.0, aBS.LastParameter());
        Assert.True(aBS.Value(0).IsEqual(aCurve.Value(1), 1e-12));
        Assert.True(aBS.Value(10).IsEqual(aCurve.Value(2), 1e-12));
        Assert.Equal(0.0, aCurve.FirstParameter());

        Assert.ThrowsAny<FailureException>(() => global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(1e-7, null, 0, 1, 0, 2, ref aRes));
    }

    // ---------------------------------------------------------------------------------------------------------
    // BuildCurve3d

    [Fact]
    public void BuildCurve3d_OnPlane_Iso_AndApproximated()
    {
        // On a plane: To3d of the 2D curve
        PlaneSurface aPlane = new(new CoordinateSystem(new Point3D(0, 0, 1), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        CircleCurve2D aCircle = new(new Axis2D(new Point2D(0, 0), new Direction2D(1, 0)), 2.0);
        CurveOnSurface aCOS = new(new CurveAdaptor2D(aCircle, 0, 1), new SurfaceAdaptor(aPlane));
        Curve3D? aRes = null;
        global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(1e-7, aCOS, 0, 1, ref aRes, out double aMax, out double anAvg);
        CircleCurve3D aC3d = Assert.IsType<CircleCurve3D>(aRes);
        Assert.Equal(2.0, aC3d.Radius());
        Assert.Equal(0.0, aMax);

        // An isoline of a cylinder: its own isoline
        CylindricalSurface aCyl = new(new CoordinateSystem(), 3.0);
        LineCurve2D anIso = new(new Point2D(0.7, -1), new Direction2D(0, 1));
        aCOS = new CurveOnSurface(new CurveAdaptor2D(anIso, 0, 2), new SurfaceAdaptor(aCyl));
        aRes = null;
        global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(1e-7, aCOS, 0, 2, ref aRes, out aMax, out anAvg);
        Assert.IsType<BSplineCurve3D>(aRes);
        Assert.True(aRes!.Value(1.5).IsEqual(aCyl.Value(0.7, 0.5), 1e-12));

        // A helix: approximated
        LineCurve2D aHelix = new(new Point2D(0, 0), new Direction2D(1, 0.2));
        aCOS = new CurveOnSurface(new CurveAdaptor2D(aHelix, 0, 8), new SurfaceAdaptor(aCyl));
        aRes = null;
        global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(1e-6, aCOS, 0, 8, ref aRes, out aMax, out anAvg, C2, 10, 20);
        BSplineCurve3D anApprox = Assert.IsType<BSplineCurve3D>(aRes);
        Assert.True(aMax < 1e-6 && aMax > 0.0 && anAvg <= aMax);
        Assert.False(anApprox.IsFrozen);
        for (double t = 0; t <= 8; t += 0.25)
        {
            Point2D aUV = aHelix.Value(t);
            Assert.True(anApprox.Value(t).Distance(aCyl.Value(aUV.X(), aUV.Y())) < 1e-6);
        }
    }

    [Fact]
    public void isIsoLine_And_buildC3dOnIsoLine()
    {
        CurveAdaptor2D aV = new(new LineCurve2D(new Point2D(2, 0), new Direction2D(0, -1)), 0, 1);
        Assert.True(global::Kerf.Engine.Geometry.Library.GeomLib.IsIsoLine(aV, out bool anIsU, out double aParam, out bool aForward));
        Assert.True(anIsU);
        Assert.Equal(2.0, aParam);
        Assert.False(aForward);

        CurveAdaptor2D aDiagonal = new(new LineCurve2D(new Point2D(2, 0), new Direction2D(1, 1)), 0, 1);
        Assert.False(global::Kerf.Engine.Geometry.Library.GeomLib.IsIsoLine(aDiagonal, out _, out _, out _));

        // On a sphere the isoline is refused
        SurfaceAdaptor aSphere = new(new SphericalSurface(new CoordinateSystem(), 1.0));
        Assert.Null(global::Kerf.Engine.Geometry.Library.GeomLib.BuildC3dOnIsoLine(aV, aSphere, 0, 1, 1e-7, true, 2.0, false));

        // On a plane B-spline surface, the isoline reversed and reparametrised
        BSplineSurface aSheet = Sheet(false);
        aSheet.Freeze();
        CurveAdaptor2D anIso = new(new LineCurve2D(new Point2D(0.25, 1), new Direction2D(0, -1)), 0, 1);
        Curve3D? aC3d = global::Kerf.Engine.Geometry.Library.GeomLib.BuildC3dOnIsoLine(anIso, new SurfaceAdaptor(aSheet), 0, 1, 1e-7, true, 0.25, false);
        Assert.NotNull(aC3d);
        Assert.True(aC3d!.Value(0.2).IsEqual(aSheet.Value(0.25, 0.8), 1e-12));
    }

    // ---------------------------------------------------------------------------------------------------------
    // Extensions

    [Fact]
    public void AdjustExtremity_MovesEndsAndTangents_FrozenInput()
    {
        BSplineCurve3D aCurve = Wavy3d();
        aCurve.Freeze();
        BoundedCurve3D aRes = aCurve;
        Point3D P1 = new(1, 0.5, 0.2), P2 = new(6.2, -0.1, 0.0);
        Vector3D T1 = new(1, 0, 0), T2 = new(1, 1, 0);
        global::Kerf.Engine.Geometry.Library.GeomLib.AdjustExtremity(ref aRes, P1, P2, T1, T2);
        Assert.NotSame(aCurve, aRes);
        Assert.True(aRes.Value(aRes.FirstParameter()).IsEqual(P1, 1e-12));
        Assert.True(aRes.Value(aRes.LastParameter()).IsEqual(P2, 1e-12));
        aRes.D1(aRes.FirstParameter(), out _, out Vector3D aD1);
        Assert.True(aD1.IsParallel(T1, 1e-9));
        aRes.D1(aRes.LastParameter(), out _, out Vector3D aD2);
        Assert.True(aD2.IsParallel(T2, 1e-9));
        Assert.True(aCurve.Pole(1).IsEqual(new Point3D(1, Math.Sin(1), 0.3 * Math.Cos(2)), 0.0));
    }

    [Fact]
    public void ExtendCurveToPoint_ReachesThePoint_FrozenInput()
    {
        for (int aCont = 1; aCont <= 3; aCont++)
        {
            foreach (bool anAfter in new[] { true, false })
            {
                BSplineCurve3D aCurve = Wavy3d();
                aCurve.Freeze();
                BoundedCurve3D aRes = aCurve;
                Point3D aTarget = anAfter ? new Point3D(7.5, -0.5, 0.5) : new Point3D(-0.2, 0.5, 0.0);
                global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref aRes, aTarget, aCont, anAfter);
                BSplineCurve3D aBS = Assert.IsType<BSplineCurve3D>(aRes);
                Assert.False(aBS.IsFrozen);
                Point3D anEnd = anAfter ? aBS.EndPoint() : aBS.StartPoint();
                Assert.True(anEnd.IsEqual(aTarget, 1e-9), $"cont {aCont} after {anAfter}");
                Assert.Equal(3.0, aCurve.LastParameter());
            }
        }

        // Nothing for a continuity out of 1..3, or a point on the end
        BoundedCurve3D aSame = Wavy3d();
        BoundedCurve3D aKept = aSame;
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref aKept, new Point3D(10, 0, 0), 0, true);
        Assert.Same(aSame, aKept);
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendCurveToPoint(ref aKept, aSame.EndPoint(), 1, true);
        Assert.Same(aSame, aKept);
    }

    [Fact]
    public void ExtendSurfByLength_TrimmedPlaneAndBSpline_FrozenInput()
    {
        // A trimmed plane is re-trimmed
        TrimmedSurface aTrimmed = new(new PlaneSurface(new CoordinateSystem()), 0, 1, 0, 2);
        BoundedSurface aRes = aTrimmed;
        global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aRes, 0.5, 1, true, true);
        aRes.Bounds(out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 1.5, 0.0, 2.0), (u1, u2, v1, v2));

        // A B-spline surface becomes a larger B-spline; the caller's is unchanged
        foreach (bool aRational in new[] { false, true })
        {
            BSplineSurface aSheet = Sheet(aRational);
            aSheet.Freeze();
            Point3D aCorner = aSheet.Value(1, 0.5);
            BoundedSurface aBS = aSheet;
            global::Kerf.Engine.Geometry.Library.GeomLib.ExtendSurfByLength(ref aBS, 1.0, 1, true, true);
            BSplineSurface anExt = Assert.IsType<BSplineSurface>(aBS);
            Assert.NotSame(aSheet, anExt);
            aSheet.Bounds(out _, out double aSheetU2, out _, out _);
            Assert.Equal(1.0, aSheetU2);
            anExt.Bounds(out _, out double anU2, out _, out _);
            Assert.True(anU2 > 1.0);
            Assert.True(anExt.Value(1, 0.5).IsEqual(aCorner, 1e-9));
            Assert.True(anExt.Value(anU2, 0.5).Distance(aCorner) > 0.5, "extended by about the length");
        }
    }

    [Fact]
    public void CancelDenominatorDerivative_KeepsTheGeometry_CopiesTheInput()
    {
        // Weights proportional on the U boundaries but not treated yet
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 3);
        NCollection_Array2<double> aWeights = new(1, 4, 1, 3);
        double[] aU = [1.0, 1.6, 0.8, 1.0];
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                aPoles[i, j] = new Point3D(i, j, 0.3 * (i - 2) * (j - 2));
                aWeights[i, j] = aU[i - 1] * (j == 2 ? 1.2 : 1.0);
            }
        }
        NCollection_Array1<int> aUMults = NCollection_Array1<int>.From([4, 4]);
        NCollection_Array1<int> aVMults = NCollection_Array1<int>.From([3, 3]);
        BSplineSurface aSurf = new(aPoles, aWeights, Reals(2, 5), Reals(0, 1), aUMults, aVMults, 3, 2);
        aSurf.Freeze();
        BSplineSurface aRes = aSurf;
        global::Kerf.Engine.Geometry.Library.GeomLib.CancelDenominatorDerivative(ref aRes, true, false);
        Assert.NotSame(aSurf, aRes);
        Assert.Equal(2.0, aRes.UKnot(1));
        Assert.Equal(5.0, aRes.UKnot(aRes.UKnotCount));
        for (double u = 2; u <= 5; u += 0.5)
        {
            for (double v = 0; v <= 1; v += 0.25)
            {
                Assert.True(aRes.Value(u, v).IsEqual(aSurf.Value(u, v), 1e-9), $"({u}, {v})");
            }
        }
        // The denominator's U derivative vanishes on the boundaries
        foreach (double u in new[] { 2.0, 5.0 })
        {
            for (double v = 0; v <= 1; v += 0.5)
            {
                double aD = Denominator(aRes, u, v), aDd = (Denominator(aRes, u + (u == 2 ? 1e-6 : -1e-6), v) - aD) / 1e-6;
                Assert.True(Math.Abs(aDd) < 1e-4 * Math.Abs(aD), $"dD/du at ({u}, {v}): {aDd}");
            }
        }

        // Nothing requested: the same surface
        BSplineSurface aKept = aSurf;
        global::Kerf.Engine.Geometry.Library.GeomLib.CancelDenominatorDerivative(ref aKept, false, false);
        Assert.Same(aSurf, aKept);
    }

    [Fact]
    public void CancelDenominatorDerivative_Untreatable_LeftReparametrised_OCCT()
    {
        // A non-rational surface can't be treated, but OCCT has already reparametrised its U knots to [0, 1] and
        // doesn't restore them (occt-issues.md). The caller's surface is unchanged in the port.
        NCollection_Array2<Point3D> aPoles = new(1, 4, 1, 2);
        for (int i = 1; i <= 4; i++)
        {
            aPoles[i, 1] = new Point3D(i, 0, 0);
            aPoles[i, 2] = new Point3D(i, 1, 0.1 * i);
        }
        NCollection_Array1<int> aUMults = NCollection_Array1<int>.From([4, 4]);
        NCollection_Array1<int> aVMults = NCollection_Array1<int>.From([2, 2]);
        BSplineSurface aSurf = new(aPoles, Reals(2, 5), Reals(0, 1), aUMults, aVMults, 3, 1);
        BSplineSurface aRes = aSurf;
        global::Kerf.Engine.Geometry.Library.GeomLib.CancelDenominatorDerivative(ref aRes, true, false);
        Assert.Equal(0.0, aRes.UKnot(1));
        Assert.Equal(1.0, aRes.UKnot(2));
        Assert.True(aRes.Value(1.0, 0.5).IsEqual(aSurf.Value(5.0, 0.5), 1e-12));
        Assert.Equal(2.0, aSurf.UKnot(1));
    }

    private static double Denominator(BSplineSurface theS, double theU, double theV)
    {
        Grid<double> aW = theS.WeightsArray();
        NCollection_Array2<Point3D> aP = new(1, theS.UPoleCount, 1, theS.VPoleCount);
        for (int i = 1; i <= theS.UPoleCount; i++)
        {
            for (int j = 1; j <= theS.VPoleCount; j++)
            {
                aP[i, j] = new Point3D(aW[i - 1, j - 1], 0, 0);
            }
        }
        NCollection_Array1<double> aUK = new(1, theS.UKnotCount), aVK = new(1, theS.VKnotCount);
        NCollection_Array1<int> aUM = new(1, theS.UKnotCount), aVM = new(1, theS.VKnotCount);
        theS.UKnots(aUK);
        theS.VKnots(aVK);
        theS.UMultiplicities(aUM);
        theS.VMultiplicities(aVM);
        return new BSplineSurface(aP, aUK, aVK, aUM, aVM, theS.UDegree(), theS.VDegree()).Value(theU, theV).X();
    }

    // ---------------------------------------------------------------------------------------------------------
    // Normals and closure

    [Fact]
    public void NormEstim_RegularSingularAndFailing()
    {
        Direction3D aNorm = new();
        Assert.Equal(0, global::Kerf.Engine.Geometry.Library.GeomLib.NormEstim(new PlaneSurface(new CoordinateSystem()), new Point2D(1, 2), 1e-7, ref aNorm));
        Assert.True(aNorm.IsEqual(new Direction3D(0, 0, 1), 1e-15));

        // The pole of a sphere: from D2
        SphericalSurface aSphere = new(new CoordinateSystem(), 2.0);
        aNorm = new Direction3D(1, 0, 0);
        Assert.Equal(1, global::Kerf.Engine.Geometry.Library.GeomLib.NormEstim(aSphere, new Point2D(0.3, Math.PI / 2), 1e-7, ref aNorm));
        Assert.True(aNorm.IsParallel(new Direction3D(0, 0, 1), 1e-6));

        // The apex of a cone: no normal; the result is left unchanged
        ConicalSurface aCone = new(new CoordinateSystem(), 0.5, 1.0);
        double anApexV = -1.0 / Math.Sin(0.5);
        aNorm = new Direction3D(1, 0, 0);
        Assert.True(global::Kerf.Engine.Geometry.Library.GeomLib.NormEstim(aCone, new Point2D(0.3, anApexV), 1e-7, ref aNorm) >= 2);
        Assert.Equal(new Direction3D(1, 0, 0), aNorm);
    }

    [Fact]
    public void IsClosed_EverySurfaceKind()
    {
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(new PlaneSurface(new CoordinateSystem()), 1e-7, out bool aU, out bool aV);
        Assert.False(aU || aV);
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(new CylindricalSurface(new CoordinateSystem(), 1), 1e-7, out aU, out aV);
        Assert.True(aU && !aV);
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(new TrimmedSurface(new CylindricalSurface(new CoordinateSystem(), 1), 0, 3, 0, 1),
                         1e-7, out aU, out aV);
        Assert.False(aU || aV);
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(new ConicalSurface(new CoordinateSystem(), 0.3, 1), 1e-7, out aU, out aV);
        Assert.True(aU && !aV);
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(new SphericalSurface(new CoordinateSystem(), 1), 1e-7, out aU, out aV);
        Assert.True(aU && !aV);
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(new ToroidalSurface(new CoordinateSystem(), 3, 1), 1e-7, out aU, out aV);
        Assert.True(aU && aV);
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(new RevolutionSurface(new LineCurve3D(new Point3D(1, 0, 0), new Direction3D(0.2, 0, 1)),
                                                      new Axis3D(new Point3D(), new Direction3D(0, 0, 1))),
                         1e-7, out aU, out aV);
        Assert.True(aU && !aV);

        // A B-spline converted from a whole cylinder is closed in U, its poles weighted
        BSplineSurface aBS = global::Kerf.Engine.Geometry.Conversion.GeomConvert.SurfaceToBSplineSurface(
            new TrimmedSurface(new CylindricalSurface(new CoordinateSystem(), 1), 0, 2 * Math.PI, 0, 1));
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(aBS, 1e-7, out aU, out aV);
        Assert.True(aU && !aV);
        Assert.True(global::Kerf.Engine.Geometry.Library.GeomLib.IsBSplUClosed(aBS, 0, 2 * Math.PI, 1e-7));
        Assert.False(global::Kerf.Engine.Geometry.Library.GeomLib.IsBSplVClosed(aBS, 0, 1, 1e-7));

        NCollection_Array2<Point3D> aPoles = new(1, 3, 1, 2);
        for (int j = 1; j <= 2; j++)
        {
            aPoles[1, j] = new Point3D(0, j, 0);
            aPoles[2, j] = new Point3D(1, j, 1);
            aPoles[3, j] = new Point3D(0, j, 0);
        }
        BezierSurface aBz = new(aPoles);
        Assert.True(global::Kerf.Engine.Geometry.Library.GeomLib.IsBzUClosed(aBz, 0, 1, 1e-7));
        Assert.False(global::Kerf.Engine.Geometry.Library.GeomLib.IsBzVClosed(aBz, 0, 1, 1e-7));
        global::Kerf.Engine.Geometry.Library.GeomLib.IsClosed(aBz, 1e-7, out aU, out aV);
        Assert.True(aU && !aV);
    }

    // ---------------------------------------------------------------------------------------------------------
    // Small classes

    [Fact]
    public void LogSample_PolyFunc_DenominatorMultiplier()
    {
        global::Kerf.Engine.Geometry.Library.LogSample aSample = new(1.0, 11.0, 10);
        Assert.Equal(1.0, aSample.GetParameter(1));
        Assert.Equal(11.0, aSample.GetParameter(10));
        Assert.Equal(0.0 + Math.Exp(Math.Log(10.0) / 10 * 5), aSample.GetParameter(5), 1e-15);
        Assert.ThrowsAny<OutOfRangeException>(() => aSample.GetParameter(11));

        // p(x) = 1 + 2x + 3x^2 + 4x^3: the function is p'(x) = 2 + 6x + 12x^2, its derivative 6 + 24x
        math_Vector aCoeffs = new(1, 4);
        aCoeffs[1] = 1;
        aCoeffs[2] = 2;
        aCoeffs[3] = 3;
        aCoeffs[4] = 4;
        global::Kerf.Engine.Geometry.Library.PolyFunc aF = new(aCoeffs);
        Assert.True(aF.Value(2.0, out double aV));
        Assert.Equal(2 + 12 + 48, aV, 1e-12);
        Assert.True(aF.Derivative(2.0, out double aD));
        Assert.Equal(6 + 48, aD, 1e-12);
        Assert.True(aF.Values(-1.0, out aV, out aD));
        Assert.Equal(2 - 6 + 12, aV, 1e-12);
        Assert.Equal(6 - 24, aD, 1e-12);

        // A non-rational surface: the multiplier is the cubic joining 1 and 1 with zero end slopes, i.e. 1
        BSplineSurface aSheet = Sheet(false);
        NCollection_Array1<double> aFlat = Reals(0, 0, 0, 0, 1, 1, 1, 1);
        global::Kerf.Engine.Geometry.Library.DenominatorMultiplier aMult = new(aSheet, aFlat);
        Assert.Equal(1.0, aMult.Value(0.3, 0.6), 1e-14);
    }

    [Fact]
    public void MakeCurvefromApprox_BuildsTheCurves()
    {
        // BuildCurve3d's approximation of a helix, through MakeCurvefromApprox
        CylindricalSurface aCyl = new(new CoordinateSystem(), 3.0);
        CurveOnSurface aCOS = new(new CurveAdaptor2D(new LineCurve2D(new Point2D(0, 0), new Direction2D(1, 0.3)), 0, 3),
                                            new SurfaceAdaptor(aCyl));
        Curve3D? aRes = null;
        global::Kerf.Engine.Geometry.Library.GeomLib.BuildCurve3d(1e-5, aCOS, 0, 3, ref aRes, out double aMax, out _);
        BSplineCurve3D aBS = Assert.IsType<BSplineCurve3D>(aRes);
        Assert.True(aMax < 1e-5);
        Assert.Equal(0.0, aBS.FirstParameter());
        Assert.Equal(3.0, aBS.LastParameter());
    }
}
