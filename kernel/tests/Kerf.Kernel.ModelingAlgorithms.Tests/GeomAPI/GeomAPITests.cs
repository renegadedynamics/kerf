// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Known answers of the GeomAPI projection, extrema, interpolation and approximation classes.</summary>
public class GeomAPITests
{
    private const double Tol = 1.0e-9;

    private static CircleCurve3D Circle(double theR) => new(Constants.XOY(), theR);

    private static LineCurve3D Line(double x, double y, double z, double dx, double dy, double dz)
        => new(new Point3D(x, y, z), new Direction3D(dx, dy, dz));

    [Fact]
    public void To3d_LineInPlane()
    {
        Plane3D aPln = new(new CoordinateSystem(new Point3D(0, 0, 2), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        LineCurve2D aL2d = new(new Point2D(1, 2), new Direction2D(0, 1));
        Curve3D aC = Projections.To3d(aL2d, aPln);
        LineCurve3D aLine = Assert.IsType<LineCurve3D>(aC);
        Assert.True(aLine.Value(0.0).IsEqual(new Point3D(1, 2, 2), Tol));
        Assert.True(aLine.Line.Direction.IsEqual(new Direction3D(0, 1, 0), Tol));
    }

    [Fact]
    public void To2d_ProjectsALineOnThePlane()
    {
        // a line above the XOY plane projects to the 2D line along u through the origin
        Curve2D? aC = Projections.To2d(Line(0, 0, 3, 1, 0, 0), new Plane3D());
        LineCurve2D aLine = Assert.IsType<LineCurve2D>(aC);
        Assert.True(aLine.Value(0.0).IsEqual(new Point2D(0, 0), Tol));
        Assert.True(aLine.Line.Direction.IsEqual(new Direction2D(1, 0), Tol));
    }

    [Fact]
    public void ProjectPointOnCurve_Circle()
    {
        ProjectPointOnCurve aProj = new(new Point3D(10, 0, 3), Circle(5));
        Assert.Equal(2, aProj.NbPoints());
        Assert.Equal(2, (int)aProj);
        Assert.Equal(Math.Sqrt(34.0), aProj.LowerDistance(), Tol);
        Assert.Equal(Math.Sqrt(34.0), (double)aProj, Tol);
        Assert.Equal(0.0, aProj.LowerDistanceParameter(), Tol);
        Assert.True(aProj.NearestPoint().IsEqual(new Point3D(5, 0, 0), Tol));
        Assert.True(((Point3D)aProj).IsEqual(new Point3D(5, 0, 0), Tol));
        double aMax = Math.Max(aProj.Distance(1), aProj.Distance(2));
        Assert.Equal(Math.Sqrt(15 * 15 + 9.0), aMax, Tol);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.OutOfRangeException>(() => aProj.Point(3));
    }

    [Fact]
    public void ProjectPointOnCurve_LineAndRange()
    {
        LineCurve3D aLine = Line(0, 0, 0, 0, 0, 1);
        ProjectPointOnCurve aProj = new(new Point3D(1, 2, 3), aLine);
        Assert.Equal(1, aProj.NbPoints());
        aProj.Parameter(1, out double aU);
        Assert.Equal(3.0, aU, Tol);
        Assert.Equal(Math.Sqrt(5.0), aProj.Distance(1), Tol);

        // The point projects outside [5, 6]: no extremum inside the range
        ProjectPointOnCurve aRanged = new(new Point3D(1, 2, 3), aLine, 5.0, 6.0);
        Assert.Equal(0, aRanged.NbPoints());
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aRanged.LowerDistance());

        // Init on a range, then several points
        ProjectPointOnCurve aReused = new();
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aReused.NearestPoint());
        aReused.Init(aLine, -10.0, 10.0);
        Assert.Equal(0, aReused.NbPoints());
        aReused.Perform(new Point3D(0, 1, 4));
        Assert.Equal(4.0, aReused.LowerDistanceParameter(), Tol);
        aReused.Perform(new Point3D(0, 1, -7));
        Assert.Equal(-7.0, aReused.LowerDistanceParameter(), Tol);
        Assert.Same(aReused.Extrema(), aReused.Extrema());
    }

    [Fact]
    public void ProjectPointOnSurf_SphereAndPlane()
    {
        SphericalSurface aSphere = new(new CoordinateSystem(Constants.XOY()), 2.0);
        ProjectPointOnSurface aProj = new(new Point3D(0, 0, 5), aSphere);
        Assert.True(aProj.IsDone);
        Assert.Equal(3.0, aProj.LowerDistance(), Tol);
        Assert.True(aProj.NearestPoint().IsEqual(new Point3D(0, 0, 2), Tol));
        aProj.LowerDistanceParameters(out _, out double aV);
        Assert.Equal(Math.PI / 2, aV, Tol);

        PlaneSurface aPlane = new(new CoordinateSystem(Constants.XOY()));
        ProjectPointOnSurface aOnPlane = new(new Point3D(1, 2, 3), aPlane);
        Assert.Equal(1, aOnPlane.NbPoints());
        aOnPlane.Parameters(1, out double aU1, out double aV1);
        Assert.Equal(1.0, aU1, Tol);
        Assert.Equal(2.0, aV1, Tol);
        Assert.Equal(3.0, (double)aOnPlane, Tol);

        // Init on a range, then several points; the minimum flag is kept across Init
        ProjectPointOnSurface aReused = new();
        aReused.SetExtremaFlag(ExtFlag.ExtFlag_MIN);
        aReused.Init(aSphere, 0.0, 2 * Math.PI, -Math.PI / 2, Math.PI / 2);
        Assert.False(aReused.IsDone);
        aReused.Perform(new Point3D(5, 0, 0));
        Assert.True(aReused.NbPoints() >= 1); // the minimum, found at U = 0 and on the seam at U = 2 pi
        Assert.Equal(3.0, aReused.LowerDistance(), Tol);
        aReused.Init(new Point3D(0, -7, 0), aSphere, ExtAlgo.ExtAlgo_Tree);
        Assert.True(aReused.NbPoints() >= 1);
        Assert.Equal(5.0, aReused.LowerDistance(), Tol);
    }

    [Fact]
    public void ExtremaCurveCurve_SkewAndParallelLines()
    {
        ExtremaCurveCurve aSkew = new(Line(0, 0, 0, 1, 0, 0), Line(3, 0, 1, 0, 1, 0));
        Assert.False(aSkew.IsParallel);
        Assert.Equal(1, aSkew.NbExtrema());
        Assert.Equal(1.0, aSkew.LowerDistance(), Tol);
        aSkew.NearestPoints(out Point3D aP1, out Point3D aP2);
        Assert.True(aP1.IsEqual(new Point3D(3, 0, 0), Tol));
        Assert.True(aP2.IsEqual(new Point3D(3, 0, 1), Tol));
        aSkew.LowerDistanceParameters(out double aU1, out double aU2);
        Assert.Equal(3.0, aU1, Tol);
        Assert.Equal(0.0, aU2, Tol);
        Assert.Equal(1.0, aSkew.TotalLowerDistance(), Tol);

        ExtremaCurveCurve aParallel = new(Line(0, 0, 0, 1, 0, 0), Line(0, 2, 0, 1, 0, 0));
        Assert.True(aParallel.IsParallel);
        Assert.Equal(2.0, aParallel.LowerDistance(), Tol);
        Assert.Equal(2.0, aParallel.TotalLowerDistance(), Tol);
        Point3D aT1 = new(), aT2 = new();
        Assert.False(aParallel.TotalNearestPoints(ref aT1, ref aT2));
    }

    [Fact]
    public void ExtremaCurveCurve_TotalReachesTheEnds()
    {
        // Two segments whose lines meet outside both, so ExtCC finds no extremum inside them: the minimum is between
        // end points, (1, 0, 0) and (0, 1, 0) at sqrt(2)
        TrimmedCurve3D aC1 = new(Line(0, 0, 0, 1, 0, 0), 1.0, 2.0);
        TrimmedCurve3D aC2 = new(Line(0, 0, 0, 0, 1, 0), 1.0, 2.0);
        ExtremaCurveCurve anExt = new(aC1, aC2);
        Assert.Equal(0, anExt.NbExtrema());
        // OCCT (occt-issues.md; decision 1462) compared each end pair with the first total distance (RealLast
        // here), never updated, so the last pair, (2, 0, 0) and (0, 2, 0), won at sqrt(8).
        Assert.Equal(Math.Sqrt(2.0), anExt.TotalLowerDistance(), Tol);
        Point3D aP1 = new(), aP2 = new();
        Assert.True(anExt.TotalNearestPoints(ref aP1, ref aP2));
        Assert.True(aP1.IsEqual(new Point3D(1, 0, 0), Tol));
        Assert.True(aP2.IsEqual(new Point3D(0, 1, 0), Tol));
        double aU1 = 0, aU2 = 0;
        Assert.True(anExt.TotalLowerDistanceParameters(ref aU1, ref aU2));
        Assert.Equal(1.0, aU1, Tol);
        Assert.Equal(1.0, aU2, Tol);
    }

    [Fact]
    public void ExtremaCurveCurve_TotalOnRangesReachesTheRangesEnds()
    {
        // The segments of the test above, as ranges [1, 2] of segments [0, 3]: the minimum is sqrt(2), between
        // (1, 0, 0) and (0, 1, 0), at parameters (1, 1). OCCT (occt-issues.md; decision 1463) loaded the whole
        // curves, so it paired the ranges' end points with the curves' own end parameters and projected them onto
        // the whole curves: 1, between (1, 0, 0) and the origin (outside the second range), at parameters (0, 0).
        TrimmedCurve3D aC1 = new(Line(0, 0, 0, 1, 0, 0), 0.0, 3.0);
        TrimmedCurve3D aC2 = new(Line(0, 0, 0, 0, 1, 0), 0.0, 3.0);
        ExtremaCurveCurve anExt = new(aC1, aC2, 1.0, 2.0, 1.0, 2.0);
        Assert.Equal(0, anExt.NbExtrema());
        Assert.Equal(Math.Sqrt(2.0), anExt.TotalLowerDistance(), Tol);
        double aU1 = -1, aU2 = -1;
        Assert.True(anExt.TotalLowerDistanceParameters(ref aU1, ref aU2));
        Point3D aP1 = new(), aP2 = new();
        Assert.True(anExt.TotalNearestPoints(ref aP1, ref aP2));
        Assert.True(aP1.IsEqual(new Point3D(1, 0, 0), Tol));
        Assert.True(aP2.IsEqual(new Point3D(0, 1, 0), Tol));
        Assert.Equal(1.0, aU1, Tol);
        Assert.Equal(1.0, aU2, Tol);
    }

    [Fact]
    public void ExtremaCurveSurface_LineAndSphere()
    {
        SphericalSurface aSphere = new(new CoordinateSystem(Constants.XOY()), 2.0);
        ExtremaCurveSurface anExt = new(Line(0, 0, 5, 1, 0, 0), aSphere);
        Assert.True(anExt.NbExtrema() > 0);
        Assert.Equal(3.0, anExt.LowerDistance(), Tol);
        anExt.NearestPoints(out Point3D aPC, out Point3D aPS);
        Assert.True(aPC.IsEqual(new Point3D(0, 0, 5), Tol));
        Assert.True(aPS.IsEqual(new Point3D(0, 0, 2), Tol));
        anExt.LowerDistanceParameters(out double aW, out _, out double aV);
        Assert.Equal(0.0, aW, Tol);
        Assert.Equal(Math.PI / 2, aV, Tol);
    }

    [Fact]
    public void ExtremaCurveSurface_ParallelLineAndPlane()
    {
        PlaneSurface aPlane = new(new CoordinateSystem(Constants.XOY()));
        ExtremaCurveSurface anExt = new(Line(0, 0, 4, 1, 1, 0), aPlane);
        Assert.True(anExt.IsParallel);
        Assert.Equal(4.0, anExt.LowerDistance(), Tol);
        Assert.Equal(4.0, (double)anExt, Tol);
    }

    [Fact]
    public void ExtremaSurfaceSurface_SphereAndPlane()
    {
        SphericalSurface aSphere = new(new CoordinateSystem(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)), 1.0);
        PlaneSurface aPlane = new(new CoordinateSystem(Constants.XOY()));
        ExtremaSurfaceSurface anExt = new(aSphere, aPlane, 0.0, 2 * Math.PI, -Math.PI / 2, Math.PI / 2,
                                                  -10.0, 10.0, -10.0, 10.0);
        Assert.False(anExt.IsParallel);
        Assert.True(anExt.NbExtrema() > 0);
        Assert.Equal(4.0, anExt.LowerDistance(), Tol);
        anExt.NearestPoints(out Point3D aP1, out Point3D aP2);
        // The general solver (GenExtSS) refines the points to about 1e-8
        Assert.True(aP1.IsEqual(new Point3D(0, 0, 4), 1.0e-6));
        Assert.True(aP2.IsEqual(new Point3D(0, 0, 0), 1.0e-6));

        PlaneSurface aPlane2 = new(new CoordinateSystem(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)));
        ExtremaSurfaceSurface aParallel = new(aPlane, aPlane2);
        Assert.True(aParallel.IsParallel);
        Assert.Equal(3.0, aParallel.LowerDistance(), Tol);
    }

    [Fact]
    public void Interpolate_PassesThroughPoints()
    {
        foreach (bool aPeriodic in new[] { false, true })
        {
            foreach (int aNb in new[] { 2, 3, 6 })
            {
                NCollection_HArray1<Point3D> aPnts = new(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    double t = 2 * Math.PI * (i - 1) / aNb;
                    aPnts.SetValue(i, new Point3D(Math.Cos(t), Math.Sin(t), 0.1 * i));
                }
                ApiInterpolate anInterp = new(aPnts, aPeriodic, global::Kerf.Engine.Foundation.Precision.Confusion());
                anInterp.Perform();
                Assert.True(anInterp.IsDone);
                BSplineCurve3D aCurve = anInterp.Curve();
                Assert.True(aCurve.IsFrozen);
                Assert.Equal(aPeriodic, aCurve.IsPeriodic);
                // Chord-length parameters: the points are at the cumulated distances
                double aU = 0.0;
                for (int i = 1; i <= aNb; i++)
                {
                    if (i > 1)
                    {
                        aU += aPnts[i - 1].DistanceTo(aPnts[i]);
                    }
                    Assert.True(aCurve.Value(aU).IsEqual(aPnts[i], 1.0e-9), $"{aPeriodic} {aNb} point {i}");
                }
            }
        }
    }

    [Fact]
    public void Interpolate_TangentsAndErrors()
    {
        NCollection_HArray1<Point3D> aPnts = new(1, 4);
        aPnts.SetValue(1, new Point3D(0, 0, 0));
        aPnts.SetValue(2, new Point3D(1, 1, 0));
        aPnts.SetValue(3, new Point3D(2, 0, 0));
        aPnts.SetValue(4, new Point3D(3, 1, 0));
        NCollection_HArray1<double> aParams = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aParams.SetValue(i, i);
        }

        ApiInterpolate anInterp = new(aPnts, aParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        NCollection_Array1<Vector3D> aTangents = new(1, 4);
        NCollection_HArray1<bool> aFlags = new(1, 4);
        aFlags.Init(false);
        aTangents[2] = new Vector3D(1, 0, 0);
        aFlags[2] = true;
        anInterp.Load(aTangents, aFlags, false);
        anInterp.Perform();
        Assert.True(anInterp.IsDone);
        BSplineCurve3D aCurve = anInterp.Curve();
        aCurve.D1(2.0, out Point3D aP, out Vector3D aD1);
        Assert.True(aP.IsEqual(aPnts[2], 1.0e-9));
        Assert.True(aD1.IsEqual(new Vector3D(1, 0, 0), 1.0e-9, 1.0e-9));
        // As in OCCT, Perform sets the flags of the end tangents it computes in the caller's array
        Assert.True(aFlags[1]);
        Assert.True(aFlags[4]);

        // Coincident points, unordered parameters, a periodic interpolation without the closing parameter
        NCollection_HArray1<Point3D> aSame = new(1, 2);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new ApiInterpolate(aSame, false, global::Kerf.Engine.Foundation.Precision.Confusion()));
        aParams.SetValue(3, 1.5);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new ApiInterpolate(aPnts, aParams, false, 1.0e-7));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new ApiInterpolate(aPnts, aParams, true, 1.0e-7));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => new ApiInterpolate(aPnts, false, 1.0e-7).Curve());
    }

    [Fact]
    public void PointsToBSpline_Approximates()
    {
        NCollection_Array1<Point3D> aPoints = new(1, 20);
        for (int i = 1; i <= 20; i++)
        {
            double t = (i - 1) / 19.0;
            aPoints[i] = new Point3D(t, t * t, Math.Sin(3 * t));
        }
        PointsToBSpline anApprox = new(aPoints);
        Assert.True(anApprox.IsDone);
        BSplineCurve3D aCurve = anApprox.Curve();
        Assert.True(aCurve.IsFrozen);
        Assert.Same(aCurve, (BSplineCurve3D?)anApprox);
        foreach (Point3D aP in aPoints)
        {
            ProjectPointOnCurve aProj = new(aP, aCurve);
            Assert.True(aProj.LowerDistance() < 1.0e-3);
        }

        NCollection_Array1<double> aParams = new(1, 20);
        for (int i = 1; i <= 20; i++)
        {
            aParams[i] = 10.0 + i;
        }
        anApprox.Init(aPoints, aParams);
        Assert.True(anApprox.IsDone);
        Assert.Equal(11.0, anApprox.Curve().FirstParameter, Tol);
        Assert.Equal(30.0, anApprox.Curve().LastParameter, Tol);

        anApprox.Init(aPoints, 1.0, 0.0, 0.0);
        Assert.True(anApprox.IsDone);

        EngineAssert.Throws<global::Kerf.Engine.Foundation.OutOfRangeException>(() => anApprox.Init(aPoints, new NCollection_Array1<double>(1, 3)));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => new PointsToBSpline().Curve());
    }
}
