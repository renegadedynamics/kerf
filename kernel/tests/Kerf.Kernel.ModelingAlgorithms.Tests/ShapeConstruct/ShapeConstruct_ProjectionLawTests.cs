// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Construct;
using Kerf.Geometry;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeConstruct_ProjectCurveOnSurface on the hard cases of periodic and degenerate surfaces: seams, poles, apexes,
/// closed isolines, uneven parametrisation and repeated points. Two laws: the surface at the pcurve's value is the
/// curve's point, at every sampled parameter; and the pcurve doesn't jump, away from a pole or apex, where a pcurve
/// has to move along the degenerate side.
/// </summary>
public class ShapeConstruct_ProjectionLawTests
{
    private static CoordinateSystem Z() => new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0));

    private static Curve2D Project(Surface theSurface, Curve3D theCurve, double theF, double theL, double thePrec = 1e-5, int theAdjust = -1)
    {
        var aSas = new SurfaceAnalysis(theSurface);
        var aProj = new ProjectCurveOnSurface();
        aProj.Init(aSas, thePrec);
        if (theAdjust >= 0)
        {
            aProj.AdjustOverDegenMode() = theAdjust;
        }
        Assert.True(aProj.Perform(theCurve, theF, theL, out Curve2D? aPCurve));
        Assert.NotNull(aPCurve);
        Assert.True(aProj.Status(Done));
        return aPCurve;
    }

    private static void AssertOnCurve(Surface theSurface, Curve3D theCurve, Curve2D thePCurve, double theF, double theL, double theTol)
    {
        const int N = 60;
        for (int i = 0; i <= N; i++)
        {
            double t = theF + i * (theL - theF) / N;
            Point2D aUV = thePCurve.Value(t);
            Point3D aOnSurface = theSurface.Value(aUV.X, aUV.Y);
            double aDist = aOnSurface.DistanceTo(theCurve.Value(t));
            Assert.True(aDist < theTol, $"at t = {t}: surface at {aUV} is {aDist} from the curve");
        }
    }

    private static void AssertNoJumps(Curve2D thePCurve, double theF, double theL, double theMaxStep)
    {
        const int N = 200;
        Point2D aPrev = thePCurve.Value(theF);
        for (int i = 1; i <= N; i++)
        {
            Point2D aNext = thePCurve.Value(theF + i * (theL - theF) / N);
            double aStep = aPrev.DistanceTo(aNext);
            Assert.True(aStep < theMaxStep, $"the pcurve jumps by {aStep} before t = {theF + i * (theL - theF) / N}");
            aPrev = aNext;
        }
    }

    // A spline through the points of the surface at (u(s), v(s)), s from 0 to 1, with its parameter s
    private static BSplineCurve3D Through(Surface theSurface, Func<double, (double U, double V)> theUV, int theN = 80)
    {
        var aPoints = new NCollection_Array1<Point3D>(1, theN + 1);
        var aParams = new NCollection_Array1<double>(1, theN + 1);
        for (int i = 0; i <= theN; i++)
        {
            double s = (double)i / theN;
            (double u, double v) = theUV(s);
            aPoints[i + 1] = theSurface.Value(u, v);
            aParams[i + 1] = s;
        }
        return new PointsToBSpline(aPoints, aParams, 3, 8, Continuity.C2, 1e-5).Curve();
    }

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    public void ACurveOverASpheresPole_StaysOnTheCurve_EitherWayRoundTheSeam(int theAdjust)
    {
        // a meridian in the XZ plane, over the north pole, from one side of the sphere to the other
        var aSphere = new SphericalSurface(Z(), 5);
        BSplineCurve3D aCurve = Through(aSphere, s => s < 0.5
            ? (0, Math.PI / 2 * (0.2 + 1.6 * s))
            : (Math.PI, Math.PI / 2 * (0.2 + 1.6 * (1 - s))));
        Curve2D aPCurve = Project(aSphere, aCurve, 0, 1, 1e-4, theAdjust);
        // the sharpest the pcurve can be next to the pole is the spacing of its points, so the worst is there
        AssertOnCurve(aSphere, aCurve, aPCurve, 0, 1, 0.3);
        AssertOnCurve(aSphere, aCurve, aPCurve, 0, 0.3, 0.03);
        AssertOnCurve(aSphere, aCurve, aPCurve, 0.7, 1, 0.03);
    }

    [Fact]
    public void ACurveEndingAtAPole_GetsAnEndOnThePolesLatitude()
    {
        // from the equator up to the north pole: the end's U is undefined, and taken from the rest of the curve
        var aSphere = new SphericalSurface(Z(), 5);
        BSplineCurve3D aCurve = Through(aSphere, s => (1.0 + 0.3 * s, s * Math.PI / 2));
        Curve2D aPCurve = Project(aSphere, aCurve, 0, 1, 1e-4);
        AssertOnCurve(aSphere, aCurve, aPCurve, 0, 1, 0.01);
        Point2D anEnd = aPCurve.Value(1);
        Assert.Equal(Math.PI / 2, anEnd.Y, 3);
        Assert.InRange(anEnd.X, 0.9, 1.6);
        AssertNoJumps(aPCurve, 0, 1, 0.2);
    }

    [Fact]
    public void ACurveStartingAtAPole_GetsAStartOnThePolesLatitude()
    {
        var aSphere = new SphericalSurface(Z(), 5);
        BSplineCurve3D aCurve = Through(aSphere, s => (1.0 + 0.3 * s, (1 - s) * -Math.PI / 2));
        Curve2D aPCurve = Project(aSphere, aCurve, 0, 1, 1e-4);
        AssertOnCurve(aSphere, aCurve, aPCurve, 0, 1, 0.01);
        Assert.Equal(-Math.PI / 2, aPCurve.Value(0).Y, 3);
        Assert.InRange(aPCurve.Value(0).X, 0.9, 1.6);
    }

    [Theory]
    [InlineData(-0.5)]
    [InlineData(5.5)]
    public void AHelixOverACylindersSeam_IsContinuousAcrossIt(double theStart)
    {
        // a turn and a half, crossing U = 0 more than once
        var aCylinder = new CylindricalSurface(Z(), 5);
        double aTurns = 3 * Math.PI;
        BSplineCurve3D aCurve = Through(aCylinder, s => (theStart + aTurns * s, 10 * s), 160);
        Curve2D aPCurve = Project(aCylinder, aCurve, 0, 1, 1e-4);
        AssertOnCurve(aCylinder, aCurve, aPCurve, 0, 1, 0.01);
        AssertNoJumps(aPCurve, 0, 1, 0.2);
        Assert.Equal(aTurns, Math.Abs(aPCurve.Value(1).X - aPCurve.Value(0).X), 2);
    }

    [Fact]
    public void AHelixRunningBackwards_IsContinuousToo()
    {
        var aCylinder = new CylindricalSurface(Z(), 5);
        BSplineCurve3D aCurve = Through(aCylinder, s => (5 - 3 * Math.PI * s, 10 * s), 160);
        Curve2D aPCurve = Project(aCylinder, aCurve, 0, 1, 1e-4);
        AssertOnCurve(aCylinder, aCurve, aPCurve, 0, 1, 0.01);
        AssertNoJumps(aPCurve, 0, 1, 0.2);
        Assert.True(aPCurve.Value(1).X < aPCurve.Value(0).X);
    }

    [Theory]
    [InlineData(1)] // up
    [InlineData(-1)] // down, so the isoline runs against V
    public void AGeneratrixOfACylinder_BecomesAnIsoline(int theSense)
    {
        var aCylinder = new CylindricalSurface(Z(), 5);
        var aPoints = new NCollection_Array1<Point3D>(1, 12);
        for (int i = 0; i < 12; i++)
        {
            double aV = theSense > 0 ? i : 11 - i;
            aPoints[i + 1] = aCylinder.Value(1.5, aV);
        }
        BSplineCurve3D aCurve = new PointsToBSpline(aPoints).Curve();
        double aF = aCurve.FirstParameter, aL = aCurve.LastParameter;
        Curve2D aPCurve = Project(aCylinder, aCurve, aF, aL, 1e-4);
        AssertOnCurve(aCylinder, aCurve, aPCurve, aF, aL, 0.01);
        Assert.Equal(1.5, aPCurve.Value(aF).X, 5);
        Assert.Equal(1.5, aPCurve.Value(aL).X, 5);
        Assert.Equal(theSense > 0, aPCurve.Value(aL).Y > aPCurve.Value(aF).Y);
    }

    [Theory]
    [InlineData(0.0)]
    [InlineData(2.0)]
    public void AClosedParallel_OfACylinder_RunsTheWholePeriod(double theStartU)
    {
        // a circle of the surface made as a spline, starting anywhere: a closed isoline
        var aCylinder = new CylindricalSurface(Z(), 5);
        BSplineCurve3D aCurve = Through(aCylinder, s => (theStartU + 2 * Math.PI * s, 3), 120);
        Curve2D aPCurve = Project(aCylinder, aCurve, 0, 1, 1e-4);
        AssertOnCurve(aCylinder, aCurve, aPCurve, 0, 1, 0.01);
        AssertNoJumps(aPCurve, 0, 1, 0.2);
        Assert.Equal(2 * Math.PI, Math.Abs(aPCurve.Value(1).X - aPCurve.Value(0).X), 3);
    }

    [Fact]
    public void ACurveOnATorus_IsContinuousAcrossBothSeams()
    {
        var aTorus = new ToroidalSurface(Z(), 10, 3);
        BSplineCurve3D aCurve = Through(aTorus, s => (-0.5 + 7 * s, -1 + 8 * s), 200);
        Curve2D aPCurve = Project(aTorus, aCurve, 0, 1, 1e-4);
        AssertOnCurve(aTorus, aCurve, aPCurve, 0, 1, 0.01);
        AssertNoJumps(aPCurve, 0, 1, 0.3);
    }

    [Fact]
    public void ACurveAlongATorusSeamInV_StaysOnOneSideOfIt()
    {
        // along V's seam: V stays at one value, either 0 or 2 pi, not jumping between them
        var aTorus = new ToroidalSurface(Z(), 10, 3);
        BSplineCurve3D aCurve = Through(aTorus, s => (1 + 2 * s, 0), 80);
        Curve2D aPCurve = Project(aTorus, aCurve, 0, 1, 1e-4);
        AssertOnCurve(aTorus, aCurve, aPCurve, 0, 1, 0.01);
        AssertNoJumps(aPCurve, 0, 1, 0.1);
    }

    [Fact]
    public void ACurveThroughAConesApex_StaysOnTheCurve()
    {
        // down one generatrix to the apex, and up the opposite one
        var aCone = new ConicalSurface(Z(), Math.PI / 6, 5);
        double anApexV = -5 / Math.Sin(Math.PI / 6);
        BSplineCurve3D aCurve = Through(aCone, s => s < 0.5 ? (1.0, anApexV * 2 * s) : (1.0 + Math.PI, anApexV * 2 * (1 - s)));
        Curve2D aPCurve = Project(aCone, aCurve, 0, 1, 1e-4);
        AssertOnCurve(aCone, aCurve, aPCurve, 0, 1, 0.03);
    }

    [Fact]
    public void ASplineWithVeryUnevenParameterSpeed_FollowsTheCurveOnItsFastStretch()
    {
        // on one knot interval the curve moves a hundred thousand times as fast as on the next; the poles aren't quite
        // on the cylinder, which is why the tolerance is wide. OCCT's standard projector, the same in native 8.0.1 to the
        // digits, is 1.413 away at t = 0 and 10 at t = 0.001, with the status "done": decision 1737
        var aCylinder = new CylindricalSurface(Z(), 5);
        var aPoles = new[]
        {
            aCylinder.Value(0.0, 0), aCylinder.Value(0.2, 1), aCylinder.Value(0.4, 2), aCylinder.Value(0.6, 3),
            aCylinder.Value(0.8, 4), aCylinder.Value(1.0, 5), aCylinder.Value(1.2, 6),
        };
        var aCurve = new BSplineCurve3D(aPoles, [0.0, 0.00001, 1.0, 2.0, 3.0], [4, 1, 1, 1, 4], 3);
        var aSas = new SurfaceAnalysis(aCylinder);
        var aProj = new ProjectCurveOnSurface();
        aProj.Init(aSas, 1e-3);
        Assert.True(aProj.Perform(aCurve, 0, 3, out Curve2D? aPCurve));
        Assert.True(aProj.Status(DONE2), "not the standard projector's result");
        AssertOnCurve(aCylinder, aCurve, aPCurve!, 0, 3, 0.1);
        AssertOnCurve(aCylinder, aCurve, aPCurve!, 0, 0.00001, 0.1);
        AssertOnCurve(aCylinder, aCurve, aPCurve!, 0, 0.01, 0.1);
        Assert.Equal(0, aCylinder.Value(aPCurve!.Value(0).X, aPCurve.Value(0).Y).DistanceTo(aCurve.Value(0)), 6);
    }

    [Fact]
    public void AnUnevenSplineOnAPlane_IsStillOnThePlane()
    {
        var aPlane = new PlaneSurface(Z());
        var aPoles = new[] { new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0), new Point3D(3, 1, 0), new Point3D(4, 0, 0) };
        var aCurve = new BSplineCurve3D(aPoles, [0.0, 0.0001, 10.0], [4, 1, 4], 3);
        Curve2D aPCurve = Project(aPlane, aCurve, 0, 10);
        AssertOnCurve(aPlane, aCurve, aPCurve, 0, 10, 1e-4);
    }

    [Fact]
    public void ACurveThatStopsPartWay_DropsItsRepeatedPoints()
    {
        // a polyline with a repeated pole: the curve waits there while its parameter runs on
        var aSphere = new SphericalSurface(Z(), 5);
        var aPoles = new[]
        {
            aSphere.Value(0.0, 0.0), aSphere.Value(0.3, 0.1), aSphere.Value(0.3, 0.1), aSphere.Value(0.3, 0.1),
            aSphere.Value(0.6, 0.2), aSphere.Value(0.9, 0.3),
        };
        var aCurve = new BSplineCurve3D(aPoles, [0.0, 1.0, 2.0, 3.0, 4.0, 5.0], [2, 1, 1, 1, 1, 2], 1);
        Curve2D aPCurve = Project(aSphere, aCurve, 0, 5, 1e-3);
        AssertOnCurve(aSphere, aCurve, aPCurve, 0, 5, 0.1);
    }

    [Fact]
    public void WithoutASurfaceOrACurve_ItFails()
    {
        var aProj = new ProjectCurveOnSurface();
        Assert.False(aProj.Perform(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0, 1, out Curve2D? aNone));
        Assert.Null(aNone);
        Assert.True(aProj.Status(FAIL1));
        aProj.Init(new CylindricalSurface(Z(), 5), 1e-5);
        Assert.False(aProj.Perform(null, 0, 1, out aNone));
        Assert.True(aProj.Status(FAIL1));
    }

    [Theory]
    [InlineData(3.5)]
    [InlineData(-1.0)]
    public void ACircleOnASphere_NotAParallel_IsContinuousAcrossTheSeam(double theStart)
    {
        // a small circle of the sphere, tilted: both U and V vary, and U crosses the seam
        var aSphere = new SphericalSurface(Z(), 5);
        var aFrame = new Frame3D(new Point3D(3, 0, 3), new Direction3D(1, 0, 1), new Direction3D(0, 1, 0));
        double aR = Math.Sqrt(25 - 18);
        var aCircle = new CircleCurve3D(aFrame, aR);
        Curve2D aPCurve = Project(aSphere, aCircle, theStart, theStart + 2 * Math.PI, 1e-4);
        AssertOnCurve(aSphere, aCircle, aPCurve, theStart, theStart + 2 * Math.PI, 0.03);
        AssertNoJumps(aPCurve, theStart, theStart + 2 * Math.PI, 0.5);
    }
}
