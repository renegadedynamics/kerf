// Port of: src/ModelingData/TKGeomBase/GTests/GeomLib_CheckCurveOnSurface_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class GeomLib_CheckCurveOnSurfaceTest
{
    // Circle of radius theRadius at height theHeight on a cylinder of radius
    // theCylRadius around OZ; the pcurve is the corresponding iso line in UV.
    private static CheckCurveOnSurface makeCircleOnCylinderCheck(double theCircRadius, double theCylRadius,
                                                                         double theHeight)
    {
        double aFirst = 0.0, aLast = 2.0 * Math.PI;

        CircleCurve3D aCirc = new(
            new Frame3D(new Point3D(0.0, 0.0, theHeight), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)),
            theCircRadius);
        CurveAdaptor aC3d = new(aCirc, aFirst, aLast);

        CylindricalSurface aCyl = new(
            new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)),
            theCylRadius);
        LineCurve2D aPLine = new(new Point2D(0.0, theHeight), new Direction2D(1.0, 0.0));

        CurveAdaptor2D aC2d = new(aPLine, aFirst, aLast);
        SurfaceAdaptor aSurf = new(aCyl);
        CurveOnSurface aCoS = new(aC2d, aSurf);

        CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, global::Kerf.Engine.Foundation.Precision.PConfusion());
        aCheck.Perform(aCoS);
        return aCheck;
    }

    private static CheckCurveOnSurface makeLineOnPlaneCheck(Direction3D theCurveDirection,
                                                                    Direction2D thePCurveDirection,
                                                                    double theFirst, double theLast)
    {
        LineCurve3D aLine = new(new Point3D(0.0, 0.0, 0.0), theCurveDirection);
        CurveAdaptor aC3d = new(aLine, theFirst, theLast);

        PlaneSurface aPlane =
            new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)));
        LineCurve2D aPLine = new(new Point2D(0.0, 0.0), thePCurveDirection);
        CurveAdaptor2D aC2d = new(aPLine, theFirst, theLast);
        SurfaceAdaptor aSurf = new(aPlane);
        CurveOnSurface aCoS = new(aC2d, aSurf);

        CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, global::Kerf.Engine.Foundation.Precision.PConfusion());
        aCheck.Perform(aCoS);
        return aCheck;
    }

    private static CheckCurveOnSurface makeCircleAndEllipseCheck(double theMajorRadius, double theMinorRadius,
                                                                         double theFirst, double theLast)
    {
        CircleCurve3D aCircle =
            new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)), 2.0);
        CurveAdaptor aC3d = new(aCircle, theFirst, theLast);

        EllipseCurve2D anEllipse =
            new(new Axis2D(new Point2D(0.0, 0.0), new Direction2D(1.0, 0.0)), theMajorRadius, theMinorRadius);
        CurveAdaptor2D aC2d = new(anEllipse, theFirst, theLast);
        PlaneSurface aPlane =
            new(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)));
        SurfaceAdaptor aSurf = new(aPlane);
        CurveOnSurface aCoS = new(aC2d, aSurf);

        CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, global::Kerf.Engine.Foundation.Precision.PConfusion());
        aCheck.Perform(aCoS);
        return aCheck;
    }

    [Fact]
    public void AnalyticCoincident_ReportsZeroDeviation()
    {
        CheckCurveOnSurface aCheck = makeCircleOnCylinderCheck(2.0, 2.0, 5.0);

        Assert.True(aCheck.IsDone);
        Assert.True(aCheck.MaxDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void AnalyticDeviating_ReportsActualDeviation()
    {
        // The 3D circle is 0.05 smaller than the cylinder carrying the pcurve: the
        // deviation is constant and equal to the radius difference. This guards the
        // constant-distance shortcut against swallowing real deviations.
        CheckCurveOnSurface aCheck = makeCircleOnCylinderCheck(1.95, 2.0, 5.0);

        Assert.True(aCheck.IsDone);
        Assert.Equal(0.05, aCheck.MaxDistance(), 1.0e-9);
    }

    [Fact]
    public void LineOnPlane_ReportsZeroDeviation()
    {
        CheckCurveOnSurface aCheck =
            makeLineOnPlaneCheck(new Direction3D(1.0, 0.0, 0.0), new Direction2D(1.0, 0.0), -10.0, 10.0);

        Assert.True(aCheck.IsDone);
        Assert.True(aCheck.MaxDistance() <= global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DeviatingLines_ReportsEndpointMaximum()
    {
        CheckCurveOnSurface aCheck =
            makeLineOnPlaneCheck(new Direction3D(1.0, 0.0, 0.0), new Direction2D(0.0, 1.0), -2.0, 3.0);

        Assert.True(aCheck.IsDone);
        Assert.Equal(Math.Sqrt(18.0), aCheck.MaxDistance(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(3.0, aCheck.MaxParameter());
    }

    [Fact]
    public void CircleAndEllipse_ReportsAnalyticMaximum()
    {
        CheckCurveOnSurface aCheck = makeCircleAndEllipseCheck(4.0, 1.0, 0.0, 2.0 * Math.PI);

        Assert.True(aCheck.IsDone);
        Assert.Equal(2.0, aCheck.MaxDistance(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void NegativeTrim_ReportsInteriorMaximum()
    {
        CheckCurveOnSurface aCheck = makeCircleAndEllipseCheck(2.0, 1.0, -2.0, -1.0);

        Assert.True(aCheck.IsDone);
        Assert.Equal(1.0, aCheck.MaxDistance(), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(-0.5 * Math.PI, aCheck.MaxParameter(), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    [Fact]
    public void AnalyticAliasing_ReportsActualDeviation()
    {
        double aFirst = 0.0, aLast = 80.0 * Math.PI;

        CircleCurve3D aCircle =
            new(new Frame3D(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)), 4.0);
        CurveAdaptor aC3d = new(aCircle, aFirst, aLast);

        ToroidalSurface aTorus = new(
            new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0), new Direction3D(1.0, 0.0, 0.0)), 3.0, 1.0);
        LineCurve2D aPLine = new(new Point2D(0.0, 0.0), new Direction2D(3.0, 4.0));
        CurveAdaptor2D aC2d = new(aPLine, aFirst, aLast);
        SurfaceAdaptor aSurf = new(aTorus);
        CurveOnSurface aCoS = new(aC2d, aSurf);

        CheckCurveOnSurface aCheck = new();
        aCheck.Init(aC3d, global::Kerf.Engine.Foundation.Precision.PConfusion());
        aCheck.Perform(aCoS);

        Assert.True(aCheck.IsDone);
        Assert.Equal(8.0, aCheck.MaxDistance(), global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    private const double IntervalBoundary = 1.0;
    private const double LastParameterOfTwo = 2.0;
    private const double CurveOffset = 0.125;

    private enum IntervalFailure
    {
        None,
        NonFinite,
        PositiveInfinite,
        NegativeInfinite,
        UndefinedValue,
    }

    // Upstream ec0167b409's GeomLib_IntervalFailureCurve: a line CurveOffset above the plane, failing past the first
    // of the pcurve's two intervals
    private sealed class IntervalFailureCurve(IntervalFailure theFailure) : CurveAdaptorBase
    {
        public override CurveAdaptorBase ShallowCopy() => new IntervalFailureCurve(theFailure);

        public override double FirstParameter() => 0.0;

        public override double LastParameter() => LastParameterOfTwo;

        public override CurveKind GetType() => CurveKind.OtherCurve;

        public override Point3D EvalD0(double u)
        {
            if (u > IntervalBoundary)
            {
                switch (theFailure)
                {
                    case IntervalFailure.UndefinedValue:
                        throw new global::Kerf.Engine.Foundation.UndefinedValueException("Undefined curve value");
                    case IntervalFailure.NonFinite:
                        return new Point3D(u, 0.0, double.NaN);
                    case IntervalFailure.PositiveInfinite:
                        return new Point3D(u, 0.0, global::Kerf.Engine.Foundation.Precision.Infinite());
                    case IntervalFailure.NegativeInfinite:
                        return new Point3D(u, 0.0, -global::Kerf.Engine.Foundation.Precision.Infinite());
                }
            }

            return new Point3D(u, 0.0, CurveOffset);
        }
    }

    // The X axis on the XY plane as a degree 1 B-spline pcurve with knots 0, 1 and 2: two intervals
    private static CurveOnSurface makeTwoIntervalPCurve()
    {
        NCollection_Array1<Point2D> aPoles = new(1, 3);
        NCollection_Array1<double> aKnots = new(1, 3);
        NCollection_Array1<int> aMults = new(1, 3);
        for (int i = 1; i <= 3; i++)
        {
            aPoles[i] = new Point2D(i - 1, 0.0);
            aKnots[i] = i - 1;
            aMults[i] = i == 1 || i == 3 ? 2 : 1;
        }

        CurveAdaptor2D aPCurve = new(new BSplineCurve2D(aPoles, aKnots, aMults, 1));
        SurfaceAdaptor aSurface =
            new(new PlaneSurface(new CoordinateSystem(new Point3D(0.0, 0.0, 0.0), new Direction3D(0.0, 0.0, 1.0),
                                                      new Direction3D(1.0, 0.0, 0.0))));
        return new CurveOnSurface(aPCurve, aSurface);
    }

    // Upstream ec0167b409's FailedIntervalDoesNotReportPartialMaximum (decision 34503): a curve whose evaluation fails
    // or isn't finite on one interval fails the check. 8.0.1 left the interval out and reported the other's 0.125.
    [Fact]
    public void A_failed_interval_fails_the_check_rather_than_giving_the_others_maximum()
    {
        double aFiniteDistance = Math.Sqrt(global::Kerf.Engine.Foundation.Precision.Infinite());
        CheckCurveOnSurface aFiniteCheck =
            new(new CurveAdaptor(new LineCurve3D(new Point3D(0.0, 0.0, aFiniteDistance), new Direction3D(1.0, 0.0, 0.0)),
                                 0.0, LastParameterOfTwo));
        aFiniteCheck.Perform(makeTwoIntervalPCurve());
        Assert.True(aFiniteCheck.IsDone);
        Assert.Equal(aFiniteDistance, aFiniteCheck.MaxDistance(), aFiniteDistance * 1e-15);

        foreach (IntervalFailure aFailure in new[] { IntervalFailure.NonFinite, IntervalFailure.PositiveInfinite,
                                                     IntervalFailure.NegativeInfinite, IntervalFailure.UndefinedValue })
        {
            foreach (bool aParallel in new[] { false, true })
            {
                CheckCurveOnSurface aCheck = new(new IntervalFailureCurve(aFailure));
                aCheck.SetParallel(aParallel);
                aCheck.Perform(makeTwoIntervalPCurve());
                Assert.False(aCheck.IsDone, $"{aFailure}, parallel {aParallel}: {aCheck.MaxDistance()}");
                Assert.Equal(3, aCheck.ErrorStatus());
                Assert.Equal(double.MaxValue, aCheck.MaxDistance());
                Assert.Equal(0.0, aCheck.MaxParameter());
            }
        }
    }

    // Upstream ec0167b409's RepeatedPerformResetsStatusAndDistance (decision 34503): 8.0.1 kept the failed check's
    // status through the next, which succeeded
    [Fact]
    public void A_repeated_check_starts_afresh()
    {
        CheckCurveOnSurface aCheck = new(new IntervalFailureCurve(IntervalFailure.None));
        CurveOnSurface aPCurve = makeTwoIntervalPCurve();
        aCheck.Perform(aPCurve);
        Assert.True(aCheck.IsDone);
        Assert.Equal(CurveOffset, aCheck.MaxDistance(), 1e-15);

        aCheck.Perform(null);
        Assert.False(aCheck.IsDone);
        Assert.Equal(1, aCheck.ErrorStatus());
        Assert.Equal(double.MaxValue, aCheck.MaxDistance());
        Assert.Equal(0.0, aCheck.MaxParameter());

        aCheck.Perform(aPCurve);
        Assert.True(aCheck.IsDone);
        Assert.Equal(0, aCheck.ErrorStatus());
        Assert.Equal(CurveOffset, aCheck.MaxDistance(), 1e-15);
    }
}
