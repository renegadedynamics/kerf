// Port of: src/ModelingAlgorithms/TKGeomAlgo/GTests/GeomFill_Gordon_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Api;
using Kerf.Geometry;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>OCCT's GeomFill_Gordon GTest (suite GeomFill_Gordon), names kept.</summary>
/// <remarks>
/// Curves that GeomAPI_Interpolate hands out are frozen in the port: where OCCT changes one in place
/// (<c>SetOrigin</c>), the test uses the non-mutating twin and replaces the curve.
/// </remarks>
public class GeomFill_Gordon_Test
{
    //! Helper: create a linear BSpline curve from two points, parametrized on [0,1].
    internal static BSplineCurve3D makeLinearBSpline(Point3D theP1, Point3D theP2)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 2);
        aPoles[1] = theP1;
        aPoles[2] = theP2;
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 1);
    }

    //! Helper: create a quadratic BSpline curve through 3 control points on [0,1].
    internal static BSplineCurve3D makeQuadraticBSpline(Point3D theP1, Point3D theP2, Point3D theP3,
                                                           double theFirstParameter = 0.0,
                                                           double theLastParameter = 1.0)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = theP1;
        aPoles[2] = theP2;
        aPoles[3] = theP3;
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = theFirstParameter;
        aKnots[2] = theLastParameter;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;
        return new BSplineCurve3D(aPoles, aKnots, aMults, 2);
    }

    private static NCollection_Array2<Point3D> makeIntersectionGrid(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                                   NCollection_Array1<double> theGuideParams)
    {
        NCollection_Array2<Point3D> aPoints = new(1, theGuideParams.Size(), 1, theProfiles.Size());
        for (int aProfileIdx = 0; aProfileIdx < theProfiles.Size(); ++aProfileIdx)
        {
            for (int aGuideIdx = 0; aGuideIdx < theGuideParams.Size(); ++aGuideIdx)
            {
                aPoints[aGuideIdx + 1, aProfileIdx + 1] = theProfiles.At(aProfileIdx)!.Value(theGuideParams.At(aGuideIdx));
            }
        }
        return aPoints;
    }

    private static NCollection_Array2<double> makeUnitWeightGrid(NCollection_Array1<BSplineCurve3D?> theProfiles,
                                                                 NCollection_Array1<double> theGuideParams)
    {
        NCollection_Array2<double> aWeights = new(1, theGuideParams.Size(), 1, theProfiles.Size());
        aWeights.Init(1.0);
        return aWeights;
    }

    private static BSplineCurve3D makeRationalQuadraticBSpline(Point3D theP1, Point3D theP2, Point3D theP3,
                                                                  double theMiddleWeight)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = theP1;
        aPoles[2] = theP2;
        aPoles[3] = theP3;
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights[1] = 1.0;
        aWeights[2] = theMiddleWeight;
        aWeights[3] = 1.0;
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;
        return new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 2);
    }

    private static BSplineCurve3D makeWeightedLinearBSpline(Point3D theP1, Point3D theP2, double theWeight)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 2);
        aPoles[1] = theP1;
        aPoles[2] = theP2;
        NCollection_Array1<double> aWeights = new(1, 2);
        aWeights[1] = theWeight;
        aWeights[2] = theWeight;
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 2;
        aMults[2] = 2;
        return new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 1, false, false);
    }

    private static BSplineCurve3D makeRationalLineBSpline(Point3D theP1, Point3D theP2, int theDegree)
    {
        NCollection_Array1<Point3D> aPoles = new(1, theDegree + 1);
        NCollection_Array1<double> aWeights = new(1, theDegree + 1);
        for (int aPoleIdx = 1; aPoleIdx <= theDegree + 1; ++aPoleIdx)
        {
            double aRatio = (double)(aPoleIdx - 1) / theDegree;
            aPoles[aPoleIdx] = new Point3D(theP1.XYZ() * (1.0 - aRatio) + theP2.XYZ() * aRatio);
            aWeights[aPoleIdx] = (aPoleIdx == (theDegree + 2) / 2) ? 0.5 : 1.0;
        }
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = theDegree + 1;
        aMults[2] = theDegree + 1;
        return new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, theDegree);
    }

    private static BSplineCurve3D makeRationalQuadraticLineBSpline(Point3D theP1, Point3D theP2,
                                                                      double theStartWeight, double theMiddleWeight,
                                                                      double theEndWeight)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = theP1;
        aPoles[2] = new Point3D((theP1.XYZ() + theP2.XYZ()) * 0.5);
        aPoles[3] = theP2;
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights[1] = theStartWeight;
        aWeights[2] = theMiddleWeight;
        aWeights[3] = theEndWeight;
        NCollection_Array1<double> aKnots = new(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        NCollection_Array1<int> aMults = new(1, 2);
        aMults[1] = 3;
        aMults[2] = 3;
        return new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, 2);
    }

    private static BSplineCurve3D makePeriodicProfile(double theY)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(1.0, theY, 0.0);
        aPoles[2] = new Point3D(0.309, theY, 0.951);
        aPoles[3] = new Point3D(-0.809, theY, 0.588);
        aPoles[4] = new Point3D(-0.809, theY, -0.588);
        aPoles[5] = new Point3D(0.309, theY, -0.951);
        NCollection_Array1<double> aKnots = new(1, 6);
        for (int aKnotIdx = 1; aKnotIdx <= 6; ++aKnotIdx)
        {
            aKnots[aKnotIdx] = 0.2 * (aKnotIdx - 1);
        }
        NCollection_Array1<int> aMults = new(1, 6);
        aMults.Init(1);
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3, true);
    }

    private static BSplineCurve3D makePeriodicGuide(double theX)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 5);
        aPoles[1] = new Point3D(theX, 1.0, 0.0);
        aPoles[2] = new Point3D(theX, 0.309, 0.951);
        aPoles[3] = new Point3D(theX, -0.809, 0.588);
        aPoles[4] = new Point3D(theX, -0.809, -0.588);
        aPoles[5] = new Point3D(theX, 0.309, -0.951);
        NCollection_Array1<double> aKnots = new(1, 6);
        for (int aKnotIdx = 1; aKnotIdx <= 6; ++aKnotIdx)
        {
            aKnots[aKnotIdx] = 0.2 * (aKnotIdx - 1);
        }
        NCollection_Array1<int> aMults = new(1, 6);
        aMults.Init(1);
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3, true);
    }

    private static BSplineCurve3D makeClosedBSplineProfile(double theY, double theXRadius, double theZRadius)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 9);
        aPoles[1] = new Point3D(theXRadius, theY, 0.0);
        aPoles[2] = new Point3D(theXRadius, theY, 0.7 * theZRadius);
        aPoles[3] = new Point3D(0.3 * theXRadius, theY, theZRadius);
        aPoles[4] = new Point3D(-0.7 * theXRadius, theY, 0.8 * theZRadius);
        aPoles[5] = new Point3D(-theXRadius, theY, 0.0);
        aPoles[6] = new Point3D(-0.7 * theXRadius, theY, -0.8 * theZRadius);
        aPoles[7] = new Point3D(0.3 * theXRadius, theY, -theZRadius);
        aPoles[8] = new Point3D(theXRadius, theY, -0.7 * theZRadius);
        aPoles[9] = new Point3D(theXRadius, theY, 0.0);
        NCollection_Array1<double> aKnots = new(1, 7);
        for (int aKnotIdx = 1; aKnotIdx <= 7; ++aKnotIdx)
        {
            aKnots[aKnotIdx] = (aKnotIdx - 1) / 6.0;
        }
        NCollection_Array1<int> aMults = new(1, 7);
        aMults[1] = 4;
        aMults[7] = 4;
        for (int aKnotIdx = 2; aKnotIdx < 7; ++aKnotIdx)
        {
            aMults[aKnotIdx] = 1;
        }
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3, false);
    }

    private static BSplineCurve3D makeInterpolatedPeriodicProfile(double theY, int theSeamShift)
    {
        const int THE_NB_POINTS = 8;
        const double THE_PI = 3.14159265358979323846;
        NCollection_HArray1<Point3D> aPoints = new(1, THE_NB_POINTS);
        for (int aPointIdx = 0; aPointIdx < THE_NB_POINTS; ++aPointIdx)
        {
            double anAngle = 2.0 * THE_PI * (aPointIdx + theSeamShift) / THE_NB_POINTS;
            aPoints.SetValue(aPointIdx + 1, new Point3D(Math.Cos(anAngle), theY, Math.Sin(anAngle)));
        }
        ApiInterpolate anInterpolator = new(aPoints, true, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterpolator.Perform();
        return anInterpolator.Curve();
    }

    //! Helper: create a cubic BSpline through 4 interpolation points on [0,1].
    private static BSplineCurve3D makeCubicInterpBSpline(Point3D theP1, Point3D theP2, Point3D theP3, Point3D theP4)
    {
        NCollection_HArray1<Point3D> aPnts = new(1, 4);
        aPnts.SetValue(1, theP1);
        aPnts.SetValue(2, theP2);
        aPnts.SetValue(3, theP3);
        aPnts.SetValue(4, theP4);
        NCollection_HArray1<double> aParams = new(1, 4);
        aParams.SetValue(1, 0.0);
        aParams.SetValue(2, 1.0 / 3.0);
        aParams.SetValue(3, 2.0 / 3.0);
        aParams.SetValue(4, 1.0);
        ApiInterpolate anInterp = new(aPnts, aParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterp.Perform();
        return anInterp.Curve();
    }

    //! Helper: create a sinusoidal BSpline curve via interpolation.
    private static BSplineCurve3D makeSineBSpline(double theXStart, double theXEnd, double theY, double theAmplitude,
                                                     int theNbPts = 11)
    {
        NCollection_HArray1<Point3D> aPnts = new(1, theNbPts);
        NCollection_HArray1<double> aParams = new(1, theNbPts);
        for (int i = 1; i <= theNbPts; ++i)
        {
            double aT = (double)(i - 1) / (theNbPts - 1);
            double aX = theXStart + aT * (theXEnd - theXStart);
            double aZ = theAmplitude * Math.Sin(Math.PI * aT);
            aPnts.SetValue(i, new Point3D(aX, theY, aZ));
            aParams.SetValue(i, aT);
        }
        ApiInterpolate anInterp = new(aPnts, aParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterp.Perform();
        return anInterp.Curve();
    }

    private static BSplineCurve3D makeMeanderingProfile(double theY)
    {
        NCollection_HArray1<Point3D> aPnts = new(1, 7);
        aPnts.SetValue(1, new Point3D(0.0, theY, 0.0));
        aPnts.SetValue(2, new Point3D(0.62, theY + 0.04, 0.01));
        aPnts.SetValue(3, new Point3D(0.38, theY - 0.04, -0.01));
        aPnts.SetValue(4, new Point3D(0.5, theY, 0.0));
        aPnts.SetValue(5, new Point3D(0.64, theY + 0.03, 0.01));
        aPnts.SetValue(6, new Point3D(0.44, theY - 0.02, -0.01));
        aPnts.SetValue(7, new Point3D(1.0, theY, 0.0));
        NCollection_HArray1<double> aParams = new(1, 7);
        for (int aParamIdx = 1; aParamIdx <= 7; ++aParamIdx)
        {
            aParams.SetValue(aParamIdx, (aParamIdx - 1) / 6.0);
        }
        ApiInterpolate anInterp = new(aPnts, aParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterp.Perform();
        return anInterp.Curve();
    }

    //! Helper: verify surface passes through a point at given parameters.
    private static void verifyPointOnSurface(BSplineSurface theSurf, double theU, double theV, Point3D theExpected,
                                             double theTol)
    {
        Point3D aSurfPt = theSurf.Value(theU, theV);
        double aDist = aSurfPt.DistanceTo(theExpected);
        Assert.True(aDist < theTol, $"Surface point at ({theU}, {theV}) differs from expected by {aDist}");
    }

    internal static double wavyHeight(double theU, double theV)
    {
        return 0.35 * Math.Sin(Math.PI * theU) * (0.4 + theV) + 0.12 * Math.Sin(2.0 * Math.PI * theV) * (1.0 - theU);
    }

    internal static double saddleHeight(double theU, double theV)
    {
        return 0.6 * (theU - 0.5) * (theV - 0.5) + 0.15 * Math.Sin(Math.PI * theU) * Math.Sin(Math.PI * theV);
    }

    internal static double rippleHeight(double theU, double theV)
    {
        return 0.2 * Math.Sin(2.0 * Math.PI * theU) * Math.Cos(Math.PI * theV) + 0.1 * Math.Sin(3.0 * Math.PI * theV);
    }

    internal sealed class AnalyticNetwork
    {
        public readonly NCollection_Array1<Curve3D> Profiles;
        public readonly NCollection_Array1<Curve3D> Guides;

        public AnalyticNetwork(int theNbProfiles, int theNbGuides, Func<double, double, double> theHeight)
            : this(theNbProfiles, theNbGuides, theHeight, new Xyz(0.0, 0.0, 0.0), 1.0)
        {
        }

        public AnalyticNetwork(int theNbProfiles, int theNbGuides, Func<double, double, double> theHeight,
                               Xyz theOrigin, double theScale)
        {
            Profiles = new NCollection_Array1<Curve3D>(1, theNbProfiles);
            Guides = new NCollection_Array1<Curve3D>(1, theNbGuides);
            for (int aProfileIdx = 0; aProfileIdx < theNbProfiles; ++aProfileIdx)
            {
                double aV = (double)aProfileIdx / (theNbProfiles - 1);
                NCollection_HArray1<Point3D> aPoints = new(1, theNbGuides);
                NCollection_HArray1<double> aParams = new(1, theNbGuides);
                for (int aGuideIdx = 0; aGuideIdx < theNbGuides; ++aGuideIdx)
                {
                    double aU = (double)aGuideIdx / (theNbGuides - 1);
                    aPoints.SetValue(aGuideIdx + 1,
                                     new Point3D(theOrigin + new Xyz(theScale * aU, theScale * aV, theScale * theHeight(aU, aV))));
                    aParams.SetValue(aGuideIdx + 1, aU);
                }
                ApiInterpolate anInterpolator = new(aPoints, aParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
                anInterpolator.Perform();
                Profiles.ChangeAt(aProfileIdx) = anInterpolator.Curve();
            }

            for (int aGuideIdx = 0; aGuideIdx < theNbGuides; ++aGuideIdx)
            {
                double aU = (double)aGuideIdx / (theNbGuides - 1);
                NCollection_HArray1<Point3D> aPoints = new(1, theNbProfiles);
                NCollection_HArray1<double> aParams = new(1, theNbProfiles);
                for (int aProfileIdx = 0; aProfileIdx < theNbProfiles; ++aProfileIdx)
                {
                    double aV = (double)aProfileIdx / (theNbProfiles - 1);
                    aPoints.SetValue(aProfileIdx + 1,
                                     new Point3D(theOrigin + new Xyz(theScale * aU, theScale * aV, theScale * theHeight(aU, aV))));
                    aParams.SetValue(aProfileIdx + 1, aV);
                }
                ApiInterpolate anInterpolator = new(aPoints, aParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
                anInterpolator.Perform();
                Guides.ChangeAt(aGuideIdx) = anInterpolator.Curve();
            }
        }
    }

    internal static void verifyUniformNetworkInterpolation(BSplineSurface theSurface, AnalyticNetwork theNetwork,
                                                           double theTolerance)
    {
        Assert.NotNull(theSurface);
        const int THE_NB_SAMPLES = 32;
        double aUMin = theSurface.UKnot(1);
        double aUMax = theSurface.UKnot(theSurface.UKnotCount);
        double aVMin = theSurface.VKnot(1);
        double aVMax = theSurface.VKnot(theSurface.VKnotCount);
        for (int aProfileIdx = 0; aProfileIdx < theNetwork.Profiles.Size(); ++aProfileIdx)
        {
            double aV = aVMin + (double)aProfileIdx / (theNetwork.Profiles.Size() - 1) * (aVMax - aVMin);
            for (int aSampleIdx = 0; aSampleIdx <= THE_NB_SAMPLES; ++aSampleIdx)
            {
                double aParam = (double)aSampleIdx / THE_NB_SAMPLES;
                double aU = aUMin + aParam * (aUMax - aUMin);
                Assert.True(theSurface.Value(aU, aV).DistanceTo(theNetwork.Profiles.At(aProfileIdx).Value(aParam)) <= theTolerance);
            }
        }
        for (int aGuideIdx = 0; aGuideIdx < theNetwork.Guides.Size(); ++aGuideIdx)
        {
            double aU = aUMin + (double)aGuideIdx / (theNetwork.Guides.Size() - 1) * (aUMax - aUMin);
            for (int aSampleIdx = 0; aSampleIdx <= THE_NB_SAMPLES; ++aSampleIdx)
            {
                double aParam = (double)aSampleIdx / THE_NB_SAMPLES;
                double aV = aVMin + aParam * (aVMax - aVMin);
                Assert.True(theSurface.Value(aU, aV).DistanceTo(theNetwork.Guides.At(aGuideIdx).Value(aParam)) <= theTolerance);
            }
        }
    }

    private static TrimmedCurve3D Segment(Point3D theOrigin, Direction3D theDir, double theLength)
        => new(new LineCurve3D(theOrigin, theDir), 0.0, theLength);

    private static NCollection_Array1<Curve3D> Curves(params Curve3D[] theCurves)
    {
        NCollection_Array1<Curve3D> anArray = new(1, theCurves.Length);
        for (int i = 0; i < theCurves.Length; ++i)
        {
            anArray[i + 1] = theCurves[i];
        }
        return anArray;
    }

    private static NCollection_Array1<BSplineCurve3D?> BSplines(params BSplineCurve3D[] theCurves)
    {
        NCollection_Array1<BSplineCurve3D?> anArray = new(1, theCurves.Length);
        for (int i = 0; i < theCurves.Length; ++i)
        {
            anArray[i + 1] = theCurves[i];
        }
        return anArray;
    }

    private static NCollection_Array1<double> Reals(params double[] theValues)
    {
        NCollection_Array1<double> anArray = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; ++i)
        {
            anArray[i + 1] = theValues[i];
        }
        return anArray;
    }

    private static double UMid(BSplineSurface theSurf) => 0.5 * (theSurf.UKnot(1) + theSurf.UKnot(theSurf.UKnotCount));

    private static double VMid(BSplineSurface theSurf) => 0.5 * (theSurf.VKnot(1) + theSurf.VKnot(theSurf.VKnotCount));

    private static NCollection_Array1<Curve3D> UnitSquareProfiles() =>
        Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), 1.0), Segment(new Point3D(0, 1, 0), new Direction3D(1, 0, 0), 1.0));

    private static NCollection_Array1<Curve3D> UnitSquareGuides() =>
        Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), 1.0), Segment(new Point3D(1, 0, 0), new Direction3D(0, 1, 0), 1.0));

    [Fact]
    public void SimpleLineNetwork_ProducesValidSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(UnitSquareProfiles(), UnitSquareGuides(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();

        Assert.True(aGordon.IsDone);
        Assert.Equal(Gordon.ResultStatus.Done, aGordon.Status());
        Assert.Equal(Gordon.ResultStatus.Done, aGordon.Report().Status);
        Assert.Equal(Gordon.BuildStage.NotStarted, aGordon.Report().FailedStage);
        Assert.False(aGordon.Report().IsApproximate);
        Assert.True(aGordon.Report().MaxContactGap <= global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aGordon.Report().MaxProfileDeviation <= 1.0e-7);
        Assert.True(aGordon.Report().MaxGuideDeviation <= 1.0e-7);

        BSplineSurface aSurf = aGordon.Surface();
        verifyPointOnSurface(aSurf, 0.0, 0.0, new Point3D(0, 0, 0), 1.0e-3);
        verifyPointOnSurface(aSurf, 1.0, 0.0, new Point3D(1, 0, 0), 1.0e-3);
        verifyPointOnSurface(aSurf, 0.0, 1.0, new Point3D(0, 1, 0), 1.0e-3);
        verifyPointOnSurface(aSurf, 1.0, 1.0, new Point3D(1, 1, 0), 1.0e-3);
    }

    [Fact]
    public void EmptyInput_NotDone()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), 1.0)),
                     Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), 1.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.False(aGordon.IsDone);
        Assert.Equal(Gordon.ResultStatus.InvalidInput, aGordon.Status());
    }

    [Fact]
    public void SingleProfileOrGuide_NotDone()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), 1.0)), UnitSquareGuides(),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.False(aGordon.IsDone);
        Assert.Equal(Gordon.ResultStatus.InvalidInput, aGordon.Status());
    }

    [Fact]
    public void NotDone_BeforePerform()
    {
        Gordon aGordon = new();
        Assert.False(aGordon.IsDone);
        Assert.Equal(Gordon.ResultStatus.NotStarted, aGordon.Status());
        Assert.Equal(Gordon.ResultStatus.NotStarted, aGordon.Report().Status);
        Assert.Equal(Gordon.BuildStage.NotStarted, aGordon.Report().FailedStage);
        Assert.False(aGordon.Report().IsApproximate);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => aGordon.Surface());
    }

    [Fact]
    public void ReinitResetsReportAndSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(UnitSquareProfiles(), UnitSquareGuides(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.Equal(Gordon.ResultStatus.Done, aGordon.Report().Status);
        Assert.NotNull(aGordon.Surface());

        aGordon.Init(Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), 1.0)),
                     Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), 1.0)), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.False(aGordon.IsDone);
        Assert.Equal(Gordon.ResultStatus.NotStarted, aGordon.Status());
        Assert.Equal(Gordon.ResultStatus.NotStarted, aGordon.Report().Status);
        Assert.Equal(Gordon.BuildStage.NotStarted, aGordon.Report().FailedStage);
        Assert.False(aGordon.Report().IsApproximate);
        Assert.Equal(0.0, aGordon.Report().MaxContactGap, 1.0e-12);
        Assert.Equal(0.0, aGordon.Report().MaxReparametrizationDeviation, 1.0e-12);
        Assert.Equal(0.0, aGordon.Report().MaxProfileDeviation, 1.0e-12);
        Assert.Equal(0.0, aGordon.Report().MaxGuideDeviation, 1.0e-12);
        Assert.Equal(0.0, aGordon.Report().MaxApproximationDeviation, 1.0e-12);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => aGordon.Surface());

        aGordon.Perform();
        Assert.False(aGordon.IsDone);
        Assert.Equal(Gordon.ResultStatus.InvalidInput, aGordon.Status());
        Assert.Equal(Gordon.ResultStatus.InvalidInput, aGordon.Report().Status);
        Assert.Equal(Gordon.BuildStage.InputConversion, aGordon.Report().FailedStage);
    }

    [Fact]
    public void DisjointNetworkReportsContactDiscoveryFailure()
    {
        Gordon aGordon = new();
        aGordon.Init(UnitSquareProfiles(),
                     Curves(Segment(new Point3D(2, 0, 0), new Direction3D(0, 1, 0), 1.0),
                            Segment(new Point3D(3, 0, 0), new Direction3D(0, 1, 0), 1.0)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();

        Assert.False(aGordon.IsDone);
        Assert.False(aGordon.IsApproximate);
        Assert.Equal(Gordon.ResultStatus.IntersectionFailed, aGordon.Status());
        Assert.Equal(Gordon.ResultStatus.IntersectionFailed, aGordon.Report().Status);
        Assert.Equal(Gordon.BuildStage.ContactDiscovery, aGordon.Report().FailedStage);
        Assert.False(aGordon.Report().IsApproximate);
        Assert.Equal(0.0, aGordon.Report().MaxContactGap, 1.0e-12);
        Assert.Equal(0.0, aGordon.Report().MaxReparametrizationDeviation, 1.0e-12);
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => aGordon.Surface());
    }

    private static NCollection_Array1<Curve3D> BumpProfiles() =>
        Curves(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0.4), new Point3D(1, 0, 0)),
               makeQuadraticBSpline(new Point3D(0, 0.5, 0), new Point3D(0.5, 0.5, 0.6), new Point3D(1, 0.5, 0)),
               makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(0.5, 1, 0.4), new Point3D(1, 1, 0)));

    private static NCollection_Array1<Curve3D> LinearSideGuides() =>
        Curves(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0)),
               makeLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0)));

    [Fact]
    public void ParallelMode_DefaultSingleThreadAndMatchesParallelResult()
    {
        NCollection_Array1<Curve3D> aProfiles = BumpProfiles();
        NCollection_Array1<Curve3D> aGuides = LinearSideGuides();

        Gordon aSingleThread = new();
        Assert.False(aSingleThread.IsParallelMode);
        aSingleThread.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aSingleThread.Perform();
        Assert.True(aSingleThread.IsDone);

        Gordon aParallel = new();
        aParallel.SetParallelMode(true);
        Assert.True(aParallel.IsParallelMode);
        aParallel.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aParallel.Perform();
        Assert.True(aParallel.IsDone);

        Point3D aMidSingle = aSingleThread.Surface().Value(0.5, 0.5);
        Point3D aMidPar = aParallel.Surface().Value(0.5, 0.5);
        Assert.True(aMidSingle.DistanceTo(aMidPar) < 1.0e-9);
    }

    [Fact]
    public void CurvedBSplineNetwork_ProducesValidSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(BumpProfiles(), LinearSideGuides(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Point3D aMidPt = aSurf.Value(UMid(aSurf), VMid(aSurf));
        Assert.Equal(0.3, aMidPt.Z, 1.0e-6);
    }

    [Fact]
    public void NonAffineProfileReparametrization_PreservesProfilesAndGuideContinuity()
    {
        NCollection_Array1<Curve3D> aProfiles =
          Curves(makeCubicInterpBSpline(new Point3D(0, 0, 0), new Point3D(0.25, 0, 0.4), new Point3D(0.75, 0, -0.3), new Point3D(1, 0, 0)),
                 makeCubicInterpBSpline(new Point3D(0, 0.5, 0), new Point3D(0.25, 0.5, 0.2), new Point3D(0.75, 0.5, -0.5), new Point3D(1, 0.5, 0)),
                 makeCubicInterpBSpline(new Point3D(0, 1, 0), new Point3D(0.25, 1, 0.5), new Point3D(0.75, 1, -0.2), new Point3D(1, 1, 0)));

        NCollection_HArray1<Point3D> aGuidePoints = new(1, 3);
        aGuidePoints.SetValue(1, aProfiles[1].Value(0.4));
        aGuidePoints.SetValue(2, aProfiles[2].Value(0.5));
        aGuidePoints.SetValue(3, aProfiles[3].Value(0.6));
        NCollection_HArray1<double> aGuideParams = new(1, 3);
        aGuideParams.SetValue(1, 0.0);
        aGuideParams.SetValue(2, 0.5);
        aGuideParams.SetValue(3, 1.0);
        ApiInterpolate anInterpolator = new(aGuidePoints, aGuideParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterpolator.Perform();
        Assert.True(anInterpolator.IsDone);

        NCollection_Array1<Curve3D> aGuides =
          Curves(makeLinearBSpline(aProfiles[1].Value(0.0), aProfiles[3].Value(0.0)), anInterpolator.Curve(),
                 makeLinearBSpline(aProfiles[1].Value(1.0), aProfiles[3].Value(1.0)));

        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();

        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        BSplineSurface aSurface = aGordon.Surface();
        Assert.True(aGordon.Report().MaxProfileDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aGordon.Report().MaxGuideDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
        double aMiddleProfileV = VMid(aSurface);
        const int THE_NB_SAMPLES = 32;
        for (int aSampleIdx = 0; aSampleIdx <= THE_NB_SAMPLES; ++aSampleIdx)
        {
            double aRatio = (double)aSampleIdx / THE_NB_SAMPLES;
            double aU = aSurface.UKnot(1) + aRatio * (aSurface.UKnot(aSurface.UKnotCount) - aSurface.UKnot(1));
            double aProfile1Param = aRatio <= 0.5 ? 0.8 * aRatio : 1.2 * aRatio - 0.2;
            double aProfile3Param = aRatio <= 0.5 ? 1.2 * aRatio : 0.2 + 0.8 * aRatio;
            Assert.True(aSurface.Value(aU, aSurface.VKnot(1)).DistanceTo(aProfiles[1].Value(aProfile1Param)) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.True(aSurface.Value(aU, aMiddleProfileV).DistanceTo(aProfiles[2].Value(aRatio)) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.True(aSurface.Value(aU, aSurface.VKnot(aSurface.VKnotCount)).DistanceTo(aProfiles[3].Value(aProfile3Param))
                        <= global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        double aGuideU = UMid(aSurface);
        for (int aSampleIdx = 0; aSampleIdx <= THE_NB_SAMPLES; ++aSampleIdx)
        {
            double aRatio = (double)aSampleIdx / THE_NB_SAMPLES;
            double aV = aSurface.VKnot(1) + aRatio * (aSurface.VKnot(aSurface.VKnotCount) - aSurface.VKnot(1));
            Assert.True(aSurface.Value(aGuideU, aV).DistanceTo(aGuides[2].Value(aRatio)) <= global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void MixedCurveTypes_ProducesValidSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), 2.0), Segment(new Point3D(0, 2, 0), new Direction3D(1, 0, 0), 2.0)),
                     Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), 2.0), Segment(new Point3D(2, 0, 0), new Direction3D(0, 1, 0), 2.0)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        verifyPointOnSurface(aGordon.Surface(), aGordon.Surface().UKnot(1), aGordon.Surface().VKnot(1), new Point3D(0, 0, 0), 0.01);
    }

    [Fact]
    public void PeriodicBSplineProfiles_AreExpandedBeforeConstruction()
    {
        NCollection_Array1<Curve3D> aProfiles = Curves(makePeriodicProfile(0.0), makePeriodicProfile(0.5), makePeriodicProfile(1.0));
        Assert.True(((BSplineCurve3D)aProfiles[1]).IsPeriodic);
        NCollection_Array1<Curve3D> aGuides =
          Curves(makeLinearBSpline(aProfiles[1].Value(0.0), aProfiles[3].Value(0.0)),
                 makeLinearBSpline(aProfiles[1].Value(0.5), aProfiles[3].Value(0.5)),
                 makeLinearBSpline(aProfiles[1].Value(1.0), aProfiles[3].Value(1.0)));
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.True(aGordon.Surface().IsUPeriodic);
    }

    [Fact]
    public void PeriodicBSplineGuides_AreExpandedBeforeConstruction()
    {
        NCollection_Array1<Curve3D> aGuides = Curves(makePeriodicGuide(0.0), makePeriodicGuide(0.5), makePeriodicGuide(1.0));
        Assert.True(((BSplineCurve3D)aGuides[1]).IsPeriodic);
        NCollection_Array1<Curve3D> aProfiles =
          Curves(makeLinearBSpline(aGuides[1].Value(0.0), aGuides[3].Value(0.0)),
                 makeLinearBSpline(aGuides[1].Value(0.5), aGuides[3].Value(0.5)),
                 makeLinearBSpline(aGuides[1].Value(1.0), aGuides[3].Value(1.0)));
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.True(aGordon.Surface().IsVPeriodic);
    }

    [Fact]
    public void InterpolatedPeriodicProfilesWithAlignedSeams_InterpolateWithoutDeviation()
    {
        NCollection_Array1<Curve3D> aProfiles = Curves(makeInterpolatedPeriodicProfile(0.0, 0), makeInterpolatedPeriodicProfile(1.0, 2));

        BSplineCurve3D aFirstProfile = (BSplineCurve3D)aProfiles[1];
        BSplineCurve3D aSecondProfile = (BSplineCurve3D)aProfiles[2];
        Point3D aFirstSeam = aFirstProfile.Value(aFirstProfile.FirstParameter);
        ProjectPointOnCurve aSeamProjection = new(new Point3D(aFirstSeam.X, 1.0, aFirstSeam.Z), aSecondProfile);
        Assert.True(aSeamProjection.NbPoints() > 0);
        // OCCT changes the (frozen) interpolated curve in place: the port uses the twin.
        aSecondProfile = aSecondProfile.WithOrigin(aSeamProjection.LowerDistanceParameter(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aProfiles[2] = aSecondProfile;
        double aFirstProfileMiddle = 0.5 * (aFirstProfile.FirstParameter + aFirstProfile.LastParameter);
        double aSecondProfileMiddle = 0.5 * (aSecondProfile.FirstParameter + aSecondProfile.LastParameter);
        Point3D aSecondSeam = aSecondProfile.Value(aSecondProfile.FirstParameter);
        Point3D aFirstMiddle = aFirstProfile.Value(aFirstProfileMiddle);
        Point3D aSecondMiddle = aSecondProfile.Value(aSecondProfileMiddle);
        Assert.True(aFirstSeam.DistanceTo(new Point3D(aSecondSeam.X, aFirstSeam.Y, aSecondSeam.Z)) <= global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aFirstMiddle.DistanceTo(new Point3D(aSecondMiddle.X, aFirstMiddle.Y, aSecondMiddle.Z)) <= global::Kerf.Engine.Foundation.Precision.Confusion());

        NCollection_Array1<Curve3D> aGuides =
          Curves(makeLinearBSpline(aFirstProfile.Value(aFirstProfile.FirstParameter), aSecondProfile.Value(aSecondProfile.FirstParameter)),
                 makeLinearBSpline(aFirstProfile.Value(aFirstProfileMiddle), aSecondProfile.Value(aSecondProfileMiddle)),
                 makeLinearBSpline(aFirstProfile.Value(aFirstProfile.FirstParameter), aSecondProfile.Value(aSecondProfile.FirstParameter)));

        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();

        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        BSplineSurface aSurface = aGordon.Surface();
        Assert.True(aSurface.IsUPeriodic);
        Assert.True(aGordon.Report().MaxProfileDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aGordon.Report().MaxGuideDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());

        double aMiddleV = VMid(aSurface);
        Surface.ResD1 aFirstSeamD1 = aSurface.EvalD1(aSurface.UKnot(1), aMiddleV);
        Surface.ResD1 aLastSeamD1 = aSurface.EvalD1(aSurface.UKnot(aSurface.UKnotCount), aMiddleV);
        Assert.True(aFirstSeamD1.D1U.AngleTo(aLastSeamD1.D1U) <= global::Kerf.Engine.Foundation.Precision.Angular());

        const int THE_NB_SAMPLES = 16;
        for (int aSampleIdx = 0; aSampleIdx <= THE_NB_SAMPLES; ++aSampleIdx)
        {
            double aRatio = (double)aSampleIdx / THE_NB_SAMPLES;
            double aU = aSurface.UKnot(1) + aRatio * (aSurface.UKnot(aSurface.UKnotCount) - aSurface.UKnot(1));
            double aFirstProfileParam = aFirstProfile.FirstParameter + aRatio * (aFirstProfile.LastParameter - aFirstProfile.FirstParameter);
            double aSecondProfileParam = aSecondProfile.FirstParameter + aRatio * (aSecondProfile.LastParameter - aSecondProfile.FirstParameter);
            Assert.True(aSurface.Value(aU, aSurface.VKnot(1)).DistanceTo(aFirstProfile.Value(aFirstProfileParam)) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.True(aSurface.Value(aU, aSurface.VKnot(aSurface.VKnotCount)).DistanceTo(aSecondProfile.Value(aSecondProfileParam))
                        <= global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }

    [Fact]
    public void MultipleIntersections_SelectsMonotoneBranch()
    {
        NCollection_Array1<Curve3D> aProfiles = Curves(makeMeanderingProfile(0.0), makeMeanderingProfile(0.5), makeMeanderingProfile(1.0));
        NCollection_Array1<Curve3D> aGuides =
          Curves(makeLinearBSpline(new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 1.0, 0.0)),
                 makeLinearBSpline(new Point3D(0.5, 0.0, 0.0), new Point3D(0.5, 1.0, 0.0)),
                 makeLinearBSpline(new Point3D(1.0, 0.0, 0.0), new Point3D(1.0, 1.0, 0.0)));
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, 1.0e-5);
        aGordon.Perform();

        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        Assert.True(aGordon.Report().MaxProfileDeviation <= 1.0e-5);
        Assert.True(aGordon.Report().MaxGuideDeviation <= 1.0e-5);
        verifyPointOnSurface(aGordon.Surface(), 0.0, 0.0, new Point3D(0.0, 0.0, 0.0), 1.0e-3);
        verifyPointOnSurface(aGordon.Surface(), 1.0, 0.0, new Point3D(1.0, 0.0, 0.0), 1.0e-3);
        verifyPointOnSurface(aGordon.Surface(), 0.0, 1.0, new Point3D(0.0, 1.0, 0.0), 1.0e-3);
        verifyPointOnSurface(aGordon.Surface(), 1.0, 1.0, new Point3D(1.0, 1.0, 0.0), 1.0e-3);

        Point3D aFirstCenter = aGordon.Surface().Value(0.5, 0.5);
        aGordon.Perform();
        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        Assert.True(aFirstCenter.DistanceTo(aGordon.Surface().Value(0.5, 0.5)) <= global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void FourByThreeGrid_NonUniformParams()
    {
        NCollection_Array1<Curve3D> aProfiles =
          Curves(Segment(new Point3D(0, 0.0, 0), new Direction3D(1, 0, 0), 1.0), Segment(new Point3D(0, 0.2, 0), new Direction3D(1, 0, 0), 1.0),
                 Segment(new Point3D(0, 0.7, 0), new Direction3D(1, 0, 0), 1.0), Segment(new Point3D(0, 1.0, 0), new Direction3D(1, 0, 0), 1.0));
        NCollection_Array1<Curve3D> aGuides =
          Curves(Segment(new Point3D(0.0, 0, 0), new Direction3D(0, 1, 0), 1.0), Segment(new Point3D(0.3, 0, 0), new Direction3D(0, 1, 0), 1.0),
                 Segment(new Point3D(1.0, 0, 0), new Direction3D(0, 1, 0), 1.0));
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.NotNull(aGordon.Surface());
    }

    [Fact]
    public void SurfaceAlongInputCurves_IsAccurate()
    {
        Gordon aGordon = new();
        aGordon.Init(UnitSquareProfiles(), UnitSquareGuides(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        double aUMin = aSurf.UKnot(1), aUMax = aSurf.UKnot(aSurf.UKnotCount);
        double aVMin = aSurf.VKnot(1), aVMax = aSurf.VKnot(aSurf.VKnotCount);
        const int THE_NB_SAMPLES = 20;
        for (int i = 0; i <= THE_NB_SAMPLES; ++i)
        {
            double aU = aUMin + i * (aUMax - aUMin) / THE_NB_SAMPLES;
            Point3D aSurfPt = aSurf.Value(aU, aVMin);
            Assert.Equal(0.0, aSurfPt.Y, 0.01);
            Assert.Equal(0.0, aSurfPt.Z, 0.01);
        }
        for (int i = 0; i <= THE_NB_SAMPLES; ++i)
        {
            double aU = aUMin + i * (aUMax - aUMin) / THE_NB_SAMPLES;
            Point3D aSurfPt = aSurf.Value(aU, aVMax);
            Assert.Equal(1.0, aSurfPt.Y, 0.01);
            Assert.Equal(0.0, aSurfPt.Z, 0.01);
        }
    }

    [Fact]
    public void ReversedCurveNetwork_ProducesValidSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(1, 0, 0), new Direction3D(-1, 0, 0), 1.0), Segment(new Point3D(0, 1, 0), new Direction3D(1, 0, 0), 1.0)),
                     Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), 1.0), Segment(new Point3D(1, 1, 0), new Direction3D(0, -1, 0), 1.0)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        double aUMin = aSurf.UKnot(1), aUMax = aSurf.UKnot(aSurf.UKnotCount);
        double aVMin = aSurf.VKnot(1), aVMax = aSurf.VKnot(aSurf.VKnotCount);
        Point3D[] aCorners = [aSurf.Value(aUMin, aVMin), aSurf.Value(aUMax, aVMin), aSurf.Value(aUMin, aVMax), aSurf.Value(aUMax, aVMax)];
        double aXMin = RealLast(), aXMax = -RealLast();
        double aYMin = RealLast(), aYMax = -RealLast();
        for (int i = 0; i < 4; ++i)
        {
            Assert.Equal(0.0, aCorners[i].Z, 0.01);
            aXMin = Math.Min(aXMin, aCorners[i].X);
            aXMax = Math.Max(aXMax, aCorners[i].X);
            aYMin = Math.Min(aYMin, aCorners[i].Y);
            aYMax = Math.Max(aYMax, aCorners[i].Y);
        }
        Assert.Equal(0.0, aXMin, 0.01);
        Assert.Equal(1.0, aXMax, 0.01);
        Assert.Equal(0.0, aYMin, 0.01);
        Assert.Equal(1.0, aYMax, 0.01);
    }

    [Fact]
    public void CubicInterpolatedNetwork_ProducesValidSurface()
    {
        NCollection_Array1<Curve3D> aProfiles =
          Curves(makeCubicInterpBSpline(new Point3D(0, 0, 0), new Point3D(0.33, 0, 0.3), new Point3D(0.66, 0, -0.3), new Point3D(1, 0, 0)),
                 makeCubicInterpBSpline(new Point3D(0, 0.5, 0), new Point3D(0.33, 0.5, 0.5), new Point3D(0.66, 0.5, -0.5), new Point3D(1, 0.5, 0)),
                 makeCubicInterpBSpline(new Point3D(0, 1, 0), new Point3D(0.33, 1, 0.3), new Point3D(0.66, 1, -0.3), new Point3D(1, 1, 0)));
        Gordon aGordon = new();
        aGordon.Init(aProfiles, LinearSideGuides(), 0.01);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        double aUMid = UMid(aSurf);
        double aVMid = VMid(aSurf);
        Point3D aP1 = aSurf.Value(aUMid * 0.5, aVMid);
        Point3D aP2 = aSurf.Value(aUMid * 1.5, aVMid);
        Assert.Equal(0.6328125, aP1.Z, 1.0e-9);
        Assert.Equal(-0.6328125, aP2.Z, 1.0e-9);
    }

    [Fact]
    public void SinusoidalProfiles_SurfacePreservesShape()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(makeSineBSpline(0.0, 1.0, 0.0, 0.3), makeSineBSpline(0.0, 1.0, 0.5, 0.5), makeSineBSpline(0.0, 1.0, 1.0, 0.3)),
                     LinearSideGuides(), 0.01);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Assert.Equal(0.5, aSurf.Value(UMid(aSurf), VMid(aSurf)).Z, 0.05);
    }

    [Fact]
    public void ThreeByThreeGrid_SurfaceInterpolatesAllCurves()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), 1.0), Segment(new Point3D(0, 0.5, 0), new Direction3D(1, 0, 0), 1.0),
                            Segment(new Point3D(0, 1, 0), new Direction3D(1, 0, 0), 1.0)),
                     Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), 1.0), Segment(new Point3D(0.5, 0, 0), new Direction3D(0, 1, 0), 1.0),
                            Segment(new Point3D(1, 0, 0), new Direction3D(0, 1, 0), 1.0)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        double aUMin = aSurf.UKnot(1), aUMax = aSurf.UKnot(aSurf.UKnotCount);
        double aVMin = aSurf.VKnot(1), aVMax = aSurf.VKnot(aSurf.VKnotCount);
        const int THE_NB = 10;
        for (int i = 0; i <= THE_NB; ++i)
        {
            double aU = aUMin + i * (aUMax - aUMin) / THE_NB;
            Assert.Equal(0.0, aSurf.Value(aU, aVMin).Y, 0.01);
            Assert.Equal(1.0, aSurf.Value(aU, aVMax).Y, 0.01);
            double aV = aVMin + i * (aVMax - aVMin) / THE_NB;
            Assert.Equal(0.0, aSurf.Value(aUMin, aV).X, 0.01);
            Assert.Equal(1.0, aSurf.Value(aUMax, aV).X, 0.01);
        }
    }

    [Fact]
    public void ScaledGeometry_LargeCoordinates()
    {
        double aScale = 1000.0;
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), aScale), Segment(new Point3D(0, aScale, 0), new Direction3D(1, 0, 0), aScale)),
                     Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), aScale), Segment(new Point3D(aScale, 0, 0), new Direction3D(0, 1, 0), aScale)),
                     0.1);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Point3D aMidPt = aSurf.Value(UMid(aSurf), VMid(aSurf));
        Assert.Equal(500.0, aMidPt.X, 1.0);
        Assert.Equal(500.0, aMidPt.Y, 1.0);
        Assert.Equal(0.0, aMidPt.Z, 1.0);
    }

    [Fact]
    public void ScaledGeometry_SmallCoordinates()
    {
        double aScale = 1.0e-3;
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), aScale), Segment(new Point3D(0, aScale, 0), new Direction3D(1, 0, 0), aScale)),
                     Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), aScale), Segment(new Point3D(aScale, 0, 0), new Direction3D(0, 1, 0), aScale)),
                     aScale * 1.0e-4);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.NotNull(aGordon.Surface());
    }

    [Fact]
    public void AllReversedCurves_ProducesValidSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(1, 0, 0), new Direction3D(-1, 0, 0), 1.0), Segment(new Point3D(1, 1, 0), new Direction3D(-1, 0, 0), 1.0)),
                     Curves(Segment(new Point3D(0, 1, 0), new Direction3D(0, -1, 0), 1.0), Segment(new Point3D(1, 1, 0), new Direction3D(0, -1, 0), 1.0)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.NotNull(aGordon.Surface());
    }

    [Fact]
    public void ShuffledInputOrder_ProducesValidSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 1.0, 0), new Direction3D(1, 0, 0), 1.0), Segment(new Point3D(0, 0.5, 0), new Direction3D(1, 0, 0), 1.0),
                            Segment(new Point3D(0, 0.0, 0), new Direction3D(1, 0, 0), 1.0)),
                     Curves(Segment(new Point3D(1, 0, 0), new Direction3D(0, 1, 0), 1.0), Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), 1.0)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Assert.Equal(0.0, aSurf.Value(UMid(aSurf), VMid(aSurf)).Z, 0.01);
    }

    [Fact]
    public void SaddleSurface_BothDirectionsCurved()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0.5), new Point3D(1, 0, 0)),
                            makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(0.5, 1, 0.5), new Point3D(1, 1, 0))),
                     Curves(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0, 0.5, -0.5), new Point3D(0, 1, 0)),
                            makeQuadraticBSpline(new Point3D(1, 0, 0), new Point3D(1, 0.5, -0.5), new Point3D(1, 1, 0))),
                     0.01);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Assert.Equal(0.0, aSurf.Value(UMid(aSurf), VMid(aSurf)).Z, 0.5);
    }

    [Fact]
    public void AsymmetricNetwork_FiveProfilesTwoGuides()
    {
        NCollection_Array1<Curve3D> aProfiles = new(1, 5);
        for (int i = 0; i < 5; ++i)
        {
            aProfiles[i + 1] = Segment(new Point3D(0, i * 0.25, 0), new Direction3D(1, 0, 0), 1.0);
        }
        Gordon aGordon = new();
        aGordon.Init(aProfiles, UnitSquareGuides(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.NotNull(aGordon.Surface());
    }

    [Fact]
    public void AsymmetricNetwork_TwoProfilesFiveGuides()
    {
        NCollection_Array1<Curve3D> aGuides = new(1, 5);
        for (int i = 0; i < 5; ++i)
        {
            aGuides[i + 1] = Segment(new Point3D(i * 0.25, 0, 0), new Direction3D(0, 1, 0), 1.0);
        }
        Gordon aGordon = new();
        aGordon.Init(UnitSquareProfiles(), aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.NotNull(aGordon.Surface());
    }

    [Fact]
    public void NonPlanarNetwork_CurvesInBothDirections()
    {
        NCollection_Array1<Curve3D> aProfiles = Curves(makeSineBSpline(0.0, 1.0, 0.0, 0.2), makeSineBSpline(0.0, 1.0, 1.0, 0.2));
        NCollection_HArray1<Point3D> aGuidPnts1 = new(1, 11);
        NCollection_HArray1<Point3D> aGuidPnts2 = new(1, 11);
        NCollection_HArray1<double> aGuidParams = new(1, 11);
        for (int i = 1; i <= 11; ++i)
        {
            double aT = (i - 1) / 10.0;
            double aZ1 = 0.15 * Math.Sin(Math.PI * aT);
            double aZ2 = 0.15 * Math.Sin(Math.PI * aT);
            aGuidPnts1.SetValue(i, new Point3D(0.0, aT, aZ1));
            aGuidPnts2.SetValue(i, new Point3D(1.0, aT, aZ2));
            aGuidParams.SetValue(i, aT);
        }
        ApiInterpolate anInterp1 = new(aGuidPnts1, aGuidParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterp1.Perform();
        ApiInterpolate anInterp2 = new(aGuidPnts2, aGuidParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        anInterp2.Perform();

        Gordon aGordon = new();
        aGordon.Init(aProfiles, Curves(anInterp1.Curve(), anInterp2.Curve()), 0.01);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Assert.Equal(0.35, aSurf.Value(UMid(aSurf), VMid(aSurf)).Z, 0.05);
    }

    [Fact]
    public void SurfaceContinuity_IsSmooth()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0.3), new Point3D(1, 0, 0)),
                            makeQuadraticBSpline(new Point3D(0, 0.5, 0), new Point3D(0.5, 0.5, 0.5), new Point3D(1, 0.5, 0)),
                            makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(0.5, 1, 0.3), new Point3D(1, 1, 0))),
                     LinearSideGuides(), 0.01);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        double aUMin = aSurf.UKnot(1), aUMax = aSurf.UKnot(aSurf.UKnotCount);
        double aV = VMid(aSurf);
        const int THE_NB = 50;
        const double THE_DERIV_TOL = 1.0;
        double aStep = (aUMax - aUMin) / THE_NB;
        aSurf.D1(aUMin + 2.0 * aStep, aV, out Point3D _, out Vector3D aPrevDU, out Vector3D _);
        for (int i = 3; i <= THE_NB - 2; ++i)
        {
            double aU = aUMin + i * aStep;
            aSurf.D1(aU, aV, out Point3D _, out Vector3D aDU, out Vector3D _);
            double aDUChange = aDU.Subtracted(aPrevDU).Magnitude;
            Assert.True(aDUChange < THE_DERIV_TOL, $"DU jump at U={aU}: {aDUChange}");
            aPrevDU = aDU;
        }
    }

    [Fact]
    public void ArcLikeProfiles_NonRational()
    {
        static BSplineCurve3D makeArcProfile(double theY)
        {
            const int THE_NB = 9;
            NCollection_HArray1<Point3D> aPnts = new(1, THE_NB);
            NCollection_HArray1<double> aParams = new(1, THE_NB);
            for (int i = 1; i <= THE_NB; ++i)
            {
                double aT = (double)(i - 1) / (THE_NB - 1);
                double aAngle = Math.PI * aT;
                double aX = 0.5 - 0.5 * Math.Cos(aAngle);
                double aZ = 0.3 * Math.Sin(aAngle);
                aPnts.SetValue(i, new Point3D(aX, theY, aZ));
                aParams.SetValue(i, aT);
            }
            ApiInterpolate anInterp = new(aPnts, aParams, false, global::Kerf.Engine.Foundation.Precision.Confusion());
            anInterp.Perform();
            return anInterp.Curve();
        }

        Gordon aGordon = new();
        aGordon.Init(Curves(makeArcProfile(0.0), makeArcProfile(1.0)), LinearSideGuides(), 0.01);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Assert.Equal(0.3, aSurf.Value(UMid(aSurf), VMid(aSurf)).Z, 0.05);
    }

    [Fact]
    public void RepeatPerform_GivesSameResult()
    {
        Gordon aGordon = new();
        aGordon.Init(UnitSquareProfiles(), UnitSquareGuides(), global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Point3D aMid1 = aGordon.Surface().Value(UMid(aGordon.Surface()), VMid(aGordon.Surface()));
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Point3D aMid2 = aGordon.Surface().Value(UMid(aGordon.Surface()), VMid(aGordon.Surface()));
        Assert.Equal(0.0, aMid1.DistanceTo(aMid2), global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(Gordon.ResultStatus.Done, aGordon.Report().Status);
    }

    [Fact]
    public void HighDensityNetwork_SixBySix()
    {
        const int THE_NB = 6;
        NCollection_Array1<Curve3D> aProfiles = new(1, THE_NB);
        for (int i = 0; i < THE_NB; ++i)
        {
            aProfiles[i + 1] = Segment(new Point3D(0, (double)i / (THE_NB - 1), 0), new Direction3D(1, 0, 0), 1.0);
        }
        NCollection_Array1<Curve3D> aGuides = new(1, THE_NB);
        for (int j = 0; j < THE_NB; ++j)
        {
            aGuides[j + 1] = Segment(new Point3D((double)j / (THE_NB - 1), 0, 0), new Direction3D(0, 1, 0), 1.0);
        }
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        double aUMin = aSurf.UKnot(1), aUMax = aSurf.UKnot(aSurf.UKnotCount);
        double aVMin = aSurf.VKnot(1), aVMax = aSurf.VKnot(aSurf.VKnotCount);
        for (int i = 0; i < THE_NB; ++i)
        {
            for (int j = 0; j < THE_NB; ++j)
            {
                double aX = (double)j / (THE_NB - 1);
                double aY = (double)i / (THE_NB - 1);
                Point3D aSurfPt = aSurf.Value(aUMin + aX * (aUMax - aUMin), aVMin + aY * (aVMax - aVMin));
                Assert.Equal(0.0, aSurfPt.Z, 0.02);
            }
        }
    }

    [Fact]
    public void QuadraticProfiles_SurfaceMatchesCurvesAtBoundaries()
    {
        NCollection_Array1<Curve3D> aProfiles =
          Curves(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 1.0), new Point3D(1, 0, 0)),
                 makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(0.5, 1, 1.0), new Point3D(1, 1, 0)));
        Gordon aGordon = new();
        aGordon.Init(aProfiles, LinearSideGuides(), 0.01);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        double aUMin = aSurf.UKnot(1), aUMax = aSurf.UKnot(aSurf.UKnotCount);
        double aVMin = aSurf.VKnot(1), aVMax = aSurf.VKnot(aSurf.VKnotCount);
        for (int i = 0; i <= 10; ++i)
        {
            double aT = i / 10.0;
            double aU = aUMin + aT * (aUMax - aUMin);
            Assert.True(aSurf.Value(aU, aVMin).DistanceTo(aProfiles[1].Value(aT)) < 0.05);
            Assert.True(aSurf.Value(aU, aVMax).DistanceTo(aProfiles[2].Value(aT)) < 0.05);
        }
    }

    [Fact]
    public void OffsetPlane_NetworkAtNonZeroZ()
    {
        double aZ = 5.0;
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 0, aZ), new Direction3D(1, 0, 0), 1.0), Segment(new Point3D(0, 1, aZ), new Direction3D(1, 0, 0), 1.0)),
                     Curves(Segment(new Point3D(0, 0, aZ), new Direction3D(0, 1, 0), 1.0), Segment(new Point3D(1, 0, aZ), new Direction3D(0, 1, 0), 1.0)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Assert.Equal(aZ, aSurf.Value(UMid(aSurf), VMid(aSurf)).Z, 0.01);
    }

    [Fact]
    public void TiltedPlane_DiagonalCurves()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 1)), makeLinearBSpline(new Point3D(0, 1, 1), new Point3D(1, 1, 2))),
                     Curves(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 1)), makeLinearBSpline(new Point3D(1, 0, 1), new Point3D(1, 1, 2))),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        double aUMin = aSurf.UKnot(1), aUMax = aSurf.UKnot(aSurf.UKnotCount);
        double aVMin = aSurf.VKnot(1), aVMax = aSurf.VKnot(aSurf.VKnotCount);
        for (int i = 0; i <= 5; ++i)
        {
            for (int j = 0; j <= 5; ++j)
            {
                Point3D aPt = aSurf.Value(aUMin + i * (aUMax - aUMin) / 5.0, aVMin + j * (aVMax - aVMin) / 5.0);
                Assert.Equal(aPt.X + aPt.Y, aPt.Z, 0.05);
            }
        }
    }

    [Fact]
    public void RectangularNotSquare_2x3AspectRatio()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(Segment(new Point3D(0, 0, 0), new Direction3D(1, 0, 0), 2.0), Segment(new Point3D(0, 3, 0), new Direction3D(1, 0, 0), 2.0)),
                     Curves(Segment(new Point3D(0, 0, 0), new Direction3D(0, 1, 0), 3.0), Segment(new Point3D(2, 0, 0), new Direction3D(0, 1, 0), 3.0)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Point3D aMidPt = aSurf.Value(UMid(aSurf), VMid(aSurf));
        Assert.Equal(1.0, aMidPt.X, 0.1);
        Assert.Equal(1.5, aMidPt.Y, 0.1);
    }

    [Fact]
    public void WavySurface_SinusoidalProfilesAndGuides()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(makeSineBSpline(0.0, 1.0, 0.0, 0.3), makeSineBSpline(0.0, 1.0, 0.5, 0.6), makeSineBSpline(0.0, 1.0, 1.0, 0.3)),
                     LinearSideGuides(), 0.01);
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        BSplineSurface aSurf = aGordon.Surface();
        Assert.Equal(0.6, aSurf.Value(UMid(aSurf), VMid(aSurf)).Z, 0.05);
    }

    private static NCollection_Array1<double> Halves() => Reals(0.0, 0.5, 1.0);

    [Fact]
    public void NetworkSurface_UClosedNetwork_ProducesUPeriodicSurface()
    {
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 0, 0)),
                   makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 1), new Point3D(0, 1, 0)),
                   makeQuadraticBSpline(new Point3D(0, 2, 0), new Point3D(1, 2, 0), new Point3D(0, 2, 0)));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 2, 0)), makeLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 2, 0)),
                   makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 2, 0)));
        NCollection_Array1<double> aGuideParams = Halves();
        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Halves(), aGuideParams, makeIntersectionGrid(aProfiles, aGuideParams),
                      makeUnitWeightGrid(aProfiles, aGuideParams), global::Kerf.Engine.Foundation.Precision.Confusion(), true, false);
        aNetwork.Perform();
        Assert.True(aNetwork.IsDone);
        Assert.True(aNetwork.Surface().IsUPeriodic);
    }

    [Fact]
    public void NetworkSurface_UClosedNetwork_UsesInputToleranceForPeriodicity()
    {
        const double aSeamGap = 1.0e-5;
        const double aTolerance = 1.0e-4;
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(aSeamGap, 0, 0)),
                   makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 1), new Point3D(aSeamGap, 1, 0)),
                   makeQuadraticBSpline(new Point3D(0, 2, 0), new Point3D(1, 2, 0), new Point3D(aSeamGap, 2, 0)));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 2, 0)), makeLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 2, 0)),
                   makeLinearBSpline(new Point3D(aSeamGap, 0, 0), new Point3D(aSeamGap, 2, 0)));
        NCollection_Array1<double> aGuideParams = Halves();
        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Halves(), aGuideParams, makeIntersectionGrid(aProfiles, aGuideParams),
                      makeUnitWeightGrid(aProfiles, aGuideParams), aTolerance, true, false);
        aNetwork.Perform();
        Assert.True(aNetwork.IsDone, aNetwork.Status().ToString());
        Assert.True(aNetwork.Surface().IsUPeriodic);
    }

    [Fact]
    public void NetworkSurface_InvalidPreparedNetworkReportsStatus()
    {
        NCollection_Array1<BSplineCurve3D?> aProfiles = BSplines(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0)));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0)), makeLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0)));
        NCollection_Array1<double> aGuideParams = Reals(0.0, 1.0);
        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Reals(0.0), aGuideParams, makeIntersectionGrid(aProfiles, aGuideParams),
                      makeUnitWeightGrid(aProfiles, aGuideParams), global::Kerf.Engine.Foundation.Precision.Confusion(), false, false);
        Assert.Equal(NetworkSurface.ResultStatus.NotStarted, aNetwork.Status());
        aNetwork.Perform();
        Assert.False(aNetwork.IsDone);
        Assert.Equal(NetworkSurface.ResultStatus.InvalidInput, aNetwork.Status());
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => aNetwork.Surface());
    }

    [Fact]
    public void NetworkSurface_InvalidWeightsReportsInvalidInput()
    {
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0)), makeLinearBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 0)));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0)), makeLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0)));
        NCollection_Array1<double> aGuideParams = Reals(0.0, 1.0);
        NCollection_Array2<double> aWeights = makeUnitWeightGrid(aProfiles, aGuideParams);
        aWeights[2, 2] = 0.0;
        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Reals(0.0, 1.0), aGuideParams, makeIntersectionGrid(aProfiles, aGuideParams), aWeights,
                      global::Kerf.Engine.Foundation.Precision.Confusion(), false, false);
        aNetwork.Perform();
        Assert.False(aNetwork.IsDone);
        Assert.Equal(NetworkSurface.ResultStatus.InvalidInput, aNetwork.Status());
    }

    [Fact]
    public void NetworkSurface_CompatibleRationalWeightsProducesRationalSurface()
    {
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(makeRationalQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0), new Point3D(1, 0, 0), 0.5),
                   makeRationalQuadraticBSpline(new Point3D(0, 0.5, 0), new Point3D(0.5, 0.5, 0), new Point3D(1, 0.5, 0), 0.5),
                   makeRationalQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(0.5, 1, 0), new Point3D(1, 1, 0), 0.5));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(makeWeightedLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0), 1.0),
                   makeWeightedLinearBSpline(new Point3D(0.5, 0, 0), new Point3D(0.5, 1, 0), 0.75),
                   makeWeightedLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0), 1.0));
        NCollection_Array1<double> aGuideParams = Halves();
        NCollection_Array2<double> aWeights = new(1, 3, 1, 3);
        for (int aProfileIdx = 1; aProfileIdx <= 3; ++aProfileIdx)
        {
            aWeights[1, aProfileIdx] = 1.0;
            aWeights[2, aProfileIdx] = 0.75;
            aWeights[3, aProfileIdx] = 1.0;
        }
        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Halves(), aGuideParams, makeIntersectionGrid(aProfiles, aGuideParams), aWeights,
                      global::Kerf.Engine.Foundation.Precision.Confusion(), false, false);
        aNetwork.Perform();
        Assert.True(aNetwork.IsDone);
        Assert.True(aNetwork.Surface().IsURational || aNetwork.Surface().IsVRational);
    }

    [Fact]
    public void NetworkSurface_IncompatibleRationalFamilyReportsStatus()
    {
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(makeRationalQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0), new Point3D(1, 0, 0), 0.5),
                   makeWeightedLinearBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 0), 1.0));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0)), makeLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0)));
        NCollection_Array1<double> aGuideParams = Reals(0.0, 1.0);
        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Reals(0.0, 1.0), aGuideParams, makeIntersectionGrid(aProfiles, aGuideParams),
                      makeUnitWeightGrid(aProfiles, aGuideParams), global::Kerf.Engine.Foundation.Precision.Confusion(), false, false);
        aNetwork.Perform();
        Assert.False(aNetwork.IsDone);
        Assert.Equal(NetworkSurface.ResultStatus.CurveCompatibilityFailed, aNetwork.Status());
    }

    private static (NCollection_Array1<Curve3D> Profiles, NCollection_Array1<Curve3D> Guides) DegreeNineNetwork()
    {
        const int aDegree = 9;
        return (Curves(makeRationalLineBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0), aDegree),
                       makeRationalLineBSpline(new Point3D(0, 0.5, 0), new Point3D(1, 0.5, 0), aDegree),
                       makeRationalLineBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 0), aDegree)),
                Curves(makeRationalLineBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0), aDegree),
                       makeRationalLineBSpline(new Point3D(0.5, 0, 0), new Point3D(0.5, 1, 0), aDegree),
                       makeRationalLineBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0), aDegree)));
    }

    [Fact]
    public void NetworkSurface_RationalDegreeOverflowReportsStatus()
    {
        const int aDegree = 9;
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(makeRationalLineBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0), aDegree),
                   makeRationalLineBSpline(new Point3D(0, 0.5, 0), new Point3D(1, 0.5, 0), aDegree),
                   makeRationalLineBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 0), aDegree));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(makeRationalLineBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0), aDegree),
                   makeRationalLineBSpline(new Point3D(0.5, 0, 0), new Point3D(0.5, 1, 0), aDegree),
                   makeRationalLineBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0), aDegree));
        NCollection_Array1<double> aGuideParams = Halves();
        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Halves(), aGuideParams, makeIntersectionGrid(aProfiles, aGuideParams),
                      makeUnitWeightGrid(aProfiles, aGuideParams), global::Kerf.Engine.Foundation.Precision.Confusion(), false, false);
        aNetwork.Perform();
        Assert.False(aNetwork.IsDone);
        Assert.Equal(NetworkSurface.ResultStatus.RationalDegreeOverflow, aNetwork.Status());
    }

    [Fact]
    public void RationalProfilesWithPolynomialGuidesProducesSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(makeRationalQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0.5, 0, 0), new Point3D(1, 0, 0), 0.5),
                            makeRationalQuadraticBSpline(new Point3D(0, 0.5, 0), new Point3D(0.5, 0.5, 0), new Point3D(1, 0.5, 0), 0.5),
                            makeRationalQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(0.5, 1, 0), new Point3D(1, 1, 0), 0.5)),
                     Curves(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0)), makeLinearBSpline(new Point3D(0.5, 0, 0), new Point3D(0.5, 1, 0)),
                            makeLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0))),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.False(aGordon.IsApproximate);
        Assert.True(aGordon.Surface().IsURational || aGordon.Surface().IsVRational);
        Assert.True(aGordon.Report().MaxProfileDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aGordon.Report().MaxGuideDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void PolynomialProfilesWithRationalGuidesProducesSurface()
    {
        Gordon aGordon = new();
        aGordon.Init(Curves(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0)), makeLinearBSpline(new Point3D(0, 0.5, 0), new Point3D(1, 0.5, 0)),
                            makeLinearBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 0))),
                     Curves(makeRationalQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0, 0.5, 0), new Point3D(0, 1, 0), 0.5),
                            makeRationalQuadraticBSpline(new Point3D(0.5, 0, 0), new Point3D(0.5, 0.5, 0), new Point3D(0.5, 1, 0), 0.5),
                            makeRationalQuadraticBSpline(new Point3D(1, 0, 0), new Point3D(1, 0.5, 0), new Point3D(1, 1, 0), 0.5)),
                     global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.False(aGordon.IsApproximate);
        Assert.True(aGordon.Surface().IsURational || aGordon.Surface().IsVRational);
        Assert.True(aGordon.Report().MaxProfileDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aGordon.Report().MaxGuideDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void ApproximateFallbackCanRecoverRationalDegreeOverflow()
    {
        (NCollection_Array1<Curve3D> aProfiles, NCollection_Array1<Curve3D> aGuides) = DegreeNineNetwork();
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.SetApproximationMode(Gordon.ApproximationMode.AllowApproximateFallback);
        aGordon.Perform();

        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        Assert.True(aGordon.IsApproximate);
        Assert.Equal(Gordon.ResultStatus.Done, aGordon.Report().Status);
        Assert.True(aGordon.Report().IsApproximate);
        Assert.Equal(Gordon.BuildStage.NotStarted, aGordon.Report().FailedStage);
        Assert.True(aGordon.Report().MaxApproximationDeviation <= 0.001);

        const int THE_NB_QUALITY_SAMPLES = 48;
        BSplineSurface aSurface = aGordon.Surface();
        double aUMin = aSurface.UKnot(1), aUMax = aSurface.UKnot(aSurface.UKnotCount);
        double aVMin = aSurface.VKnot(1), aVMax = aSurface.VKnot(aSurface.VKnotCount);
        double aMaxDeviation = 0.0;
        for (int aUIdx = 0; aUIdx <= THE_NB_QUALITY_SAMPLES; ++aUIdx)
        {
            double aUParam = (double)aUIdx / THE_NB_QUALITY_SAMPLES;
            double aU = aUMin + aUParam * (aUMax - aUMin);
            for (int aVIdx = 0; aVIdx <= THE_NB_QUALITY_SAMPLES; ++aVIdx)
            {
                double aVParam = (double)aVIdx / THE_NB_QUALITY_SAMPLES;
                double aV = aVMin + aVParam * (aVMax - aVMin);
                Point3D aProfilePoint = aProfiles.At(0).Value(aUParam);
                Point3D aGuidePoint = aGuides.At(0).Value(aVParam);
                Point3D anExpectedPoint = new(aProfilePoint.X, aGuidePoint.Y, 0.0);
                aMaxDeviation = Math.Max(aMaxDeviation, aSurface.Value(aU, aV).DistanceTo(anExpectedPoint));
            }
        }
        Assert.True(aMaxDeviation <= 0.002);
    }

    [Fact]
    public void ExactOnlyReportsRationalDegreeOverflow()
    {
        (NCollection_Array1<Curve3D> aProfiles, NCollection_Array1<Curve3D> aGuides) = DegreeNineNetwork();
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(Gordon.ApproximationMode.ExactOnly, aGordon.GetApproximationMode());
        aGordon.Perform();
        Assert.False(aGordon.IsDone);
        Assert.False(aGordon.IsApproximate);
        Assert.Equal(Gordon.ResultStatus.RationalDegreeOverflow, aGordon.Status());
        Assert.Equal(Gordon.ResultStatus.RationalDegreeOverflow, aGordon.Report().Status);
        Assert.Equal(Gordon.BuildStage.ExactConstruction, aGordon.Report().FailedStage);
        Assert.False(aGordon.Report().IsApproximate);
    }

    private static (NCollection_Array1<Curve3D> Profiles, NCollection_Array1<Curve3D> Guides) RationalLineNetwork() =>
        (Curves(makeRationalQuadraticLineBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0), 1.0, 0.45, 1.0),
                makeRationalQuadraticLineBSpline(new Point3D(0, 0.4, 0), new Point3D(1, 0.4, 0), 1.0, 1.8, 0.7),
                makeRationalQuadraticLineBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 0), 0.8, 0.65, 1.5)),
         Curves(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0)), makeLinearBSpline(new Point3D(0.4, 0, 0), new Point3D(0.4, 1, 0)),
                makeLinearBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0))));

    [Fact]
    public void ExactOnlyReparametrizesRationalCurves()
    {
        (NCollection_Array1<Curve3D> aProfiles, NCollection_Array1<Curve3D> aGuides) = RationalLineNetwork();
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone);
        Assert.False(aGordon.IsApproximate);
        Assert.Equal(Gordon.ResultStatus.Done, aGordon.Status());
        Assert.Equal(Gordon.ResultStatus.Done, aGordon.Report().Status);
        Assert.Equal(Gordon.BuildStage.NotStarted, aGordon.Report().FailedStage);
        Assert.False(aGordon.Report().IsApproximate);
        Assert.Equal(0.0, aGordon.Report().MaxReparametrizationDeviation, 1.0e-12);
    }

    [Fact]
    public void ApproximationModeUsesExactRationalReparametrizationWhenPossible()
    {
        (NCollection_Array1<Curve3D> aProfiles, NCollection_Array1<Curve3D> aGuides) = RationalLineNetwork();
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.SetApproximationMode(Gordon.ApproximationMode.AllowApproximateFallback);
        aGordon.Perform();
        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        Assert.False(aGordon.IsApproximate);
        Assert.Equal(Gordon.ResultStatus.Done, aGordon.Report().Status);
        Assert.False(aGordon.Report().IsApproximate);
        Assert.Equal(0.0, aGordon.Report().MaxReparametrizationDeviation, 1.0e-12);
        Assert.NotNull(aGordon.Surface());
    }

    [Fact]
    public void ExactRationalReparametrizationRepeatPerformKeepsReportStable()
    {
        (NCollection_Array1<Curve3D> aProfiles, NCollection_Array1<Curve3D> aGuides) = RationalLineNetwork();
        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.SetApproximationMode(Gordon.ApproximationMode.AllowApproximateFallback);

        Point3D? aFirstCenter = null;
        for (int aRun = 0; aRun < 2; ++aRun)
        {
            aGordon.Perform();
            Assert.True(aGordon.IsDone, aGordon.Status().ToString());
            Assert.False(aGordon.IsApproximate);
            Assert.Equal(Gordon.ResultStatus.Done, aGordon.Report().Status);
            Assert.Equal(Gordon.BuildStage.NotStarted, aGordon.Report().FailedStage);
            Assert.False(aGordon.Report().IsApproximate);
            Assert.Equal(0.0, aGordon.Report().MaxContactGap, 1.0e-12);
            Assert.Equal(0.0, aGordon.Report().MaxReparametrizationDeviation, 1.0e-12);
            Assert.Equal(0.0, aGordon.Report().MaxApproximationDeviation, 1.0e-12);
            Point3D aCenter = aGordon.Surface().Value(0.5, 0.5);
            if (aFirstCenter is { } aFirst)
            {
                Assert.Equal(0.0, aFirst.DistanceTo(aCenter), global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            aFirstCenter = aCenter;
        }
    }

    [Fact]
    public void NetworkSurface_VClosedNetwork_ProducesVPeriodicSurface()
    {
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(2, 0, 0)), makeLinearBSpline(new Point3D(0, 1, 0), new Point3D(2, 1, 1)),
                   makeLinearBSpline(new Point3D(0, 0, 0), new Point3D(2, 0, 0)));
        NCollection_Array1<BSplineCurve3D?> aGuides =
          BSplines(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(0, 1, 0), new Point3D(0, 0, 0)),
                   makeQuadraticBSpline(new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(1, 0, 0)),
                   makeQuadraticBSpline(new Point3D(2, 0, 0), new Point3D(2, 1, 0), new Point3D(2, 0, 0)));
        NCollection_Array1<double> aGuideParams = Halves();
        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Halves(), aGuideParams, makeIntersectionGrid(aProfiles, aGuideParams),
                      makeUnitWeightGrid(aProfiles, aGuideParams), global::Kerf.Engine.Foundation.Precision.Confusion(), false, true);
        aNetwork.Perform();
        Assert.True(aNetwork.IsDone);
        Assert.True(aNetwork.Surface().IsVPeriodic);
    }

    private static void denseNetwork(int theNbProfiles, int theNbGuides, Func<double, double, double> theHeight)
    {
        AnalyticNetwork aNetwork = new(theNbProfiles, theNbGuides, theHeight);
        Gordon aGordon = new();
        aGordon.Init(aNetwork.Profiles, aNetwork.Guides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        Assert.False(aGordon.IsApproximate);
        verifyUniformNetworkInterpolation(aGordon.Surface(), aNetwork, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DenseWavyFiveByFiveNetwork_InterpolatesAllInputCurves() => denseNetwork(5, 5, wavyHeight);

    [Fact]
    public void DenseSaddleSevenByFourNetwork_InterpolatesAllInputCurves() => denseNetwork(7, 4, saddleHeight);

    [Fact]
    public void DenseRippleFourBySevenNetwork_InterpolatesAllInputCurves() => denseNetwork(4, 7, rippleHeight);

    [Fact]
    public void DenseWavyEightByEightNetwork_InterpolatesAllInputCurves() => denseNetwork(8, 8, wavyHeight);

    [Fact]
    public void TranslatedScaledSixByFiveNetwork_InterpolatesAllInputCurves()
    {
        AnalyticNetwork aNetwork = new(6, 5, rippleHeight, new Xyz(1.0e5, -2.0e5, 3.0e5), 250.0);
        Gordon aGordon = new();
        aGordon.Init(aNetwork.Profiles, aNetwork.Guides, 1.0e-5);
        aGordon.Perform();
        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        Assert.False(aGordon.IsApproximate);
        verifyUniformNetworkInterpolation(aGordon.Surface(), aNetwork, 1.0e-5);
    }

    [Fact]
    public void SmallScaledFiveBySixNetwork_InterpolatesAllInputCurves()
    {
        AnalyticNetwork aNetwork = new(5, 6, saddleHeight, new Xyz(0.0, 0.0, 0.0), 0.01);
        Gordon aGordon = new();
        aGordon.Init(aNetwork.Profiles, aNetwork.Guides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        Assert.False(aGordon.IsApproximate);
        verifyUniformNetworkInterpolation(aGordon.Surface(), aNetwork, global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DenseWavyNetwork_ParallelConstructionMatchesSerialQuality()
    {
        AnalyticNetwork aNetwork = new(6, 6, wavyHeight);
        Gordon aSerial = new();
        aSerial.Init(aNetwork.Profiles, aNetwork.Guides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aSerial.Perform();
        Assert.True(aSerial.IsDone, aSerial.Status().ToString());

        Gordon aParallel = new();
        aParallel.SetParallelMode(true);
        aParallel.Init(aNetwork.Profiles, aNetwork.Guides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aParallel.Perform();
        Assert.True(aParallel.IsDone, aParallel.Status().ToString());

        verifyUniformNetworkInterpolation(aSerial.Surface(), aNetwork, global::Kerf.Engine.Foundation.Precision.Confusion());
        verifyUniformNetworkInterpolation(aParallel.Surface(), aNetwork, global::Kerf.Engine.Foundation.Precision.Confusion());
        const int THE_NB_SAMPLES = 24;
        for (int aUIdx = 0; aUIdx <= THE_NB_SAMPLES; ++aUIdx)
        {
            for (int aVIdx = 0; aVIdx <= THE_NB_SAMPLES; ++aVIdx)
            {
                double aU = (double)aUIdx / THE_NB_SAMPLES;
                double aV = (double)aVIdx / THE_NB_SAMPLES;
                Assert.True(aSerial.Surface().Value(aU, aV).DistanceTo(aParallel.Surface().Value(aU, aV)) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
    }

    [Fact]
    public void DenseSaddleNetwork_RepeatedPerformMaintainsQuality()
    {
        AnalyticNetwork aNetwork = new(6, 5, saddleHeight);
        Gordon aGordon = new();
        aGordon.Init(aNetwork.Profiles, aNetwork.Guides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        BSplineSurface aFirstSurface = aGordon.Surface();

        aGordon.Perform();
        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        verifyUniformNetworkInterpolation(aGordon.Surface(), aNetwork, global::Kerf.Engine.Foundation.Precision.Confusion());
        const int THE_NB_SAMPLES = 24;
        for (int aUIdx = 0; aUIdx <= THE_NB_SAMPLES; ++aUIdx)
        {
            for (int aVIdx = 0; aVIdx <= THE_NB_SAMPLES; ++aVIdx)
            {
                double aU = (double)aUIdx / THE_NB_SAMPLES;
                double aV = (double)aVIdx / THE_NB_SAMPLES;
                Assert.True(aFirstSurface.Value(aU, aV).DistanceTo(aGordon.Surface().Value(aU, aV)) <= global::Kerf.Engine.Foundation.Precision.Confusion());
            }
        }
    }

    [Fact]
    public void DenseRippleNineByThreeNetwork_InterpolatesAllInputCurves() => denseNetwork(9, 3, rippleHeight);

    [Fact]
    public void DenseWavyThreeByNineNetwork_InterpolatesAllInputCurves() => denseNetwork(3, 9, wavyHeight);

    [Fact]
    public void ClosedBSplineProfilesWithInteriorGuides_ProducePeriodicSurface()
    {
        NCollection_Array1<Curve3D> aProfiles = Curves(makeClosedBSplineProfile(0.0, 1.0, 1.0), makeClosedBSplineProfile(1.0, 1.15, 0.85));
        Assert.False(((BSplineCurve3D)aProfiles[1]).IsPeriodic);
        Assert.True(aProfiles[1].Value(aProfiles[1].FirstParameter).DistanceTo(aProfiles[1].Value(aProfiles[1].LastParameter))
                    <= global::Kerf.Engine.Foundation.Precision.Confusion());

        NCollection_Array1<double> aGuideParameters = Reals(0.12, 0.34, 0.59, 0.83);
        NCollection_Array1<Curve3D> aGuides = new(1, 4);
        for (int aGuideIdx = aGuides.Lower(); aGuideIdx <= aGuides.Upper(); ++aGuideIdx)
        {
            double aParameter = aGuideParameters[aGuideIdx];
            aGuides[aGuideIdx] = makeLinearBSpline(aProfiles[1].Value(aParameter), aProfiles[2].Value(aParameter));
        }

        Gordon aGordon = new();
        aGordon.Init(aProfiles, aGuides, global::Kerf.Engine.Foundation.Precision.Confusion());
        aGordon.Perform();
        Assert.True(aGordon.IsDone, aGordon.Status().ToString());
        Assert.False(aGordon.IsApproximate);
        Assert.True(aGordon.Surface().IsUPeriodic);
        Assert.True(aGordon.Report().MaxProfileDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.True(aGordon.Report().MaxGuideDeviation <= global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void NetworkSurface_ClosedNetworkWithNonUnitUParameters_ProducesUPeriodicSurface()
    {
        NCollection_Array1<BSplineCurve3D?> aProfiles =
          BSplines(makeQuadraticBSpline(new Point3D(0, 0, 0), new Point3D(1, 0, 0), new Point3D(0, 0, 0), 2.0, 5.0),
                   makeQuadraticBSpline(new Point3D(0, 1, 0), new Point3D(1, 1, 1), new Point3D(0, 1, 0), 2.0, 5.0));
        NCollection_Array1<double> aGuideParameters = Reals(2.0, 3.5);
        NCollection_Array1<BSplineCurve3D?> aGuides = new(1, 2);
        for (int aGuideIdx = aGuides.Lower(); aGuideIdx <= aGuides.Upper(); ++aGuideIdx)
        {
            double aParameter = aGuideParameters[aGuideIdx];
            aGuides[aGuideIdx] = makeLinearBSpline(aProfiles[1]!.Value(aParameter), aProfiles[2]!.Value(aParameter));
        }

        NetworkSurface aNetwork = new();
        aNetwork.Init(aProfiles, aGuides, Reals(0.0, 1.0), aGuideParameters, makeIntersectionGrid(aProfiles, aGuideParameters),
                      makeUnitWeightGrid(aProfiles, aGuideParameters), global::Kerf.Engine.Foundation.Precision.Confusion(), true, false);
        aNetwork.Perform();
        Assert.True(aNetwork.IsDone, aNetwork.Status().ToString());
        Assert.True(aNetwork.Surface().IsUPeriodic);
    }
}
