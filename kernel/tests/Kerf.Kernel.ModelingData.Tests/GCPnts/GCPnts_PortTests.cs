// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Points;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Tests of the port's CPnts and GCPnts beyond OCCT's GTest: the properties each algorithm promises, on 3D and 2D
/// curves, and the 2D instantiations against the 3D ones on the same planar curves (decision 187).
/// </summary>
public class GCPnts_PortTests
{
    // A planar degree 3 B-spline, C2 at u = 1, C0 at u = 2 (multiplicity 3): composite for every algorithm.
    private static readonly double[] s_X = [0, 1, 2, 3, 4, 5, 6, 7];
    private static readonly double[] s_Y = [0, 2, -1, 1, 3, 0, 2, 0.5];

    private static BSplineCurve3D BSpline3d(bool theComposite = true)
    {
        Point3D[] aPoles = new Point3D[s_X.Length];
        for (int i = 0; i < aPoles.Length; i++)
        {
            aPoles[i] = new Point3D(s_X[i], s_Y[i], 0.0);
        }
        return theComposite
            ? new BSplineCurve3D(NCollection_Array1<Point3D>.From(aPoles), NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
                NCollection_Array1<int>.From([4, 1, 3, 4]), 3)
            : new BSplineCurve3D(NCollection_Array1<Point3D>.From(aPoles[..4]), NCollection_Array1<double>.From([0.0, 1.0]),
                NCollection_Array1<int>.From([4, 4]), 3);
    }

    private static BSplineCurve2D BSpline2d(bool theComposite = true)
    {
        Point2D[] aPoles = new Point2D[s_X.Length];
        for (int i = 0; i < aPoles.Length; i++)
        {
            aPoles[i] = new Point2D(s_X[i], s_Y[i]);
        }
        return theComposite
            ? new BSplineCurve2D(NCollection_Array1<Point2D>.From(aPoles), NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
                NCollection_Array1<int>.From([4, 1, 3, 4]), 3)
            : new BSplineCurve2D(NCollection_Array1<Point2D>.From(aPoles[..4]), NCollection_Array1<double>.From([0.0, 1.0]),
                NCollection_Array1<int>.From([4, 4]), 3);
    }

    private static TestGeomAdaptor_Curve Ellipse3d() =>
        new(new EllipseCurve3D(new Frame3D(new Point3D(1, 2, 3), new Direction3D(1, 1, 1)), 5.0, 2.0), 0.3, 5.0);

    private static TestGeom2dAdaptor_Curve Ellipse2d() =>
        new(new EllipseCurve2D(new Frame2D(new Point2D(1, 2), new Direction2D(1, 0)), 5.0, 2.0), 0.3, 5.0);

    // The length of the curve between two parameters by a fine polygon, split at the integer parameters (the knots of
    // the B-spline, where it may have a corner).
    private static double PolygonLength(CurveAdaptorBase theC, double theU1, double theU2, int theNb = 100000)
    {
        double aKnot = Math.Floor(theU1) + 1.0;
        if (aKnot < theU2)
        {
            return PolygonLength(theC, theU1, aKnot, theNb) + PolygonLength(theC, aKnot, theU2, theNb);
        }
        double aL = 0.0;
        Point3D aPrev = theC.Value(theU1);
        for (int i = 1; i <= theNb; i++)
        {
            Point3D aP = theC.Value(theU1 + (theU2 - theU1) * i / theNb);
            aL += aPrev.DistanceTo(aP);
            aPrev = aP;
        }
        return aL;
    }

    // The largest distance between the curve and the chord between two parameters, sampled.
    private static double Sag(CurveAdaptorBase theC, double theU1, double theU2, int theNb = 50)
    {
        Point3D aP1 = theC.Value(theU1), aP2 = theC.Value(theU2);
        if (aP1.DistanceTo(aP2) < 1e-12)
        {
            return 0.0;
        }
        Line3D aChord = new(aP1, new Direction3D(new Vector3D(aP1, aP2)));
        double aMax = 0.0;
        for (int i = 1; i < theNb; i++)
        {
            aMax = Math.Max(aMax, aChord.DistanceTo(theC.Value(theU1 + (theU2 - theU1) * i / theNb)));
        }
        return aMax;
    }

    [Fact]
    public void Length_BSpline_MatchesPolygon()
    {
        TestGeomAdaptor_Curve aC = new(BSpline3d());
        Assert.True(aC.NbIntervals(CN) > 1);
        // The default length uses a Gauss rule of fixed order (OCCT), close to the polygon; with a tolerance it is adaptive.
        Assert.Equal(PolygonLength(aC, 0.0, 3.0), AbscissaPoint.Length(aC), 1e-7);
        Assert.Equal(PolygonLength(aC, 0.0, 3.0), AbscissaPoint.Length(aC, 1e-12), 1e-9);
        Assert.Equal(PolygonLength(aC, 0.5, 2.5), AbscissaPoint.Length(aC, 0.5, 2.5, 1e-9), 1e-6);
        Assert.Equal(PolygonLength(aC, 0.5, 2.5), CurveAbscissaPoint.Length(aC, 0.5, 1.0)
            + CurveAbscissaPoint.Length(aC, 1.0, 2.0) + CurveAbscissaPoint.Length(aC, 2.0, 2.5), 1e-6);
    }

    [Fact]
    public void Length_Ellipse_MatchesPolygon()
    {
        TestGeomAdaptor_Curve aC = Ellipse3d();
        Assert.Equal(PolygonLength(aC, 0.3, 5.0), AbscissaPoint.Length(aC, 1e-10), 1e-6);
    }

    [Theory]
    [InlineData(0.2, 3.0)]
    [InlineData(2.9, -4.0)]
    [InlineData(0.0, 7.5)]
    public void AbscissaPoint_BSpline_IsAtTheDistance(double theU0, double theAbscissa)
    {
        TestGeomAdaptor_Curve aC = new(BSpline3d());
        AbscissaPoint aDefault = new(aC, theAbscissa, theU0);
        AbscissaPoint aWithTol = new(1e-9, aC, theAbscissa, theU0);
        foreach (AbscissaPoint anAP in new[] { aDefault, aWithTol })
        {
            Assert.True(anAP.IsDone);
            double aU = anAP.Parameter();
            Assert.Equal(Math.Abs(theAbscissa), AbscissaPoint.Length(aC, theU0, aU), 1e-5);
            Assert.Equal(theAbscissa > 0, aU > theU0);
        }
    }

    [Fact]
    public void AbscissaPoint_ZeroLength_Throws()
    {
        TestGeomAdaptor_Curve aC = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 1.0, 1.0);
        EngineAssert.Throws<ConstructionErrorException>(() => new AbscissaPoint(aC, 1.0, 1.0));
        Assert.False(new AbscissaPoint().IsDone);
        EngineAssert.Throws<NotDoneException>(() => new AbscissaPoint().Parameter());
    }

    [Fact]
    public void UniformAbscissa_ByDistance_PointsAreEquidistant()
    {
        foreach (CurveAdaptorBase aC in new CurveAdaptorBase[] { new TestGeomAdaptor_Curve(BSpline3d()), Ellipse3d() })
        {
            double aTotal = AbscissaPoint.Length(aC);
            UniformAbscissa anUA = new(aC, 1.0, 1e-9);
            Assert.True(anUA.IsDone);
            Assert.Equal(1.0, anUA.Abscissa());
            Assert.Equal((int)Math.Ceiling(aTotal) + 1, anUA.PointCount);
            Assert.Equal(aC.FirstParameter(), anUA.Parameter(1));
            Assert.Equal(aC.LastParameter(), anUA.Parameter(anUA.PointCount));
            for (int i = 1; i < anUA.PointCount - 1; i++)
            {
                Assert.Equal(1.0, AbscissaPoint.Length(aC, anUA.Parameter(i), anUA.Parameter(i + 1)), 1e-6);
            }
        }
    }

    [Fact]
    public void UniformAbscissa_ByNumber_PointsAreEquidistant()
    {
        TestGeomAdaptor_Curve aC = new(BSpline3d());
        UniformAbscissa anUA = new(aC, 11, 0.25, 2.75);
        Assert.True(anUA.IsDone);
        Assert.Equal(11, anUA.PointCount);
        double aStep = AbscissaPoint.Length(aC, 0.25, 2.75) / 10;
        Assert.Equal(aStep, anUA.Abscissa(), 1e-9);
        for (int i = 1; i < 11; i++)
        {
            Assert.Equal(aStep, AbscissaPoint.Length(aC, anUA.Parameter(i), anUA.Parameter(i + 1)), 1e-5);
        }
        EngineAssert.Throws<ConstructionErrorException>(() => new UniformAbscissa(aC, 1));
        EngineAssert.Throws<NotDoneException>(() => new UniformAbscissa().PointCount);
    }

    [Fact]
    public void UniformAbscissa_Circle_IsLengthParametrized()
    {
        TestGeomAdaptor_Curve aC = new(new CircleCurve3D(new Frame3D(), 2.0), 0.0, 2 * M_PI);
        UniformAbscissa anUA = new(aC, 8);
        Assert.Equal(8, anUA.PointCount);
        for (int i = 1; i <= 8; i++)
        {
            Assert.Equal((i - 1) * 2 * M_PI / 7, anUA.Parameter(i), 1e-12);
        }
    }

    [Fact]
    public void QuasiUniformAbscissa_BSpline()
    {
        TestGeomAdaptor_Curve aC = new(BSpline3d());
        QuasiUniformAbscissa aQUA = new(aC, 20);
        Assert.True(aQUA.IsDone);
        Assert.Equal(20, aQUA.PointCount);
        Assert.Equal(0.0, aQUA.Parameter(1));
        Assert.Equal(3.0, aQUA.Parameter(20));
        double aStep = AbscissaPoint.Length(aC) / 19;
        for (int i = 1; i < 20; i++)
        {
            Assert.True(aQUA.Parameter(i) < aQUA.Parameter(i + 1));
            Assert.Equal(aStep, AbscissaPoint.Length(aC, aQUA.Parameter(i), aQUA.Parameter(i + 1)), aStep * 0.05);
        }
        // Other curves go through GCPnts_UniformAbscissa.
        QuasiUniformAbscissa anEllipse = new(Ellipse3d(), 7);
        UniformAbscissa aUA = new(Ellipse3d(), 7);
        Assert.Equal(aUA.PointCount, anEllipse.PointCount);
        for (int i = 1; i <= 7; i++)
        {
            Assert.Equal(aUA.Parameter(i), anEllipse.Parameter(i));
        }
    }

    [Fact]
    public void UniformDeflection_RespectsTheDeflection()
    {
        foreach (CurveAdaptorBase aC in new CurveAdaptorBase[] { new TestGeomAdaptor_Curve(BSpline3d()), Ellipse3d() })
        {
            foreach (bool aWithControl in new[] { true, false })
            {
                UniformDeflection anUD = new(aC, 0.01, aWithControl);
                Assert.True(anUD.IsDone);
                Assert.Equal(0.01, anUD.Deflection());
                Assert.Equal(aC.FirstParameter(), anUD.Parameter(1));
                Assert.Equal(aC.LastParameter(), anUD.Parameter(anUD.PointCount));
                for (int i = 1; i < anUD.PointCount; i++)
                {
                    Assert.True(anUD.Parameter(i) < anUD.Parameter(i + 1));
                    Assert.True(anUD.Value(i).IsEqual(aC.Value(anUD.Parameter(i)), 0.0));
                    // Without control the deflection is only estimated from the second derivative.
                    Assert.True(Sag(aC, anUD.Parameter(i), anUD.Parameter(i + 1)) < (aWithControl ? 0.015 : 0.03),
                        $"{aC.GetType()} {aWithControl} {anUD.Parameter(i)} {Sag(aC, anUD.Parameter(i), anUD.Parameter(i + 1))}");
                }
            }
        }
    }

    [Fact]
    public void QuasiUniformDeflection_RespectsTheDeflection()
    {
        foreach (CurveAdaptorBase aC in new CurveAdaptorBase[] { new TestGeomAdaptor_Curve(BSpline3d()), Ellipse3d() })
        {
            foreach (Continuity aCont in new[] { C0, C1, C2 })
            {
                QuasiUniformDeflection aQUD = new(aC, 0.01, aCont);
                Assert.True(aQUD.IsDone);
                Assert.Equal(aC.FirstParameter(), aQUD.Parameter(1));
                Assert.Equal(aC.LastParameter(), aQUD.Parameter(aQUD.PointCount));
                for (int i = 1; i < aQUD.PointCount; i++)
                {
                    Assert.True(aQUD.Parameter(i) < aQUD.Parameter(i + 1));
                    Assert.True(Sag(aC, aQUD.Parameter(i), aQUD.Parameter(i + 1)) < 0.015,
                        $"{aC.GetType()} {aCont} {aQUD.Parameter(i)} {Sag(aC, aQUD.Parameter(i), aQUD.Parameter(i + 1))}");
                }
            }
        }
    }

    // OCCT's estimate fails on an arc whose chord's midpoint is on the curve and whose end tangents are parallel
    // (an S): the span [2, 3] of this B-spline is kept whole, 0.29 away from its chord, at every continuity.
    [Fact]
    public void QuasiUniformDeflection_SymmetricS_IsNotSplit()
    {
        NCollection_Array1<Point3D> aPoles = NCollection_Array1<Point3D>.From(
            [new Point3D(4, 3, 0), new Point3D(5, 0, 0), new Point3D(6, 2, 0), new Point3D(7, -1, 0)]);
        TestGeomAdaptor_Curve aC = new(new BSplineCurve3D(aPoles, NCollection_Array1<double>.From([2.0, 3.0]),
            NCollection_Array1<int>.From([4, 4]), 3));
        foreach (Continuity aCont in new[] { C0, C1 })
        {
            QuasiUniformDeflection aQUD = new(aC, 0.01, aCont);
            Assert.Equal(2, aQUD.PointCount);
            Assert.Equal(0.288288, Sag(aC, 2.0, 3.0), 1e-6);
        }
    }

    [Fact]
    public void Deflections_Line_And_Circle()
    {
        TestGeomAdaptor_Curve aLine = new(new LineCurve3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), -1.0, 4.0);
        Assert.Equal(2, new UniformDeflection(aLine, 0.1).PointCount);
        Assert.Equal(2, new QuasiUniformDeflection(aLine, 0.1).PointCount);
        Assert.Equal(5, new TangentialDeflection(aLine, 0.1, 0.1, 5).PointCount);

        // Circle: N = (int)(range / (2 acos(1 - d / R))) + 2 points.
        TestGeomAdaptor_Curve aCircle = new(new CircleCurve3D(new Frame3D(), 10.0), 0.0, M_PI);
        int aNb = (int)(M_PI / (2.0 * Math.Acos(1.0 - 0.1 / 10.0))) + 2;
        Assert.Equal(aNb, new UniformDeflection(aCircle, 0.1).PointCount);
        Assert.Equal(aNb, new QuasiUniformDeflection(aCircle, 0.1).PointCount);
    }

    [Fact]
    public void TangentialDeflection_RespectsBothDeflections()
    {
        foreach (CurveAdaptorBase aC in new CurveAdaptorBase[] { new TestGeomAdaptor_Curve(BSpline3d()), Ellipse3d() })
        {
            TangentialDeflection aTD = new(aC, 0.2, 0.01, 4);
            Assert.True(aTD.PointCount >= 4);
            Assert.Equal(aC.FirstParameter(), aTD.Parameter(1));
            Assert.Equal(aC.LastParameter(), aTD.Parameter(aTD.PointCount));
            for (int i = 1; i < aTD.PointCount; i++)
            {
                Assert.True(aTD.Parameter(i) < aTD.Parameter(i + 1));
                Assert.True(aTD.Value(i).IsEqual(aC.Value(aTD.Parameter(i)), 0.0));
                Assert.True(Sag(aC, aTD.Parameter(i), aTD.Parameter(i + 1)) < 0.015);
            }
        }
    }

    [Fact]
    public void TangentialDeflection_Circle_AddPoint_ArcAngularStep()
    {
        TestGeomAdaptor_Curve aC = new(new CircleCurve3D(new Frame3D(), 2.0), 0.0, 2 * M_PI);
        TangentialDeflection aTD = new(aC, 0.5, 0.01);
        double aStep = TangentialDeflection.ArcAngularStep(2.0, 0.01, 0.5, 1e-7);
        Assert.Equal(2.0 * Math.Acos(1.0 - 0.01 / 2.0), aStep, 1e-15);
        Assert.Equal((int)Math.Ceiling(2 * M_PI / aStep) + 1, aTD.PointCount);

        int aNb = aTD.PointCount;
        Point3D aP = aC.Value(0.05);
        Assert.Equal(2, aTD.AddPoint(aP, 0.05));
        Assert.Equal(aNb + 1, aTD.PointCount);
        Assert.Equal(2, aTD.AddPoint(new Point3D(9, 9, 9), 0.05, false));
        Assert.True(aTD.Value(2).IsEqual(aP, 0.0));
        Assert.Equal(2, aTD.AddPoint(new Point3D(9, 9, 9), 0.05));
        Assert.True(aTD.Value(2).IsEqual(new Point3D(9, 9, 9), 0.0));
        Assert.Equal(aNb + 2, aTD.AddPoint(aP, 10.0));

        EngineAssert.Throws<ConstructionErrorException>(() => new TangentialDeflection(aC, 0.5, 0.0));
        EngineAssert.Throws<ConstructionErrorException>(() => TangentialDeflection.ArcAngularStep(-1.0, 0.1, 0.1, 0.0));
    }

    [Fact]
    public void CPnts_UniformDeflection_Iterates()
    {
        TestGeomAdaptor_Curve aC = new(BSpline3d(false));
        CurveUniformDeflection anIt = new(aC, 0.01, 1e-9, true);
        Assert.True(anIt.IsAllDone);
        double aPrev = double.NegativeInfinity;
        int aNb = 0;
        for (; anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Value() > aPrev);
            Assert.True(anIt.Point().IsEqual(aC.Value(anIt.Value()), 0.0));
            aPrev = anIt.Value();
            aNb++;
        }
        Assert.Equal(1.0, aPrev);
        Assert.True(aNb > 3);
        EngineAssert.Throws<OutOfRangeException>(() => anIt.Next());
    }

    // The 2D instantiations are generated from the 3D ones: on a planar curve built from the same numbers they must
    // give the same parameters, and the same points with Z = 0.
    [Fact]
    public void TwoD_MatchesThreeD()
    {
        foreach (bool aComposite in new[] { true, false })
        {
            TestGeomAdaptor_Curve aC3 = new(BSpline3d(aComposite));
            TestGeom2dAdaptor_Curve aC2 = new(BSpline2d(aComposite));

            Assert.Equal(AbscissaPoint.Length(aC3), AbscissaPoint.Length(aC2));
            Assert.Equal(AbscissaPoint.Length(aC3, 0.1, 0.9, 1e-8), AbscissaPoint.Length(aC2, 0.1, 0.9, 1e-8));
            Assert.Equal(new AbscissaPoint(aC3, 2.5, 0.1).Parameter(), new AbscissaPoint(aC2, 2.5, 0.1).Parameter());
            Assert.Equal(new AbscissaPoint(1e-8, aC3, -0.5, 0.9).Parameter(),
                new AbscissaPoint(1e-8, aC2, -0.5, 0.9).Parameter());

            AssertSame(new UniformAbscissa(aC3, 0.7), new UniformAbscissa(aC2, 0.7));
            AssertSame(new UniformAbscissa(aC3, 9, 1e-6), new UniformAbscissa(aC2, 9, 1e-6));

            QuasiUniformAbscissa aQ3 = new(aC3, 13), aQ2 = new(aC2, 13);
            for (int i = 1; i <= 13; i++)
            {
                Assert.Equal(aQ3.Parameter(i), aQ2.Parameter(i));
            }

            UniformDeflection aU3 = new(aC3, 0.005), aU2 = new(aC2, 0.005);
            AssertSame(aU3.PointCount, aU3.Parameter, aU3.Value, aU2.PointCount, aU2.Parameter, aU2.Value);
            QuasiUniformDeflection aD3 = new(aC3, 0.005, C2), aD2 = new(aC2, 0.005, C2);
            AssertSame(aD3.PointCount, aD3.Parameter, aD3.Value, aD2.PointCount, aD2.Parameter, aD2.Value);
            TangentialDeflection aT3 = new(aC3, 0.1, 0.005), aT2 = new(aC2, 0.1, 0.005);
            AssertSame(aT3.PointCount, aT3.Parameter, aT3.Value, aT2.PointCount, aT2.Parameter, aT2.Value);
        }
    }

    private static void AssertSame(UniformAbscissa the3d, UniformAbscissa the2d)
    {
        Assert.Equal(the3d.PointCount, the2d.PointCount);
        for (int i = 1; i <= the3d.PointCount; i++)
        {
            Assert.Equal(the3d.Parameter(i), the2d.Parameter(i));
        }
    }

    private static void AssertSame(int theNb3, Func<int, double> theParam3, Func<int, Point3D> thePnt3,
        int theNb2, Func<int, double> theParam2, Func<int, Point3D> thePnt2)
    {
        Assert.Equal(theNb3, theNb2);
        for (int i = 1; i <= theNb3; i++)
        {
            Assert.Equal(theParam3(i), theParam2(i));
            Assert.Equal(thePnt3(i), thePnt2(i));
        }
    }
}
