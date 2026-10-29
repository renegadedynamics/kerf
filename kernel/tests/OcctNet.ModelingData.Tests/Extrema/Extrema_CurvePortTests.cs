// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Tests of the port's curve extrema beyond OCCT's GTest (unit 3B): point/curve (ExtPC, LocateExtPC) and curve/curve
/// (ExtCC, ECC, LocateExtCC) in 3D and 2D, against known answers and dense sampling, and the 2D instantiations against
/// the 3D ones on the same planar curves (decision 2201).
/// </summary>
public class Extrema_CurvePortTests
{
    // A planar degree 3 B-spline, C2 at u = 1, C0 at u = 2 (multiplicity 3).
    private static readonly double[] s_X = [0, 1, 2, 3, 4, 5, 6, 7];
    private static readonly double[] s_Y = [0, 2, -1, 1, 3, 0, 2, 0.5];

    private static Geom_BSplineCurve BSpline3d(int theDegree = 3, double theDy = 0.0)
    {
        if (theDegree == 1)
        {
            var aPoles1 = new gp_Pnt[s_X.Length];
            for (int i = 0; i < aPoles1.Length; i++)
            {
                aPoles1[i] = new gp_Pnt(s_X[i], s_Y[i] + theDy, 0.0);
            }
            return new Geom_BSplineCurve(NCollection_Array1<gp_Pnt>.From(aPoles1),
                NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]),
                NCollection_Array1<int>.From([2, 1, 1, 1, 1, 1, 1, 2]), 1);
        }
        var aPoles = new gp_Pnt[s_X.Length];
        for (int i = 0; i < aPoles.Length; i++)
        {
            aPoles[i] = new gp_Pnt(s_X[i], s_Y[i] + theDy, 0.0);
        }
        return new Geom_BSplineCurve(NCollection_Array1<gp_Pnt>.From(aPoles), NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
            NCollection_Array1<int>.From([4, 1, 3, 4]), 3);
    }

    private static Geom2d_BSplineCurve BSpline2d(int theDegree = 3, double theDy = 0.0)
    {
        if (theDegree == 1)
        {
            var aPoles1 = new gp_Pnt2d[s_X.Length];
            for (int i = 0; i < aPoles1.Length; i++)
            {
                aPoles1[i] = new gp_Pnt2d(s_X[i], s_Y[i] + theDy);
            }
            return new Geom2d_BSplineCurve(NCollection_Array1<gp_Pnt2d>.From(aPoles1),
                NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0]),
                NCollection_Array1<int>.From([2, 1, 1, 1, 1, 1, 1, 2]), 1);
        }
        var aPoles = new gp_Pnt2d[s_X.Length];
        for (int i = 0; i < aPoles.Length; i++)
        {
            aPoles[i] = new gp_Pnt2d(s_X[i], s_Y[i] + theDy);
        }
        return new Geom2d_BSplineCurve(NCollection_Array1<gp_Pnt2d>.From(aPoles), NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
            NCollection_Array1<int>.From([4, 1, 3, 4]), 3);
    }

    private static Geom_BezierCurve Bezier3d() => new(NCollection_Array1<gp_Pnt>.From(
        [new gp_Pnt(0, 0, 0), new gp_Pnt(1, 3, 0), new gp_Pnt(3, -2, 0), new gp_Pnt(4, 1, 0), new gp_Pnt(6, 0, 0)]));

    private static Geom2d_BezierCurve Bezier2d() => new(NCollection_Array1<gp_Pnt2d>.From(
        [new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 3), new gp_Pnt2d(3, -2), new gp_Pnt2d(4, 1), new gp_Pnt2d(6, 0)]));

    // The least square distance from theP to the curve on a dense sampling, with its parameter.
    private static (double SqDist, double U) Sampled(Adaptor3d_Curve theC, in gp_Pnt theP, int theN = 20000)
    {
        double aBest = double.MaxValue, aBestU = 0.0;
        double aF = theC.FirstParameter(), aL = theC.LastParameter();
        for (int i = 0; i <= theN; i++)
        {
            double u = aF + (aL - aF) * i / theN;
            double d = theC.Value(u).SquareDistance(theP);
            if (d < aBest)
            {
                aBest = d;
                aBestU = u;
            }
        }
        return (aBest, aBestU);
    }

    private static double MinSqDist(Extrema_ExtPC theExt)
    {
        double aMin = double.MaxValue;
        for (int i = 1; i <= theExt.NbExt(); i++)
        {
            aMin = Math.Min(aMin, theExt.SquareDistance(i));
        }
        return aMin;
    }

    // ------------------------------------------------------------------------------------------
    // Point / curve
    // ------------------------------------------------------------------------------------------

    [Fact]
    public void ExtPC_Circle_MinAndMax()
    {
        TestGeomAdaptor_Curve aC = new(new Geom_Circle(gp.XOY(), 2.0));
        Extrema_ExtPC anExt = new(new gp_Pnt(5, 0, 1), aC);
        Assert.True(anExt.IsDone());
        Assert.Equal(2, anExt.NbExt());
        for (int i = 1; i <= 2; i++)
        {
            double u = anExt.Point(i).Parameter();
            if (anExt.IsMin(i))
            {
                Assert.Equal(0.0, u, 1e-12);
                Assert.Equal(10.0, anExt.SquareDistance(i), 1e-12);
            }
            else
            {
                Assert.Equal(Math.PI, u, 1e-12);
                Assert.Equal(50.0, anExt.SquareDistance(i), 1e-12);
            }
        }

        // On an arc, only the extremum inside the range is kept
        Extrema_ExtPC anArc = new(new gp_Pnt(5, 0, 1), aC, 2.0, 4.0);
        Assert.True(anArc.IsDone());
        Assert.Equal(1, anArc.NbExt());
        Assert.False(anArc.IsMin(1));
        anArc.TrimmedSquareDistances(out double aD1, out double aD2, out gp_Pnt aP1, out gp_Pnt aP2);
        Assert.Equal(aC.Value(2.0).SquareDistance(new gp_Pnt(5, 0, 1)), aD1);
        Assert.Equal(aC.Value(4.0), aP2);
        Assert.True(aD2 > 0.0 && aP1.Z() == 0.0);
    }

    [Fact]
    public void ExtPC_Line_Segment()
    {
        TestGeomAdaptor_Curve aC = new(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), -1.0, 10.0);
        Extrema_ExtPC anExt = new(new gp_Pnt(3, 4, 0), aC);
        Assert.True(anExt.IsDone());
        Assert.Equal(1, anExt.NbExt());
        Assert.True(anExt.IsMin(1));
        Assert.Equal(3.0, anExt.Point(1).Parameter(), 1e-14);
        Assert.Equal(16.0, anExt.SquareDistance(1), 1e-12);

        // The projection falls outside the segment: Extrema_ExtPElC is not done, nor is Extrema_ExtPC
        Extrema_ExtPC anOut = new(new gp_Pnt(30, 4, 0), aC);
        Assert.False(anOut.IsDone());
    }

    [Theory]
    [InlineData(3)]
    [InlineData(1)]
    public void ExtPC_BSpline_FindsTheMinimum(int theDegree)
    {
        TestGeomAdaptor_Curve aC = new(BSpline3d(theDegree));
        foreach (gp_Pnt aP in new[] { new gp_Pnt(1.5, 3, 0.5), new gp_Pnt(3.2, -2, 0), new gp_Pnt(6, 4, -1), new gp_Pnt(2.5, 0.3, 0) })
        {
            Extrema_ExtPC anExt = new(aP, aC);
            Assert.True(anExt.IsDone());
            Assert.True(anExt.NbExt() > 0);
            (double aSampled, _) = Sampled(aC, aP);
            Assert.True(MinSqDist(anExt) <= aSampled + 1e-9, $"{MinSqDist(anExt)} > {aSampled}");
            for (int i = 1; i <= anExt.NbExt(); i++)
            {
                Extrema_POnCurv aPC = anExt.Point(i);
                Assert.Equal(0.0, aC.Value(aPC.Parameter()).Distance(aPC.Value()), 1e-12);
                Assert.Equal(aPC.Value().SquareDistance(aP), anExt.SquareDistance(i), 1e-9);
                // (an extremum at the C0 knot u = 2 is a corner, not a root of the function)
                if (theDegree == 3 && aPC.Parameter() > 0.0 && aPC.Parameter() < 3.0 && Math.Abs(aPC.Parameter() - 2.0) > 1e-3)
                {
                    // An extremum inside a smooth span is orthogonal
                    aC.D1(aPC.Parameter(), out gp_Pnt aPt, out gp_Vec aD);
                    Assert.Equal(0.0, new gp_Vec(aP, aPt).Dot(aD) / aD.Magnitude(), 1e-6);
                }
            }
        }
    }

    [Fact]
    public void ExtPC_Bezier_And_Offset()
    {
        TestGeomAdaptor_Curve aBez = new(Bezier3d());
        gp_Pnt aP = new(2.0, 2.0, 0.5);
        Extrema_ExtPC anExt = new(aP, aBez);
        Assert.True(anExt.IsDone());
        Assert.True(MinSqDist(anExt) <= Sampled(aBez, aP).SqDist + 1e-9);

        // An offset curve takes the general path (C2 intervals or deflection intervals)
        Geom_OffsetCurve anOffset = new(Bezier3d(), 0.25, new gp_Dir(0, 0, 1));
        TestGeomAdaptor_Curve aOff = new(anOffset);
        Assert.Equal(GeomAbs_CurveType.GeomAbs_OffsetCurve, aOff.GetType());
        Extrema_ExtPC anExtOff = new(new gp_Pnt(2.0, 3, 0), aOff);
        Assert.True(anExtOff.IsDone());
        Assert.True(MinSqDist(anExtOff) <= Sampled(aOff, new gp_Pnt(2.0, 3, 0)).SqDist + 1e-9);
    }

    [Fact]
    public void ExtPC_EndPointOnCurve_IsAdded()
    {
        TestGeomAdaptor_Curve aC = new(BSpline3d());
        gp_Pnt aStart = aC.Value(0.0);
        Extrema_ExtPC anExt = new(aStart, aC);
        Assert.True(anExt.IsDone());
        bool aFound = false;
        for (int i = 1; i <= anExt.NbExt(); i++)
        {
            aFound |= anExt.SquareDistance(i) < Precision.SquareConfusion() && Math.Abs(anExt.Point(i).Parameter()) < 1e-9;
        }
        Assert.True(aFound);
    }

    [Fact]
    public void ExtPC_NotDone_Throws()
    {
        Extrema_ExtPC anExt = new();
        Assert.False(anExt.IsDone());
        Assert.Throws<StdFail_NotDone>(() => anExt.NbExt());
        TestGeomAdaptor_Curve aC = new(new Geom_Circle(gp.XOY(), 2.0));
        anExt.Initialize(aC, 0.0, 2.0 * Math.PI);
        anExt.Perform(new gp_Pnt(3, 0, 0));
        Assert.Throws<Standard_OutOfRange>(() => anExt.Point(0));
        Assert.Throws<Standard_OutOfRange>(() => anExt.SquareDistance(anExt.NbExt() + 1));
    }

    [Fact]
    public void ExtPC2d_Elementary_And_BSpline()
    {
        Geom2dAdaptor_Curve aCirc = new(new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 0)), 2.0));
        Extrema_ExtPC2d anExt = new(new gp_Pnt2d(1, 5), aCirc);
        Assert.True(anExt.IsDone());
        Assert.Equal(2, anExt.NbExt());
        // Extrema_ExtPElC2d marks both points of a circle as minima (occt-issues.md, unit 1G)
        Assert.Equal(4.0, Math.Min(anExt.SquareDistance(1), anExt.SquareDistance(2)), 1e-12);
        Assert.Equal(36.0, Math.Max(anExt.SquareDistance(1), anExt.SquareDistance(2)), 1e-12);
        Assert.True(anExt.IsMin(1) && anExt.IsMin(2));

        Geom2dAdaptor_Curve aBS = new(BSpline2d());
        Extrema_ExtPC2d anExtBS = new(new gp_Pnt2d(1.5, 3), aBS);
        Assert.True(anExtBS.IsDone());
        Assert.True(anExtBS.NbExt() > 0);
    }

    [Fact]
    public void LocateExtPC_FindsTheNearbyExtremum()
    {
        // B-spline: the local search from a parameter near the projection
        TestGeomAdaptor_Curve aC = new(BSpline3d());
        gp_Pnt aP = new(1.5, 3, 0.5);
        Extrema_ExtPC aGlobal = new(aP, aC);
        int aBest = 1;
        for (int i = 2; i <= aGlobal.NbExt(); i++)
        {
            if (aGlobal.SquareDistance(i) < aGlobal.SquareDistance(aBest))
            {
                aBest = i;
            }
        }
        double aU = aGlobal.Point(aBest).Parameter();
        Extrema_LocateExtPC aLoc = new(aP, aC, aU + 0.02, 1e-10);
        Assert.True(aLoc.IsDone());
        Assert.Equal(aU, aLoc.Point().Parameter(), 1e-7);
        Assert.Equal(aGlobal.SquareDistance(aBest), aLoc.SquareDistance(), 1e-10);
        Assert.True(aLoc.IsMin());

        // Circle: the extremum of Extrema_ExtPC whose parameter is nearest U0
        TestGeomAdaptor_Curve aCirc = new(new Geom_Circle(gp.XOY(), 2.0));
        Extrema_LocateExtPC aLocC = new(new gp_Pnt(5, 0, 0), aCirc, 3.0, 1e-10);
        Assert.True(aLocC.IsDone());
        Assert.Equal(Math.PI, aLocC.Point().Parameter(), 1e-12);
        Assert.False(aLocC.IsMin());

        // Bezier: the local search alone
        TestGeomAdaptor_Curve aBez = new(Bezier3d());
        Extrema_LocateExtPC aLocB = new(new gp_Pnt(2, 2, 0), aBez, 0.3, 1e-10);
        if (aLocB.IsDone())
        {
            gp_Pnt aPt = aLocB.Point().Value();
            aBez.D1(aLocB.Point().Parameter(), out _, out gp_Vec aD);
            Assert.Equal(0.0, new gp_Vec(new gp_Pnt(2, 2, 0), aPt).Dot(aD) / aD.Magnitude(), 1e-6);
        }

        // 2D
        Geom2dAdaptor_Curve aC2 = new(BSpline2d());
        Extrema_LocateExtPC2d aLoc2 = new(new gp_Pnt2d(1.5, 3), aC2, aU + 0.02, 1e-10);
        Assert.True(aLoc2.IsDone());
        Assert.Equal(aU, aLoc2.Point().Parameter(), 1e-7);
    }

    // ------------------------------------------------------------------------------------------
    // Curve / curve
    // ------------------------------------------------------------------------------------------

    [Fact]
    public void ExtCC_SkewLines()
    {
        TestGeomAdaptor_Curve aL1 = new(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), -10, 10);
        TestGeomAdaptor_Curve aL2 = new(new Geom_Line(new gp_Pnt(2, 0, 3), new gp_Dir(0, 1, 0)), -10, 10);
        Extrema_ExtCC anExt = new(aL1, aL2);
        Assert.True(anExt.IsDone());
        Assert.False(anExt.IsParallel());
        Assert.Equal(1, anExt.NbExt());
        Assert.Equal(9.0, anExt.SquareDistance(1), 1e-12);
        anExt.Points(1, out Extrema_POnCurv aP1, out Extrema_POnCurv aP2);
        Assert.Equal(2.0, aP1.Parameter(), 1e-12);
        Assert.Equal(0.0, aP2.Parameter(), 1e-12);

        // Parallel lines overlapping
        TestGeomAdaptor_Curve aL3 = new(new Geom_Line(new gp_Pnt(0, 1, 0), new gp_Dir(1, 0, 0)), -5, 5);
        Extrema_ExtCC aPar = new(aL1, aL3);
        Assert.True(aPar.IsDone());
        Assert.True(aPar.IsParallel());
        Assert.Equal(1.0, aPar.SquareDistance(1), 1e-12);

        // Parallel lines not overlapping: the nearest ends
        TestGeomAdaptor_Curve aL4 = new(new Geom_Line(new gp_Pnt(0, 1, 0), new gp_Dir(1, 0, 0)), 20, 30);
        Extrema_ExtCC aEnds = new(aL1, aL4);
        Assert.True(aEnds.IsDone());
        Assert.False(aEnds.IsParallel());
        Assert.Equal(1, aEnds.NbExt());
        Assert.Equal(101.0, aEnds.SquareDistance(1), 1e-12);
    }

    [Fact]
    public void ExtCC_ConcentricCircles_AreParallel()
    {
        TestGeomAdaptor_Curve aC1 = new(new Geom_Circle(gp.XOY(), 2.0));
        TestGeomAdaptor_Curve aC2 = new(new Geom_Circle(gp.XOY(), 3.0));
        Extrema_ExtCC anExt = new(aC1, aC2);
        Assert.True(anExt.IsDone());
        Assert.True(anExt.IsParallel());
        Assert.Equal(1.0, anExt.SquareDistance(1), 1e-12);
    }

    [Fact]
    public void ExtCC_BSplineAndCircle_ThroughECC()
    {
        TestGeomAdaptor_Curve aBS = new(BSpline3d());
        TestGeomAdaptor_Curve aCirc = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(3, 6, 0), new gp_Dir(0, 0, 1)), 1.0));
        Extrema_ExtCC anExt = new(aBS, aCirc);
        Assert.True(anExt.IsDone());
        Assert.True(anExt.NbExt() > 0);
        double aMin = double.MaxValue;
        for (int i = 1; i <= anExt.NbExt(); i++)
        {
            aMin = Math.Min(aMin, anExt.SquareDistance(i));
            anExt.Points(i, out Extrema_POnCurv aP1, out Extrema_POnCurv aP2);
            Assert.Equal(aP1.Value().SquareDistance(aP2.Value()), anExt.SquareDistance(i), 1e-12);
        }

        // Dense sampling of both curves
        double aSampled = double.MaxValue;
        for (int i = 0; i <= 600; i++)
        {
            gp_Pnt aPt = aBS.Value(3.0 * i / 600);
            for (int j = 0; j <= 600; j++)
            {
                aSampled = Math.Min(aSampled, aPt.SquareDistance(aCirc.Value(2 * Math.PI * j / 600)));
            }
        }
        Assert.True(aMin <= aSampled + 1e-9, $"{aMin} > {aSampled}");
    }

    [Fact]
    public void ExtCC_CrossingBSplines_DistanceZero()
    {
        TestGeomAdaptor_Curve aBS = new(BSpline3d());
        TestGeomAdaptor_Curve aL = new(new Geom_Line(new gp_Pnt(3.5, -5, 0), new gp_Dir(0, 1, 0)), 0, 10);
        Extrema_ExtCC anExt = new(aL, aBS);
        Assert.True(anExt.IsDone());
        double aMin = double.MaxValue;
        for (int i = 1; i <= anExt.NbExt(); i++)
        {
            aMin = Math.Min(aMin, anExt.SquareDistance(i));
        }
        Assert.True(aMin < 1e-12);
    }

    [Fact]
    public void ExtCC2d_LineCircle_And_BSplines()
    {
        Geom2dAdaptor_Curve aL = new(new Geom2d_Line(new gp_Pnt2d(0, 5), new gp_Dir2d(1, 0)), -10, 10);
        Geom2dAdaptor_Curve aC = new(new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0));
        Extrema_ExtCC2d anExt = new(aC, aL);
        Assert.True(anExt.IsDone());
        Assert.False(anExt.IsParallel());
        double aMin = double.MaxValue;
        for (int i = 1; i <= anExt.NbExt(); i++)
        {
            aMin = Math.Min(aMin, anExt.SquareDistance(i));
            anExt.Points(i, out Extrema_POnCurv2d aP1, out Extrema_POnCurv2d aP2);
            Assert.Equal(0.0, aC.Value(aP1.Parameter()).Distance(aP1.Value()), 1e-12);
            Assert.Equal(0.0, aL.Value(aP2.Parameter()).Distance(aP2.Value()), 1e-12);
        }
        Assert.Equal(9.0, aMin, 1e-12);

        Geom2dAdaptor_Curve aB1 = new(BSpline2d()), aB2 = new(BSpline2d(3, 5.0));
        Extrema_ExtCC2d anExtBS = new(aB1, aB2);
        Assert.True(anExtBS.IsDone());
        Assert.True(anExtBS.NbExt() > 0);
    }

    [Fact]
    public void LocateExtCC_FromNearbyParameters()
    {
        TestGeomAdaptor_Curve aL1 = new(new Geom_Line(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0)), -10, 10);
        TestGeomAdaptor_Curve aL2 = new(new Geom_Line(new gp_Pnt(2, 0, 3), new gp_Dir(0, 1, 0)), -10, 10);
        Extrema_LocateExtCC aLoc = new(aL1, aL2, 1.0, 1.0);
        Assert.True(aLoc.IsDone());
        Assert.Equal(9.0, aLoc.SquareDistance(), 1e-10);
        aLoc.Point(out Extrema_POnCurv aP1, out Extrema_POnCurv aP2);
        Assert.Equal(2.0, aP1.Parameter(), 1e-8);
        Assert.Equal(0.0, aP2.Parameter(), 1e-8);

        Geom2dAdaptor_Curve aC1 = new(new Geom2d_Circle(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0));
        Geom2dAdaptor_Curve aC2 = new(new Geom2d_Line(new gp_Pnt2d(0, 5), new gp_Dir2d(1, 0)), -10, 10);
        Extrema_LocateExtCC2d aLoc2 = new(aC1, aC2, 1.4, 0.3);
        Assert.True(aLoc2.IsDone());
        Assert.Equal(9.0, aLoc2.SquareDistance(), 1e-10);
    }

    // ------------------------------------------------------------------------------------------
    // The 2D instantiations against the 3D ones
    // ------------------------------------------------------------------------------------------

    private static void AssertSame(Extrema_ExtPC the3d, Extrema_ExtPC2d the2d)
    {
        Assert.Equal(the3d.IsDone(), the2d.IsDone());
        if (!the3d.IsDone())
        {
            return;
        }
        Assert.Equal(the3d.NbExt(), the2d.NbExt());
        for (int i = 1; i <= the3d.NbExt(); i++)
        {
            Assert.Equal(the3d.Point(i).Parameter(), the2d.Point(i).Parameter());
            Assert.Equal(the3d.SquareDistance(i), the2d.SquareDistance(i));
            Assert.Equal(the3d.IsMin(i), the2d.IsMin(i));
            Assert.Equal(the3d.Point(i).Value().X(), the2d.Point(i).Value().X());
            Assert.Equal(the3d.Point(i).Value().Y(), the2d.Point(i).Value().Y());
        }
    }

    [Fact]
    public void TwoD_MatchesThreeD()
    {
        (Adaptor3d_Curve C3, Adaptor2d_Curve2d C2)[] aCurves =
        [
            (new TestGeomAdaptor_Curve(BSpline3d()), new TestGeom2dAdaptor_Curve(BSpline2d())),
            (new TestGeomAdaptor_Curve(BSpline3d(1)), new TestGeom2dAdaptor_Curve(BSpline2d(1))),
            (new TestGeomAdaptor_Curve(Bezier3d()), new TestGeom2dAdaptor_Curve(Bezier2d())),
            (new TestGeomAdaptor_Curve(BSpline3d(), 0.3, 2.6), new TestGeom2dAdaptor_Curve(BSpline2d(), 0.3, 2.6)),
        ];
        double[][] aPoints = [[1.5, 3], [3.2, -2], [6, 4], [2.5, 0.3], [0, 0], [-1, 1]];
        foreach ((Adaptor3d_Curve aC3, Adaptor2d_Curve2d aC2) in aCurves)
        {
            foreach (double[] aXY in aPoints)
            {
                gp_Pnt aP3 = new(aXY[0], aXY[1], 0.0);
                gp_Pnt2d aP2 = new(aXY[0], aXY[1]);
                AssertSame(new Extrema_ExtPC(aP3, aC3), new Extrema_ExtPC2d(aP2, aC2));
                AssertSame(new Extrema_ExtPC(aP3, aC3, 0.5, 1.7, 1e-8), new Extrema_ExtPC2d(aP2, aC2, 0.5, 1.7, 1e-8));

                foreach (double aU0 in new[] { 0.2, 0.9, 1.5, 2.4 })
                {
                    Extrema_LocateExtPC aL3 = new(aP3, aC3, aU0, 1e-10);
                    Extrema_LocateExtPC2d aL2 = new(aP2, aC2, aU0, 1e-10);
                    Assert.Equal(aL3.IsDone(), aL2.IsDone());
                    if (aL3.IsDone())
                    {
                        Assert.Equal(aL3.Point().Parameter(), aL2.Point().Parameter());
                        Assert.Equal(aL3.SquareDistance(), aL2.SquareDistance());
                        Assert.Equal(aL3.IsMin(), aL2.IsMin());
                    }
                }
            }
        }

        // Curve / curve between two B-splines (Extrema_ECC and Extrema_ECC2d) and the local search
        TestGeomAdaptor_Curve aA3 = new(BSpline3d()), aB3 = new(BSpline3d(3, 2.5));
        TestGeom2dAdaptor_Curve aA2 = new(BSpline2d()), aB2 = new(BSpline2d(3, 2.5));
        Extrema_ExtCC aCC3 = new(aA3, aB3);
        Extrema_ExtCC2d aCC2 = new(aA2, aB2);
        Assert.Equal(aCC3.IsDone(), aCC2.IsDone());
        Assert.True(aCC3.IsDone());
        Assert.Equal(aCC3.NbExt(), aCC2.NbExt());
        Assert.Equal(aCC3.IsParallel(), aCC2.IsParallel());
        for (int i = 1; i <= aCC3.NbExt(); i++)
        {
            Assert.Equal(aCC3.SquareDistance(i), aCC2.SquareDistance(i));
            aCC3.Points(i, out Extrema_POnCurv aP31, out Extrema_POnCurv aP32);
            aCC2.Points(i, out Extrema_POnCurv2d aP21, out Extrema_POnCurv2d aP22);
            Assert.Equal(aP31.Parameter(), aP21.Parameter());
            Assert.Equal(aP32.Parameter(), aP22.Parameter());
        }

        Extrema_LocateExtCC aLCC3 = new(aA3, aB3, 1.2, 1.3);
        Extrema_LocateExtCC2d aLCC2 = new(aA2, aB2, 1.2, 1.3);
        Assert.Equal(aLCC3.IsDone(), aLCC2.IsDone());
        if (aLCC3.IsDone())
        {
            Assert.Equal(aLCC3.SquareDistance(), aLCC2.SquareDistance());
        }
    }
}
