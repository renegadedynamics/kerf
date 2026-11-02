// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

/// <summary>OCCT.NET's own tests of Geom2dConvert (OCCT has GTests only for two of its classes).</summary>
public class Geom2dConvertTests
{
    private const double Tol = 1e-9;

    private static Geom2d_BSplineCurve Cubic()
    {
        // A smooth cubic with three spans
        NCollection_Array1<gp_Pnt2d> aPoles = NCollection_Array1<gp_Pnt2d>.From([
            new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 2), new gp_Pnt2d(3, 3), new gp_Pnt2d(5, 1), new gp_Pnt2d(7, 2), new gp_Pnt2d(8, 0)]);
        return new Geom2d_BSplineCurve(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
                                       NCollection_Array1<int>.From([4, 1, 1, 4]), 3);
    }

    private static void AssertSamePoints(Geom2d_Curve theExpected, double theU1, double theU2, Geom2d_Curve theActual,
                                         double theV1, double theV2, double theTol)
    {
        for (int i = 0; i <= 20; i++)
        {
            gp_Pnt2d aP = theExpected.Value(theU1 + (theU2 - theU1) * i / 20.0);
            gp_Pnt2d aQ = theActual.Value(theV1 + (theV2 - theV1) * i / 20.0);
            Assert.True(aP.Distance(aQ) < theTol, $"point {i}: {aP} vs {aQ}");
        }
    }

    private static double MinDistance(Geom2d_Curve theCurve, gp_Pnt2d theP)
    {
        double aMin = double.MaxValue;
        double aF = theCurve.FirstParameter(), aL = theCurve.LastParameter();
        for (int i = 0; i <= 2000; i++)
        {
            aMin = Math.Min(aMin, theCurve.Value(aF + (aL - aF) * i / 2000.0).Distance(theP));
        }

        return aMin;
    }

    [Fact]
    public void CurveToBSplineCurve_WholeCircle_IsPeriodic()
    {
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(1, 2), new gp_Dir2d(1, 1)), 3.0);
        foreach (Convert_ParameterisationType aParam in new[] { Convert_TgtThetaOver2, Convert_QuasiAngular, Convert_RationalC1, Convert_Polynomial })
        {
            Geom2d_BSplineCurve aBS = Geom2dConvert.CurveToBSplineCurve(aCircle, aParam);
            Assert.True(aBS.IsPeriodic());
            Assert.False(aBS.IsFrozen());
            for (int i = 0; i < 16; i++)
            {
                double aU = aBS.FirstParameter() + (aBS.LastParameter() - aBS.FirstParameter()) * i / 16.0;
                Assert.Equal(3.0, aBS.Value(aU).Distance(new gp_Pnt2d(1, 2)), aParam == Convert_Polynomial ? 1e-4 : Tol);
            }
        }
    }

    [Fact]
    public void CurveToBSplineCurve_LeftHandedEllipse()
    {
        Geom2d_Ellipse anEllipse = new(new gp_Ax22d(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1), false), 4.0, 2.0);
        Geom2d_TrimmedCurve aTrimmed = new(anEllipse, 0.3, 2.5);
        Geom2d_BSplineCurve aBS = Geom2dConvert.CurveToBSplineCurve(aTrimmed);
        Assert.True(aBS.StartPoint().Distance(aTrimmed.StartPoint()) < Tol);
        Assert.True(aBS.EndPoint().Distance(aTrimmed.EndPoint()) < Tol);
        Assert.True(MinDistance(aTrimmed, aBS.Value(0.5 * (aBS.FirstParameter() + aBS.LastParameter()))) < 1e-3);
    }

    [Fact]
    public void CurveToBSplineCurve_LongRationalC1Arc_JoinsTwoHalves()
    {
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        Geom2d_TrimmedCurve anArc = new(aCircle, 0.1, 6.2);
        Geom2d_BSplineCurve aBS = Geom2dConvert.CurveToBSplineCurve(anArc, Convert_RationalC1);
        Assert.False(aBS.IsFrozen());
        Assert.True(aBS.StartPoint().Distance(anArc.StartPoint()) < Tol);
        Assert.True(aBS.EndPoint().Distance(anArc.EndPoint()) < Tol);
        for (int i = 0; i <= 50; i++)
        {
            double aU = aBS.FirstParameter() + (aBS.LastParameter() - aBS.FirstParameter()) * i / 50.0;
            Assert.Equal(2.0, aBS.Value(aU).Distance(new gp_Pnt2d(0, 0)), Tol);
        }
    }

    [Fact]
    public void CurveToBSplineCurve_TrimmedLineHyperbolaParabolaBezier()
    {
        Geom2d_Line aLine = new(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 2));
        Geom2d_BSplineCurve aBS = Geom2dConvert.CurveToBSplineCurve(new Geom2d_TrimmedCurve(aLine, -2.0, 3.0));
        Assert.Equal(1, aBS.Degree());
        Assert.Equal(2, aBS.NbPoles());
        AssertSamePoints(aLine, -2.0, 3.0, aBS, -2.0, 3.0, Tol);

        Geom2d_Hyperbola aHyp = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 3.0, 2.0);
        Geom2d_TrimmedCurve aHypTrimmed = new(aHyp, -1.0, 1.5);
        aBS = Geom2dConvert.CurveToBSplineCurve(aHypTrimmed);
        Assert.True(aBS.StartPoint().Distance(aHypTrimmed.StartPoint()) < Tol);
        Assert.True(aBS.EndPoint().Distance(aHypTrimmed.EndPoint()) < Tol);

        Geom2d_Parabola aParab = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(0, 1)), 1.5);
        Geom2d_TrimmedCurve aParabTrimmed = new(aParab, -2.0, 4.0);
        aBS = Geom2dConvert.CurveToBSplineCurve(aParabTrimmed);
        Assert.Equal(2, aBS.Degree());
        AssertSamePoints(aParabTrimmed, -2.0, 4.0, aBS, aBS.FirstParameter(), aBS.LastParameter(), Tol);

        Geom2d_BezierCurve aBez = new(NCollection_Array1<gp_Pnt2d>.From([new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 3), new gp_Pnt2d(4, 2)]),
                                      NCollection_Array1<double>.From([1.0, 2.0, 1.0]));
        aBS = Geom2dConvert.CurveToBSplineCurve(aBez);
        Assert.True(aBS.IsRational());
        AssertSamePoints(aBez, 0.0, 1.0, aBS, 0.0, 1.0, Tol);
        aBS = Geom2dConvert.CurveToBSplineCurve(new Geom2d_TrimmedCurve(aBez, 0.2, 0.7));
        AssertSamePoints(aBez, 0.2, 0.7, aBS, 0.0, 1.0, Tol);
    }

    [Fact]
    public void CurveToBSplineCurve_FrozenTrimmedBSpline_IsCopied()
    {
        Geom2d_BSplineCurve aCubic = Cubic();
        aCubic.Freeze();
        Geom2d_TrimmedCurve aTrimmed = new(aCubic, 0.5, 2.5);
        Geom2d_BSplineCurve aBS = Geom2dConvert.CurveToBSplineCurve(aTrimmed);
        Assert.False(aBS.IsFrozen());
        Assert.Equal(0.5, aBS.FirstParameter());
        Assert.Equal(2.5, aBS.LastParameter());
        AssertSamePoints(aCubic, 0.5, 2.5, aBS, 0.5, 2.5, Tol);
        Assert.Equal(0.0, aCubic.FirstParameter());
    }

    [Fact]
    public void CurveToBSplineCurve_OffsetCurve_Approximated()
    {
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        Geom2d_OffsetCurve anOffset = new(new Geom2d_TrimmedCurve(aCircle, 0.0, 2.0), 0.5);
        Geom2d_BSplineCurve aBS = Geom2dConvert.CurveToBSplineCurve(anOffset);
        Assert.False(aBS.IsFrozen());
        for (int i = 0; i <= 20; i++)
        {
            double aU = aBS.FirstParameter() + (aBS.LastParameter() - aBS.FirstParameter()) * i / 20.0;
            Assert.Equal(2.5, aBS.Value(aU).Distance(new gp_Pnt2d(0, 0)), 1e-4);
        }
    }

    [Fact]
    public void CurveToBSplineCurve_UnboundedCurves_Throw()
    {
        Assert.ThrowsAny<Standard_DomainError>(() => Geom2dConvert.CurveToBSplineCurve(new Geom2d_Line(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0))));
        Assert.ThrowsAny<Standard_DomainError>(() => Geom2dConvert.CurveToBSplineCurve(
            new Geom2d_Parabola(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 1.0)));
    }

    [Fact]
    public void SplitBSplineCurve_ByKnotsAndParameters()
    {
        Geom2d_BSplineCurve aC = Cubic();
        Geom2d_BSplineCurve aPart = Geom2dConvert.SplitBSplineCurve(aC, 2, 3);
        Assert.Equal(1.0, aPart.FirstParameter());
        Assert.Equal(2.0, aPart.LastParameter());
        AssertSamePoints(aC, 1.0, 2.0, aPart, 1.0, 2.0, Tol);

        Geom2d_BSplineCurve aReversed = Geom2dConvert.SplitBSplineCurve(aC, 3, 2);
        Assert.True(aReversed.StartPoint().Distance(aC.Value(2.0)) < Tol);
        Assert.True(aReversed.EndPoint().Distance(aC.Value(1.0)) < Tol);

        Geom2d_BSplineCurve aByParams = Geom2dConvert.SplitBSplineCurve(aC, 2.5, 0.5, 1e-9);
        Assert.True(aByParams.StartPoint().Distance(aC.Value(2.5)) < Tol);
        Assert.True(aByParams.EndPoint().Distance(aC.Value(0.5)) < Tol);

        Assert.ThrowsAny<Standard_DomainError>(() => Geom2dConvert.SplitBSplineCurve(aC, 2, 2));
        Assert.ThrowsAny<Standard_OutOfRange>(() => Geom2dConvert.SplitBSplineCurve(aC, 0, 2));
    }

    [Fact]
    public void BSplineCurveKnotSplitting()
    {
        // Knot 2 with multiplicity 3 (C0), knot 3 with multiplicity 2 (C1)
        Geom2d_BSplineCurve aC = Cubic().KnotInserted(1.0, 3).KnotInserted(2.0, 2);
        Geom2dConvert_BSplineCurveKnotSplitting aSplit0 = new(aC, 0);
        Assert.Equal(2, aSplit0.NbSplits());
        Geom2dConvert_BSplineCurveKnotSplitting aSplit1 = new(aC, 1);
        Assert.Equal(3, aSplit1.NbSplits());
        Assert.Equal(1, aSplit1.SplitValue(1));
        Assert.Equal(2, aSplit1.SplitValue(2));
        Assert.Equal(4, aSplit1.SplitValue(3));
        Geom2dConvert_BSplineCurveKnotSplitting aSplit2 = new(aC, 2);
        NCollection_Array1<int> aValues = new(1, aSplit2.NbSplits());
        aSplit2.Splitting(aValues);
        Assert.Equal([1, 2, 3, 4], aValues.AsSpan().ToArray());
        Assert.ThrowsAny<Standard_RangeError>(() => new Geom2dConvert_BSplineCurveKnotSplitting(aC, -1));
        Assert.ThrowsAny<Standard_RangeError>(() => aSplit2.SplitValue(5));
    }

    [Fact]
    public void BSplineCurveToBezierCurve()
    {
        Geom2d_BSplineCurve aC = Cubic();
        Geom2dConvert_BSplineCurveToBezierCurve aConv = new(aC);
        Assert.Equal(3, aConv.NbArcs());
        NCollection_Array1<Geom2d_BezierCurve> anArcs = new(1, 3);
        aConv.Arcs(anArcs);
        NCollection_Array1<double> aKnots = new(1, 4);
        aConv.Knots(aKnots);
        Assert.Equal([0.0, 1.0, 2.0, 3.0], aKnots.AsSpan().ToArray());
        for (int i = 1; i <= 3; i++)
        {
            Assert.Equal(3, anArcs[i].Degree());
            AssertSamePoints(aC, i - 1.0, i, anArcs[i], 0.0, 1.0, Tol);
            Assert.NotSame(anArcs[i], aConv.Arc(i));
        }

        Assert.ThrowsAny<Standard_OutOfRange>(() => aConv.Arc(4));

        // A part, with bounds snapped onto knots within half the tolerance
        Geom2dConvert_BSplineCurveToBezierCurve aPart = new(aC, 0.5, 2.0 + 1e-4, 1e-3);
        Assert.Equal(2, aPart.NbArcs());
        AssertSamePoints(aC, 0.5, 1.0, aPart.Arc(1), 0.0, 1.0, Tol);
        Assert.ThrowsAny<Standard_DomainError>(() => new Geom2dConvert_BSplineCurveToBezierCurve(aC, 1.0, 1.0, 1e-3));
    }

    [Fact]
    public void CompCurveToBSplineCurve_AddsBeforeAndAfter_AndKeepsHandedOutCurves()
    {
        Geom2d_BSplineCurve aC = Cubic();
        Geom2d_BSplineCurve aMiddle = Geom2dConvert.SplitBSplineCurve(aC, 2, 3);
        Geom2d_BSplineCurve aFirst = Geom2dConvert.SplitBSplineCurve(aC, 1, 2);
        Geom2d_BSplineCurve aLast = Geom2dConvert.SplitBSplineCurve(aC, 3, 4);
        Geom2dConvert_CompCurveToBSplineCurve aComp = new(aMiddle);
        Geom2d_BSplineCurve? aHandedOut = aComp.BSplineCurve();
        Assert.NotNull(aHandedOut);
        Assert.True(aHandedOut.IsFrozen());
        Assert.Same(aHandedOut, aComp.BSplineCurve());
        Assert.NotSame(aMiddle, aHandedOut);

        Assert.True(aComp.Add(aLast, 1e-7, true));
        Assert.True(aComp.Add(aFirst, 1e-7));
        Assert.Equal(1.0, aHandedOut.FirstParameter());
        Assert.Equal(2.0, aHandedOut.LastParameter());

        Geom2d_BSplineCurve aResult = aComp.BSplineCurve()!;
        Assert.True(aResult.StartPoint().Distance(aC.StartPoint()) < Tol);
        Assert.True(aResult.EndPoint().Distance(aC.EndPoint()) < Tol);
        // The C2 junctions are merged back: the original knots
        Assert.Equal(4, aResult.NbKnots());

        // A curve that touches neither end
        Geom2d_Line aLine = new(new gp_Pnt2d(100, 100), new gp_Dir2d(1, 0));
        Assert.False(aComp.Add(new Geom2d_TrimmedCurve(aLine, 0.0, 1.0), 1e-7));

        aComp.Clear();
        Assert.Null(aComp.BSplineCurve());
        Assert.True(aComp.Add(aLast, 1e-7));
        Assert.Equal(aLast.NbPoles(), aComp.BSplineCurve()!.NbPoles());
    }

    [Fact]
    public void ConcatG1_OpenAndClosed()
    {
        Geom2d_BSplineCurve aC = Cubic();
        NCollection_Array1<Geom2d_BSplineCurve> aCurves = new(0, 2);
        aCurves[0] = Geom2dConvert.SplitBSplineCurve(aC, 1, 2);
        aCurves[1] = Geom2dConvert.SplitBSplineCurve(aC, 2, 3);
        aCurves[2] = Geom2dConvert.SplitBSplineCurve(aC, 3, 4);
        foreach (Geom2d_BSplineCurve aCurve in new[] { aCurves[0], aCurves[1], aCurves[2] })
        {
            aCurve.Freeze();
        }

        NCollection_Array1<double> aTols = NCollection_Array1<double>.From([1e-7, 1e-7], 0);
        bool isClosed = false;
        Geom2dConvert.ConcatG1(aCurves, aTols, out NCollection_HArray1<Geom2d_BSplineCurve> aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        AssertSamePoints(aC, 0.0, 3.0, aResult[0], 0.0, 3.0, 1e-9);

        // A closed G1 contour: four quarter circles, as rational B-splines
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        NCollection_Array1<Geom2d_BSplineCurve> aQuarters = new(0, 3);
        for (int i = 0; i < 4; i++)
        {
            aQuarters[i] = Geom2dConvert.CurveToBSplineCurve(new Geom2d_TrimmedCurve(aCircle, i * M_PI / 2, (i + 1) * M_PI / 2));
            aQuarters[i].Freeze();
        }

        isClosed = true;
        Geom2dConvert.ConcatG1(aQuarters, NCollection_Array1<double>.From([1e-7, 1e-7, 1e-7], 0), out aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        Assert.True(aResult[0].IsPeriodic());
        for (int i = 0; i <= 40; i++)
        {
            double aU = aResult[0].FirstParameter() + (aResult[0].LastParameter() - aResult[0].FirstParameter()) * i / 40.0;
            Assert.Equal(2.0, aResult[0].Value(aU).Distance(new gp_Pnt2d(0, 0)), 1e-7);
        }
    }

    [Fact]
    public void ConcatG1_SingleClosedCurve_DoesNotChangeTheCaller()
    {
        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        Geom2d_BSplineCurve aWhole = Geom2dConvert.CurveToBSplineCurve(new Geom2d_TrimmedCurve(aCircle, 0.0, 2 * M_PI));
        aWhole.Freeze();
        NCollection_Array1<Geom2d_BSplineCurve> aCurves = new(0, 0);
        aCurves[0] = aWhole;
        bool isClosed = true;
        Geom2dConvert.ConcatG1(aCurves, new NCollection_Array1<double>(0, -1), out NCollection_HArray1<Geom2d_BSplineCurve> aResult, ref isClosed, 1e-7);
        Assert.True(aResult[0].IsPeriodic());
        Assert.False(aWhole.IsPeriodic());
        Assert.NotSame(aWhole, aResult[0]);
    }

    [Fact]
    public void ConcatG1_CurvesThatDontJoin_Throw()
    {
        Geom2d_BSplineCurve aC = Cubic();
        NCollection_Array1<Geom2d_BSplineCurve> aCurves = new(0, 1);
        aCurves[0] = Geom2dConvert.SplitBSplineCurve(aC, 1, 2);
        aCurves[1] = Geom2dConvert.SplitBSplineCurve(aC, 3, 4);
        bool isClosed = false;
        Assert.ThrowsAny<Standard_Failure>(() => Geom2dConvert.ConcatG1(aCurves, NCollection_Array1<double>.From([1e-7], 0),
                                                                         out _, ref isClosed, 1e-7));
    }

    private static Geom2d_BSplineCurve Segment(gp_Pnt2d theP1, gp_Pnt2d theP2) =>
        new(NCollection_Array1<gp_Pnt2d>.From([theP1, theP2]), NCollection_Array1<double>.From([0.0, 1.0]),
            NCollection_Array1<int>.From([2, 2]), 1);

    [Fact]
    public void ConcatG1_ClosedWire_ReadsTheCallersTolerances_OCCT()
    {
        // A closed wire of four segments: s0-s1 collinear (one G1 group), corners at s1-s2 and s2-s3, and a gap of
        // 1e-5 at the closure s3-s0, within ClosedTolerance. ConcatG1 reorders the curves to s2, s3, s0, s1 (the
        // closure made G1) and its local tolerances with them, but merges with the caller's array: s3 and s0 are
        // merged with the tolerance of the junction s1-s2 instead of ClosedTolerance (occt-issues.md).
        Geom2d_BSplineCurve[] aSegments =
        [
            Segment(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 0)),
            Segment(new gp_Pnt2d(1, 0), new gp_Pnt2d(2, 0)),
            Segment(new gp_Pnt2d(2, 0), new gp_Pnt2d(2, 1)),
            Segment(new gp_Pnt2d(2, 1), new gp_Pnt2d(1e-5, 0)),
        ];

        NCollection_Array1<Geom2d_BSplineCurve> aCurves = NCollection_Array1<Geom2d_BSplineCurve>.From(aSegments, 0);
        bool isClosed = true;
        Assert.ThrowsAny<Standard_ConstructionError>(() => Geom2dConvert.ConcatG1(
            aCurves, NCollection_Array1<double>.From([1e-7, 1e-12, 1e-7], 0), out _, ref isClosed, 1e-4));

        // With the junction s1-s2 given the closure's tolerance, the closure merges
        aCurves = NCollection_Array1<Geom2d_BSplineCurve>.From(aSegments, 0);
        isClosed = true;
        Geom2dConvert.ConcatG1(aCurves, NCollection_Array1<double>.From([1e-7, 1e-4, 1e-7], 0),
                               out NCollection_HArray1<Geom2d_BSplineCurve> aResult, ref isClosed, 1e-4);
        Assert.Equal(2, aResult.Length());
        Assert.Same(aSegments[2], aResult[0]);
        Assert.True(aResult[1].StartPoint().Distance(new gp_Pnt2d(2, 1)) < Tol);
        Assert.True(aResult[1].EndPoint().Distance(new gp_Pnt2d(2, 0)) < Tol);
    }

    [Fact]
    public void ConcatC1_GroupsAndIndices()
    {
        // A polyline with a corner after the second segment: two groups
        NCollection_Array1<Geom2d_BSplineCurve> aCurves = new(0, 2);
        gp_Pnt2d[] aPts = [new(0, 0), new(1, 0), new(2, 0), new(2, 1)];
        for (int i = 0; i < 3; i++)
        {
            aCurves[i] = new Geom2d_BSplineCurve(NCollection_Array1<gp_Pnt2d>.From([aPts[i], aPts[i + 1]]),
                                                 NCollection_Array1<double>.From([0.0, 1.0]), NCollection_Array1<int>.From([2, 2]), 1);
        }

        bool isClosed = false;
        Geom2dConvert.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7, 1e-7], 0), out NCollection_HArray1<int> anIndices,
                               out NCollection_HArray1<Geom2d_BSplineCurve> aResult, ref isClosed, 1e-7);
        Assert.Equal(2, aResult.Length());
        Assert.Equal(3, anIndices.Length());
        Assert.Equal(0, anIndices[0]);
        Assert.Equal(2, anIndices[1]);
        Assert.Equal(0, anIndices[2]);
        Assert.True(aResult[0].StartPoint().Distance(aPts[0]) < Tol);
        Assert.True(aResult[0].EndPoint().Distance(aPts[2]) < Tol);
        Assert.Same(aCurves[2], aResult[1]);
    }

    [Fact]
    public void ConcatC1_RationalPieces_AreNormalisedCopies()
    {
        // Two rational quartics joining with G1 continuity, whose two first and two last weights are 2:
        // Pretreatment divides their weights by 2, in copies that replace them in the array
        gp_Pnt2d[][] aPoles =
        [
            [new(0, 0), new(1, 0), new(2, 1), new(3, 1), new(4, 1)],
            [new(4, 1), new(5, 1), new(6, 2), new(7, 0), new(8, 0)],
        ];
        NCollection_Array1<Geom2d_BSplineCurve> aCurves = new(0, 1);
        Geom2d_BSplineCurve[] anOriginals = new Geom2d_BSplineCurve[2];
        for (int i = 0; i < 2; i++)
        {
            anOriginals[i] = new Geom2d_BSplineCurve(NCollection_Array1<gp_Pnt2d>.From(aPoles[i]),
                                                     NCollection_Array1<double>.From([2.0, 2.0, 1.0, 2.0, 2.0]),
                                                     NCollection_Array1<double>.From([0.0, 1.0]),
                                                     NCollection_Array1<int>.From([5, 5]), 4);
            anOriginals[i].Freeze();
            aCurves[i] = anOriginals[i];
        }

        bool isClosed = false;
        Geom2dConvert.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7], 0), out _,
                               out NCollection_HArray1<Geom2d_BSplineCurve> aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        Assert.NotSame(anOriginals[0], aCurves[0]);
        Assert.Equal(2.0, anOriginals[0].Weight(1));
        Assert.Equal(1.0, aCurves[0].Weight(1));
        Assert.Equal(0.5, aCurves[0].Weight(3));
        Assert.True(aResult[0].StartPoint().Distance(new gp_Pnt2d(0, 0)) < Tol);
        Assert.True(aResult[0].EndPoint().Distance(new gp_Pnt2d(8, 0)) < Tol);
        Assert.True(MinDistance(aResult[0], anOriginals[0].Value(0.5)) < 1e-2);
        Assert.True(MinDistance(aResult[0], anOriginals[1].Value(0.5)) < 1e-2);
    }

    [Fact]
    public void C0BSplineToC1BSplineCurve_RemovesG1Knots()
    {
        Geom2d_BSplineCurve aC = Cubic();
        // Structurally C0 at knot 1, geometrically smooth
        Geom2d_BSplineCurve aC0 = aC.KnotInserted(1.0, 3);
        Assert.Equal(3, aC0.Multiplicity(2));
        aC0.Freeze();
        Geom2d_BSplineCurve aResult = aC0;
        Geom2dConvert.C0BSplineToC1BSplineCurve(ref aResult, 1e-7);
        Assert.NotSame(aC0, aResult);
        Assert.Equal(3, aC0.Multiplicity(2));
        Assert.True(aResult.Continuity() >= GeomAbs_C1);
        Assert.True(aResult.StartPoint().Distance(aC.StartPoint()) < Tol);
        Assert.True(aResult.EndPoint().Distance(aC.EndPoint()) < Tol);

        // Nothing to do
        Geom2d_BSplineCurve aSmooth = aC;
        Geom2dConvert.C0BSplineToC1BSplineCurve(ref aSmooth, 1e-7);
        Assert.Same(aC, aSmooth);
    }

    [Fact]
    public void C0BSplineToArrayOfC1BSplineCurve_SplitsAtCorners()
    {
        Geom2d_BSplineCurve aPolyline = new(NCollection_Array1<gp_Pnt2d>.From([new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 0), new gp_Pnt2d(1, 1)]),
                                            NCollection_Array1<double>.From([0.0, 1.0, 2.0]), NCollection_Array1<int>.From([2, 1, 2]), 1);
        Geom2dConvert.C0BSplineToArrayOfC1BSplineCurve(aPolyline, out NCollection_HArray1<Geom2d_BSplineCurve> aTab, 1e-7);
        Assert.Equal(2, aTab.Length());
        Assert.True(aTab[0].EndPoint().Distance(new gp_Pnt2d(1, 0)) < Tol);
        Assert.True(aTab[1].StartPoint().Distance(new gp_Pnt2d(1, 0)) < Tol);

        Geom2d_BSplineCurve aC = Cubic();
        Geom2dConvert.C0BSplineToArrayOfC1BSplineCurve(aC, out aTab, 1e-7);
        Assert.Equal(1, aTab.Length());
        Assert.Same(aC, aTab[0]);
    }

    [Fact]
    public void ApproxCurve_OfAnOffsetCurve()
    {
        Geom2d_BSplineCurve aC = Cubic();
        Geom2d_OffsetCurve anOffset = new(aC, 0.2);
        Geom2dConvert_ApproxCurve anApprox = new(anOffset, 1e-5, GeomAbs_C2, 50, 9);
        Assert.True(anApprox.HasResult());
        Assert.True(anApprox.IsDone());
        Assert.True(anApprox.MaxError() < 1e-5);
        Geom2d_BSplineCurve aResult = anApprox.Curve()!;
        Assert.True(aResult.IsFrozen());
        Assert.Same(aResult, anApprox.Curve());
        Assert.True(aResult.Degree() <= 9);
        AssertSamePoints(anOffset, 0.0, 3.0, aResult, aResult.FirstParameter(), aResult.LastParameter(), 1e-4);
        Assert.Contains("Dump of ApproxCurve", anApprox.ToString());

        // Through an adaptor on part of the curve
        Geom2dConvert_ApproxCurve aPart = new(new Geom2dAdaptor_Curve(anOffset, 0.5, 2.5), 1e-5, GeomAbs_C1, 50, 9);
        Assert.True(aPart.HasResult());
        Assert.Equal(0.5, aPart.Curve()!.FirstParameter());
        Assert.Equal(2.5, aPart.Curve()!.LastParameter());
    }

    [Fact]
    public void ApproxArcsSegments_LineAndCircle()
    {
        Geom2d_Line aLine = new(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 1));
        Geom2dConvert_ApproxArcsSegments aLineApprox = new(new Geom2dAdaptor_Curve(aLine, 0.0, 5.0), 1e-3, 0.01);
        Schematiq.Collections.ISchematiqList<Geom2d_Curve> aResult = aLineApprox.GetResult();
        Assert.Single(aResult);
        Assert.True(aResult[0].IsFrozen());
        Assert.True(aResult[0].Value(aResult[0].LastParameter()).Distance(aLine.Value(5.0)) < 1e-9);

        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(1, 1), new gp_Dir2d(1, 0)), 2.0);
        Geom2dConvert_ApproxArcsSegments aCircleApprox = new(new Geom2dAdaptor_Curve(aCircle, 0.0, 3.0), 1e-3, 0.01);
        aResult = aCircleApprox.GetResult();
        Assert.Equal(2, aResult.Count);
        foreach (Geom2d_Curve aCurve in aResult)
        {
            Geom2d_Circle aBasis = (Geom2d_Circle)((Geom2d_TrimmedCurve)aCurve).BasisCurve();
            Assert.Equal(2.0, aBasis.Radius(), 1e-9);
        }
    }

    [Fact]
    public void ApproxArcsSegments_FreeForm()
    {
        Geom2d_BSplineCurve aC = Cubic();
        const double aTol = 1e-2;
        Geom2dConvert_ApproxArcsSegments anApprox = new(new Geom2dAdaptor_Curve(aC), aTol, 0.1);
        Schematiq.Collections.ISchematiqList<Geom2d_Curve> aResult = anApprox.GetResult();
        Assert.True(aResult.Count > 1);
        Assert.True(aResult[0].Value(aResult[0].FirstParameter()).Distance(aC.StartPoint()) < 1e-7);
        Geom2d_Curve aLastCurve = aResult[aResult.Count - 1];
        Assert.True(aLastCurve.Value(aLastCurve.LastParameter()).Distance(aC.EndPoint()) < 1e-7);
        for (int i = 0; i < aResult.Count; i++)
        {
            Geom2d_Curve aCurve = aResult[i];
            Assert.True(aCurve is Geom2d_TrimmedCurve { } aTrimmed && (aTrimmed.BasisCurve() is Geom2d_Line or Geom2d_Circle));
            if (i > 0)
            {
                Geom2d_Curve aPrev = aResult[i - 1];
                Assert.True(aPrev.Value(aPrev.LastParameter()).Distance(aCurve.Value(aCurve.FirstParameter())) < 1e-6);
            }

            // Every piece lies within the tolerance of the curve
            for (int k = 0; k <= 10; k++)
            {
                gp_Pnt2d aP = aCurve.Value(aCurve.FirstParameter() + (aCurve.LastParameter() - aCurve.FirstParameter()) * k / 10.0);
                Assert.True(MinDistance(aC, aP) < aTol + 2e-3, $"piece {i} point {k}");
            }
        }
    }

    [Fact]
    public void PPoint()
    {
        Geom2dConvert_PPoint anEmpty = new();
        Assert.Equal(RealLast(), anEmpty.Parameter());
        Geom2dConvert_PPoint aP = new(1.0, new gp_XY(1, 2), new gp_XY(0, 1));
        Geom2dConvert_PPoint aQ = new(1.0 + 1e-12, new gp_XY(5, 5), new gp_XY(1, 0));
        Assert.True(aP.IsEqual(aQ));
        Assert.False(aP == aQ);
        Assert.Equal(new gp_XY(3, 3), aP.WithD1(new gp_XY(3, 3)).D1());
        Assert.Equal(Math.Sqrt(9 + 16), aP.Dist(new Geom2dConvert_PPoint(0.0, new gp_XY(4, 6), new gp_XY(0, 1))), 1e-15);

        Geom2d_Circle aCircle = new(new gp_Ax2d(new gp_Pnt2d(0, 0), new gp_Dir2d(1, 0)), 2.0);
        Geom2dConvert_PPoint anOnCurve = new(M_PI / 2, new Geom2dAdaptor_Curve(aCircle));
        Assert.True(new gp_Pnt2d(anOnCurve.Point()).Distance(new gp_Pnt2d(0, 2)) < 1e-15);
        Assert.True(new gp_Vec2d(anOnCurve.D1()).IsParallel(new gp_Vec2d(-1, 0), 1e-12));
    }
}
