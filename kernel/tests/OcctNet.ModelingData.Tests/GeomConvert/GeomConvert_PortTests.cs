// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// OCCT.NET's own tests of GeomConvert's statics, GeomConvert_ApproxCurve, GeomConvert_ApproxSurface,
/// GeomConvert_BSplineCurveKnotSplitting, GeomConvert_BSplineCurveToBezierCurve and the freezing rules of
/// GeomConvert_CompCurveToBSplineCurve (OCCT's GTests cover little of them).
/// </summary>
public class GeomConvert_PortTests
{
    private const double Tol = 1e-9;

    private static gp_Pnt P(double theX, double theY, double theZ = 0.0) => new(theX, theY, theZ);

    private static Geom_BSplineCurve Cubic()
    {
        // A smooth cubic with three spans
        NCollection_Array1<gp_Pnt> aPoles = NCollection_Array1<gp_Pnt>.From([
            P(0, 0, 0), P(1, 2, 1), P(3, 3, 0), P(5, 1, -1), P(7, 2, 0), P(8, 0, 1)]);
        return new Geom_BSplineCurve(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0]),
                                     NCollection_Array1<int>.From([4, 1, 1, 4]), 3);
    }

    private static Geom_BSplineCurve Segment(gp_Pnt theP1, gp_Pnt theP2) =>
        new(NCollection_Array1<gp_Pnt>.From([theP1, theP2]), NCollection_Array1<double>.From([0.0, 1.0]),
            NCollection_Array1<int>.From([2, 2]), 1);

    private static void AssertSamePoints(Geom_Curve theExpected, double theU1, double theU2, Geom_Curve theActual,
                                         double theV1, double theV2, double theTol)
    {
        for (int i = 0; i <= 20; i++)
        {
            gp_Pnt aP = theExpected.Value(theU1 + (theU2 - theU1) * i / 20.0);
            gp_Pnt aQ = theActual.Value(theV1 + (theV2 - theV1) * i / 20.0);
            Assert.True(aP.Distance(aQ) < theTol, $"point {i}: {aP} vs {aQ}");
        }
    }

    [Fact]
    public void CurveToBSplineCurve_WholeCircle_IsPeriodic_AndLongRationalC1Arc_IsJoined()
    {
        Geom_Circle aCircle = new(new gp_Ax2(P(1, 2, 3), new gp_Dir(0, 1, 1), new gp_Dir(1, 0, 0)), 4.0);
        Geom_BSplineCurve aWhole = GeomConvert.CurveToBSplineCurve(aCircle);
        Assert.True(aWhole.IsPeriodic());
        Assert.False(aWhole.IsFrozen());
        Assert.True(aWhole.StartPoint().Distance(aCircle.Value(0.0)) < Tol);
        for (int i = 0; i <= 20; i++)
        {
            Assert.Equal(4.0, aWhole.Value(i * 0.3).Distance(P(1, 2, 3)), 1e-12);
        }

        // An arc of 6.2 radians with RationalC1: two halves joined by GeomConvert_CompCurveToBSplineCurve
        Geom_BSplineCurve aLong = GeomConvert.CurveToBSplineCurve(new Geom_TrimmedCurve(aCircle, 0.0, 6.2), Convert_RationalC1);
        Assert.True(aLong.StartPoint().Distance(aCircle.Value(0.0)) < Tol);
        Assert.True(aLong.EndPoint().Distance(aCircle.Value(6.2)) < Tol);
        Assert.True(aLong.Continuity() >= GeomAbs_C1);
    }

    [Fact]
    public void CurveToBSplineCurve_FrozenTrimmedPeriodicBSpline_IsCopied()
    {
        Geom_BSplineCurve aPeriodic = Cubic().KnotInserted(1.5, 1, 0.0, false);
        aPeriodic = GeomConvert.CurveToBSplineCurve(new Geom_TrimmedCurve(aPeriodic, 0.0, 3.0));
        aPeriodic.Freeze();
        Geom_TrimmedCurve aTrimmed = new(aPeriodic, 0.5, 2.5);
        Geom_BSplineCurve aResult = GeomConvert.CurveToBSplineCurve(aTrimmed);
        Assert.False(aResult.IsFrozen());
        AssertSamePoints(aPeriodic, 0.5, 2.5, aResult, 0.5, 2.5, Tol);
        Assert.Equal(3.0, aPeriodic.LastParameter());
    }

    [Fact]
    public void CurveToBSplineCurve_OffsetCurve_IsApproximated_AndUnboundedCurvesThrow()
    {
        Geom_OffsetCurve anOffset = new(Cubic(), 0.5, new gp_Dir(0, 0, 1));
        Geom_BSplineCurve aResult = GeomConvert.CurveToBSplineCurve(anOffset);
        Assert.False(aResult.IsFrozen());
        for (int i = 0; i <= 10; i++)
        {
            double aU = 0.3 * i;
            Assert.True(aResult.Value(aU).Distance(anOffset.Value(aU)) < 1e-3);
        }

        Assert.ThrowsAny<Standard_DomainError>(() => GeomConvert.CurveToBSplineCurve(new Geom_Line(P(0, 0), new gp_Dir(1, 0, 0))));
        Assert.ThrowsAny<Standard_DomainError>(
            () => GeomConvert.CurveToBSplineCurve(new Geom_Parabola(new gp_Ax2(P(0, 0), new gp_Dir(0, 0, 1)), 1.0)));
    }

    [Fact]
    public void SplitBSplineCurve_ByKnotsAndParameters()
    {
        Geom_BSplineCurve aC = Cubic();
        Geom_BSplineCurve aPart = GeomConvert.SplitBSplineCurve(aC, 2, 3);
        Assert.Equal(1.0, aPart.FirstParameter());
        Assert.Equal(2.0, aPart.LastParameter());
        AssertSamePoints(aC, 1.0, 2.0, aPart, 1.0, 2.0, Tol);

        Geom_BSplineCurve aReversed = GeomConvert.SplitBSplineCurve(aC, 3, 2);
        Assert.True(aReversed.StartPoint().Distance(aC.Value(2.0)) < Tol);
        Assert.True(aReversed.EndPoint().Distance(aC.Value(1.0)) < Tol);

        Geom_BSplineCurve aByParams = GeomConvert.SplitBSplineCurve(aC, 2.5, 0.5, 1e-9);
        Assert.True(aByParams.StartPoint().Distance(aC.Value(2.5)) < Tol);
        Assert.True(aByParams.EndPoint().Distance(aC.Value(0.5)) < Tol);

        // GeomConvert throws Standard_DomainError for both (Geom2dConvert: Standard_OutOfRange for the second)
        Assert.ThrowsAny<Standard_DomainError>(() => GeomConvert.SplitBSplineCurve(aC, 2, 2));
        Assert.ThrowsAny<Standard_DomainError>(() => GeomConvert.SplitBSplineCurve(aC, 0, 2));
    }

    private static Geom_BSplineSurface BiCubic()
    {
        NCollection_Array2<gp_Pnt> aPoles = new(1, 5, 1, 4);
        for (int i = 1; i <= 5; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                aPoles[i, j] = P(i, j, Math.Sin(i + 2.0 * j));
            }
        }

        return new Geom_BSplineSurface(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 3.0]),
                                       NCollection_Array1<double>.From([0.0, 2.0]), NCollection_Array1<int>.From([4, 1, 4]),
                                       NCollection_Array1<int>.From([4, 4]), 3, 3);
    }

    [Fact]
    public void SplitBSplineSurface_AllOverloads()
    {
        Geom_BSplineSurface aS = BiCubic();
        Geom_BSplineSurface aKnots = GeomConvert.SplitBSplineSurface(aS, 3, 2, 1, 2);
        aKnots.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((1.0, 3.0, 0.0, 2.0), (aU1, aU2, aV1, aV2));
        // reversed in U: the parameter u of the result is 4 - u on S
        Assert.True(aKnots.Value(1.5, 0.5).Distance(aS.Value(2.5, 0.5)) < Tol);

        Geom_BSplineSurface aU = GeomConvert.SplitBSplineSurface(aS, 1, 2, true);
        aU.Bounds(out aU1, out aU2, out _, out _);
        Assert.Equal((0.0, 1.0), (aU1, aU2));

        Geom_BSplineSurface aParams = GeomConvert.SplitBSplineSurface(aS, 0.5, 2.5, 1.5, 0.5, 1e-9);
        Assert.True(aParams.Value(1.0, 1.0).Distance(aS.Value(1.0, 1.0)) < Tol);
        aParams.Bounds(out _, out _, out aV1, out aV2);
        // reversed in V
        Assert.True(aParams.Value(1.0, aV1).Distance(aS.Value(1.0, 1.5)) < Tol);

        Assert.ThrowsAny<Standard_DomainError>(() => GeomConvert.SplitBSplineSurface(aS, 1, 1, 1, 2));
        Assert.ThrowsAny<Standard_DomainError>(() => GeomConvert.SplitBSplineSurface(aS, 1, 4, false));
        Assert.ThrowsAny<Standard_DomainError>(() => GeomConvert.SplitBSplineSurface(aS, 1.0, 1.0, true, 1e-9));
    }

    [Fact]
    public void SplitBSplineSurface_InV_ReversesU_OCCT()
    {
        // OCCT's split by parameters in V tests FromParam1 > ToParam2 and reverses U where V is meant
        // (occt-issues.md): the V range is not reversed, the U one is.
        Geom_BSplineSurface aS = BiCubic();
        Geom_BSplineSurface aSplit = GeomConvert.SplitBSplineSurface(aS, 1.5, 0.5, false, 1e-9);
        aSplit.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        Assert.Equal((0.5, 1.5), (aV1, aV2));
        Assert.True(aSplit.Value(aU1, 0.5).Distance(aS.Value(3.0, 0.5)) < Tol);
        Assert.True(aSplit.Value(aU2, 1.5).Distance(aS.Value(0.0, 1.5)) < Tol);
    }

    [Fact]
    public void BSplineCurveKnotSplitting()
    {
        // Knot 2 with multiplicity 3 (C0), knot 3 with multiplicity 2 (C1)
        Geom_BSplineCurve aC = Cubic().KnotInserted(1.0, 3, 0.0, false).KnotInserted(2.0, 2, 0.0, false);
        GeomConvert_BSplineCurveKnotSplitting aSplit0 = new(aC, 0);
        Assert.Equal(2, aSplit0.NbSplits());
        GeomConvert_BSplineCurveKnotSplitting aSplit1 = new(aC, 1);
        Assert.Equal(3, aSplit1.NbSplits());
        Assert.Equal(1, aSplit1.SplitValue(1));
        Assert.Equal(2, aSplit1.SplitValue(2));
        Assert.Equal(4, aSplit1.SplitValue(3));
        GeomConvert_BSplineCurveKnotSplitting aSplit2 = new(aC, 2);
        NCollection_Array1<int> aValues = new(1, aSplit2.NbSplits());
        aSplit2.Splitting(aValues);
        Assert.Equal([1, 2, 3, 4], aValues.AsSpan().ToArray());
        Assert.ThrowsAny<Standard_RangeError>(() => new GeomConvert_BSplineCurveKnotSplitting(aC, -1));
        Assert.ThrowsAny<Standard_RangeError>(() => aSplit2.SplitValue(5));
    }

    [Fact]
    public void BSplineCurveToBezierCurve()
    {
        Geom_BSplineCurve aC = Cubic();
        aC.Freeze();
        GeomConvert_BSplineCurveToBezierCurve aConv = new(aC);
        Assert.Equal(3, aConv.NbArcs());
        NCollection_Array1<Geom_BezierCurve> anArcs = new(1, 3);
        aConv.Arcs(anArcs);
        NCollection_Array1<double> aKnots = new(1, 4);
        aConv.Knots(aKnots);
        Assert.Equal([0.0, 1.0, 2.0, 3.0], aKnots.AsSpan().ToArray());
        for (int i = 1; i <= 3; i++)
        {
            Assert.Equal(3, anArcs[i].Degree());
            AssertSamePoints(aC, i - 1.0, i, anArcs[i], 0.0, 1.0, Tol);
            Assert.NotSame(anArcs[i], aConv.Arc(i));
            Assert.False(anArcs[i].IsFrozen());
        }

        Assert.ThrowsAny<Standard_OutOfRange>(() => aConv.Arc(4));

        // A part, with bounds snapped onto knots within half the tolerance
        GeomConvert_BSplineCurveToBezierCurve aPart = new(aC, 0.5, 2.0 + 1e-4, 1e-3);
        Assert.Equal(2, aPart.NbArcs());
        AssertSamePoints(aC, 0.5, 1.0, aPart.Arc(1), 0.0, 1.0, Tol);
        Assert.ThrowsAny<Standard_DomainError>(() => new GeomConvert_BSplineCurveToBezierCurve(aC, 1.0, 1.0, 1e-3));

        // Rational
        Geom_BSplineCurve aRational = GeomConvert.CurveToBSplineCurve(
            new Geom_TrimmedCurve(new Geom_Circle(new gp_Ax2(P(0, 0), new gp_Dir(0, 0, 1)), 2.0), 0.0, 3.0));
        GeomConvert_BSplineCurveToBezierCurve aRConv = new(aRational);
        for (int i = 1; i <= aRConv.NbArcs(); i++)
        {
            Geom_BezierCurve anArc = aRConv.Arc(i);
            Assert.True(anArc.IsRational());
            Assert.Equal(2.0, anArc.Value(0.37).Distance(P(0, 0)), 1e-12);
        }
    }

    [Fact]
    public void BSplineCurveToBezierCurve_Periodic()
    {
        // GeomConvert keeps a periodic curve periodic (Geom2dConvert makes it non-periodic first, OCC7372)
        NCollection_Array1<gp_Pnt> aPoles = NCollection_Array1<gp_Pnt>.From([P(0, 0), P(4, 0, 1), P(4, 4), P(0, 4, -1), P(-1, 2)]);
        Geom_BSplineCurve aPeriodic = new(aPoles, NCollection_Array1<double>.From([0.0, 1.0, 2.0, 3.0, 4.0, 5.0]),
                                          NCollection_Array1<int>.From([1, 1, 1, 1, 1, 1]), 3, true);
        GeomConvert_BSplineCurveToBezierCurve aConv = new(aPeriodic);
        Assert.Equal(5, aConv.NbArcs());
        for (int i = 1; i <= 5; i++)
        {
            AssertSamePoints(aPeriodic, i - 1.0, i, aConv.Arc(i), 0.0, 1.0, Tol);
        }
    }

    [Fact]
    public void CompCurveToBSplineCurve_KeepsHandedOutCurves_AndOptions()
    {
        Geom_BSplineCurve aC = Cubic();
        Geom_BSplineCurve aMiddle = GeomConvert.SplitBSplineCurve(aC, 2, 3);
        Geom_BSplineCurve aFirst = GeomConvert.SplitBSplineCurve(aC, 1, 2);
        Geom_BSplineCurve aLast = GeomConvert.SplitBSplineCurve(aC, 3, 4);
        aFirst.Freeze();
        GeomConvert_CompCurveToBSplineCurve aComp = new(aMiddle);
        Geom_BSplineCurve? aHandedOut = aComp.BSplineCurve();
        Assert.NotNull(aHandedOut);
        Assert.True(aHandedOut.IsFrozen());
        Assert.Same(aHandedOut, aComp.BSplineCurve());
        Assert.NotSame(aMiddle, aHandedOut);

        Assert.True(aComp.Add(aLast, 1e-7, true));
        Assert.True(aComp.Add(aFirst, 1e-7));
        Assert.Equal(1.0, aHandedOut.FirstParameter());
        Assert.Equal(2.0, aHandedOut.LastParameter());

        Geom_BSplineCurve aResult = aComp.BSplineCurve()!;
        Assert.True(aResult.StartPoint().Distance(aC.StartPoint()) < Tol);
        Assert.True(aResult.EndPoint().Distance(aC.EndPoint()) < Tol);
        // The C2 junctions are merged back: the original knots
        Assert.Equal(4, aResult.NbKnots());

        // MinM keeps the joint knot's multiplicity
        GeomConvert_CompCurveToBSplineCurve aKept = new(aFirst);
        Assert.True(aKept.Add(aMiddle, 1e-7, true, true, 2));
        Assert.Equal(2, aKept.BSplineCurve()!.Multiplicity(2));

        // A lower degree is raised; WithRatio false keeps the added curve's parametrisation length
        Geom_TrimmedCurve aLine = new(new Geom_Line(aC.EndPoint(), new gp_Dir(1, 0, 0)), 0.0, 10.0);
        GeomConvert_CompCurveToBSplineCurve aWithLine = new(aC);
        Assert.True(aWithLine.Add(aLine, 1e-7, true, false));
        Geom_BSplineCurve aJoined = aWithLine.BSplineCurve()!;
        Assert.Equal(3, aJoined.Degree());
        Assert.Equal(13.0, aJoined.LastParameter(), 1e-12);
        Assert.True(aJoined.EndPoint().Distance(aLine.EndPoint()) < Tol);
    }

    [Fact]
    public void ConcatG1_OpenAndClosed()
    {
        Geom_BSplineCurve aC = Cubic();
        NCollection_Array1<Geom_BSplineCurve> aCurves = new(0, 2);
        aCurves[0] = GeomConvert.SplitBSplineCurve(aC, 1, 2);
        aCurves[1] = GeomConvert.SplitBSplineCurve(aC, 2, 3);
        aCurves[2] = GeomConvert.SplitBSplineCurve(aC, 3, 4);
        foreach (Geom_BSplineCurve aCurve in new[] { aCurves[0], aCurves[1], aCurves[2] })
        {
            aCurve.Freeze();
        }

        NCollection_Array1<double> aTols = NCollection_Array1<double>.From([1e-7, 1e-7], 0);
        bool isClosed = false;
        GeomConvert.ConcatG1(aCurves, aTols, out NCollection_HArray1<Geom_BSplineCurve> aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        AssertSamePoints(aC, 0.0, 3.0, aResult[0], 0.0, 3.0, 1e-9);

        // A closed G1 contour: four quarter circles, as rational B-splines
        Geom_Circle aCircle = new(new gp_Ax2(P(0, 0, 1), new gp_Dir(1, 1, 1)), 2.0);
        NCollection_Array1<Geom_BSplineCurve> aQuarters = new(0, 3);
        for (int i = 0; i < 4; i++)
        {
            aQuarters[i] = GeomConvert.CurveToBSplineCurve(new Geom_TrimmedCurve(aCircle, i * M_PI / 2, (i + 1) * M_PI / 2));
            aQuarters[i].Freeze();
        }

        isClosed = true;
        GeomConvert.ConcatG1(aQuarters, NCollection_Array1<double>.From([1e-7, 1e-7, 1e-7], 0), out aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        Assert.True(aResult[0].IsPeriodic());
        for (int i = 0; i <= 40; i++)
        {
            double aU = aResult[0].FirstParameter() + (aResult[0].LastParameter() - aResult[0].FirstParameter()) * i / 40.0;
            Assert.Equal(2.0, aResult[0].Value(aU).Distance(P(0, 0, 1)), 1e-7);
        }
    }

    [Fact]
    public void ConcatG1_SingleClosedCurve_DoesNotChangeTheCaller()
    {
        Geom_Circle aCircle = new(new gp_Ax2(P(0, 0), new gp_Dir(0, 0, 1)), 2.0);
        Geom_BSplineCurve aWhole = GeomConvert.CurveToBSplineCurve(new Geom_TrimmedCurve(aCircle, 0.0, 2 * M_PI));
        aWhole.Freeze();
        NCollection_Array1<Geom_BSplineCurve> aCurves = new(0, 0);
        aCurves[0] = aWhole;
        bool isClosed = true;
        GeomConvert.ConcatG1(aCurves, new NCollection_Array1<double>(0, -1), out NCollection_HArray1<Geom_BSplineCurve> aResult,
                             ref isClosed, 1e-7);
        Assert.True(aResult[0].IsPeriodic());
        Assert.False(aWhole.IsPeriodic());
        Assert.NotSame(aWhole, aResult[0]);
    }

    [Fact]
    public void ConcatG1_CurvesThatDontJoin_Throw()
    {
        Geom_BSplineCurve aC = Cubic();
        NCollection_Array1<Geom_BSplineCurve> aCurves = new(0, 1);
        aCurves[0] = GeomConvert.SplitBSplineCurve(aC, 1, 2);
        aCurves[1] = GeomConvert.SplitBSplineCurve(aC, 3, 4);
        bool isClosed = false;
        // GeomLProp::Continuity: "Courbes non jointives"
        Assert.ThrowsAny<Standard_Failure>(() => GeomConvert.ConcatG1(aCurves, NCollection_Array1<double>.From([1e-7], 0),
                                                                       out _, ref isClosed, 1e-7));
    }

    [Fact]
    public void ConcatG1_ClosedWire_ReadsTheCallersTolerances_OCCT()
    {
        // As in Geom2dConvert (occt-issues.md): after reordering a closed wire, ConcatG1 merges each group with the
        // caller's tolerances, not the reordered ones, so s3 and s0 are merged with the tolerance of the junction s1-s2
        // instead of ClosedTolerance. (ConcatC1 reads the reordered ones in 3D.)
        Geom_BSplineCurve[] aSegments =
        [
            Segment(P(0, 0), P(1, 0)),
            Segment(P(1, 0), P(2, 0)),
            Segment(P(2, 0), P(2, 1)),
            Segment(P(2, 1), P(1e-5, 0)),
        ];

        NCollection_Array1<Geom_BSplineCurve> aCurves = NCollection_Array1<Geom_BSplineCurve>.From(aSegments, 0);
        bool isClosed = true;
        Assert.ThrowsAny<Standard_ConstructionError>(() => GeomConvert.ConcatG1(
            aCurves, NCollection_Array1<double>.From([1e-7, 1e-12, 1e-7], 0), out _, ref isClosed, 1e-4));

        aCurves = NCollection_Array1<Geom_BSplineCurve>.From(aSegments, 0);
        isClosed = true;
        GeomConvert.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7, 1e-12, 1e-7], 0), out _,
                             out NCollection_HArray1<Geom_BSplineCurve> aResult, ref isClosed, 1e-4);
        Assert.Equal(2, aResult.Length());
        Assert.True(aResult[1].StartPoint().Distance(P(2, 1)) < Tol);
        Assert.True(aResult[1].EndPoint().Distance(P(2, 0)) < Tol);
    }

    [Fact]
    public void ConcatC1_GroupsAndIndices()
    {
        // A polyline with a corner after the second segment: two groups
        gp_Pnt[] aPts = [P(0, 0), P(1, 0), P(2, 0), P(2, 1)];
        NCollection_Array1<Geom_BSplineCurve> aCurves = new(0, 2);
        for (int i = 0; i < 3; i++)
        {
            aCurves[i] = Segment(aPts[i], aPts[i + 1]);
        }

        bool isClosed = false;
        GeomConvert.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7, 1e-7], 0), out NCollection_HArray1<int> anIndices,
                             out NCollection_HArray1<Geom_BSplineCurve> aResult, ref isClosed, 1e-7);
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
        // Two rational quartics joining with G1 continuity, whose two first and two last weights are 2: Pretreatment
        // divides their weights by 2, in copies that replace them in the array; then Hermit makes them C1
        gp_Pnt[][] aPoles =
        [
            [P(0, 0), P(1, 0), P(2, 1, 1), P(3, 1), P(4, 1)],
            [P(4, 1), P(5, 1), P(6, 2, -1), P(7, 0), P(8, 0)],
        ];
        NCollection_Array1<Geom_BSplineCurve> aCurves = new(0, 1);
        Geom_BSplineCurve[] anOriginals = new Geom_BSplineCurve[2];
        for (int i = 0; i < 2; i++)
        {
            anOriginals[i] = new Geom_BSplineCurve(NCollection_Array1<gp_Pnt>.From(aPoles[i]),
                                                   NCollection_Array1<double>.From([2.0, 2.0, 1.0, 2.0, 2.0]),
                                                   NCollection_Array1<double>.From([0.0, 1.0]),
                                                   NCollection_Array1<int>.From([5, 5]), 4);
            anOriginals[i].Freeze();
            aCurves[i] = anOriginals[i];
        }

        bool isClosed = false;
        GeomConvert.ConcatC1(aCurves, NCollection_Array1<double>.From([1e-7], 0), out _,
                             out NCollection_HArray1<Geom_BSplineCurve> aResult, ref isClosed, 1e-7);
        Assert.Equal(1, aResult.Length());
        Assert.NotSame(anOriginals[0], aCurves[0]);
        Assert.Equal(2.0, anOriginals[0].Weight(1));
        Assert.Equal(1.0, aCurves[0].Weight(1));
        Assert.Equal(0.5, aCurves[0].Weight(3));
        Assert.True(aResult[0].StartPoint().Distance(P(0, 0)) < Tol);
        Assert.True(aResult[0].EndPoint().Distance(P(8, 0)) < Tol);
    }

    [Fact]
    public void C0BSplineToC1BSplineCurve_RemovesG1Knots()
    {
        Geom_BSplineCurve aC = Cubic();
        // Structurally C0 at knot 1, geometrically smooth
        Geom_BSplineCurve aC0 = aC.KnotInserted(1.0, 3, 0.0, false);
        Assert.Equal(3, aC0.Multiplicity(2));
        aC0.Freeze();
        Geom_BSplineCurve aResult = aC0;
        GeomConvert.C0BSplineToC1BSplineCurve(ref aResult, 1e-7);
        Assert.NotSame(aC0, aResult);
        Assert.False(aResult.IsFrozen());
        Assert.Equal(3, aC0.Multiplicity(2));
        Assert.True(aResult.Continuity() >= GeomAbs_C1);
        Assert.True(aResult.StartPoint().Distance(aC.StartPoint()) < Tol);
        Assert.True(aResult.EndPoint().Distance(aC.EndPoint()) < Tol);

        // Nothing to do: still a new curve (GeomConvert, unlike Geom2dConvert, always goes through the concatenation)
        Geom_BSplineCurve aSmooth = aC;
        GeomConvert.C0BSplineToC1BSplineCurve(ref aSmooth, 1e-7);
        Assert.NotSame(aC, aSmooth);
        AssertSamePoints(aC, 0.0, 3.0, aSmooth, 0.0, 3.0, Tol);
    }

    [Fact]
    public void C0BSplineToArrayOfC1BSplineCurve_SplitsAtCorners()
    {
        Geom_BSplineCurve aPolyline = new(NCollection_Array1<gp_Pnt>.From([P(0, 0), P(1, 0), P(1, 1)]),
                                          NCollection_Array1<double>.From([0.0, 1.0, 2.0]), NCollection_Array1<int>.From([2, 1, 2]), 1);
        GeomConvert.C0BSplineToArrayOfC1BSplineCurve(aPolyline, out NCollection_HArray1<Geom_BSplineCurve> aTab, 1e-7);
        Assert.Equal(2, aTab.Length());
        Assert.True(aTab[0].EndPoint().Distance(P(1, 0)) < Tol);
        Assert.True(aTab[1].StartPoint().Distance(P(1, 0)) < Tol);

        Geom_BSplineCurve aC = Cubic();
        GeomConvert.C0BSplineToArrayOfC1BSplineCurve(aC, out aTab, 1e-7);
        Assert.Equal(1, aTab.Length());
        Assert.Same(aC, aTab[0]);
    }

    [Fact]
    public void ApproxCurve_OfAnOffsetCurve_AndThroughAnAdaptor()
    {
        Geom_OffsetCurve anOffset = new(Cubic(), -0.3, new gp_Dir(0, 0, 1));
        GeomConvert_ApproxCurve anApprox = new(anOffset, 1e-5, GeomAbs_C2, 20, 9);
        Assert.True(anApprox.HasResult());
        Assert.True(anApprox.IsDone());
        Assert.True(anApprox.MaxError() < 1e-5);
        Geom_BSplineCurve aCurve = anApprox.Curve()!;
        Assert.True(aCurve.IsFrozen());
        Assert.Same(aCurve, anApprox.Curve());
        for (int i = 0; i <= 30; i++)
        {
            double aU = 0.1 * i;
            Assert.True(aCurve.Value(aU).Distance(anOffset.Value(aU)) < 1e-5);
        }

        GeomConvert_ApproxCurve aPart = new(new GeomAdaptor_Curve(anOffset, 0.5, 2.0), 1e-5, GeomAbs_C1, 20, 9);
        Assert.True(aPart.HasResult());
        Assert.Equal(0.5, aPart.Curve()!.FirstParameter());
        Assert.Equal(2.0, aPart.Curve()!.LastParameter());
        Assert.StartsWith("******* Dump of ApproxCurve *******\n*******Error   ", aPart.ToString());
    }

    [Fact]
    public void ApproxSurface_OfAnOffsetSurface()
    {
        Geom_BSplineSurface aBasis = BiCubic();
        Geom_OffsetSurface anOffset = new(aBasis, 0.2);
        GeomConvert_ApproxSurface anApprox = new(anOffset, 1e-4, GeomAbs_C1, GeomAbs_C1, 9, 9, 50, 1);
        Assert.True(anApprox.HasResult());
        Geom_BSplineSurface aSurface = anApprox.Surface()!;
        Assert.True(aSurface.IsFrozen());
        Assert.True(anApprox.MaxError() >= 0.0);
        for (int i = 0; i <= 6; i++)
        {
            for (int j = 0; j <= 4; j++)
            {
                double aU = 0.5 * i, aV = 0.5 * j;
                Assert.True(aSurface.Value(aU, aV).Distance(anOffset.Value(aU, aV)) < 1e-3);
            }
        }

        Assert.Contains("Result max error :", anApprox.ToString());

        // Through SurfaceToBSplineSurface: an unfrozen copy for the caller
        Geom_BSplineSurface aConverted = GeomConvert.SurfaceToBSplineSurface(anOffset);
        Assert.False(aConverted.IsFrozen());
        Assert.True(aConverted.Value(1.0, 1.0).Distance(anOffset.Value(1.0, 1.0)) < 1e-3);
    }

    [Fact]
    public void SurfaceToBSplineSurface_SweptAndElementary()
    {
        // A trimmed surface of revolution of a circle arc (a torus patch) and an extrusion of a B-spline curve
        Geom_Circle aMeridian = new(new gp_Ax2(P(5, 0, 0), new gp_Dir(0, 1, 0)), 1.0);
        Geom_SurfaceOfRevolution aRevol = new(aMeridian, new gp_Ax1(P(0, 0, 0), new gp_Dir(0, 0, 1)));
        Geom_RectangularTrimmedSurface aRevolPatch = new(aRevol, 0.0, 4.0, 0.5, 2.0);
        Geom_BSplineSurface aR = GeomConvert.SurfaceToBSplineSurface(aRevolPatch);
        Geom_SurfaceOfLinearExtrusion anExtru = new(Cubic(), new gp_Dir(0, 1, 1));
        Geom_BSplineSurface anE = GeomConvert.SurfaceToBSplineSurface(new Geom_RectangularTrimmedSurface(anExtru, 0.5, 2.5, -1.0, 2.0));
        for (int i = 0; i <= 8; i++)
        {
            double aT = i / 8.0;
            gp_Pnt aOnR = aR.Value(4.0 * aT, 0.5 + 1.5 * aT);
            Assert.Equal(1.0, Math.Sqrt(Math.Pow(Math.Sqrt(aOnR.X() * aOnR.X() + aOnR.Y() * aOnR.Y()) - 5.0, 2) + aOnR.Z() * aOnR.Z()), 1e-12);
            Assert.True(anE.Value(0.5 + 2 * aT, -1 + 3 * aT).Distance(anExtru.Value(0.5 + 2 * aT, -1 + 3 * aT)) < Tol);
        }

        Assert.ThrowsAny<Standard_DomainError>(
            () => GeomConvert.SurfaceToBSplineSurface(new Geom_CylindricalSurface(new gp_Ax3(gp.XOY()), 1.0)));
        Geom_BSplineSurface aSphere = GeomConvert.SurfaceToBSplineSurface(new Geom_SphericalSurface(new gp_Ax3(gp.XOY()), 3.0));
        Assert.True(aSphere.IsUPeriodic());
        Assert.Equal(3.0, aSphere.Value(1.0, 0.3).Distance(P(0, 0)), 1e-12);
    }
}
