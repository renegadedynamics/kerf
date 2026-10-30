// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the surfaces through sections: GeomFill_Profiler, GeomFill_Generator, GeomFill_SectionGenerator,
/// GeomFill_Line and GeomFill_AppSurf (AppBlend_AppSurf). Native results are compared by <see cref="AppSurfRecordingsTests"/>.
/// </summary>
public class GeomFill_AppSurfTests
{
    // A cubic B-spline section in the plane y = theH, with knots theKnots (multiplicity 4 at the ends, 1 inside)
    private static Geom_BSplineCurve Section(double theH, double[] theKnots, double theWave)
    {
        int aNbK = theKnots.Length;
        int aNbP = aNbK + 2;
        NCollection_Array1<gp_Pnt> aPoles = new(1, aNbP);
        for (int i = 1; i <= aNbP; i++)
        {
            aPoles[i] = new gp_Pnt(i - 1.0, theH, theWave * Math.Sin(i + theH));
        }
        NCollection_Array1<double> aKnots = new(1, aNbK);
        NCollection_Array1<int> aMults = new(1, aNbK);
        for (int i = 1; i <= aNbK; i++)
        {
            aKnots[i] = theKnots[i - 1];
            aMults[i] = (i == 1 || i == aNbK) ? 4 : 1;
        }
        return new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    private static List<Geom_Curve> Sections() =>
    [
        Section(0.0, [0.0, 0.5, 1.0], 0.5),
        Section(2.0, [0.0, 0.25, 1.0], 1.0),
        Section(4.0, [0.0, 0.75, 1.0], 0.25),
        Section(7.0, [0.0, 1.0], 0.75),
    ];

    [Fact]
    public void Profiler_brings_the_curves_to_one_profile_and_keeps_their_shapes()
    {
        List<Geom_Curve> aCurves = Sections();
        GeomFill_Profiler aP = new();
        foreach (Geom_Curve aC in aCurves)
        {
            aP.AddCurve(aC);
        }
        Assert.Throws<StdFail_NotDone>(() => aP.Degree());
        aP.Perform(Precision.PConfusion());

        Assert.Equal(3, aP.Degree());
        Assert.False(aP.IsPeriodic());
        int aNbK = aP.NbKnots();
        Assert.Equal(5, aNbK); // 0, 0.25, 0.5, 0.75, 1
        NCollection_Array1<double> aK = new(1, aNbK);
        NCollection_Array1<int> aM = new(1, aNbK);
        aP.KnotsAndMults(aK, aM);
        Assert.Equal([0.0, 0.25, 0.5, 0.75, 1.0], aK.AsSpan().ToArray());
        for (int c = 1; c <= aCurves.Count; c++)
        {
            Geom_BSplineCurve aU = (Geom_BSplineCurve)aP.Curve(c);
            Assert.Equal(aP.NbPoles(), aU.NbPoles());
            for (int s = 0; s <= 10; s++)
            {
                double t = s / 10.0;
                Assert.True(aU.Value(t).Distance(aCurves[c - 1].Value(t)) < 1e-12);
            }
        }

        // the arrays must have the profile's lengths
        Assert.Throws<Standard_DomainError>(() => aP.Poles(1, new NCollection_Array1<gp_Pnt>(1, 2)));
        Assert.Throws<Standard_DomainError>(() => aP.Poles(5, new NCollection_Array1<gp_Pnt>(1, aP.NbPoles())));
        Assert.Throws<Standard_DomainError>(() => aP.KnotsAndMults(new NCollection_Array1<double>(1, 2), aM));
    }

    [Fact]
    public void Profiler_never_changes_the_callers_curves_and_hands_out_frozen_ones()
    {
        List<Geom_Curve> aCurves = Sections();
        foreach (Geom_Curve aC in aCurves)
        {
            aC.Freeze();
        }
        GeomFill_Profiler aP = new();
        foreach (Geom_Curve aC in aCurves)
        {
            aP.AddCurve(aC);
        }
        // handed out before Perform: frozen; Perform then changes copies (decision 844)
        Geom_Curve aBefore = aP.Curve(1);
        Assert.True(aBefore.IsFrozen());
        int aNbPolesBefore = ((Geom_BSplineCurve)aBefore).NbPoles();
        aP.Perform(Precision.PConfusion());
        Assert.Equal(aNbPolesBefore, ((Geom_BSplineCurve)aBefore).NbPoles());
        Assert.NotSame(aBefore, aP.Curve(1));
        Assert.True(aP.Curve(1).IsFrozen());
        Assert.False(aP.InternalCurve(2).IsFrozen());
        Assert.Equal(4, ((Geom_BSplineCurve)aCurves[3]).NbPoles());
        Assert.NotSame(aCurves[0], aP.Curve(1));
    }

    [Fact]
    public void Profiler_on_periodic_sections_throws_where_native_corrupts_its_heap()
    {
        // OCCT: UnifyByInsertingAllKnots inserts with Add = false, and BSplCLib::PrepareInsertKnots mispredicts the
        // pole count of periodic curves; BSplCLib::InsertKnots then writes past its arrays natively (occt-issues.md).
        static Geom_BSplineCurve Loop(double theH, int theNbPoles, int theDeg)
        {
            NCollection_Array1<gp_Pnt> aPoles = new(1, theNbPoles);
            for (int i = 1; i <= theNbPoles; i++)
            {
                double t = 2 * Math.PI * (i - 1) / theNbPoles;
                aPoles[i] = new gp_Pnt(Math.Cos(t), theH, Math.Sin(t));
            }
            NCollection_Array1<double> aKnots = new(1, theNbPoles + 1);
            NCollection_Array1<int> aMults = new(1, theNbPoles + 1);
            for (int i = 1; i <= theNbPoles + 1; i++)
            {
                aKnots[i] = i - 1;
                aMults[i] = 1;
            }
            return new Geom_BSplineCurve(aPoles, aKnots, aMults, theDeg, true);
        }

        GeomFill_Profiler aP = new();
        aP.AddCurve(Loop(0.0, 5, 4));
        aP.AddCurve(Loop(3.0, 6, 1));
        aP.AddCurve(Loop(7.0, 7, 1));
        Assert.True(aP.IsPeriodic());
        Assert.ThrowsAny<Standard_RangeError>(() => aP.Perform(Precision.PConfusion()));
    }

    [Fact]
    public void Generator_makes_a_ruled_surface_through_the_sections()
    {
        List<Geom_Curve> aCurves = Sections();
        GeomFill_Generator aGen = new();
        foreach (Geom_Curve aC in aCurves)
        {
            aGen.AddCurve(aC);
        }
        Assert.Null(aGen.Surface());
        aGen.Perform(Precision.PConfusion());
        Geom_BSplineSurface aS = (Geom_BSplineSurface)aGen.Surface()!;
        Assert.True(aS.IsFrozen());
        Assert.Same(aS, aGen.Surface());
        Assert.Equal(3, aS.UDegree());
        Assert.Equal(1, aS.VDegree());
        Assert.Equal(aCurves.Count, aS.NbVPoles());
        for (int j = 0; j < aCurves.Count; j++)
        {
            Assert.Equal(j, aS.VKnot(j + 1));
            for (int s = 0; s <= 8; s++)
            {
                double u = s / 8.0;
                Assert.True(aS.Value(u, j).Distance(aCurves[j].Value(u)) < 1e-12);
            }
        }
        // between two sections the surface is the segment joining them
        gp_Pnt aMid = aS.Value(0.3, 1.5);
        gp_Pnt aA = aCurves[1].Value(0.3), aB = aCurves[2].Value(0.3);
        Assert.True(aMid.Distance(new gp_Pnt((aA.XYZ() + aB.XYZ()) * 0.5)) < 1e-12);
    }

    [Fact]
    public void SectionGenerator_gives_the_sections_and_their_parameters()
    {
        List<Geom_Curve> aCurves = Sections();
        GeomFill_SectionGenerator aSec = new();
        foreach (Geom_Curve aC in aCurves)
        {
            aSec.AddCurve(aC);
        }
        aSec.Perform(Precision.PConfusion());
        aSec.GetShape(out int aNbPoles, out int aNbKnots, out int aDegree, out int aNbPoles2d);
        Assert.Equal((aSec.NbPoles(), aSec.NbKnots(), 3, 0), (aNbPoles, aNbKnots, aDegree, aNbPoles2d));

        NCollection_Array1<gp_Pnt> aPoles = new(1, aNbPoles);
        NCollection_Array1<gp_Vec> aDPoles = new(1, aNbPoles);
        NCollection_Array1<gp_Pnt2d> aPoles2d = new(1, 1);
        NCollection_Array1<gp_Vec2d> aDPoles2d = new(1, 1);
        NCollection_Array1<double> aW = new(1, aNbPoles), aDW = new(1, aNbPoles);
        Assert.False(aSec.Section(2, aPoles, aDPoles, aPoles2d, aDPoles2d, aW, aDW));
        NCollection_Array1<gp_Pnt> aExpected = new(1, aNbPoles);
        aSec.Poles(2, aExpected);
        Assert.Equal(aExpected.AsSpan().ToArray(), aPoles.AsSpan().ToArray());
        Assert.All(aW.AsSpan().ToArray(), w => Assert.Equal(1.0, w));

        NCollection_HArray1<double> aParams = new(1, 4);
        for (int i = 1; i <= 4; i++)
        {
            aParams.SetValue(i, 10.0 * i);
        }
        aSec.SetParam(aParams);
        Assert.Equal(30.0, aSec.Parameter(3));

        // OCCT keeps the caller's array and shifts it into place: a 0-based array loses its first value
        NCollection_HArray1<double> aZero = new(0, 3);
        for (int i = 0; i <= 3; i++)
        {
            aZero.SetValue(i, 10.0 * i);
        }
        Assert.ThrowsAny<Standard_RangeError>(() => aSec.SetParam(aZero));
    }

    [Fact]
    public void Line_points_are_their_indices()
    {
        GeomFill_Line aLine = new(5);
        Assert.Equal(5, aLine.NbPoints());
        Assert.Equal(3, aLine.Point(3));
        Assert.Equal(0, new GeomFill_Line().NbPoints());
    }

    private static (GeomFill_SectionGenerator, GeomFill_Line, List<Geom_Curve>) Approximation()
    {
        List<Geom_Curve> aCurves = Sections();
        GeomFill_SectionGenerator aSec = new();
        foreach (Geom_Curve aC in aCurves)
        {
            aSec.AddCurve(aC);
        }
        aSec.Perform(Precision.PConfusion());
        NCollection_HArray1<double> aParams = new(1, aCurves.Count);
        double[] aV = [0.0, 2.0, 4.0, 7.0];
        for (int i = 1; i <= aCurves.Count; i++)
        {
            aParams.SetValue(i, aV[i - 1]);
        }
        aSec.SetParam(aParams);
        return (aSec, new GeomFill_Line(aCurves.Count), aCurves);
    }

    private static Geom_BSplineSurface Surface(GeomFill_AppSurf theApp) =>
        new(theApp.SurfPoles(), theApp.SurfWeights(), theApp.SurfUKnots(), theApp.SurfVKnots(), theApp.SurfUMults(),
            theApp.SurfVMults(), theApp.UDegree(), theApp.VDegree());

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void AppSurf_passes_through_the_sections_at_their_parameters(int theWay)
    {
        (GeomFill_SectionGenerator aSec, GeomFill_Line aLine, List<Geom_Curve> aCurves) = Approximation();
        GeomFill_AppSurf anApp = new(2, 8, 1e-7, 1e-7, theWay == 2 ? 2 : 0, KnownParameters: true);
        anApp.SetContinuity(GeomAbs_C2);
        switch (theWay)
        {
            case 0: anApp.Perform(aLine, aSec); break;
            case 1: anApp.Perform(aLine, aSec, true); break;
            case 2: anApp.PerformSmoothing(aLine, aSec); break;
            default: anApp.Perform(aLine, aSec, 3); break;
        }
        Assert.True(anApp.IsDone());
        anApp.SurfShape(out int ud, out int vd, out int nup, out int nvp, out int nuk, out int nvk);
        Assert.Equal((3, aSec.NbPoles(), aSec.NbKnots()), (ud, nup, nuk));
        Assert.Equal((anApp.VDegree(), anApp.SurfVKnots().Length(), anApp.SurfPoles().RowLength()), (vd, nvk, nvp));
        Assert.Equal(0, anApp.NbCurves2d());
        Assert.Throws<Standard_DomainError>(() => anApp.Curves2dDegree());

        Geom_BSplineSurface aS = Surface(anApp);
        double[] aV = [0.0, 2.0, 4.0, 7.0];
        if (theWay != 2)
        {
            // known parameters: V runs over the sections' parameters
            Assert.Equal((0.0, 7.0), (aS.VKnot(1), aS.VKnot(aS.NbVKnots())));
        }
        double aTol = theWay == 2 ? 1e-3 : 1e-6;
        for (int j = 0; j < aCurves.Count; j++)
        {
            // The B-spline least squares with known parameters passes through every section at its parameter.
            // The others pass through the end sections at the ends of V (the Bezier pieces' knots are not the
            // sections' parameters; the smoothing keeps its own parameters).
            if (theWay != 1 && j != 0 && j != aCurves.Count - 1)
            {
                continue;
            }
            double v = theWay == 1 ? aV[j] : (j == 0 ? aS.VKnot(1) : aS.VKnot(aS.NbVKnots()));
            for (int s = 0; s <= 8; s++)
            {
                double u = s / 8.0;
                Assert.True(aS.Value(u, v).Distance(aCurves[j].Value(u)) < aTol, $"section {j} at {u}");
            }
        }

        // the copy into the caller's arrays
        NCollection_Array2<gp_Pnt> aP = new(1, nup, 1, nvp);
        NCollection_Array2<double> aW = new(1, nup, 1, nvp);
        NCollection_Array1<double> aUK = new(1, nuk), aVK = new(1, nvk);
        NCollection_Array1<int> aUM = new(1, nuk), aVM = new(1, nvk);
        anApp.Surface(aP, aW, aUK, aVK, aUM, aVM);
        Assert.Equal(anApp.SurfPoles()[1, 1], aP[1, 1]);
        Assert.Equal(anApp.SurfVKnots()[nvk], aVK[nvk]);
    }

    [Fact]
    public void AppSurf_checks_its_state_and_arguments()
    {
        GeomFill_AppSurf anApp = new();
        Assert.False(anApp.IsDone());
        Assert.Throws<StdFail_NotDone>(() => anApp.UDegree());
        Assert.Throws<StdFail_NotDone>(() => anApp.SurfPoles());
        Assert.Throws<StdFail_NotDone>(() => anApp.TolReached(out _, out _));
        Assert.Throws<Standard_DomainError>(() => anApp.SetCriteriumWeight(1, -1, 1));
        anApp.SetCriteriumWeight(0.1, 0.2, 0.7);
        anApp.CriteriumWeight(out double w1, out double w2, out double w3);
        Assert.Equal((0.1, 0.2, 0.7), (w1, w2, w3));
        anApp.SetContinuity(GeomAbs_C1);
        anApp.SetParType(Approx_ParametrizationType.Approx_Centripetal);
        anApp.Init(3, 8, 1e-5, 1e-5, 0);
        // Init restores the defaults
        Assert.Equal(GeomAbs_C2, anApp.Continuity());
        Assert.Equal(Approx_ParametrizationType.Approx_ChordLength, anApp.ParType());
        anApp.CriteriumWeight(out w1, out w2, out w3);
        Assert.Equal((0.4, 0.2, 0.4), (w1, w2, w3));

        // no line: not done
        (GeomFill_SectionGenerator aSec, _, _) = Approximation();
        anApp.Perform(null, aSec);
        Assert.False(anApp.IsDone());
    }

    [Fact]
    public void AppSurf_leaves_the_sections_unchanged_and_approximates_the_same_twice()
    {
        (GeomFill_SectionGenerator aSec, GeomFill_Line aLine, List<Geom_Curve> aCurves) = Approximation();
        foreach (Geom_Curve aC in aCurves)
        {
            aC.Freeze();
        }
        GeomFill_AppSurf anApp = new(3, 8, 1e-6, 1e-6, 0);
        anApp.Perform(aLine, aSec, true);
        NCollection_Array2<gp_Pnt> aFirst = new(anApp.SurfPoles());
        anApp.Perform(aLine, aSec, true);
        Assert.Equal(aFirst.AsSpan().ToArray(), anApp.SurfPoles().AsSpan().ToArray());
    }
}
