// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of the surfaces through sections: GeomFill_Profiler, GeomFill_Generator, GeomFill_SectionGenerator,
/// GeomFill_Line and GeomFill_AppSurf (AppBlend_AppSurf). Native results are compared by <see cref="AppSurfRecordingsTests"/>.
/// </summary>
public class GeomFill_AppSurfTests
{
    // A cubic B-spline section in the plane y = theH, with knots theKnots (multiplicity 4 at the ends, 1 inside)
    private static BSplineCurve3D Section(double theH, double[] theKnots, double theWave)
    {
        int aNbK = theKnots.Length;
        int aNbP = aNbK + 2;
        NCollection_Array1<Point3D> aPoles = new(1, aNbP);
        for (int i = 1; i <= aNbP; i++)
        {
            aPoles[i] = new Point3D(i - 1.0, theH, theWave * Math.Sin(i + theH));
        }
        NCollection_Array1<double> aKnots = new(1, aNbK);
        NCollection_Array1<int> aMults = new(1, aNbK);
        for (int i = 1; i <= aNbK; i++)
        {
            aKnots[i] = theKnots[i - 1];
            aMults[i] = (i == 1 || i == aNbK) ? 4 : 1;
        }
        return new BSplineCurve3D(aPoles, aKnots, aMults, 3);
    }

    private static List<Curve3D> Sections() =>
    [
        Section(0.0, [0.0, 0.5, 1.0], 0.5),
        Section(2.0, [0.0, 0.25, 1.0], 1.0),
        Section(4.0, [0.0, 0.75, 1.0], 0.25),
        Section(7.0, [0.0, 1.0], 0.75),
    ];

    [Fact]
    public void Profiler_brings_the_curves_to_one_profile_and_keeps_their_shapes()
    {
        List<Curve3D> aCurves = Sections();
        Profiler aP = new();
        foreach (Curve3D aC in aCurves)
        {
            aP.AddCurve(aC);
        }
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aP.Degree());
        aP.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());

        Assert.Equal(3, aP.Degree());
        Assert.False(aP.IsPeriodic);
        int aNbK = aP.NbKnots();
        Assert.Equal(5, aNbK); // 0, 0.25, 0.5, 0.75, 1
        NCollection_Array1<double> aK = new(1, aNbK);
        NCollection_Array1<int> aM = new(1, aNbK);
        aP.KnotsAndMults(aK, aM);
        Assert.Equal([0.0, 0.25, 0.5, 0.75, 1.0], aK.AsSpan().ToArray());
        for (int c = 1; c <= aCurves.Count; c++)
        {
            BSplineCurve3D aU = (BSplineCurve3D)aP.Curve(c);
            Assert.Equal(aP.NbPoles(), aU.PoleCount);
            for (int s = 0; s <= 10; s++)
            {
                double t = s / 10.0;
                Assert.True(aU.Value(t).DistanceTo(aCurves[c - 1].Value(t)) < 1e-12);
            }
        }

        // the arrays must have the profile's lengths
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => aP.Poles(1, new NCollection_Array1<Point3D>(1, 2)));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => aP.Poles(5, new NCollection_Array1<Point3D>(1, aP.NbPoles())));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => aP.KnotsAndMults(new NCollection_Array1<double>(1, 2), aM));
    }

    [Fact]
    public void Profiler_never_changes_the_callers_curves_and_hands_out_frozen_ones()
    {
        List<Curve3D> aCurves = Sections();
        foreach (Curve3D aC in aCurves)
        {
            aC.Freeze();
        }
        Profiler aP = new();
        foreach (Curve3D aC in aCurves)
        {
            aP.AddCurve(aC);
        }
        // handed out before Perform: frozen; Perform then changes copies (decision 844)
        Curve3D aBefore = aP.Curve(1);
        Assert.True(aBefore.IsFrozen);
        int aNbPolesBefore = ((BSplineCurve3D)aBefore).PoleCount;
        aP.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.Equal(aNbPolesBefore, ((BSplineCurve3D)aBefore).PoleCount);
        Assert.NotSame(aBefore, aP.Curve(1));
        Assert.True(aP.Curve(1).IsFrozen);
        Assert.False(aP.InternalCurve(2).IsFrozen);
        Assert.Equal(4, ((BSplineCurve3D)aCurves[3]).PoleCount);
        Assert.NotSame(aCurves[0], aP.Curve(1));
    }

    /// <summary>
    /// Periodic sections with different knots: UnifyByInsertingAllKnots inserts with Add = false, and
    /// BSplCLib::PrepareInsertKnots mispredicts the pole count of periodic curves, so BSplCLib::InsertKnots wrote past
    /// its arrays natively and threw Standard_RangeError in the port (occt-issues.md, unit 5L2's GeomFill_Profiler
    /// rows; Kerf unit 10K1, decision 1439). The profile is now made of the sections' non-periodic forms: one degree,
    /// one knot vector, one number of poles, and each curve the section it came from.
    /// </summary>
    [Fact]
    public void Profiler_on_periodic_sections_with_different_knots_unifies_them()
    {
        static BSplineCurve3D Loop(double theH, int theNbPoles, int theDeg)
        {
            NCollection_Array1<Point3D> aPoles = new(1, theNbPoles);
            for (int i = 1; i <= theNbPoles; i++)
            {
                double t = 2 * Math.PI * (i - 1) / theNbPoles;
                aPoles[i] = new Point3D(Math.Cos(t), theH, Math.Sin(t));
            }
            NCollection_Array1<double> aKnots = new(1, theNbPoles + 1);
            NCollection_Array1<int> aMults = new(1, theNbPoles + 1);
            for (int i = 1; i <= theNbPoles + 1; i++)
            {
                aKnots[i] = i - 1;
                aMults[i] = 1;
            }
            return new BSplineCurve3D(aPoles, aKnots, aMults, theDeg, true);
        }

        BSplineCurve3D[] aLoops = [Loop(0.0, 5, 4), Loop(3.0, 6, 1), Loop(7.0, 7, 1)];
        Profiler aP = new();
        foreach (BSplineCurve3D aLoop in aLoops)
        {
            aP.AddCurve(aLoop);
        }
        Assert.True(aP.IsPeriodic);
        aP.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        Assert.False(aP.IsPeriodic);
        for (int c = 1; c <= aLoops.Length; c++)
        {
            BSplineCurve3D aU = (BSplineCurve3D)aP.Curve(c);
            Assert.Equal(aP.NbPoles(), aU.PoleCount);
            Assert.Equal(aP.Degree(), aU.Degree);
            Assert.Equal(aP.NbKnots(), aU.KnotCount);
            // the same closed curve, reparametrised linearly onto the profile's range
            BSplineCurve3D anOriginal = aLoops[c - 1];
            for (int s = 0; s <= 20; s++)
            {
                double t = s / 20.0;
                Point3D aOnU = aU.Value(aU.FirstParameter + t * (aU.LastParameter - aU.FirstParameter));
                Point3D aOnOriginal = anOriginal.Value(anOriginal.FirstParameter + t * (anOriginal.LastParameter - anOriginal.FirstParameter));
                Assert.True(aOnU.DistanceTo(aOnOriginal) < 1e-9, $"curve {c} at {t}");
            }
        }
    }

    [Fact]
    public void Generator_makes_a_ruled_surface_through_the_sections()
    {
        List<Curve3D> aCurves = Sections();
        SurfaceGenerator aGen = new();
        foreach (Curve3D aC in aCurves)
        {
            aGen.AddCurve(aC);
        }
        Assert.Null(aGen.Surface());
        aGen.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        BSplineSurface aS = (BSplineSurface)aGen.Surface()!;
        Assert.True(aS.IsFrozen);
        Assert.Same(aS, aGen.Surface());
        Assert.Equal(3, aS.UDegree);
        Assert.Equal(1, aS.VDegree);
        Assert.Equal(aCurves.Count, aS.VPoleCount);
        for (int j = 0; j < aCurves.Count; j++)
        {
            Assert.Equal(j, aS.VKnot(j + 1));
            for (int s = 0; s <= 8; s++)
            {
                double u = s / 8.0;
                Assert.True(aS.Value(u, j).DistanceTo(aCurves[j].Value(u)) < 1e-12);
            }
        }
        // between two sections the surface is the segment joining them
        Point3D aMid = aS.Value(0.3, 1.5);
        Point3D aA = aCurves[1].Value(0.3), aB = aCurves[2].Value(0.3);
        Assert.True(aMid.DistanceTo(new Point3D((aA.XYZ() + aB.XYZ()) * 0.5)) < 1e-12);
    }

    [Fact]
    public void SectionGenerator_gives_the_sections_and_their_parameters()
    {
        List<Curve3D> aCurves = Sections();
        SectionGenerator aSec = new();
        foreach (Curve3D aC in aCurves)
        {
            aSec.AddCurve(aC);
        }
        aSec.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        aSec.GetShape(out int aNbPoles, out int aNbKnots, out int aDegree, out int aNbPoles2d);
        Assert.Equal((aSec.NbPoles(), aSec.NbKnots(), 3, 0), (aNbPoles, aNbKnots, aDegree, aNbPoles2d));

        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        NCollection_Array1<Vector3D> aDPoles = new(1, aNbPoles);
        NCollection_Array1<Point2D> aPoles2d = new(1, 1);
        NCollection_Array1<Vector2D> aDPoles2d = new(1, 1);
        NCollection_Array1<double> aW = new(1, aNbPoles), aDW = new(1, aNbPoles);
        Assert.False(aSec.Section(2, aPoles, aDPoles, aPoles2d, aDPoles2d, aW, aDW));
        NCollection_Array1<Point3D> aExpected = new(1, aNbPoles);
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
        EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.RangeErrorException>(() => aSec.SetParam(aZero));
    }

    [Fact]
    public void Line_points_are_their_indices()
    {
        SurfaceLine aLine = new(5);
        Assert.Equal(5, aLine.NbPoints());
        Assert.Equal(3, aLine.Point(3));
        Assert.Equal(0, new SurfaceLine().NbPoints());
    }

    private static (SectionGenerator, SurfaceLine, List<Curve3D>) Approximation()
    {
        List<Curve3D> aCurves = Sections();
        SectionGenerator aSec = new();
        foreach (Curve3D aC in aCurves)
        {
            aSec.AddCurve(aC);
        }
        aSec.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());
        NCollection_HArray1<double> aParams = new(1, aCurves.Count);
        double[] aV = [0.0, 2.0, 4.0, 7.0];
        for (int i = 1; i <= aCurves.Count; i++)
        {
            aParams.SetValue(i, aV[i - 1]);
        }
        aSec.SetParam(aParams);
        return (aSec, new SurfaceLine(aCurves.Count), aCurves);
    }

    private static BSplineSurface Surface(SurfaceAppSurf theApp) =>
        new(theApp.SurfPoles(), theApp.SurfWeights(), theApp.SurfUKnots(), theApp.SurfVKnots(), theApp.SurfUMults(),
            theApp.SurfVMults(), theApp.UDegree(), theApp.VDegree());

    [Theory]
    [InlineData(0)]
    [InlineData(1)]
    [InlineData(2)]
    [InlineData(3)]
    public void AppSurf_passes_through_the_sections_at_their_parameters(int theWay)
    {
        (SectionGenerator aSec, SurfaceLine aLine, List<Curve3D> aCurves) = Approximation();
        SurfaceAppSurf anApp = new(2, 8, 1e-7, 1e-7, theWay == 2 ? 2 : 0, knownParameters: true);
        anApp.SetContinuity(C2);
        switch (theWay)
        {
            case 0: anApp.Perform(aLine, aSec); break;
            case 1: anApp.Perform(aLine, aSec, true); break;
            case 2: anApp.PerformSmoothing(aLine, aSec); break;
            default: anApp.Perform(aLine, aSec, 3); break;
        }
        Assert.True(anApp.IsDone);
        anApp.SurfShape(out int ud, out int vd, out int nup, out int nvp, out int nuk, out int nvk);
        Assert.Equal((3, aSec.NbPoles(), aSec.NbKnots()), (ud, nup, nuk));
        Assert.Equal((anApp.VDegree(), anApp.SurfVKnots().Length(), anApp.SurfPoles().RowLength()), (vd, nvk, nvp));
        Assert.Equal(0, anApp.Curves2dCount);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => anApp.Curves2dDegree());

        BSplineSurface aS = Surface(anApp);
        double[] aV = [0.0, 2.0, 4.0, 7.0];
        if (theWay != 2)
        {
            // known parameters: V runs over the sections' parameters
            Assert.Equal((0.0, 7.0), (aS.VKnot(1), aS.VKnot(aS.VKnotCount)));
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
            double v = theWay == 1 ? aV[j] : (j == 0 ? aS.VKnot(1) : aS.VKnot(aS.VKnotCount));
            for (int s = 0; s <= 8; s++)
            {
                double u = s / 8.0;
                Assert.True(aS.Value(u, v).DistanceTo(aCurves[j].Value(u)) < aTol, $"section {j} at {u}");
            }
        }

        // the copy into the caller's arrays
        NCollection_Array2<Point3D> aP = new(1, nup, 1, nvp);
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
        SurfaceAppSurf anApp = new();
        Assert.False(anApp.IsDone);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anApp.UDegree());
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anApp.SurfPoles());
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anApp.TolReached(out _, out _));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => anApp.SetCriteriumWeight(1, -1, 1));
        anApp.SetCriteriumWeight(0.1, 0.2, 0.7);
        anApp.CriteriumWeight(out double w1, out double w2, out double w3);
        Assert.Equal((0.1, 0.2, 0.7), (w1, w2, w3));
        anApp.SetContinuity(C1);
        anApp.SetParType(global::Kerf.Geometry.ParametrizationType.Centripetal);
        anApp.Init(3, 8, 1e-5, 1e-5, 0);
        // Init restores the defaults
        Assert.Equal(C2, anApp.Continuity());
        Assert.Equal(global::Kerf.Geometry.ParametrizationType.ChordLength, anApp.ParType());
        anApp.CriteriumWeight(out w1, out w2, out w3);
        Assert.Equal((0.4, 0.2, 0.4), (w1, w2, w3));

        // no line: not done
        (SectionGenerator aSec, _, _) = Approximation();
        anApp.Perform(null, aSec);
        Assert.False(anApp.IsDone);
    }

    [Fact]
    public void AppSurf_leaves_the_sections_unchanged_and_approximates_the_same_twice()
    {
        (SectionGenerator aSec, SurfaceLine aLine, List<Curve3D> aCurves) = Approximation();
        foreach (Curve3D aC in aCurves)
        {
            aC.Freeze();
        }
        SurfaceAppSurf anApp = new(3, 8, 1e-6, 1e-6, 0);
        anApp.Perform(aLine, aSec, true);
        NCollection_Array2<Point3D> aFirst = new(anApp.SurfPoles());
        anApp.Perform(aLine, aSec, true);
        Assert.Equal(aFirst.AsSpan().ToArray(), anApp.SurfPoles().AsSpan().ToArray());
    }
}
