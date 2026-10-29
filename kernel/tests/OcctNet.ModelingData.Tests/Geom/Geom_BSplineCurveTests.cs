// Port of: src/ModelingData/TKG3d/GTests/Geom_BSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_BSplKnotDistribution;

namespace OcctNet.ModelingData.Tests;

public class Geom_BSplineCurve_Test
{
    private readonly Geom_BSplineCurve myOriginalCurve;

    public Geom_BSplineCurve_Test()
    {
        // Create a simple BSpline curve for testing
        NCollection_Array1<gp_Pnt> aPoles = Pnts(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 1, 0), new gp_Pnt(3, 0, 0));
        NCollection_Array1<double> aKnots = Reals(0.0, 1.0);
        NCollection_Array1<int> aMults = Ints(4, 4);

        myOriginalCurve = new Geom_BSplineCurve(aPoles, aKnots, aMults, 3);
    }

    // OCCT's tests fill 1-based arrays element by element; these build the same arrays.
    private static NCollection_Array1<gp_Pnt> Pnts(params gp_Pnt[] theValues) => NCollection_Array1<gp_Pnt>.From(theValues);

    private static NCollection_Array1<double> Reals(params double[] theValues) => NCollection_Array1<double>.From(theValues);

    private static NCollection_Array1<int> Ints(params int[] theValues) => NCollection_Array1<int>.From(theValues);

    private static Geom_BSplineCurve RationalQuadratic(double w2)
    {
        return new Geom_BSplineCurve(Pnts(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 0, 0)),
                                     Reals(1.0, w2, 1.0),
                                     Reals(0.0, 1.0),
                                     Ints(3, 3),
                                     2);
    }

    private static NCollection_Array1<gp_Pnt> PentagonPoles() =>
        Pnts(new gp_Pnt(1, 0, 0),
             new gp_Pnt(0.309, 0.951, 0),
             new gp_Pnt(-0.809, 0.588, 0),
             new gp_Pnt(-0.809, -0.588, 0),
             new gp_Pnt(0.309, -0.951, 0));

    private static NCollection_Array1<double> PentagonKnots()
    {
        NCollection_Array1<double> aKnots = new(1, 6);
        for (int i = 1; i <= 6; ++i)
        {
            aKnots[i] = (i - 1) * 0.2;
        }
        return aKnots;
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        // Test copy constructor
        Geom_BSplineCurve aCopiedCurve = new(myOriginalCurve);

        // Verify basic properties are identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.NbPoles(), aCopiedCurve.NbPoles());
        Assert.Equal(myOriginalCurve.NbKnots(), aCopiedCurve.NbKnots());
        Assert.Equal(myOriginalCurve.IsPeriodic(), aCopiedCurve.IsPeriodic());
        Assert.Equal(myOriginalCurve.IsRational(), aCopiedCurve.IsRational());
    }

    [Fact]
    public void CopyConstructorPoles()
    {
        Geom_BSplineCurve aCopiedCurve = new(myOriginalCurve);

        // Verify all poles are identical
        for (int i = 1; i <= myOriginalCurve.NbPoles(); ++i)
        {
            gp_Pnt anOrigPole = myOriginalCurve.Pole(i);
            gp_Pnt aCopyPole = aCopiedCurve.Pole(i);
            Assert.True(anOrigPole.IsEqual(aCopyPole, 1e-10));
        }
    }

    [Fact]
    public void CopyConstructorKnots()
    {
        Geom_BSplineCurve aCopiedCurve = new(myOriginalCurve);

        // Verify all knots are identical
        for (int i = 1; i <= myOriginalCurve.NbKnots(); ++i)
        {
            Assert.Equal(myOriginalCurve.Knot(i), aCopiedCurve.Knot(i));
            Assert.Equal(myOriginalCurve.Multiplicity(i), aCopiedCurve.Multiplicity(i));
        }
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        // Test that Copy() method uses the optimized copy constructor
        Geom_BSplineCurve? aCopiedCurve = myOriginalCurve.Copy() as Geom_BSplineCurve;
        Assert.NotNull(aCopiedCurve);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.NbPoles(), aCopiedCurve.NbPoles());

        // Test evaluation at several points
        for (double u = 0.0; u <= 1.0; u += 0.25)
        {
            gp_Pnt anOrigPnt = myOriginalCurve.Value(u);
            gp_Pnt aCopyPnt = aCopiedCurve.Value(u);
            Assert.True(anOrigPnt.IsEqual(aCopyPnt, 1e-10));
        }
    }

    [Fact]
    public void RationalCurveCopyConstructor()
    {
        // Create a rational BSpline curve
        Geom_BSplineCurve aRationalCurve = RationalQuadratic(2.0);
        Geom_BSplineCurve aCopiedRational = new(aRationalCurve);

        Assert.True(aCopiedRational.IsRational());

        // Verify weights are copied correctly
        for (int i = 1; i <= aRationalCurve.NbPoles(); ++i)
        {
            Assert.Equal(aRationalCurve.Weight(i), aCopiedRational.Weight(i));
        }
    }

    [Fact]
    public void Evaluation_D0()
    {
        myOriginalCurve.D0(0.0, out gp_Pnt aPnt);
        Assert.True(aPnt.IsEqual(new gp_Pnt(0, 0, 0), 1e-10));

        myOriginalCurve.D0(1.0, out aPnt);
        Assert.True(aPnt.IsEqual(new gp_Pnt(3, 0, 0), 1e-10));
    }

    [Fact]
    public void Evaluation_D1()
    {
        myOriginalCurve.D1(0.5, out _, out gp_Vec aV1);
        Assert.True(aV1.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_D2()
    {
        myOriginalCurve.D2(0.5, out _, out gp_Vec aV1, out _);
        Assert.True(aV1.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_D3()
    {
        myOriginalCurve.D3(0.5, out _, out _, out _, out gp_Vec aV3);
        // D3 of a degree 3 curve is constant
        myOriginalCurve.D3(0.25, out _, out _, out _, out gp_Vec aV3b);
        Assert.Equal(aV3b.X(), aV3.X(), 1e-8);
        Assert.Equal(aV3b.Y(), aV3.Y(), 1e-8);
        Assert.Equal(aV3b.Z(), aV3.Z(), 1e-8);
    }

    [Fact]
    public void Evaluation_DN()
    {
        gp_Vec aDN = myOriginalCurve.DN(0.5, 1);
        Assert.True(aDN.Magnitude() > 0.0);

        // DN(4) for a degree 3 curve should be zero
        gp_Vec aDN4 = myOriginalCurve.DN(0.5, 4);
        Assert.Equal(0.0, aDN4.Magnitude(), 1e-10);
    }

    [Fact]
    public void StartEndPoints()
    {
        gp_Pnt aStart = myOriginalCurve.StartPoint();
        gp_Pnt anEnd = myOriginalCurve.EndPoint();
        Assert.True(aStart.IsEqual(new gp_Pnt(0, 0, 0), 1e-10));
        Assert.True(anEnd.IsEqual(new gp_Pnt(3, 0, 0), 1e-10));
    }

    [Fact]
    public void SetPole()
    {
        gp_Pnt aNewPole = new(1, 2, 3);
        myOriginalCurve.SetPole(2, aNewPole);
        Assert.True(myOriginalCurve.Pole(2).IsEqual(aNewPole, 1e-10));
        // End points should not change
        Assert.True(myOriginalCurve.StartPoint().IsEqual(new gp_Pnt(0, 0, 0), 1e-10));
    }

    [Fact]
    public void SetWeight()
    {
        // Create rational curve
        Geom_BSplineCurve aCurve = RationalQuadratic(1.0);
        gp_Pnt aMidBefore = aCurve.Value(0.5);

        aCurve.SetWeight(2, 5.0);
        Assert.Equal(5.0, aCurve.Weight(2));
        Assert.True(aCurve.IsRational());

        gp_Pnt aMidAfter = aCurve.Value(0.5);
        Assert.False(aMidBefore.IsEqual(aMidAfter, 1e-6));
    }

    [Fact]
    public void IncreaseDegree()
    {
        gp_Pnt aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.IncreaseDegree(5);
        Assert.Equal(5, myOriginalCurve.Degree());
        gp_Pnt aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void InsertKnot()
    {
        gp_Pnt aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.InsertKnot(0.5);
        Assert.Equal(3, myOriginalCurve.NbKnots());
        gp_Pnt aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void RemoveKnot()
    {
        myOriginalCurve.InsertKnot(0.5);
        Assert.Equal(3, myOriginalCurve.NbKnots());

        gp_Pnt aValBefore = myOriginalCurve.Value(0.25);
        bool isRemoved = myOriginalCurve.RemoveKnot(2, 0, 1e-6);
        Assert.True(isRemoved);
        Assert.Equal(2, myOriginalCurve.NbKnots());
        gp_Pnt aValAfter = myOriginalCurve.Value(0.25);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-6));
    }

    [Fact]
    public void Segment()
    {
        gp_Pnt aP25 = myOriginalCurve.Value(0.25);
        gp_Pnt aP75 = myOriginalCurve.Value(0.75);
        myOriginalCurve.Segment(0.25, 0.75);
        Assert.Equal(0.25, myOriginalCurve.FirstParameter(), 1e-10);
        Assert.Equal(0.75, myOriginalCurve.LastParameter(), 1e-10);
        Assert.True(myOriginalCurve.StartPoint().IsEqual(aP25, 1e-6));
        Assert.True(myOriginalCurve.EndPoint().IsEqual(aP75, 1e-6));
    }

    [Fact]
    public void Reverse()
    {
        gp_Pnt aStart = myOriginalCurve.StartPoint();
        gp_Pnt anEnd = myOriginalCurve.EndPoint();
        myOriginalCurve.Reverse();
        Assert.True(myOriginalCurve.StartPoint().IsEqual(anEnd, 1e-10));
        Assert.True(myOriginalCurve.EndPoint().IsEqual(aStart, 1e-10));
    }

    [Fact]
    public void Resolution()
    {
        myOriginalCurve.Resolution(1.0, out double aUTol);
        Assert.True(aUTol > 0.0);
    }

    [Fact]
    public void Properties()
    {
        Assert.Equal(3, myOriginalCurve.Degree());
        Assert.False(myOriginalCurve.IsPeriodic());
        Assert.False(myOriginalCurve.IsRational());
        Assert.False(myOriginalCurve.IsClosed());
        Assert.True(myOriginalCurve.IsCN(3));
        Assert.Equal(GeomAbs_Shape.GeomAbs_CN, myOriginalCurve.Continuity());
    }

    [Fact]
    public void Transform()
    {
        gp_Trsf aTrsf = new();
        aTrsf.SetTranslation(new gp_Vec(10, 20, 30));
        gp_Pnt aPtBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.Transform(aTrsf);
        gp_Pnt aPtAfter = myOriginalCurve.Value(0.5);
        Assert.Equal(aPtBefore.X() + 10.0, aPtAfter.X(), 1e-10);
        Assert.Equal(aPtBefore.Y() + 20.0, aPtAfter.Y(), 1e-10);
        Assert.Equal(aPtBefore.Z() + 30.0, aPtAfter.Z(), 1e-10);
    }

    [Fact]
    public void PeriodicCurve()
    {
        // Periodic BSpline: NbPoles = Sum(Mults) - Mults(last)
        // 5 poles, degree 3, 6 knots of mult 1 -> NbPoles = 1+4 = 5
        NCollection_Array1<int> aMults = new(1, 6);
        aMults.Init(1);

        Geom_BSplineCurve aCurve = new(PentagonPoles(), PentagonKnots(), aMults, 3, true);
        Assert.True(aCurve.IsPeriodic());

        gp_Pnt aVal1 = aCurve.Value(0.5);
        aCurve.SetNotPeriodic();
        Assert.False(aCurve.IsPeriodic());
        gp_Pnt aVal2 = aCurve.Value(0.5);
        Assert.True(aVal1.IsEqual(aVal2, 1e-10));
    }

    [Fact]
    public void MultiKnotSpan()
    {
        // BSpline with 3 knot spans, degree 2
        // NbPoles = Mf + Ml - (Deg+1) + Sum(interior) = 3+3-3+1+1 = 5
        NCollection_Array1<gp_Pnt> aPoles = Pnts(new gp_Pnt(0, 0, 0),
                                                 new gp_Pnt(1, 2, 0),
                                                 new gp_Pnt(2, 0, 0),
                                                 new gp_Pnt(3, 2, 0),
                                                 new gp_Pnt(4, 0, 0));
        Geom_BSplineCurve aCurve = new(aPoles, Reals(0.0, 1.0, 2.0, 3.0), Ints(3, 1, 1, 3), 2);
        Assert.Equal(4, aCurve.NbKnots());
        Assert.Equal(5, aCurve.NbPoles());

        // Evaluate at knot boundaries
        aCurve.D0(0.0, out gp_Pnt aPnt);
        Assert.True(aPnt.IsEqual(new gp_Pnt(0, 0, 0), 1e-10));
        aCurve.D0(3.0, out aPnt);
        Assert.True(aPnt.IsEqual(new gp_Pnt(4, 0, 0), 1e-10));
    }

    [Fact]
    public void IncreaseMultiplicity()
    {
        // Add a knot first to have an interior knot
        myOriginalCurve.InsertKnot(0.5, 1);
        Assert.Equal(3, myOriginalCurve.NbKnots());
        Assert.Equal(1, myOriginalCurve.Multiplicity(2));

        gp_Pnt aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.IncreaseMultiplicity(2, 2);
        Assert.Equal(2, myOriginalCurve.Multiplicity(2));
        gp_Pnt aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void KnotAccess()
    {
        Assert.Equal(0.0, myOriginalCurve.Knot(1));
        Assert.Equal(1.0, myOriginalCurve.Knot(2));
        Assert.Equal(1, myOriginalCurve.FirstUKnotIndex());
        Assert.Equal(2, myOriginalCurve.LastUKnotIndex());

        // The public arrays are immutable copies indexed from 0 (OCCT: references indexed from 1)
        PackedList<double> aKnots = myOriginalCurve.Knots();
        Assert.Equal(2, aKnots.Count);

        PackedList<double> aFlatKnots = myOriginalCurve.KnotSequence();
        Assert.Equal(8, aFlatKnots.Count);

        PackedList<int> aMults = myOriginalCurve.Multiplicities();
        Assert.Equal(2, aMults.Count);
        Assert.Equal(4, aMults[0]);
        Assert.Equal(4, aMults[1]);
    }

    [Fact]
    public void PolesAccess()
    {
        PackedList<gp_Pnt> aPoles = myOriginalCurve.Poles();
        Assert.Equal(4, aPoles.Count);
        Assert.True(aPoles[0].IsEqual(new gp_Pnt(0, 0, 0), 1e-10));
    }

    [Fact]
    public void WeightsAccess_NonRational()
    {
        PackedList<double>? aWeights = myOriginalCurve.Weights();
        Assert.Null(aWeights);
        Assert.Null(myOriginalCurve.InternalWeights());
    }

    [Fact]
    public void WeightsAccess_Rational()
    {
        Geom_BSplineCurve aCurve = RationalQuadratic(2.0);
        PackedList<double>? aW = aCurve.Weights();
        Assert.NotNull(aW);
        Assert.Equal(2.0, aW[1]);
        Assert.Equal(2.0, aCurve.InternalWeights()![2]);
    }

    [Fact]
    public void LocalEvaluation()
    {
        // Insert a knot to create 2 spans
        myOriginalCurve.InsertKnot(0.5);
        Assert.Equal(3, myOriginalCurve.NbKnots());

        gp_Pnt aGlobal = myOriginalCurve.Value(0.25);
        gp_Pnt aLocal = myOriginalCurve.LocalValue(0.25, 1, 2);
        Assert.True(aGlobal.IsEqual(aLocal, 1e-10));
    }

    [Fact]
    public void SetKnot()
    {
        // Create a curve with 3 knots, degree 2
        // NbPoles = 3+3-3+1 = 4
        NCollection_Array1<gp_Pnt> aPoles = Pnts(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(3, 1, 0));
        Geom_BSplineCurve aCurve = new(aPoles, Reals(0.0, 0.5, 1.0), Ints(3, 1, 3), 2);
        aCurve.SetKnot(2, 0.6);
        Assert.Equal(0.6, aCurve.Knot(2));
    }

    [Fact]
    public void CopyIndependence()
    {
        Geom_BSplineCurve aCopy = new(myOriginalCurve);
        myOriginalCurve.SetPole(2, new gp_Pnt(10, 10, 10));
        Assert.False(aCopy.Pole(2).IsEqual(new gp_Pnt(10, 10, 10), 1e-10));
    }

    [Fact]
    public void MovePoint()
    {
        gp_Pnt aTarget = new(1.5, 0.8, 0);
        myOriginalCurve.MovePoint(0.5, aTarget, 1, myOriginalCurve.NbPoles(), out int aFirst, out int aLast);
        Assert.True(aFirst > 0);
        gp_Pnt aMoved = myOriginalCurve.Value(0.5);
        Assert.True(aMoved.IsEqual(aTarget, 1e-6));
    }

    [Fact]
    public void LocateU()
    {
        myOriginalCurve.InsertKnot(0.5);
        myOriginalCurve.LocateU(0.25, 1e-10, out int anI1, out int anI2);
        Assert.True(anI1 >= 1);
        Assert.True(anI2 <= myOriginalCurve.NbKnots());
        Assert.True(myOriginalCurve.Knot(anI1) <= 0.25 + 1e-10);
        Assert.True(myOriginalCurve.Knot(anI2) >= 0.25 - 1e-10);
    }

    [Fact]
    public void IsEqual()
    {
        Geom_BSplineCurve aCopy = new(myOriginalCurve);
        Assert.True(myOriginalCurve.IsEqual(aCopy, 1e-10));

        aCopy.SetPole(2, new gp_Pnt(10, 10, 10));
        Assert.False(myOriginalCurve.IsEqual(aCopy, 1e-10));
    }

    [Fact]
    public void KnotDistribution()
    {
        GeomAbs_BSplKnotDistribution aDistr = myOriginalCurve.KnotDistribution();
        // Single span with end mults = degree+1 -> PiecewiseBezier
        Assert.Equal(GeomAbs_PiecewiseBezier, aDistr);
    }

    [Fact]
    public void RationalCurveSegment()
    {
        Geom_BSplineCurve aCurve = new(Pnts(new gp_Pnt(1, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(0, 1, 0)),
                                       Reals(1.0, 1.0 / Math.Sqrt(2.0), 1.0),
                                       Reals(0.0, 1.0),
                                       Ints(3, 3),
                                       2);
        Assert.True(aCurve.IsRational());

        gp_Pnt aMid = aCurve.Value(0.5);
        aCurve.Segment(0.25, 0.75);
        gp_Pnt aMidAfter = aCurve.Value(0.5);
        Assert.True(aMid.IsEqual(aMidAfter, 1e-6));
        Assert.True(aCurve.IsRational());
    }

    [Fact]
    public void RationalCurveIncreaseDegree()
    {
        Geom_BSplineCurve aCurve = RationalQuadratic(2.0);
        gp_Pnt aValBefore = aCurve.Value(0.5);
        aCurve.IncreaseDegree(4);
        Assert.Equal(4, aCurve.Degree());
        Assert.True(aCurve.IsRational());
        gp_Pnt aValAfter = aCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void PeriodicCurve_SetOrigin()
    {
        NCollection_Array1<int> aMults = new(1, 6);
        aMults.Init(1);

        Geom_BSplineCurve aCurve = new(PentagonPoles(), PentagonKnots(), aMults, 3, true);
        Assert.True(aCurve.IsPeriodic());

        aCurve.SetOrigin(3);
        Assert.True(aCurve.IsPeriodic());
        // Curve shape unchanged, just reparameterized -- can evaluate without crash
        gp_Pnt aVal2 = aCurve.Value(aCurve.FirstParameter());
        Assert.True(aVal2.X() * aVal2.X() + aVal2.Y() * aVal2.Y() > 0.0);
    }

    [Fact]
    public void InsertKnots_Multiple()
    {
        NCollection_Array1<double> aNewKnots = Reals(0.25, 0.75);
        NCollection_Array1<int> aNewMults = Ints(1, 1);

        gp_Pnt aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.InsertKnots(aNewKnots, aNewMults);
        Assert.Equal(4, myOriginalCurve.NbKnots());
        gp_Pnt aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void ClosedCurve()
    {
        // Create a closed BSpline (first pole == last pole)
        NCollection_Array1<gp_Pnt> aPoles = Pnts(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 0, 0), new gp_Pnt(0, 0, 0));
        Geom_BSplineCurve aCurve = new(aPoles, Reals(0.0, 1.0), Ints(4, 4), 3);
        Assert.True(aCurve.IsClosed());
    }

    [Fact]
    public void LocalD1()
    {
        myOriginalCurve.InsertKnot(0.5);
        myOriginalCurve.D1(0.25, out gp_Pnt aPnt, out gp_Vec aV1);
        myOriginalCurve.LocalD1(0.25, 1, 2, out gp_Pnt aPntL, out gp_Vec aV1L);
        Assert.True(aPnt.IsEqual(aPntL, 1e-10));
        Assert.Equal(aV1L.X(), aV1.X(), 1e-10);
        Assert.Equal(aV1L.Y(), aV1.Y(), 1e-10);
    }

    // OCCT checks the reference returned by WeightsArray(); the port's public WeightsArray() is a copy, so the
    // identity and ownership checks use the internal accessor.

    [Fact]
    public void WeightsArray_NonRational_ReturnsUnitWeights()
    {
        Assert.False(myOriginalCurve.IsRational());
        NCollection_Array1<double> aWeights = myOriginalCurve.InternalWeightsArray();
        Assert.Equal(myOriginalCurve.NbPoles(), aWeights.Length());
        Assert.False(aWeights.IsDeletable());
        for (int i = 1; i <= aWeights.Length(); ++i)
        {
            Assert.Equal(1.0, aWeights[i]);
        }
        // Verify that the reference is stable
        Assert.Same(aWeights, myOriginalCurve.InternalWeightsArray());
        Assert.Equal(aWeights.AsSpan().ToArray(), myOriginalCurve.WeightsArray().AsSpan().ToArray());
    }

    [Fact]
    public void WeightsArray_Rational_ReturnsOwning()
    {
        // Create a rational BSpline
        NCollection_Array1<gp_Pnt> aPoles = Pnts(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 1, 0), new gp_Pnt(2, 1, 0), new gp_Pnt(3, 0, 0));
        Geom_BSplineCurve aRational = new(aPoles, Reals(1.0, 2.0, 3.0, 1.0), Reals(0.0, 1.0), Ints(4, 4), 3);
        Assert.True(aRational.IsRational());

        NCollection_Array1<double> aWeights = aRational.InternalWeightsArray();
        Assert.Equal(4, aWeights.Length());
        Assert.True(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1]);
        Assert.Equal(2.0, aWeights[2]);
        Assert.Equal(3.0, aWeights[3]);
        Assert.Equal(1.0, aWeights[4]);
        // Verify that the reference is stable
        Assert.Same(aWeights, aRational.InternalWeightsArray());
        Assert.Equal([1.0, 2.0, 3.0, 1.0], aRational.WeightsArray().AsSpan().ToArray());
    }
}
