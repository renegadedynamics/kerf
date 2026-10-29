// Port of: src/ModelingData/TKG2d/GTests/Geom2d_BSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.ModelingData.Tests;

public class Geom2d_BSplineCurve_Test
{
    private readonly Geom2d_BSplineCurve myOriginalCurve;

    public Geom2d_BSplineCurve_Test()
    {
        // Create a simple BSpline curve for testing
        NCollection_Array1<gp_Pnt2d> aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0, 0);
        aPoles[2] = new gp_Pnt2d(1, 1);
        aPoles[3] = new gp_Pnt2d(2, 1);
        aPoles[4] = new gp_Pnt2d(3, 0);
        myOriginalCurve = new Geom2d_BSplineCurve(aPoles, Knots01(), Mults(4, 4), 3);
    }

    private static NCollection_Array1<double> Knots01()
    {
        NCollection_Array1<double> aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0.0;
        aKnots[2] = 1.0;
        return aKnots;
    }

    private static NCollection_Array1<int> Mults(int theM1, int theM2)
    {
        NCollection_Array1<int> aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = theM1;
        aMults[2] = theM2;
        return aMults;
    }

    private static NCollection_Array1<gp_Pnt2d> Poles3(gp_Pnt2d theP1, gp_Pnt2d theP2, gp_Pnt2d theP3)
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new NCollection_Array1<gp_Pnt2d>(1, 3);
        aPoles[1] = theP1;
        aPoles[2] = theP2;
        aPoles[3] = theP3;
        return aPoles;
    }

    private static NCollection_Array1<double> Weights3(double theW1, double theW2, double theW3)
    {
        NCollection_Array1<double> aWeights = new NCollection_Array1<double>(1, 3);
        aWeights[1] = theW1;
        aWeights[2] = theW2;
        aWeights[3] = theW3;
        return aWeights;
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        // Test copy constructor
        Geom2d_BSplineCurve aCopiedCurve = new Geom2d_BSplineCurve(myOriginalCurve);

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
        Geom2d_BSplineCurve aCopiedCurve = new Geom2d_BSplineCurve(myOriginalCurve);

        // Verify all poles are identical
        for (int i = 1; i <= myOriginalCurve.NbPoles(); ++i)
        {
            gp_Pnt2d anOrigPole = myOriginalCurve.Pole(i);
            gp_Pnt2d aCopyPole = aCopiedCurve.Pole(i);
            Assert.True(anOrigPole.IsEqual(aCopyPole, 1e-10));
        }
    }

    [Fact]
    public void CopyConstructorKnots()
    {
        Geom2d_BSplineCurve aCopiedCurve = new Geom2d_BSplineCurve(myOriginalCurve);

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
        Geom2d_Geometry aCopiedGeom = myOriginalCurve.Copy();
        Geom2d_BSplineCurve? aCopiedCurve = aCopiedGeom as Geom2d_BSplineCurve;

        Assert.NotNull(aCopiedCurve);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.NbPoles(), aCopiedCurve.NbPoles());

        // Test evaluation at several points
        for (double u = 0.0; u <= 1.0; u += 0.25)
        {
            gp_Pnt2d anOrigPnt = myOriginalCurve.Value(u);
            gp_Pnt2d aCopyPnt = aCopiedCurve.Value(u);
            Assert.True(anOrigPnt.IsEqual(aCopyPnt, 1e-10));
        }
    }

    [Fact]
    public void RationalCurveCopyConstructor()
    {
        // Create a rational BSpline curve
        Geom2d_BSplineCurve aRationalCurve =
            new Geom2d_BSplineCurve(Poles3(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 1), new gp_Pnt2d(2, 0)),
                                    Weights3(1.0, 2.0, 1.0),
                                    Knots01(),
                                    Mults(3, 3),
                                    2);
        Geom2d_BSplineCurve aCopiedRational = new Geom2d_BSplineCurve(aRationalCurve);

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
        gp_Pnt2d aPnt;
        myOriginalCurve.D0(0.0, out aPnt);
        Assert.True(aPnt.IsEqual(new gp_Pnt2d(0, 0), 1e-10));
        myOriginalCurve.D0(1.0, out aPnt);
        Assert.True(aPnt.IsEqual(new gp_Pnt2d(3, 0), 1e-10));
    }

    [Fact]
    public void Evaluation_D1()
    {
        myOriginalCurve.D1(0.5, out _, out gp_Vec2d aV1);
        Assert.True(aV1.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_D2()
    {
        myOriginalCurve.D2(0.5, out _, out gp_Vec2d aV1, out _);
        Assert.True(aV1.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_DN()
    {
        gp_Vec2d aDN1 = myOriginalCurve.DN(0.5, 1);
        Assert.True(aDN1.Magnitude() > 0.0);
        // DN(4) for degree 3 should be zero
        gp_Vec2d aDN4 = myOriginalCurve.DN(0.5, 4);
        Assert.Equal(0.0, aDN4.Magnitude(), 1e-10);
    }

    [Fact]
    public void StartEndPoints()
    {
        gp_Pnt2d aStart = myOriginalCurve.StartPoint();
        gp_Pnt2d anEnd = myOriginalCurve.EndPoint();
        Assert.True(aStart.IsEqual(new gp_Pnt2d(0, 0), 1e-10));
        Assert.True(anEnd.IsEqual(new gp_Pnt2d(3, 0), 1e-10));
    }

    [Fact]
    public void Properties()
    {
        Assert.Equal(3, myOriginalCurve.Degree());
        Assert.False(myOriginalCurve.IsPeriodic());
        Assert.False(myOriginalCurve.IsRational());
        Assert.False(myOriginalCurve.IsClosed());
        Assert.True(myOriginalCurve.IsCN(3));
    }

    [Fact]
    public void SetPole()
    {
        gp_Pnt2d aNewPole = new gp_Pnt2d(1, 5);
        myOriginalCurve.SetPole(2, aNewPole);
        Assert.True(myOriginalCurve.Pole(2).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void IncreaseDegree()
    {
        gp_Pnt2d aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.IncreaseDegree(5);
        Assert.Equal(5, myOriginalCurve.Degree());
        gp_Pnt2d aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void InsertKnot()
    {
        gp_Pnt2d aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.InsertKnot(0.5);
        Assert.Equal(3, myOriginalCurve.NbKnots());
        gp_Pnt2d aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void RemoveKnot()
    {
        myOriginalCurve.InsertKnot(0.5);
        Assert.Equal(3, myOriginalCurve.NbKnots());
        bool isRemoved = myOriginalCurve.RemoveKnot(2, 0, 1e-6);
        Assert.True(isRemoved);
        Assert.Equal(2, myOriginalCurve.NbKnots());
    }

    [Fact]
    public void Segment()
    {
        gp_Pnt2d aP25 = myOriginalCurve.Value(0.25);
        gp_Pnt2d aP75 = myOriginalCurve.Value(0.75);
        myOriginalCurve.Segment(0.25, 0.75);
        Assert.Equal(0.25, myOriginalCurve.FirstParameter(), 1e-10);
        Assert.Equal(0.75, myOriginalCurve.LastParameter(), 1e-10);
        Assert.True(myOriginalCurve.StartPoint().IsEqual(aP25, 1e-6));
        Assert.True(myOriginalCurve.EndPoint().IsEqual(aP75, 1e-6));
    }

    [Fact]
    public void Reverse()
    {
        gp_Pnt2d aStart = myOriginalCurve.StartPoint();
        gp_Pnt2d anEnd = myOriginalCurve.EndPoint();
        myOriginalCurve.Reverse();
        Assert.True(myOriginalCurve.StartPoint().IsEqual(anEnd, 1e-10));
        Assert.True(myOriginalCurve.EndPoint().IsEqual(aStart, 1e-10));
    }

    [Fact]
    public void Resolution()
    {
        double aUTol;
        myOriginalCurve.Resolution(1.0, out aUTol);
        Assert.True(aUTol > 0.0);
    }

    [Fact]
    public void Transform()
    {
        gp_Trsf2d aTrsf = new gp_Trsf2d();
        aTrsf.SetTranslation(new gp_Vec2d(10, 20));
        gp_Pnt2d aPtBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.Transform(aTrsf);
        gp_Pnt2d aPtAfter = myOriginalCurve.Value(0.5);
        Assert.Equal(aPtBefore.X() + 10.0, aPtAfter.X(), 1e-10);
        Assert.Equal(aPtBefore.Y() + 20.0, aPtAfter.Y(), 1e-10);
    }

    [Fact]
    public void PeriodicCurve()
    {
        // Periodic BSpline: degree 3, 6 knots of mult 1 -> NbPoles = 1+4 = 5
        NCollection_Array1<gp_Pnt2d> aPoles = new NCollection_Array1<gp_Pnt2d>(1, 5);
        aPoles[1] = new gp_Pnt2d(1, 0);
        aPoles[2] = new gp_Pnt2d(0.309, 0.951);
        aPoles[3] = new gp_Pnt2d(-0.809, 0.588);
        aPoles[4] = new gp_Pnt2d(-0.809, -0.588);
        aPoles[5] = new gp_Pnt2d(0.309, -0.951);
        NCollection_Array1<double> aKnots = new NCollection_Array1<double>(1, 6);
        for (int i = 1; i <= 6; ++i)
        {
            aKnots[i] = (i - 1) * 0.2;
        }
        NCollection_Array1<int> aMults = new NCollection_Array1<int>(1, 6);
        aMults.Init(1);
        Geom2d_BSplineCurve aCurve = new Geom2d_BSplineCurve(aPoles, aKnots, aMults, 3, true);
        Assert.True(aCurve.IsPeriodic());
        gp_Pnt2d aVal1 = aCurve.Value(0.5);
        aCurve.SetNotPeriodic();
        Assert.False(aCurve.IsPeriodic());
        gp_Pnt2d aVal2 = aCurve.Value(0.5);
        Assert.True(aVal1.IsEqual(aVal2, 1e-10));
    }

    [Fact]
    public void WeightsAccess_NonRational()
    {
        PackedList<double>? aWeights = myOriginalCurve.Weights();
        Assert.Null(aWeights);
    }

    [Fact]
    public void KnotAccess()
    {
        Assert.Equal(0.0, myOriginalCurve.Knot(1));
        Assert.Equal(1.0, myOriginalCurve.Knot(2));
        // OCCT returns references to its arrays; the port returns PackedLists (indexed from 0)
        PackedList<double> aKnots = myOriginalCurve.Knots();
        Assert.Equal(2, aKnots.Count);
        PackedList<int> aMults = myOriginalCurve.Multiplicities();
        Assert.Equal(2, aMults.Count);
        Assert.Equal(4, aMults[0]);
    }

    [Fact]
    public void CopyIndependence()
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(myOriginalCurve);
        myOriginalCurve.SetPole(2, new gp_Pnt2d(10, 10));
        Assert.False(aCopy.Pole(2).IsEqual(new gp_Pnt2d(10, 10), 1e-10));
    }

    [Fact]
    public void SetWeight()
    {
        Geom2d_BSplineCurve aCurve =
            new Geom2d_BSplineCurve(Poles3(new gp_Pnt2d(0, 0), new gp_Pnt2d(1, 1), new gp_Pnt2d(2, 0)),
                                    Weights3(1.0, 1.0, 1.0),
                                    Knots01(),
                                    Mults(3, 3),
                                    2);
        gp_Pnt2d aMidBefore = aCurve.Value(0.5);
        aCurve.SetWeight(2, 5.0);
        Assert.Equal(5.0, aCurve.Weight(2));
        Assert.True(aCurve.IsRational());
        gp_Pnt2d aMidAfter = aCurve.Value(0.5);
        Assert.True(aMidAfter.Y() > aMidBefore.Y());
    }

    [Fact]
    public void MovePoint()
    {
        gp_Pnt2d aTarget = new gp_Pnt2d(1.5, 0.8);
        myOriginalCurve.MovePoint(0.5, aTarget, 1, myOriginalCurve.NbPoles(), out int aFirst, out int aLast);
        Assert.True(aFirst > 0);
        gp_Pnt2d aMoved = myOriginalCurve.Value(0.5);
        Assert.True(aMoved.IsEqual(aTarget, 1e-6));
    }

    [Fact]
    public void LocateU()
    {
        myOriginalCurve.InsertKnot(0.5);
        myOriginalCurve.LocateU(0.25, 1e-10, out int anI1, out int anI2);
        Assert.True(anI1 >= 1);
        Assert.True(anI2 <= myOriginalCurve.NbKnots());
    }

    [Fact]
    public void LocalD1()
    {
        myOriginalCurve.InsertKnot(0.5);
        myOriginalCurve.D1(0.25, out gp_Pnt2d aPnt, out gp_Vec2d aV1);
        myOriginalCurve.LocalD1(0.25, 1, 2, out gp_Pnt2d aPntL, out gp_Vec2d aV1L);
        Assert.True(aPnt.IsEqual(aPntL, 1e-10));
        Assert.Equal(aV1L.X(), aV1.X(), 1e-10);
        Assert.Equal(aV1L.Y(), aV1.Y(), 1e-10);
    }

    [Fact]
    public void RationalCurveSegment()
    {
        Geom2d_BSplineCurve aCurve =
            new Geom2d_BSplineCurve(Poles3(new gp_Pnt2d(1, 0), new gp_Pnt2d(1, 1), new gp_Pnt2d(0, 1)),
                                    Weights3(1.0, 1.0 / Math.Sqrt(2.0), 1.0),
                                    Knots01(),
                                    Mults(3, 3),
                                    2);
        gp_Pnt2d aMid = aCurve.Value(0.5);
        aCurve.Segment(0.25, 0.75);
        gp_Pnt2d aMidAfter = aCurve.Value(0.5);
        Assert.True(aMid.IsEqual(aMidAfter, 1e-6));
        Assert.True(aCurve.IsRational());
    }

    [Fact]
    public void ClosedCurve()
    {
        NCollection_Array1<gp_Pnt2d> aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0, 0);
        aPoles[2] = new gp_Pnt2d(1, 1);
        aPoles[3] = new gp_Pnt2d(2, 0);
        aPoles[4] = new gp_Pnt2d(0, 0);
        Geom2d_BSplineCurve aCurve = new Geom2d_BSplineCurve(aPoles, Knots01(), Mults(4, 4), 3);
        Assert.True(aCurve.IsClosed());
    }

    [Fact]
    public void InsertKnots_Multiple()
    {
        NCollection_Array1<double> aNewKnots = new NCollection_Array1<double>(1, 2);
        aNewKnots[1] = 0.25;
        aNewKnots[2] = 0.75;
        NCollection_Array1<int> aNewMults = Mults(1, 1);
        gp_Pnt2d aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.InsertKnots(aNewKnots, aNewMults);
        Assert.Equal(4, myOriginalCurve.NbKnots());
        gp_Pnt2d aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void WeightsArray_NonRational_ReturnsUnitWeights()
    {
        Assert.False(myOriginalCurve.IsRational());
        // The array itself is reached through the internal WeightsArrayRef() (decision 131)
        NCollection_Array1<double> aWeights = myOriginalCurve.WeightsArrayRef();
        Assert.Equal(myOriginalCurve.NbPoles(), aWeights.Length());
        Assert.False(aWeights.IsDeletable());
        for (int i = 1; i <= aWeights.Length(); ++i)
        {
            Assert.Equal(1.0, aWeights[i]);
        }
        // Verify that the reference is stable
        Assert.Same(aWeights, myOriginalCurve.WeightsArrayRef());
    }

    [Fact]
    public void WeightsArray_Rational_ReturnsOwning()
    {
        // Create a rational BSpline
        NCollection_Array1<gp_Pnt2d> aPoles = new NCollection_Array1<gp_Pnt2d>(1, 4);
        aPoles[1] = new gp_Pnt2d(0, 0);
        aPoles[2] = new gp_Pnt2d(1, 1);
        aPoles[3] = new gp_Pnt2d(2, 1);
        aPoles[4] = new gp_Pnt2d(3, 0);
        NCollection_Array1<double> aWeightsIn = new NCollection_Array1<double>(1, 4);
        aWeightsIn[1] = 1.0;
        aWeightsIn[2] = 2.0;
        aWeightsIn[3] = 3.0;
        aWeightsIn[4] = 1.0;
        Geom2d_BSplineCurve aRational = new Geom2d_BSplineCurve(aPoles, aWeightsIn, Knots01(), Mults(4, 4), 3);
        Assert.True(aRational.IsRational());
        NCollection_Array1<double> aWeights = aRational.WeightsArrayRef();
        Assert.Equal(4, aWeights.Length());
        Assert.True(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1]);
        Assert.Equal(2.0, aWeights[2]);
        Assert.Equal(3.0, aWeights[3]);
        Assert.Equal(1.0, aWeights[4]);
        // Verify that the reference is stable
        Assert.Same(aWeights, aRational.WeightsArrayRef());
    }
}
