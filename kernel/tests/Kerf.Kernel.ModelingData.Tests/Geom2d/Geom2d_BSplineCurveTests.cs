// Port of: src/ModelingData/TKG2d/GTests/Geom2d_BSplineCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom2d_BSplineCurve_Test
{
    private readonly BSplineCurve2D myOriginalCurve;

    public Geom2d_BSplineCurve_Test()
    {
        // Create a simple BSpline curve for testing
        NCollection_Array1<Point2D> aPoles = new NCollection_Array1<Point2D>(1, 4);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 1);
        aPoles[3] = new Point2D(2, 1);
        aPoles[4] = new Point2D(3, 0);
        myOriginalCurve = new BSplineCurve2D(aPoles, Knots01(), Mults(4, 4), 3);
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

    private static NCollection_Array1<Point2D> Poles3(Point2D theP1, Point2D theP2, Point2D theP3)
    {
        NCollection_Array1<Point2D> aPoles = new NCollection_Array1<Point2D>(1, 3);
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
        BSplineCurve2D aCopiedCurve = new BSplineCurve2D(myOriginalCurve);

        // Verify basic properties are identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.PoleCount, aCopiedCurve.PoleCount);
        Assert.Equal(myOriginalCurve.KnotCount, aCopiedCurve.KnotCount);
        Assert.Equal(myOriginalCurve.IsPeriodic, aCopiedCurve.IsPeriodic);
        Assert.Equal(myOriginalCurve.IsRational, aCopiedCurve.IsRational);
    }

    [Fact]
    public void CopyConstructorPoles()
    {
        BSplineCurve2D aCopiedCurve = new BSplineCurve2D(myOriginalCurve);

        // Verify all poles are identical
        for (int i = 1; i <= myOriginalCurve.PoleCount; ++i)
        {
            Point2D anOrigPole = myOriginalCurve.Pole(i);
            Point2D aCopyPole = aCopiedCurve.Pole(i);
            Assert.True(anOrigPole.IsEqual(aCopyPole, 1e-10));
        }
    }

    [Fact]
    public void CopyConstructorKnots()
    {
        BSplineCurve2D aCopiedCurve = new BSplineCurve2D(myOriginalCurve);

        // Verify all knots are identical
        for (int i = 1; i <= myOriginalCurve.KnotCount; ++i)
        {
            Assert.Equal(myOriginalCurve.Knot(i), aCopiedCurve.Knot(i));
            Assert.Equal(myOriginalCurve.Multiplicity(i), aCopiedCurve.Multiplicity(i));
        }
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        // Test that Copy() method uses the optimized copy constructor
        Geometry2D aCopiedGeom = myOriginalCurve.Copy();
        BSplineCurve2D? aCopiedCurve = aCopiedGeom as BSplineCurve2D;

        Assert.NotNull(aCopiedCurve);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.PoleCount, aCopiedCurve.PoleCount);

        // Test evaluation at several points
        for (double u = 0.0; u <= 1.0; u += 0.25)
        {
            Point2D anOrigPnt = myOriginalCurve.Value(u);
            Point2D aCopyPnt = aCopiedCurve.Value(u);
            Assert.True(anOrigPnt.IsEqual(aCopyPnt, 1e-10));
        }
    }

    [Fact]
    public void RationalCurveCopyConstructor()
    {
        // Create a rational BSpline curve
        BSplineCurve2D aRationalCurve =
            new BSplineCurve2D(Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                    Weights3(1.0, 2.0, 1.0),
                                    Knots01(),
                                    Mults(3, 3),
                                    2);
        BSplineCurve2D aCopiedRational = new BSplineCurve2D(aRationalCurve);

        Assert.True(aCopiedRational.IsRational);

        // Verify weights are copied correctly
        for (int i = 1; i <= aRationalCurve.PoleCount; ++i)
        {
            Assert.Equal(aRationalCurve.Weight(i), aCopiedRational.Weight(i));
        }
    }

    [Fact]
    public void Evaluation_D0()
    {
        Point2D aPnt;
        myOriginalCurve.D0(0.0, out aPnt);
        Assert.True(aPnt.IsEqual(new Point2D(0, 0), 1e-10));
        myOriginalCurve.D0(1.0, out aPnt);
        Assert.True(aPnt.IsEqual(new Point2D(3, 0), 1e-10));
    }

    [Fact]
    public void Evaluation_D1()
    {
        myOriginalCurve.D1(0.5, out _, out Vector2D aV1);
        Assert.True(aV1.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_D2()
    {
        myOriginalCurve.D2(0.5, out _, out Vector2D aV1, out _);
        Assert.True(aV1.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_DN()
    {
        Vector2D aDN1 = myOriginalCurve.DN(0.5, 1);
        Assert.True(aDN1.Magnitude() > 0.0);
        // DN(4) for degree 3 should be zero
        Vector2D aDN4 = myOriginalCurve.DN(0.5, 4);
        Assert.Equal(0.0, aDN4.Magnitude(), 1e-10);
    }

    [Fact]
    public void StartEndPoints()
    {
        Point2D aStart = myOriginalCurve.StartPoint();
        Point2D anEnd = myOriginalCurve.EndPoint();
        Assert.True(aStart.IsEqual(new Point2D(0, 0), 1e-10));
        Assert.True(anEnd.IsEqual(new Point2D(3, 0), 1e-10));
    }

    [Fact]
    public void Properties()
    {
        Assert.Equal(3, myOriginalCurve.Degree());
        Assert.False(myOriginalCurve.IsPeriodic);
        Assert.False(myOriginalCurve.IsRational);
        Assert.False(myOriginalCurve.IsClosed);
        Assert.True(myOriginalCurve.IsCN(3));
    }

    [Fact]
    public void SetPole()
    {
        Point2D aNewPole = new Point2D(1, 5);
        myOriginalCurve.SetPole(2, aNewPole);
        Assert.True(myOriginalCurve.Pole(2).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void IncreaseDegree()
    {
        Point2D aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.IncreaseDegree(5);
        Assert.Equal(5, myOriginalCurve.Degree());
        Point2D aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void InsertKnot()
    {
        Point2D aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.InsertKnot(0.5);
        Assert.Equal(3, myOriginalCurve.KnotCount);
        Point2D aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void RemoveKnot()
    {
        myOriginalCurve.InsertKnot(0.5);
        Assert.Equal(3, myOriginalCurve.KnotCount);
        bool isRemoved = myOriginalCurve.RemoveKnot(2, 0, 1e-6);
        Assert.True(isRemoved);
        Assert.Equal(2, myOriginalCurve.KnotCount);
    }

    [Fact]
    public void Segment()
    {
        Point2D aP25 = myOriginalCurve.Value(0.25);
        Point2D aP75 = myOriginalCurve.Value(0.75);
        myOriginalCurve.Segment(0.25, 0.75);
        Assert.Equal(0.25, myOriginalCurve.FirstParameter(), 1e-10);
        Assert.Equal(0.75, myOriginalCurve.LastParameter(), 1e-10);
        Assert.True(myOriginalCurve.StartPoint().IsEqual(aP25, 1e-6));
        Assert.True(myOriginalCurve.EndPoint().IsEqual(aP75, 1e-6));
    }

    [Fact]
    public void Reverse()
    {
        Point2D aStart = myOriginalCurve.StartPoint();
        Point2D anEnd = myOriginalCurve.EndPoint();
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
        Transform2D aTrsf = new Transform2D();
        aTrsf.SetTranslation(new Vector2D(10, 20));
        Point2D aPtBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.Transform(aTrsf);
        Point2D aPtAfter = myOriginalCurve.Value(0.5);
        Assert.Equal(aPtBefore.X() + 10.0, aPtAfter.X(), 1e-10);
        Assert.Equal(aPtBefore.Y() + 20.0, aPtAfter.Y(), 1e-10);
    }

    [Fact]
    public void PeriodicCurve()
    {
        // Periodic BSpline: degree 3, 6 knots of mult 1 -> NbPoles = 1+4 = 5
        NCollection_Array1<Point2D> aPoles = new NCollection_Array1<Point2D>(1, 5);
        aPoles[1] = new Point2D(1, 0);
        aPoles[2] = new Point2D(0.309, 0.951);
        aPoles[3] = new Point2D(-0.809, 0.588);
        aPoles[4] = new Point2D(-0.809, -0.588);
        aPoles[5] = new Point2D(0.309, -0.951);
        NCollection_Array1<double> aKnots = new NCollection_Array1<double>(1, 6);
        for (int i = 1; i <= 6; ++i)
        {
            aKnots[i] = (i - 1) * 0.2;
        }
        NCollection_Array1<int> aMults = new NCollection_Array1<int>(1, 6);
        aMults.Init(1);
        BSplineCurve2D aCurve = new BSplineCurve2D(aPoles, aKnots, aMults, 3, true);
        Assert.True(aCurve.IsPeriodic);
        Point2D aVal1 = aCurve.Value(0.5);
        aCurve.SetNotPeriodic();
        Assert.False(aCurve.IsPeriodic);
        Point2D aVal2 = aCurve.Value(0.5);
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
        BSplineCurve2D aCopy = new BSplineCurve2D(myOriginalCurve);
        myOriginalCurve.SetPole(2, new Point2D(10, 10));
        Assert.False(aCopy.Pole(2).IsEqual(new Point2D(10, 10), 1e-10));
    }

    [Fact]
    public void SetWeight()
    {
        BSplineCurve2D aCurve =
            new BSplineCurve2D(Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                    Weights3(1.0, 1.0, 1.0),
                                    Knots01(),
                                    Mults(3, 3),
                                    2);
        Point2D aMidBefore = aCurve.Value(0.5);
        aCurve.SetWeight(2, 5.0);
        Assert.Equal(5.0, aCurve.Weight(2));
        Assert.True(aCurve.IsRational);
        Point2D aMidAfter = aCurve.Value(0.5);
        Assert.True(aMidAfter.Y() > aMidBefore.Y());
    }

    [Fact]
    public void MovePoint()
    {
        Point2D aTarget = new Point2D(1.5, 0.8);
        myOriginalCurve.MovePoint(0.5, aTarget, 1, myOriginalCurve.PoleCount, out int aFirst, out int aLast);
        Assert.True(aFirst > 0);
        Point2D aMoved = myOriginalCurve.Value(0.5);
        Assert.True(aMoved.IsEqual(aTarget, 1e-6));
    }

    [Fact]
    public void LocateU()
    {
        myOriginalCurve.InsertKnot(0.5);
        myOriginalCurve.LocateU(0.25, 1e-10, out int anI1, out int anI2);
        Assert.True(anI1 >= 1);
        Assert.True(anI2 <= myOriginalCurve.KnotCount);
    }

    [Fact]
    public void LocalD1()
    {
        myOriginalCurve.InsertKnot(0.5);
        myOriginalCurve.D1(0.25, out Point2D aPnt, out Vector2D aV1);
        myOriginalCurve.LocalD1(0.25, 1, 2, out Point2D aPntL, out Vector2D aV1L);
        Assert.True(aPnt.IsEqual(aPntL, 1e-10));
        Assert.Equal(aV1L.X(), aV1.X(), 1e-10);
        Assert.Equal(aV1L.Y(), aV1.Y(), 1e-10);
    }

    [Fact]
    public void RationalCurveSegment()
    {
        BSplineCurve2D aCurve =
            new BSplineCurve2D(Poles3(new Point2D(1, 0), new Point2D(1, 1), new Point2D(0, 1)),
                                    Weights3(1.0, 1.0 / Math.Sqrt(2.0), 1.0),
                                    Knots01(),
                                    Mults(3, 3),
                                    2);
        Point2D aMid = aCurve.Value(0.5);
        aCurve.Segment(0.25, 0.75);
        Point2D aMidAfter = aCurve.Value(0.5);
        Assert.True(aMid.IsEqual(aMidAfter, 1e-6));
        Assert.True(aCurve.IsRational);
    }

    [Fact]
    public void ClosedCurve()
    {
        NCollection_Array1<Point2D> aPoles = new NCollection_Array1<Point2D>(1, 4);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 1);
        aPoles[3] = new Point2D(2, 0);
        aPoles[4] = new Point2D(0, 0);
        BSplineCurve2D aCurve = new BSplineCurve2D(aPoles, Knots01(), Mults(4, 4), 3);
        Assert.True(aCurve.IsClosed);
    }

    [Fact]
    public void InsertKnots_Multiple()
    {
        NCollection_Array1<double> aNewKnots = new NCollection_Array1<double>(1, 2);
        aNewKnots[1] = 0.25;
        aNewKnots[2] = 0.75;
        NCollection_Array1<int> aNewMults = Mults(1, 1);
        Point2D aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.InsertKnots(aNewKnots, aNewMults);
        Assert.Equal(4, myOriginalCurve.KnotCount);
        Point2D aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void WeightsArray_NonRational_ReturnsUnitWeights()
    {
        Assert.False(myOriginalCurve.IsRational);
        // The array itself is reached through the internal InternalWeightsArray() (decision 131)
        NCollection_Array1<double> aWeights = myOriginalCurve.InternalWeightsArray();
        Assert.Equal(myOriginalCurve.PoleCount, aWeights.Length());
        Assert.False(aWeights.IsDeletable());
        for (int i = 1; i <= aWeights.Length(); ++i)
        {
            Assert.Equal(1.0, aWeights[i]);
        }
        // Verify that the reference is stable
        Assert.Same(aWeights, myOriginalCurve.InternalWeightsArray());
    }

    [Fact]
    public void WeightsArray_Rational_ReturnsOwning()
    {
        // Create a rational BSpline
        NCollection_Array1<Point2D> aPoles = new NCollection_Array1<Point2D>(1, 4);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 1);
        aPoles[3] = new Point2D(2, 1);
        aPoles[4] = new Point2D(3, 0);
        NCollection_Array1<double> aWeightsIn = new NCollection_Array1<double>(1, 4);
        aWeightsIn[1] = 1.0;
        aWeightsIn[2] = 2.0;
        aWeightsIn[3] = 3.0;
        aWeightsIn[4] = 1.0;
        BSplineCurve2D aRational = new BSplineCurve2D(aPoles, aWeightsIn, Knots01(), Mults(4, 4), 3);
        Assert.True(aRational.IsRational);
        NCollection_Array1<double> aWeights = aRational.InternalWeightsArray();
        Assert.Equal(4, aWeights.Length());
        Assert.True(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1]);
        Assert.Equal(2.0, aWeights[2]);
        Assert.Equal(3.0, aWeights[3]);
        Assert.Equal(1.0, aWeights[4]);
        // Verify that the reference is stable
        Assert.Same(aWeights, aRational.InternalWeightsArray());
    }
}
