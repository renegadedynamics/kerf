// Port of: src/ModelingData/TKG2d/GTests/Geom2d_BezierCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace OcctNet.ModelingData.Tests;

public class Geom2d_BezierCurve_Test
{
    private readonly Geom2d_BezierCurve myOriginalCurve;

    public Geom2d_BezierCurve_Test()
    {
        // Create a simple Bezier curve for testing
        NCollection_Array1<Point2D> aPoles = new NCollection_Array1<Point2D>(1, 4);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 1);
        aPoles[3] = new Point2D(2, 1);
        aPoles[4] = new Point2D(3, 0);
        myOriginalCurve = new Geom2d_BezierCurve(aPoles);
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
        Geom2d_BezierCurve aCopiedCurve = new Geom2d_BezierCurve(myOriginalCurve);

        // Verify basic properties are identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.NbPoles(), aCopiedCurve.NbPoles());
        Assert.Equal(myOriginalCurve.IsRational(), aCopiedCurve.IsRational());
        Assert.Equal(myOriginalCurve.IsClosed(), aCopiedCurve.IsClosed());
    }

    [Fact]
    public void CopyConstructorPoles()
    {
        Geom2d_BezierCurve aCopiedCurve = new Geom2d_BezierCurve(myOriginalCurve);

        // Verify all poles are identical
        for (int i = 1; i <= myOriginalCurve.NbPoles(); ++i)
        {
            Point2D anOrigPole = myOriginalCurve.Pole(i);
            Point2D aCopyPole = aCopiedCurve.Pole(i);
            Assert.True(anOrigPole.IsEqual(aCopyPole, 1e-10));
        }
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        // Test that Copy() method uses the optimized copy constructor
        Geom2d_Geometry aCopiedGeom = myOriginalCurve.Copy();
        Geom2d_BezierCurve? aCopiedCurve = aCopiedGeom as Geom2d_BezierCurve;

        Assert.NotNull(aCopiedCurve);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.NbPoles(), aCopiedCurve.NbPoles());

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
        // Create a rational Bezier curve
        NCollection_Array1<Point2D> aPoles = Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0));
        NCollection_Array1<double> aWeights = Weights3(1.0, 2.0, 1.0);

        Geom2d_BezierCurve aRationalCurve = new Geom2d_BezierCurve(aPoles, aWeights);
        Geom2d_BezierCurve aCopiedRational = new Geom2d_BezierCurve(aRationalCurve);

        Assert.True(aCopiedRational.IsRational());

        // Verify weights are copied correctly
        for (int i = 1; i <= aRationalCurve.NbPoles(); ++i)
        {
            Assert.Equal(aRationalCurve.Weight(i), aCopiedRational.Weight(i));
        }
    }

    [Fact]
    public void CopyIndependence()
    {
        Geom2d_BezierCurve aCopiedCurve = new Geom2d_BezierCurve(myOriginalCurve);

        // Modify the original curve
        Point2D aNewPole = new Point2D(10, 10);
        myOriginalCurve.SetPole(2, aNewPole);

        // Verify the copied curve is not affected
        Point2D anOrigPole = aCopiedCurve.Pole(2);
        Assert.False(anOrigPole.IsEqual(aNewPole, 1e-10));
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
        Assert.False(myOriginalCurve.IsPeriodic());
        Assert.False(myOriginalCurve.IsRational());
        Assert.False(myOriginalCurve.IsClosed());
        Assert.True(myOriginalCurve.IsCN(100));
        Assert.Equal(0.0, myOriginalCurve.FirstParameter());
        Assert.Equal(1.0, myOriginalCurve.LastParameter());
    }

    [Fact]
    public void SetPole()
    {
        Point2D aNewPole = new Point2D(1, 5);
        myOriginalCurve.SetPole(2, aNewPole);
        Assert.True(myOriginalCurve.Pole(2).IsEqual(aNewPole, 1e-10));
        Assert.True(myOriginalCurve.StartPoint().IsEqual(new Point2D(0, 0), 1e-10));
    }

    [Fact]
    public void SetWeight()
    {
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                                           Weights3(1.0, 1.0, 1.0));
        Point2D aMidBefore = aCurve.Value(0.5);
        aCurve.SetWeight(2, 10.0);
        Assert.Equal(10.0, aCurve.Weight(2));
        Assert.True(aCurve.IsRational());
        Point2D aMidAfter = aCurve.Value(0.5);
        Assert.True(aMidAfter.Y() > aMidBefore.Y());
    }

    [Fact]
    public void InsertPoleAfter()
    {
        int aNbBefore = myOriginalCurve.NbPoles();
        myOriginalCurve.InsertPoleAfter(2, new Point2D(1.5, 2));
        Assert.Equal(aNbBefore + 1, myOriginalCurve.NbPoles());
        Assert.Equal(4, myOriginalCurve.Degree());
    }

    [Fact]
    public void RemovePole()
    {
        myOriginalCurve.InsertPoleAfter(2, new Point2D(1.5, 2));
        Assert.Equal(5, myOriginalCurve.NbPoles());
        myOriginalCurve.RemovePole(3);
        Assert.Equal(4, myOriginalCurve.NbPoles());
    }

    [Fact]
    public void Increase()
    {
        Point2D aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.Increase(5);
        Assert.Equal(5, myOriginalCurve.Degree());
        Point2D aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void Segment()
    {
        Point2D aP25 = myOriginalCurve.Value(0.25);
        Point2D aP75 = myOriginalCurve.Value(0.75);
        myOriginalCurve.Segment(0.25, 0.75);
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
    public void PolesAccess()
    {
        // OCCT returns a reference to its array; the port returns a PackedList (indexed from 0)
        PackedList<Point2D> aPoles = myOriginalCurve.Poles();
        Assert.Equal(4, aPoles.Count);
        Assert.True(aPoles[0].IsEqual(new Point2D(0, 0), 1e-10));
    }

    [Fact]
    public void WeightsAccess_NonRational()
    {
        PackedList<double>? aWeights = myOriginalCurve.Weights();
        Assert.Null(aWeights);
    }

    [Fact]
    public void MaxDegree()
    {
        Assert.True(Geom2d_BezierCurve.MaxDegree() >= 25);
    }

    [Fact]
    public void RationalCurveEvaluation()
    {
        // Rational quadratic Bezier approximating circular arc
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(Poles3(new Point2D(1, 0), new Point2D(1, 1), new Point2D(0, 1)),
                                                           Weights3(1.0, 1.0 / Math.Sqrt(2.0), 1.0));
        Assert.True(aCurve.IsRational());
        Point2D aMid = aCurve.Value(0.5);
        double aRadius = Math.Sqrt(aMid.X() * aMid.X() + aMid.Y() * aMid.Y());
        Assert.Equal(1.0, aRadius, 1e-6);
    }

    [Fact]
    public void SetPoleWithWeight()
    {
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                                           Weights3(1.0, 1.0, 1.0));
        aCurve.SetPole(2, new Point2D(1, 2), 5.0);
        Assert.True(aCurve.Pole(2).IsEqual(new Point2D(1, 2), 1e-10));
        Assert.Equal(5.0, aCurve.Weight(2));
        Assert.True(aCurve.IsRational());
    }

    [Fact]
    public void InsertPoleBeforeWithWeight()
    {
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                                           Weights3(1.0, 2.0, 1.0));
        aCurve.InsertPoleBefore(2, new Point2D(0.5, 0.5), 3.0);
        Assert.Equal(4, aCurve.NbPoles());
        Assert.Equal(3.0, aCurve.Weight(2));
    }

    [Fact]
    public void ClosedCurve()
    {
        NCollection_Array1<Point2D> aPoles = new NCollection_Array1<Point2D>(1, 4);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(1, 1);
        aPoles[3] = new Point2D(2, 0);
        aPoles[4] = new Point2D(0, 0);
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(aPoles);
        Assert.True(aCurve.IsClosed());
    }

    [Fact]
    public void RationalSegment()
    {
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(Poles3(new Point2D(1, 0), new Point2D(1, 1), new Point2D(0, 1)),
                                                           Weights3(1.0, 1.0 / Math.Sqrt(2.0), 1.0));
        Point2D aMid = aCurve.Value(0.5);
        aCurve.Segment(0.25, 0.75);
        Point2D aMidAfter = aCurve.Value(0.5);
        Assert.True(aMid.IsEqual(aMidAfter, 1e-6));
        Assert.True(aCurve.IsRational());
    }

    [Fact]
    public void RationalReverse()
    {
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                                           Weights3(1.0, 3.0, 2.0));
        Point2D aStart = aCurve.StartPoint();
        Point2D anEnd = aCurve.EndPoint();
        aCurve.Reverse();
        Assert.True(aCurve.StartPoint().IsEqual(anEnd, 1e-10));
        Assert.True(aCurve.EndPoint().IsEqual(aStart, 1e-10));
        Assert.Equal(2.0, aCurve.Weight(1));
        Assert.Equal(1.0, aCurve.Weight(3));
    }

    [Fact]
    public void Evaluation_D3()
    {
        myOriginalCurve.D3(0.5, out _, out _, out _, out Vector2D aV3);
        // D3 of degree 3 is constant
        myOriginalCurve.D3(0.25, out _, out _, out _, out Vector2D aV3b);
        Assert.Equal(aV3b.X(), aV3.X(), 1e-8);
        Assert.Equal(aV3b.Y(), aV3.Y(), 1e-8);
    }

    [Fact]
    public void ReversedParameter()
    {
        Assert.Equal(0.7, myOriginalCurve.ReversedParameter(0.3), 1e-15);
        Assert.Equal(1.0, myOriginalCurve.ReversedParameter(0.0));
    }

    [Fact]
    public void LinearCurve()
    {
        NCollection_Array1<Point2D> aPoles = new NCollection_Array1<Point2D>(1, 2);
        aPoles[1] = new Point2D(0, 0);
        aPoles[2] = new Point2D(3, 4);
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(aPoles);
        Assert.Equal(1, aCurve.Degree());
        Point2D aMid = aCurve.Value(0.5);
        Assert.True(aMid.IsEqual(new Point2D(1.5, 2.0), 1e-10));
    }

    [Fact]
    public void RationalIncrease()
    {
        Geom2d_BezierCurve aCurve = new Geom2d_BezierCurve(Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                                           Weights3(1.0, 2.0, 1.0));
        Point2D aValBefore = aCurve.Value(0.5);
        aCurve.Increase(5);
        Assert.Equal(5, aCurve.Degree());
        Assert.True(aCurve.IsRational());
        Point2D aValAfter = aCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void WeightsArray_NonRational_ReturnsUnitWeights()
    {
        Assert.False(myOriginalCurve.IsRational());
        // The array itself is reached through the internal InternalWeightsArray() (decision 131)
        NCollection_Array1<double> aWeights = myOriginalCurve.InternalWeightsArray();
        Assert.Equal(myOriginalCurve.NbPoles(), aWeights.Length());
        Assert.False(aWeights.IsDeletable());
        for (int i = 1; i <= aWeights.Length(); ++i)
        {
            Assert.Equal(1.0, aWeights[i]);
        }
        Assert.Same(aWeights, myOriginalCurve.InternalWeightsArray());
    }

    [Fact]
    public void WeightsArray_Rational_ReturnsOwning()
    {
        Geom2d_BezierCurve aRational = new Geom2d_BezierCurve(Poles3(new Point2D(0, 0), new Point2D(1, 1), new Point2D(2, 0)),
                                                              Weights3(1.0, 2.0, 1.0));
        Assert.True(aRational.IsRational());
        NCollection_Array1<double> aWeights = aRational.InternalWeightsArray();
        Assert.Equal(3, aWeights.Length());
        Assert.True(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1]);
        Assert.Equal(2.0, aWeights[2]);
        Assert.Equal(1.0, aWeights[3]);
        Assert.Same(aWeights, aRational.InternalWeightsArray());
    }
}
