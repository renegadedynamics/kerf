// Port of: src/ModelingData/TKG3d/GTests/Geom_BezierCurve_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

namespace Kerf.Kernel.ModelingData.Tests;

public class Geom_BezierCurve_Test
{
    private readonly BezierCurve3D myOriginalCurve;

    public Geom_BezierCurve_Test()
    {
        // Create a simple Bezier curve for testing
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 1, 0);
        aPoles[3] = new Point3D(2, 1, 0);
        aPoles[4] = new Point3D(3, 0, 0);

        myOriginalCurve = new BezierCurve3D(aPoles);
    }

    private static NCollection_Array1<Point3D> Poles3(Point3D p1, Point3D p2, Point3D p3)
    {
        NCollection_Array1<Point3D> aPoles = new(1, 3);
        aPoles[1] = p1;
        aPoles[2] = p2;
        aPoles[3] = p3;
        return aPoles;
    }

    private static NCollection_Array1<double> Weights3(double w1, double w2, double w3)
    {
        NCollection_Array1<double> aWeights = new(1, 3);
        aWeights[1] = w1;
        aWeights[2] = w2;
        aWeights[3] = w3;
        return aWeights;
    }

    [Fact]
    public void CopyConstructorBasicProperties()
    {
        // Test copy constructor
        BezierCurve3D aCopiedCurve = new(myOriginalCurve);

        // Verify basic properties are identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.PoleCount, aCopiedCurve.PoleCount);
        Assert.Equal(myOriginalCurve.IsRational, aCopiedCurve.IsRational);
        Assert.Equal(myOriginalCurve.IsClosed, aCopiedCurve.IsClosed);
    }

    [Fact]
    public void CopyConstructorPoles()
    {
        BezierCurve3D aCopiedCurve = new(myOriginalCurve);

        // Verify all poles are identical
        for (int i = 1; i <= myOriginalCurve.PoleCount; ++i)
        {
            Point3D anOrigPole = myOriginalCurve.Pole(i);
            Point3D aCopyPole = aCopiedCurve.Pole(i);
            Assert.True(anOrigPole.IsEqual(aCopyPole, 1e-10));
        }
    }

    [Fact]
    public void CopyMethodUsesOptimizedConstructor()
    {
        // Test that Copy() method uses the optimized copy constructor
        BezierCurve3D? aCopiedCurve = myOriginalCurve.Copy() as BezierCurve3D;
        Assert.NotNull(aCopiedCurve);

        // Verify the copy is functionally identical
        Assert.Equal(myOriginalCurve.Degree(), aCopiedCurve.Degree());
        Assert.Equal(myOriginalCurve.PoleCount, aCopiedCurve.PoleCount);

        // Test evaluation at several points
        for (double u = 0.0; u <= 1.0; u += 0.25)
        {
            Point3D anOrigPnt = myOriginalCurve.Value(u);
            Point3D aCopyPnt = aCopiedCurve.Value(u);
            Assert.True(anOrigPnt.IsEqual(aCopyPnt, 1e-10));
        }
    }

    [Fact]
    public void RationalCurveCopyConstructor()
    {
        // Create a rational Bezier curve
        BezierCurve3D aRationalCurve = new(Poles3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)),
                                              Weights3(1.0, 2.0, 1.0));
        BezierCurve3D aCopiedRational = new(aRationalCurve);

        Assert.True(aCopiedRational.IsRational);

        // Verify weights are copied correctly
        for (int i = 1; i <= aRationalCurve.PoleCount; ++i)
        {
            Assert.Equal(aRationalCurve.Weight(i), aCopiedRational.Weight(i));
        }
    }

    [Fact]
    public void CopyIndependence()
    {
        BezierCurve3D aCopiedCurve = new(myOriginalCurve);

        // Modify the original curve
        Point3D aNewPole = new(10, 10, 10);
        myOriginalCurve.SetPole(2, aNewPole);

        // Verify the copied curve is not affected
        Point3D anOrigPole = aCopiedCurve.Pole(2);
        Assert.False(anOrigPole.IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void Evaluation_D0()
    {
        myOriginalCurve.D0(0.0, out Point3D aPnt);
        Assert.True(aPnt.IsEqual(new Point3D(0, 0, 0), 1e-10));

        myOriginalCurve.D0(1.0, out aPnt);
        Assert.True(aPnt.IsEqual(new Point3D(3, 0, 0), 1e-10));
    }

    [Fact]
    public void Evaluation_D1()
    {
        myOriginalCurve.D1(0.5, out _, out Vector3D aV1);
        Assert.True(aV1.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_D2()
    {
        myOriginalCurve.D2(0.5, out _, out Vector3D aV1, out _);
        Assert.True(aV1.Magnitude() > 0.0);
    }

    [Fact]
    public void Evaluation_D3()
    {
        myOriginalCurve.D3(0.5, out _, out _, out _, out Vector3D aV3);
        // D3 of a degree 3 curve is constant
        myOriginalCurve.D3(0.25, out _, out _, out _, out Vector3D aV3b);
        Assert.Equal(aV3b.X(), aV3.X(), 1e-8);
        Assert.Equal(aV3b.Y(), aV3.Y(), 1e-8);
    }

    [Fact]
    public void Evaluation_DN()
    {
        Vector3D aDN1 = myOriginalCurve.DN(0.5, 1);
        Assert.True(aDN1.Magnitude() > 0.0);

        // DN(4) for a degree 3 curve should be zero
        Vector3D aDN4 = myOriginalCurve.DN(0.5, 4);
        Assert.Equal(0.0, aDN4.Magnitude(), 1e-10);
    }

    [Fact]
    public void StartEndPoints()
    {
        Point3D aStart = myOriginalCurve.StartPoint();
        Point3D anEnd = myOriginalCurve.EndPoint();
        Assert.True(aStart.IsEqual(new Point3D(0, 0, 0), 1e-10));
        Assert.True(anEnd.IsEqual(new Point3D(3, 0, 0), 1e-10));
    }

    [Fact]
    public void Properties()
    {
        Assert.Equal(3, myOriginalCurve.Degree());
        Assert.False(myOriginalCurve.IsPeriodic);
        Assert.False(myOriginalCurve.IsRational);
        Assert.False(myOriginalCurve.IsClosed);
        Assert.True(myOriginalCurve.IsCN(100));
        Assert.Equal(Continuity.CN, myOriginalCurve.Continuity());
        Assert.Equal(0.0, myOriginalCurve.FirstParameter());
        Assert.Equal(1.0, myOriginalCurve.LastParameter());
    }

    [Fact]
    public void SetPole()
    {
        Point3D aNewPole = new(1, 5, 0);
        myOriginalCurve.SetPole(2, aNewPole);
        Assert.True(myOriginalCurve.Pole(2).IsEqual(aNewPole, 1e-10));
        // Endpoints unchanged
        Assert.True(myOriginalCurve.StartPoint().IsEqual(new Point3D(0, 0, 0), 1e-10));
        Assert.True(myOriginalCurve.EndPoint().IsEqual(new Point3D(3, 0, 0), 1e-10));
    }

    [Fact]
    public void SetWeight()
    {
        BezierCurve3D aCurve = new(Poles3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)),
                                      Weights3(1.0, 1.0, 1.0));
        Point3D aMidBefore = aCurve.Value(0.5);

        aCurve.SetWeight(2, 10.0);
        Assert.Equal(10.0, aCurve.Weight(2));
        Assert.True(aCurve.IsRational);

        Point3D aMidAfter = aCurve.Value(0.5);
        // High weight pulls curve toward the control point
        Assert.True(aMidAfter.Y() > aMidBefore.Y());
    }

    [Fact]
    public void InsertPoleAfter()
    {
        int aNbBefore = myOriginalCurve.PoleCount;
        Point3D aNewPole = new(1.5, 2.0, 0);
        myOriginalCurve.InsertPoleAfter(2, aNewPole);
        Assert.Equal(aNbBefore + 1, myOriginalCurve.PoleCount);
        Assert.Equal(4, myOriginalCurve.Degree());
        Assert.True(myOriginalCurve.Pole(3).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void InsertPoleBefore()
    {
        int aNbBefore = myOriginalCurve.PoleCount;
        Point3D aNewPole = new(0.5, 2.0, 0);
        myOriginalCurve.InsertPoleBefore(2, aNewPole);
        Assert.Equal(aNbBefore + 1, myOriginalCurve.PoleCount);
        Assert.True(myOriginalCurve.Pole(2).IsEqual(aNewPole, 1e-10));
    }

    [Fact]
    public void RemovePole()
    {
        // Add a pole first so we have 5, then remove one
        myOriginalCurve.InsertPoleAfter(2, new Point3D(1.5, 2, 0));
        Assert.Equal(5, myOriginalCurve.PoleCount);
        myOriginalCurve.RemovePole(3);
        Assert.Equal(4, myOriginalCurve.PoleCount);
        Assert.Equal(3, myOriginalCurve.Degree());
    }

    [Fact]
    public void Increase()
    {
        Point3D aValBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.Increase(5);
        Assert.Equal(5, myOriginalCurve.Degree());
        Assert.Equal(6, myOriginalCurve.PoleCount);
        Point3D aValAfter = myOriginalCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void Segment()
    {
        Point3D aP25 = myOriginalCurve.Value(0.25);
        Point3D aP75 = myOriginalCurve.Value(0.75);
        myOriginalCurve.Segment(0.25, 0.75);
        Assert.True(myOriginalCurve.StartPoint().IsEqual(aP25, 1e-6));
        Assert.True(myOriginalCurve.EndPoint().IsEqual(aP75, 1e-6));
    }

    [Fact]
    public void Reverse()
    {
        Point3D aStart = myOriginalCurve.StartPoint();
        Point3D anEnd = myOriginalCurve.EndPoint();
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
    public void Transform()
    {
        Transform3D aTrsf = new Transform3D().WithTranslation(new Vector3D(10, 20, 30));
        Point3D aPtBefore = myOriginalCurve.Value(0.5);
        myOriginalCurve.Transform(aTrsf);
        Point3D aPtAfter = myOriginalCurve.Value(0.5);
        Assert.Equal(aPtBefore.X() + 10.0, aPtAfter.X(), 1e-10);
        Assert.Equal(aPtBefore.Y() + 20.0, aPtAfter.Y(), 1e-10);
        Assert.Equal(aPtBefore.Z() + 30.0, aPtAfter.Z(), 1e-10);
    }

    [Fact]
    public void PolesAccess()
    {
        // Poles() returns an immutable copy indexed from 0 (OCCT: a reference indexed from 1)
        PackedList<Point3D> aPoles = myOriginalCurve.Poles();
        Assert.Equal(4, aPoles.Count);
        Assert.True(aPoles[0].IsEqual(new Point3D(0, 0, 0), 1e-10));
    }

    [Fact]
    public void WeightsAccess_NonRational()
    {
        PackedList<double>? aWeights = myOriginalCurve.Weights();
        Assert.Null(aWeights);
        Assert.Null(myOriginalCurve.InternalWeights());
    }

    [Fact]
    public void RationalCurveEvaluation()
    {
        // Create a rational quadratic Bezier (approximation of circular arc)
        BezierCurve3D aCurve = new(Poles3(new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0)),
                                      Weights3(1.0, 1.0 / Math.Sqrt(2.0), 1.0));
        Assert.True(aCurve.IsRational);

        // At u=0.5, the rational curve should be close to the unit circle
        Point3D aMid = aCurve.Value(0.5);
        double aRadius = Math.Sqrt(aMid.X() * aMid.X() + aMid.Y() * aMid.Y());
        Assert.Equal(1.0, aRadius, 1e-6);
    }

    [Fact]
    public void MaxDegree()
    {
        Assert.True(BezierCurve3D.MaxDegree() >= 25);
    }

    [Fact]
    public void SetPoleWithWeight()
    {
        BezierCurve3D aCurve = new(Poles3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)),
                                      Weights3(1.0, 1.0, 1.0));
        Point3D aNewPole = new(1, 2, 0);
        aCurve.SetPole(2, aNewPole, 5.0);
        Assert.True(aCurve.Pole(2).IsEqual(aNewPole, 1e-10));
        Assert.Equal(5.0, aCurve.Weight(2));
        Assert.True(aCurve.IsRational);
    }

    [Fact]
    public void InsertPoleAfterWithWeight()
    {
        BezierCurve3D aCurve = new(Poles3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)),
                                      Weights3(1.0, 2.0, 1.0));
        aCurve.InsertPoleAfter(2, new Point3D(1.5, 0.5, 0), 3.0);
        Assert.Equal(4, aCurve.PoleCount);
        Assert.Equal(3.0, aCurve.Weight(3));
        Assert.True(aCurve.IsRational);
    }

    [Fact]
    public void ClosedCurve()
    {
        NCollection_Array1<Point3D> aPoles = new(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 1, 0);
        aPoles[3] = new Point3D(2, 0, 0);
        aPoles[4] = new Point3D(0, 0, 0);
        BezierCurve3D aCurve = new(aPoles);
        Assert.True(aCurve.IsClosed);
    }

    [Fact]
    public void RationalSegment()
    {
        BezierCurve3D aCurve = new(Poles3(new Point3D(1, 0, 0), new Point3D(1, 1, 0), new Point3D(0, 1, 0)),
                                      Weights3(1.0, 1.0 / Math.Sqrt(2.0), 1.0));
        Point3D aMid = aCurve.Value(0.5);
        aCurve.Segment(0.25, 0.75);
        Point3D aMidAfter = aCurve.Value(0.5);
        Assert.True(aMid.IsEqual(aMidAfter, 1e-6));
        Assert.True(aCurve.IsRational);
    }

    [Fact]
    public void RationalIncrease()
    {
        BezierCurve3D aCurve = new(Poles3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)),
                                      Weights3(1.0, 2.0, 1.0));
        Point3D aValBefore = aCurve.Value(0.5);
        aCurve.Increase(5);
        Assert.Equal(5, aCurve.Degree());
        Assert.True(aCurve.IsRational);
        Point3D aValAfter = aCurve.Value(0.5);
        Assert.True(aValBefore.IsEqual(aValAfter, 1e-10));
    }

    [Fact]
    public void RationalReverse()
    {
        BezierCurve3D aCurve = new(Poles3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)),
                                      Weights3(1.0, 3.0, 2.0));
        Point3D aStart = aCurve.StartPoint();
        Point3D anEnd = aCurve.EndPoint();
        aCurve.Reverse();
        Assert.True(aCurve.StartPoint().IsEqual(anEnd, 1e-10));
        Assert.True(aCurve.EndPoint().IsEqual(aStart, 1e-10));
        // Weights should be reversed
        Assert.Equal(2.0, aCurve.Weight(1));
        Assert.Equal(1.0, aCurve.Weight(3));
    }

    [Fact]
    public void ReversedParameter()
    {
        Assert.Equal(0.7, myOriginalCurve.ReversedParameter(0.3), 15);
        Assert.Equal(1.0, myOriginalCurve.ReversedParameter(0.0));
    }

    [Fact]
    public void LinearCurve()
    {
        // Degree 1 Bezier = line segment
        NCollection_Array1<Point3D> aPoles = new(1, 2);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(3, 4, 0);
        BezierCurve3D aCurve = new(aPoles);
        Assert.Equal(1, aCurve.Degree());

        Point3D aMid = aCurve.Value(0.5);
        Assert.True(aMid.IsEqual(new Point3D(1.5, 2.0, 0), 1e-10));

        // D2 should be zero for a line
        aCurve.D2(0.5, out _, out _, out Vector3D aV2);
        Assert.Equal(0.0, aV2.Magnitude(), 1e-10);
    }

    // OCCT checks the reference returned by WeightsArray(); the port's public WeightsArray() is a copy, so the
    // identity and ownership checks use the internal accessor.

    [Fact]
    public void WeightsArray_NonRational_ReturnsUnitWeights()
    {
        Assert.False(myOriginalCurve.IsRational);
        NCollection_Array1<double> aWeights = myOriginalCurve.InternalWeightsArray();
        Assert.Equal(myOriginalCurve.PoleCount, aWeights.Length());
        Assert.False(aWeights.IsDeletable());
        for (int i = 1; i <= aWeights.Length(); ++i)
        {
            Assert.Equal(1.0, aWeights[i]);
        }
        Assert.Same(aWeights, myOriginalCurve.InternalWeightsArray());
        Assert.Equal(aWeights.AsSpan().ToArray(), myOriginalCurve.WeightsArray().AsSpan().ToArray());
    }

    [Fact]
    public void WeightsArray_Rational_ReturnsOwning()
    {
        BezierCurve3D aRational = new(Poles3(new Point3D(0, 0, 0), new Point3D(1, 1, 0), new Point3D(2, 0, 0)),
                                         Weights3(1.0, 2.0, 1.0));
        Assert.True(aRational.IsRational);

        NCollection_Array1<double> aWeights = aRational.InternalWeightsArray();
        Assert.Equal(3, aWeights.Length());
        Assert.True(aWeights.IsDeletable());
        Assert.Equal(1.0, aWeights[1]);
        Assert.Equal(2.0, aWeights[2]);
        Assert.Equal(1.0, aWeights[3]);
        Assert.Same(aWeights, aRational.InternalWeightsArray());
        Assert.Equal([1.0, 2.0, 1.0], aRational.WeightsArray().AsSpan().ToArray());
    }
}

public class Geom_BezierCurveTest
{
    [Fact]
    public void OCC2569_DegreeEqualsNbPolesMinusOne()
    {
        const int aNbPoles = 26;
        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; ++i)
        {
            aPoles.SetValue(i, new Point3D(i + 10, i * 2 + 20, i * 3 + 45));
        }

        BezierCurve3D aCurve = new(aPoles);
        Assert.Equal(aNbPoles - 1, aCurve.Degree());
    }

    [Fact]
    public void OCC2569_ThrowsForTooManyPoles()
    {
        const int aNbPoles = 29;
        NCollection_Array1<Point3D> aPoles = new(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; ++i)
        {
            aPoles.SetValue(i, new Point3D(i + 10, i * 2 + 20, i * 3 + 45));
        }

        Assert.ThrowsAny<FailureException>(() => new BezierCurve3D(aPoles));
    }
}
