// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;

namespace Kerf.Kernel.ModelingData.Tests;

// Approx_FitAndDivide and Approx_FitAndDivide2d (Approx_ComputeCLine.gxx) on functions with known approximations
// (OCCT has no GTests for them).
public class Approx_FitAndDivideTests
{
    // A cubic 3d curve on [U0, U1].
    private sealed class CubicFunction : Function
    {
        private readonly double myU0, myU1;

        public CubicFunction(double theU0, double theU1)
        {
            myU0 = theU0;
            myU1 = theU1;
            _nbPnt = 1;
            _nbPnt2d = 0;
        }

        public static Point3D P(double u) => new(1.0 + 2.0 * u - u * u * u, 0.5 * u * u, 3.0 - u + 0.25 * u * u * u);

        public override double FirstParameter() => myU0;

        public override double LastParameter() => myU1;

        public override bool Value(double theU, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
        {
            thePnt[1] = P(theU);
            return true;
        }

        public override bool D1(double theU, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec)
        {
            theVec[1] = new Vector3D(2.0 - 3.0 * theU * theU, theU, -1.0 + 0.75 * theU * theU);
            return true;
        }
    }

    // A circle of radius R, in 3d (XY plane) or in 2d, on [U0, U1].
    private sealed class CircleFunction : Function
    {
        private readonly double myU0, myU1, myR;

        public CircleFunction(double theU0, double theU1, double theR, bool the2d)
        {
            myU0 = theU0;
            myU1 = theU1;
            myR = theR;
            _nbPnt = the2d ? 0 : 1;
            _nbPnt2d = the2d ? 1 : 0;
        }

        public override double FirstParameter() => myU0;

        public override double LastParameter() => myU1;

        public override bool Value(double theU, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
        {
            if (_nbPnt == 1)
            {
                thePnt[1] = new Point3D(myR * Math.Cos(theU), myR * Math.Sin(theU), 0.0);
            }
            else
            {
                thePnt2d[1] = new Point2D(myR * Math.Cos(theU), myR * Math.Sin(theU));
            }
            return true;
        }

        public override bool D1(double theU, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec)
        {
            if (_nbPnt == 1)
            {
                theVec[1] = new Vector3D(-myR * Math.Sin(theU), myR * Math.Cos(theU), 0.0);
            }
            else
            {
                theVec2d[1] = new Vector2D(-myR * Math.Sin(theU), myR * Math.Cos(theU));
            }
            return true;
        }
    }

    // Evaluates the Bezier MultiCurve's first 3d curve at the parameter u of [U0, U1].
    private static Point3D Value3d(MultiCurve theCurve, double U0, double U1, double u)
    {
        theCurve.Value(1, (u - U0) / (U1 - U0), out Point3D aP);
        return aP;
    }

    private static Point2D Value2d(MultiCurve theCurve, double U0, double U1, double u)
    {
        theCurve.Value(1, (u - U0) / (U1 - U0), out Point2D aP);
        return aP;
    }

    [Fact]
    public void Cubic_WithFreeEnds_IsApproximated()
    {
        // An unconstrained continuous approximation is done: one curve through the cubic (OCCT's AppCont_LeastSquare
        // reported not done with free ends, and so did Approx_ComputeCLine: occt-issues.md, decision 1470)
        CubicFunction aFunc = new(-1.0, 2.0);
        FitAndDivide anApprox = new(aFunc, 3, 8, 1.0e-6, 1.0e-6, false, NoConstraint, NoConstraint);
        Assert.True(anApprox.IsAllApproximated);
        Assert.True(anApprox.IsToleranceReached);
        Assert.Equal(1, anApprox.MultiCurveCount);
        MultiCurve aCurve = anApprox.Value(1);
        for (int i = 0; i <= 10; i++)
        {
            double u = -1.0 + 0.3 * i;
            Assert.True(Value3d(aCurve, -1.0, 2.0, u).Distance(CubicFunction.P(u)) < 1.0e-9);
        }
    }

    [Fact]
    public void Cubic_IsReproducedByOneCurve()
    {
        CubicFunction aFunc = new(-1.0, 2.0);
        FitAndDivide anApprox = new(aFunc, 3, 8, 1.0e-6, 1.0e-6);

        Assert.True(anApprox.IsAllApproximated);
        Assert.True(anApprox.IsToleranceReached);
        Assert.Equal(1, anApprox.MultiCurveCount);
        anApprox.Error(1, out double aTol3d, out double aTol2d);
        Assert.True(aTol3d < 1.0e-9);
        Assert.Equal(0.0, aTol2d);
        anApprox.Parameters(1, out double aFirst, out double aLast);
        Assert.Equal(-1.0, aFirst);
        Assert.Equal(2.0, aLast);

        // The inverse order (the default) keeps the lowest degree within the tolerance that AppCont_LeastSquare
        // solves: with tangency at both ends, degree 3 leaves no free pole (IsDone false), so degree 4.
        MultiCurve aCurve = anApprox.Value(1);
        Assert.Equal(4, aCurve.Degree());
        for (int i = 0; i <= 10; i++)
        {
            double u = -1.0 + 0.3 * i;
            Assert.True(Value3d(aCurve, aFirst, aLast, u).Distance(CubicFunction.P(u)) < 1.0e-9);
        }
    }

    [Fact]
    public void Cubic_ForwardOrder_StopsAtFirstDegreeWithinTolerance()
    {
        CubicFunction aFunc = new(0.0, 1.0);
        FitAndDivide anApprox = new(3, 6, 1.0e-6, 1.0e-6, false, PassPoint, PassPoint);
        anApprox.SetInvOrder(false);
        anApprox.Perform(aFunc);
        Assert.True(anApprox.IsAllApproximated);
        Assert.Equal(1, anApprox.MultiCurveCount);
        Assert.Equal(3, anApprox.Value().Degree());
    }

    [Fact]
    public void Circle_WithCutting_CoversTheRangeWithinTolerance()
    {
        const double aTol = 1.0e-7;
        CircleFunction aFunc = new(0.0, 2.0 * Math.PI, 2.0, false);
        FitAndDivide anApprox = new(aFunc, 3, 6, aTol, aTol, true);

        Assert.True(anApprox.IsAllApproximated);
        Assert.True(anApprox.IsToleranceReached);
        int aNb = anApprox.MultiCurveCount;
        Assert.True(aNb > 1);

        double aPrevLast = 0.0;
        for (int i = 1; i <= aNb; i++)
        {
            anApprox.Parameters(i, out double aFirst, out double aLast);
            Assert.Equal(aPrevLast, aFirst);
            aPrevLast = aLast;
            anApprox.Error(i, out double aTol3d, out _);
            Assert.True(aTol3d <= aTol);

            MultiCurve aCurve = anApprox.Value(i);
            Assert.True(aCurve.Degree() >= 3 && aCurve.Degree() <= 6);
            for (int j = 0; j <= 8; j++)
            {
                double u = aFirst + (aLast - aFirst) * j / 8.0;
                Point3D aP = Value3d(aCurve, aFirst, aLast, u);
                Assert.True(aP.Distance(new Point3D(2.0 * Math.Cos(u), 2.0 * Math.Sin(u), 0.0)) < 10.0 * aTol);
            }
        }
        Assert.Equal(2.0 * Math.PI, aPrevLast);
    }

    [Fact]
    public void Circle_WithoutCutting_KeepsTheBestCurveOutOfTolerance()
    {
        CircleFunction aFunc = new(0.0, 2.0 * Math.PI, 1.0, false);
        FitAndDivide anApprox = new(aFunc, 2, 4, 1.0e-9, 1.0e-9, false);

        Assert.False(anApprox.IsAllApproximated);
        Assert.False(anApprox.IsToleranceReached);
        Assert.Equal(1, anApprox.MultiCurveCount);
        anApprox.Error(1, out double aTol3d, out _);
        Assert.True(aTol3d > 1.0e-9);
        // The kept curve is the one of the highest degree.
        Assert.Equal(4, anApprox.Value(1).Degree());
    }

    [Fact]
    public void Circle2d_WithCutting_AndMaxSegments()
    {
        const double aTol = 1.0e-6;
        CircleFunction aFunc = new(0.0, Math.PI, 1.5, true);
        FitAndDivide2d anApprox = new(3, 5, aTol, aTol, true);
        anApprox.SetHangChecking(false);
        anApprox.Perform(aFunc);

        Assert.True(anApprox.IsAllApproximated);
        for (int i = 1; i <= anApprox.MultiCurveCount; i++)
        {
            anApprox.Parameters(i, out double aFirst, out double aLast);
            anApprox.Error(i, out double aTol3d, out double aTol2d);
            Assert.Equal(0.0, aTol3d);
            Assert.True(aTol2d <= aTol);
            MultiCurve aCurve = anApprox.Value(i);
            double u = 0.5 * (aFirst + aLast);
            Point2D aP = Value2d(aCurve, aFirst, aLast, u);
            Assert.True(aP.Distance(new Point2D(1.5 * Math.Cos(u), 1.5 * Math.Sin(u))) < 10.0 * aTol);
        }

        // With one segment allowed, the cutting stops at once with the best curve found.
        FitAndDivide2d aLimited = new(3, 4, 1.0e-12, 1.0e-12, true);
        aLimited.SetMaxSegments(1);
        aLimited.Perform(new CircleFunction(0.0, Math.PI, 1.5, true));
        Assert.Equal(1, aLimited.MultiCurveCount);
        Assert.False(aLimited.IsToleranceReached);
    }

    [Fact]
    public void Value_IsACopySharingThePoles()
    {
        FitAndDivide anApprox = new(new CubicFunction(0.0, 1.0));
        MultiCurve aFirst = anApprox.Value(1);
        MultiCurve aSecond = anApprox.Value(1);
        Assert.NotSame(aFirst, aSecond);
        Assert.Equal(aFirst.Pole(1, 2), aSecond.Pole(1, 2));
    }
}
