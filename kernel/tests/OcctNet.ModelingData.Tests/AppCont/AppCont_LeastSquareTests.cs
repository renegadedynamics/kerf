// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Engine.Approximation.Constraint;

namespace OcctNet.ModelingData.Tests;

// AppCont_LeastSquare on functions with known approximations (OCCT has no GTests for it).
public class AppCont_LeastSquareTests
{
    // f(u) = a cubic 3d curve and, optionally, a quadratic 2d curve, on [U0, U1].
    private sealed class PolynomialFunction : Function
    {
        private readonly double myU0, myU1;

        public PolynomialFunction(double theU0, double theU1, bool theWith2d)
        {
            myU0 = theU0;
            myU1 = theU1;
            _nbPnt = 1;
            _nbPnt2d = theWith2d ? 1 : 0;
        }

        public static Point3D P3d(double u) => new(1.0 + 2.0 * u - u * u * u, 0.5 * u * u, 3.0 - u + 0.25 * u * u * u);

        public static Vector3D V3d(double u) => new(2.0 - 3.0 * u * u, u, -1.0 + 0.75 * u * u);

        public static Point2D P2d(double u) => new(u * u - 1.0, 2.0 * u + 0.5);

        public static Vector2D V2d(double u) => new(2.0 * u, 2.0);

        public override double FirstParameter() => myU0;

        public override double LastParameter() => myU1;

        public override bool Value(double theU, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
        {
            thePnt[1] = P3d(theU);
            if (_nbPnt2d == 1)
            {
                thePnt2d[1] = P2d(theU);
            }
            return true;
        }

        public override bool D1(double theU, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec)
        {
            theVec[1] = V3d(theU);
            if (_nbPnt2d == 1)
            {
                theVec2d[1] = V2d(theU);
            }
            return true;
        }
    }

    // A unit circle arc in the XY plane, 3d, with or without derivatives.
    private sealed class CircleFunction : Function
    {
        private readonly double myU0, myU1;
        private readonly bool myHasD1;

        public CircleFunction(double theU0, double theU1, bool theHasD1)
        {
            myU0 = theU0;
            myU1 = theU1;
            myHasD1 = theHasD1;
            _nbPnt = 1;
            _nbPnt2d = 0;
        }

        public override double FirstParameter() => myU0;

        public override double LastParameter() => myU1;

        public override bool Value(double theU, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
        {
            thePnt[1] = new Point3D(Math.Cos(theU), Math.Sin(theU), 0.0);
            return true;
        }

        public override bool D1(double theU, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec)
        {
            theVec[1] = new Vector3D(-Math.Sin(theU), Math.Cos(theU), 0.0);
            return myHasD1;
        }
    }

    private static readonly Constraint[] s_constraints =
        [NoConstraint, PassPoint, TangencyPoint, CurvaturePoint];

    [Fact]
    public void Polynomial_IsReproducedUnderEveryEndConstraint()
    {
        const double U0 = -0.5, U1 = 1.5;
        foreach (Constraint aFirst in s_constraints)
        {
            foreach (Constraint aLast in s_constraints)
            {
                PolynomialFunction aFunc = new(U0, U1, true);
                LeastSquare aLSQ = new(aFunc, U0, U1, aFirst, aLast, 5, 12);
                // OCCT sets IsDone only when it solves a constrained system (occt-issues.md)
                bool aFree = aFirst == NoConstraint && aLast == NoConstraint;
                Assert.Equal(!aFree, aLSQ.IsDone);

                MultiCurve aCurve = aLSQ.Value();
                Assert.Equal(6, aCurve.NbPoles());
                Assert.Equal(2, aCurve.NbCurves());
                for (int i = 0; i <= 20; i++)
                {
                    double t = i / 20.0, u = U0 + t * (U1 - U0);
                    aCurve.Value(1, t, out Point3D aP);
                    aCurve.Value(2, t, out Point2D aP2d);
                    Assert.True(aP.Distance(PolynomialFunction.P3d(u)) < 1e-10, $"{aFirst}/{aLast} 3d at {t}");
                    Assert.True(aP2d.Distance(PolynomialFunction.P2d(u)) < 1e-10, $"{aFirst}/{aLast} 2d at {t}");
                }
                aLSQ.Error(out double F, out double aMax3d, out double aMax2d);
                Assert.True(F < 1e-20 && aMax3d < 1e-10 && aMax2d < 1e-10, $"{aFirst}/{aLast}: {F} {aMax3d} {aMax2d}");
            }
        }
    }

    [Fact]
    public void Constraints_FixTheEndPolesExactly()
    {
        const double U0 = 0.0, U1 = 1.0;
        PolynomialFunction aFunc = new(U0, U1, false);
        LeastSquare aLSQ = new(aFunc, U0, U1, TangencyPoint, PassPoint, 4, 10);
        MultiCurve aCurve = aLSQ.Value();
        Assert.Equal(PolynomialFunction.P3d(U0), aCurve.Pole(1, 1));
        Assert.Equal(PolynomialFunction.P3d(U1), aCurve.Pole(1, 5));
        // second pole = first pole + tangent * (U1 - U0) / degree
        Point3D anExpected = PolynomialFunction.P3d(U0).Translated(PolynomialFunction.V3d(U0) * ((U1 - U0) / 4));
        Assert.True(aCurve.Pole(1, 2).Distance(anExpected) < 1e-15);
    }

    [Theory]
    [InlineData(NoConstraint, NoConstraint)]
    [InlineData(PassPoint, PassPoint)]
    [InlineData(TangencyPoint, TangencyPoint)]
    [InlineData(PassPoint, TangencyPoint)]
    [InlineData(CurvaturePoint, CurvaturePoint)]
    public void CircleArc_IsApproximatedWithinTolerance(Constraint theFirst, Constraint theLast)
    {
        const double U0 = 0.2, U1 = 0.2 + M_PI / 2;
        CircleFunction aFunc = new(U0, U1, true);
        LeastSquare aLSQ = new(aFunc, U0, U1, theFirst, theLast, 6, 24);
        MultiCurve aCurve = aLSQ.Value();
        aLSQ.Error(out _, out double aMax3d, out double aMax2d);
        Assert.True(aMax3d < 1e-5, $"error {aMax3d}");
        Assert.Equal(0.0, aMax2d);
        for (int i = 0; i <= 40; i++)
        {
            double t = i / 40.0, u = U0 + t * (U1 - U0);
            aCurve.Value(1, t, out Point3D aP);
            // radial distance from the unit circle
            Assert.True(Math.Abs(Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()) - 1.0) < 1e-5);
            Assert.Equal(0.0, aP.Z());
            Assert.True(aP.Distance(new Point3D(Math.Cos(u), Math.Sin(u), 0.0)) < 1e-4);
        }
        if (theFirst is PassPoint or TangencyPoint)
        {
            Assert.True(aCurve.Pole(1, 1).Distance(new Point3D(Math.Cos(U0), Math.Sin(U0), 0.0)) < 1e-15);
        }
    }

    [Fact]
    public void Tangency_WithoutDerivative_FallsBackToPassPoint()
    {
        const double U0 = 0.0, U1 = 1.0;
        LeastSquare aWithout = new(new CircleFunction(U0, U1, false), U0, U1,
                                           TangencyPoint, TangencyPoint, 5, 12);
        LeastSquare aPass = new(new CircleFunction(U0, U1, true), U0, U1,
                                        PassPoint, PassPoint, 5, 12);
        MultiCurve aC1 = aWithout.Value(), aC2 = aPass.Value();
        for (int i = 1; i <= 6; i++)
        {
            Assert.Equal(aC2.Pole(1, i), aC1.Pole(1, i));
        }
    }

    [Fact]
    public void Value_IsTheObjectsOwnMultiCurve()
    {
        PolynomialFunction aFunc = new(0.0, 1.0, false);
        LeastSquare aLSQ = new(aFunc, 0.0, 1.0, PassPoint, PassPoint, 3, 8);
        Assert.Same(aLSQ.Value(), aLSQ.Value());
    }

    [Fact]
    public void Degrees_AboveTheTables_Throw()
    {
        PolynomialFunction aFunc = new(0.0, 1.0, false);
        // MMatrix holds classe up to 24 (degree 23), VBernstein up to 24 points
        Assert.ThrowsAny<DimensionErrorException>(() => new LeastSquare(aFunc, 0.0, 1.0, PassPoint,
                                                                               PassPoint, 24, 24));
        Assert.ThrowsAny<DimensionErrorException>(() => new LeastSquare(aFunc, 0.0, 1.0, PassPoint,
                                                                               PassPoint, 5, 25));
    }
}
