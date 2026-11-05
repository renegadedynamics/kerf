// Port of: src/FoundationClasses/TKMath/GTests/math_FunctionAllRoots_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_FunctionAllRootsTest
{
    // Test function with multiple roots: f(x) = x^3 - 6x^2 + 11x - 6 = (x-1)(x-2)(x-3)
    private sealed class CubicFunction : FunctionWithDerivative
    {
        public override bool Value(double X, out double F)
        {
            F = X * X * X - 6.0 * X * X + 11.0 * X - 6.0;
            return true;
        }

        public override bool Derivative(double X, out double D)
        {
            D = 3.0 * X * X - 12.0 * X + 11.0;
            return true;
        }

        public override bool Values(double X, out double F, out double D)
        {
            F = X * X * X - 6.0 * X * X + 11.0 * X - 6.0;
            D = 3.0 * X * X - 12.0 * X + 11.0;
            return true;
        }
    }

    // Sine function with multiple roots
    private sealed class SineFunction : FunctionWithDerivative
    {
        public override bool Value(double X, out double F)
        {
            F = Math.Sin(X);
            return true;
        }

        public override bool Derivative(double X, out double D)
        {
            D = Math.Cos(X);
            return true;
        }

        public override bool Values(double X, out double F, out double D)
        {
            F = Math.Sin(X);
            D = Math.Cos(X);
            return true;
        }
    }

    // Function with a null interval: f(x) = 0 for x in [2, 4]
    private sealed class NullIntervalFunctionImpl : FunctionWithDerivative
    {
        public override bool Value(double X, out double F)
        {
            if (X >= 2.0 && X <= 4.0)
            {
                F = 0.0;
            }
            else if (X < 2.0)
            {
                F = X - 2.0;
            }
            else
            {
                F = X - 4.0;
            }
            return true;
        }

        public override bool Derivative(double X, out double D)
        {
            if (X >= 2.0 && X <= 4.0)
            {
                D = 0.0;
            }
            else
            {
                D = 1.0;
            }
            return true;
        }

        public override bool Values(double X, out double F, out double D)
        {
            if (X >= 2.0 && X <= 4.0)
            {
                F = 0.0;
                D = 0.0;
            }
            else if (X < 2.0)
            {
                F = X - 2.0;
                D = 1.0;
            }
            else
            {
                F = X - 4.0;
                D = 1.0;
            }
            return true;
        }
    }

    // Function with one root: f(x) = (x-1.5)^2 - 0.25 = (x-1)(x-2)
    private sealed class QuadraticFunction : FunctionWithDerivative
    {
        public override bool Value(double X, out double F)
        {
            F = (X - 1.5) * (X - 1.5) - 0.25;
            return true;
        }

        public override bool Derivative(double X, out double D)
        {
            D = 2.0 * (X - 1.5);
            return true;
        }

        public override bool Values(double X, out double F, out double D)
        {
            F = (X - 1.5) * (X - 1.5) - 0.25;
            D = 2.0 * (X - 1.5);
            return true;
        }
    }

    // Constant zero function
    private sealed class ZeroFunctionImpl : FunctionWithDerivative
    {
        public override bool Value(double _, out double F)
        {
            F = 0.0;
            return true;
        }

        public override bool Derivative(double _, out double D)
        {
            D = 0.0;
            return true;
        }

        public override bool Values(double _, out double F, out double D)
        {
            F = 0.0;
            D = 0.0;
            return true;
        }
    }

    [Fact]
    public void CubicFunctionBasic()
    {
        CubicFunction func = new();
        FunctionSample sample = new(0.0, 5.0, 21);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-8);

        Assert.True(solver.IsDone);

        // Should find the three roots at x = 1, 2, 3
        Assert.True(solver.PointCount >= 2);

        if (solver.PointCount >= 2)
        {
            double root1 = solver.GetPoint(1);
            double root2 = solver.GetPoint(2);

            // Check if we found roots near 1, 2, or 3
            Assert.True(Math.Abs(root1 - 1.0) < 1.0e-4 || Math.Abs(root1 - 2.0) < 1.0e-4
                                    || Math.Abs(root1 - 3.0) < 1.0e-4);
            Assert.True(Math.Abs(root2 - 1.0) < 1.0e-4 || Math.Abs(root2 - 2.0) < 1.0e-4
                                    || Math.Abs(root2 - 3.0) < 1.0e-4);
        }
    }

    [Fact]
    public void SineFunctionRoots()
    {
        SineFunction func = new();
        FunctionSample sample = new(-1.0, 7.0, 41);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-8);

        Assert.True(solver.IsDone);

        // Should find roots near 0, PI, 2*PI within the range
        Assert.True(solver.PointCount >= 2);

        if (solver.PointCount >= 1)
        {
            double root1 = solver.GetPoint(1);
            // Check if we found a root near 0 or PI
            Assert.True(Math.Abs(root1) < 1.0e-4 || Math.Abs(root1 - M_PI) < 1.0e-4
                                    || Math.Abs(root1 - 2 * M_PI) < 1.0e-4);
        }
    }

    [Fact]
    public void NullIntervalFunction()
    {
        NullIntervalFunctionImpl func = new();
        FunctionSample sample = new(0.0, 6.0, 31);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-6);

        Assert.True(solver.IsDone);

        // Should find a null interval from x = 2 to x = 4
        Assert.True(solver.IntervalCount >= 1);

        if (solver.IntervalCount >= 1)
        {
            double a, b;
            solver.GetInterval(1, out a, out b);
            Assert.Equal(2.0, a, 1.0e-3);
            Assert.Equal(4.0, b, 1.0e-3);
        }
    }

    [Fact]
    public void QuadraticTwoRoots()
    {
        QuadraticFunction func = new();
        FunctionSample sample = new(0.0, 3.0, 16);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-8);

        Assert.True(solver.IsDone);

        // Should find roots at x = 1 and x = 2
        Assert.True(solver.PointCount >= 1);

        if (solver.PointCount >= 1)
        {
            double root1 = solver.GetPoint(1);
            Assert.True(Math.Abs(root1 - 1.0) < 1.0e-4 || Math.Abs(root1 - 2.0) < 1.0e-4);
        }
    }

    [Fact]
    public void ZeroFunction()
    {
        ZeroFunctionImpl func = new();
        FunctionSample sample = new(0.0, 1.0, 11);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-6);

        Assert.True(solver.IsDone);

        // Should find the entire interval as null
        Assert.True(solver.IntervalCount >= 1);

        if (solver.IntervalCount >= 1)
        {
            double a, b;
            solver.GetInterval(1, out a, out b);
            Assert.Equal(0.0, a, 1.0e-3);
            Assert.Equal(1.0, b, 1.0e-3);
        }
    }

    [Fact]
    public void GetIntervalState()
    {
        NullIntervalFunctionImpl func = new();
        FunctionSample sample = new(0.0, 6.0, 31);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-6);

        Assert.True(solver.IsDone);

        if (solver.IntervalCount >= 1)
        {
            int iFirst, iLast;
            solver.GetIntervalState(1, out iFirst, out iLast);
            Assert.True(iFirst >= 0);
            Assert.True(iLast >= 0);
            Assert.True(iLast >= iFirst);
        }
    }

    [Fact]
    public void GetPointState()
    {
        CubicFunction func = new();
        FunctionSample sample = new(0.0, 5.0, 21);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-8);

        Assert.True(solver.IsDone);

        if (solver.PointCount >= 1)
        {
            int state = solver.GetPointState(1);
            Assert.True(state >= 0);
        }
    }

    [Fact]
    public void LargeSampleSize()
    {
        CubicFunction func = new();
        FunctionSample sample = new(0.0, 5.0, 101);
        FunctionAllRoots solver = new(func, sample, 1.0e-8, 1.0e-8, 1.0e-10);

        Assert.True(solver.IsDone);
        Assert.True(solver.PointCount >= 2);
    }

    [Fact]
    public void SmallSampleSize()
    {
        CubicFunction func = new();
        FunctionSample sample = new(0.0, 5.0, 5);
        FunctionAllRoots solver = new(func, sample, 1.0e-4, 1.0e-4, 1.0e-6);

        Assert.True(solver.IsDone);
        // With small sample, might miss some roots
        Assert.True(solver.PointCount >= 0);
    }

    [Fact]
    public void TightTolerances()
    {
        QuadraticFunction func = new();
        FunctionSample sample = new(0.0, 3.0, 31);
        FunctionAllRoots solver = new(func, sample, 1.0e-10, 1.0e-10, 1.0e-12);

        Assert.True(solver.IsDone);
        Assert.True(solver.PointCount >= 1);
    }

    [Fact]
    public void LooseTolerances()
    {
        CubicFunction func = new();
        FunctionSample sample = new(0.0, 5.0, 21);
        FunctionAllRoots solver = new(func, sample, 1.0e-2, 1.0e-2, 1.0e-4);

        Assert.True(solver.IsDone);
        Assert.True(solver.PointCount >= 0);
    }

    [Fact]
    public void NarrowRange()
    {
        QuadraticFunction func = new();
        FunctionSample sample = new(0.5, 2.5, 21);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-8);

        Assert.True(solver.IsDone);
        // Should find roots at x = 1 and x = 2 within this range
        Assert.True(solver.PointCount >= 1);
    }

    [Fact]
    public void WideRange()
    {
        SineFunction func = new();
        FunctionSample sample = new(-10.0, 10.0, 81);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-8);

        Assert.True(solver.IsDone);
        // Should find multiple roots in this wide range
        Assert.True(solver.PointCount >= 5);
    }

    [Fact]
    public void EmptyRange()
    {
        CubicFunction func = new();
        FunctionSample sample = new(10.0, 20.0, 11);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-8);

        Assert.True(solver.IsDone);
        // No roots expected in this range for cubic function
        Assert.Equal(0, solver.PointCount);
        Assert.Equal(0, solver.IntervalCount);
    }

    [Fact]
    public void IntervalBounds()
    {
        NullIntervalFunctionImpl func = new();
        FunctionSample sample = new(0.0, 6.0, 31);
        FunctionAllRoots solver = new(func, sample, 1.0e-6, 1.0e-6, 1.0e-6);

        Assert.True(solver.IsDone);

        if (solver.IntervalCount >= 1)
        {
            double a, b;
            solver.GetInterval(1, out a, out b);
            Assert.True(a < b);
            Assert.True(a >= 0.0);
            Assert.True(b <= 6.0);
        }
    }

    [Fact]
    public void MultiplePrecisionLevels()
    {
        CubicFunction func = new();

        // Test with different precision levels
        List<double> tolerances = [1.0e-4, 1.0e-6, 1.0e-8];

        foreach (double tol in tolerances)
        {
            FunctionSample sample = new(0.0, 5.0, 21);
            FunctionAllRoots solver = new(func, sample, tol, tol, tol * 0.01);

            Assert.True(solver.IsDone);
            Assert.True(solver.PointCount >= 0);
        }
    }
}
