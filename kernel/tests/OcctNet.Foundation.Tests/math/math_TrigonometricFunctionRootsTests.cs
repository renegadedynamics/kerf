// Port of: src/FoundationClasses/TKMath/GTests/math_TrigonometricFunctionRoots_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace OcctNet.Foundation.Tests;

public class math_TrigonometricFunctionRootsTest
{
    double PI        = M_PI;
    double TOLERANCE = 1.0e-6;

    [Fact]
    public void FullEquationBasic()
    {
        // Test a*cos^2(x) + 2*b*Math.Cos(x)*Math.Sin(x) + c*Math.Cos(x) + d*Math.Sin(x) + e = 0
        // Example: cos^2(x) - sin^2(x) = 0 => Math.Cos(2x) = 0
        // a=1, b=0, c=0, d=0, e=-sin^2(x) equivalent to: cos^2(x) - sin^2(x) = Math.Cos(2x) = 0
        // But let's use: cos^2(x) + c*Math.Cos(x) = 0 => Math.Cos(x)(Math.Cos(x) + c) = 0
        double                          a = 1.0, b = 0.0, c = 1.0, d = 0.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(a, b, c, d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount > 0);
    }

    [Fact]
    public void LinearSineOnly()
    {
        // Test d*Math.Sin(x) + e = 0 => Math.Sin(x) = -e/d
        // Example: Math.Sin(x) - 0.5 = 0 => Math.Sin(x) = 0.5 => x = PI/6, 5*PI/6
        double                          d = 1.0, e = -0.5;
        TrigonometricFunctionRoots solver = new(d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount > 0);

        if (solver.SolutionCount >= 1)
        {
            double x1      = solver.Value(1);
            double sin_val = Math.Sin(x1);
            Assert.Equal(0.5, sin_val, TOLERANCE);
        }
    }

    [Fact]
    public void LinearCosineAndSine()
    {
        // Test c*Math.Cos(x) + d*Math.Sin(x) + e = 0
        // Example: Math.Cos(x) + Math.Sin(x) = 0 => Math.Tan(x) = -1 => x = 3*PI/4, 7*PI/4
        double                          c = 1.0, d = 1.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(c, d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount > 0);

        if (solver.SolutionCount >= 1)
        {
            double x1     = solver.Value(1);
            double result = Math.Cos(x1) + Math.Sin(x1);
            Assert.Equal(0.0, result, TOLERANCE);
        }
    }

    [Fact]
    public void PureCosineEquation()
    {
        // Test Math.Cos(x) = 0 => x = PI/2, 3*PI/2
        double                          c = 1.0, d = 0.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(c, d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount >= 1);

        if (solver.SolutionCount >= 1)
        {
            double x1 = solver.Value(1);
            Assert.Equal(0.0, Math.Abs(Math.Cos(x1)), TOLERANCE);
        }
    }

    [Fact]
    public void PureSineEquation()
    {
        // Test Math.Sin(x) = 0 => x = 0, PI, 2*PI
        double                          d = 1.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount >= 2);

        if (solver.SolutionCount >= 1)
        {
            double x1 = solver.Value(1);
            Assert.Equal(0.0, Math.Abs(Math.Sin(x1)), TOLERANCE);
        }
    }

    [Fact]
    public void NoSolution()
    {
        // Test Math.Sin(x) + 2 = 0 => Math.Sin(x) = -2 (impossible)
        double                          d = 1.0, e = 2.0;
        TrigonometricFunctionRoots solver = new(d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.Equal(0, solver.SolutionCount);
    }

    [Fact]
    public void InfiniteSolutions()
    {
        // Test 0*Math.Sin(x) + 0 = 0 (always true)
        double                          d = 0.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.True(solver.InfiniteRoots());
    }

    [Fact]
    public void CustomBounds()
    {
        // Test Math.Sin(x) = 0 in range [PI/2, 3*PI/2]
        double                          d = 1.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(d, e, PI / 2.0, 3.0 * PI / 2.0);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount >= 1);

        for (int i = 1; i <= solver.SolutionCount; i++)
        {
            double x = solver.Value(i);
            Assert.True(x >= PI / 2.0);
            Assert.True(x <= 3.0 * PI / 2.0);
            Assert.Equal(0.0, Math.Abs(Math.Sin(x)), TOLERANCE);
        }
    }

    [Fact]
    public void NarrowBounds()
    {
        // Test Math.Cos(x) = 0 in range [0, PI/4]
        double                          c = 1.0, d = 0.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(c, d, e, 0.0, PI / 4.0);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        // No solutions expected in this narrow range
        Assert.Equal(0, solver.SolutionCount);
    }

    [Fact]
    public void QuadraticTerms()
    {
        // Test cos^2(x) - 0.5 = 0 => cos^2(x) = 0.5 => Math.Cos(x) = +/-Math.Sqrt(0.5)
        double                          a = 1.0, b = 0.0, c = 0.0, d = 0.0, e = -0.5;
        TrigonometricFunctionRoots solver = new(a, b, c, d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount >= 2);

        if (solver.SolutionCount >= 1)
        {
            double x1          = solver.Value(1);
            double cos_squared = Math.Cos(x1) * Math.Cos(x1);
            Assert.Equal(0.5, cos_squared, TOLERANCE);
        }
    }

    [Fact]
    public void MixedTerms()
    {
        // Test Math.Cos(x) + Math.Sin(x) - Math.Sqrt(2) = 0
        // This gives Math.Cos(x) + Math.Sin(x) = Math.Sqrt(2)
        // Which is Math.Sqrt(2)*Math.Sin(x + PI/4) = Math.Sqrt(2)
        // So Math.Sin(x + PI/4) = 1 => x + PI/4 = PI/2 => x = PI/4
        double                          c = 1.0, d = 1.0, e = -Math.Sqrt(2.0);
        TrigonometricFunctionRoots solver = new(c, d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount >= 1);

        if (solver.SolutionCount >= 1)
        {
            double x1     = solver.Value(1);
            double result = Math.Cos(x1) + Math.Sin(x1);
            Assert.Equal(Math.Sqrt(2.0), result, TOLERANCE);
        }
    }

    [Fact]
    public void AllCoefficients()
    {
        // Test a more complex equation with all coefficients non-zero
        // a*cos^2(x) + 2*b*Math.Cos(x)*Math.Sin(x) + c*Math.Cos(x) + d*Math.Sin(x) + e = 0
        double                          a = 1.0, b = 0.5, c = 0.5, d = 0.5, e = -0.25;
        TrigonometricFunctionRoots solver = new(a, b, c, d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());

        // Check that found solutions are valid
        for (int i = 1; i <= solver.SolutionCount; i++)
        {
            double x      = solver.Value(i);
            double result = a * Math.Cos(x) * Math.Cos(x) + 2.0 * b * Math.Cos(x) * Math.Sin(x) + c * Math.Cos(x) + d * Math.Sin(x) + e;
            Assert.Equal(0.0, result, TOLERANCE);
        }
    }

    [Fact]
    public void LargeBounds()
    {
        // Test Math.Sin(x) = 0 over multiple periods
        double                          d = 1.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(d, e, 0.0, 4.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount >= 2); // Should find multiple roots
    }

    [Fact]
    public void NegativeBounds()
    {
        // Test Math.Cos(x) = 0 with negative bounds
        double                          c = 1.0, d = 0.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(c, d, e, -PI, PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount >= 1);

        for (int i = 1; i <= solver.SolutionCount; i++)
        {
            double x = solver.Value(i);
            Assert.True(x >= -PI);
            Assert.True(x <= PI);
            Assert.Equal(0.0, Math.Abs(Math.Cos(x)), TOLERANCE);
        }
    }

    [Fact]
    public void HighFrequencyTest()
    {
        // Test Math.Sin(x) - 0.5 = 0 with precise expected solutions
        double                          d = 1.0, e = -0.5;
        TrigonometricFunctionRoots solver = new(d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        Assert.False(solver.InfiniteRoots());
        Assert.True(solver.SolutionCount >= 2);

        // Expected solutions: PI/6 approximately 0.5236 and 5*PI/6 approximately 2.618
        if (solver.SolutionCount >= 2)
        {
            List<double> solutions = [];
            for (int i = 1; i <= solver.SolutionCount; i++)
            {
                solutions.Add(solver.Value(i));
            }

            // Check that we have solutions near PI/6 and 5*PI/6
            bool found_first = false, found_second = false;
            foreach (double sol in solutions)
            {
                if (Math.Abs(sol - PI / 6.0) < 0.1)
                {
                    found_first = true;
                }
                if (Math.Abs(sol - 5.0 * PI / 6.0) < 0.1)
                {
                    found_second = true;
                }
            }
            Assert.True(found_first || found_second);
        }
    }

    [Fact]
    public void EdgeCaseSmallCoefficients()
    {
        // Test with very small coefficients
        double                          c = 1.0e-10, d = 1.0, e = 0.0;
        TrigonometricFunctionRoots solver = new(c, d, e, 0.0, 2.0 * PI);

        Assert.True(solver.IsDone);
        // Should behave approximately like Math.Sin(x) = 0
        if (!solver.InfiniteRoots())
        {
            Assert.True(solver.SolutionCount >= 2);
        }
    }
}
