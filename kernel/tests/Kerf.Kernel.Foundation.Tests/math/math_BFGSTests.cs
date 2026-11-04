// Port of: src/FoundationClasses/TKMath/GTests/math_BFGS_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_BFGSTest
{
    // Test function classes for optimization

    // Quadratic function: f(x,y) = (x-1)^2 + (y-2)^2 (minimum at (1,2), value = 0)
    private sealed class QuadraticFunction2D : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0; // C++ leaves it unset when returning false
            if (theX.Length() != 2)
            {
                return false;
            }
            double dx = theX[1] - 1.0;
            double dy = theX[2] - 2.0;
            theF = dx * dx + dy * dy;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            if (theX.Length() != 2 || theG.Length() != 2)
            {
                return false;
            }
            theG[1] = 2.0 * (theX[1] - 1.0);
            theG[2] = 2.0 * (theX[2] - 2.0);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }
    }

    // Rosenbrock function: f(x,y) = 100*(y-x^2)^2 + (1-x)^2 (minimum at (1,1), value = 0)
    private sealed class RosenbrockFunction : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0; // C++ leaves it unset when returning false
            if (theX.Length() != 2)
            {
                return false;
            }
            double x = theX[1];
            double y = theX[2];
            double t1 = y - x * x;
            double t2 = 1.0 - x;
            theF = 100.0 * t1 * t1 + t2 * t2;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            if (theX.Length() != 2 || theG.Length() != 2)
            {
                return false;
            }
            double x = theX[1];
            double y = theX[2];
            theG[1] = -400.0 * x * (y - x * x) - 2.0 * (1.0 - x);
            theG[2] = 200.0 * (y - x * x);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }
    }

    // 3D Paraboloid: f(x,y,z) = x^2 + 2*y^2 + 3*z^2 (minimum at origin, value = 0)
    private sealed class Paraboloid3D : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 3; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0; // C++ leaves it unset when returning false
            if (theX.Length() != 3)
            {
                return false;
            }
            theF = theX[1] * theX[1] + 2.0 * theX[2] * theX[2] + 3.0 * theX[3] * theX[3];
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            if (theX.Length() != 3 || theG.Length() != 3)
            {
                return false;
            }
            theG[1] = 2.0 * theX[1];
            theG[2] = 4.0 * theX[2];
            theG[3] = 6.0 * theX[3];
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }
    }

    // Tests for math_BFGS optimization
    [Fact]
    public void QuadraticFunction2DOptimization()
    {
        QuadraticFunction2D aFunc = new();
        double aTolerance = 1.0e-8;
        int aMaxIterations = 100;

        Bfgs anOptimizer = new(2, aTolerance, aMaxIterations);

        // Start from point (5, 7), should converge to (1, 2)
        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 5.0;
        aStartPoint[2] = 7.0;

        anOptimizer.Perform(aFunc, aStartPoint);

        Assert.True(anOptimizer.IsDone, "BFGS optimization should succeed");

        math_Vector aLocation = anOptimizer.Location();
        Assert.Equal(1.0, aLocation[1], 1.0e-6);
        Assert.Equal(2.0, aLocation[2], 1.0e-6);

        Assert.Equal(0.0, anOptimizer.Minimum(), 1.0e-10);

        math_Vector aGradient = anOptimizer.Gradient();
        Assert.Equal(0.0, aGradient[1], 1.0e-6);
        Assert.Equal(0.0, aGradient[2], 1.0e-6);

        Assert.True(anOptimizer.IterationCount > 0, "Should require at least one iteration");
        Assert.True(anOptimizer.IterationCount <= aMaxIterations, "Should not exceed max iterations");
    }

    [Fact]
    public void RosenbrockFunctionOptimization()
    {
        RosenbrockFunction aFunc = new();
        double aTolerance = 1.0e-6;
        int aMaxIterations = 1000; // Rosenbrock can be challenging

        Bfgs anOptimizer = new(2, aTolerance, aMaxIterations);

        // Start from point (-1, 1), should converge to (1, 1)
        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = -1.0;
        aStartPoint[2] = 1.0;

        anOptimizer.Perform(aFunc, aStartPoint);

        Assert.True(anOptimizer.IsDone, "BFGS optimization should succeed for Rosenbrock");

        math_Vector aLocation = anOptimizer.Location();
        Assert.Equal(1.0, aLocation[1], 1.0e-4);
        Assert.Equal(1.0, aLocation[2], 1.0e-4);

        Assert.Equal(0.0, anOptimizer.Minimum(), 1.0e-8);
    }

    [Fact]
    public void Paraboloid3DOptimization()
    {
        Paraboloid3D aFunc = new();
        double aTolerance = 1.0e-8;
        int aMaxIterations = 100;

        Bfgs anOptimizer = new(3, aTolerance, aMaxIterations);

        // Start from point (3, 4, 5), should converge to origin (0, 0, 0)
        math_Vector aStartPoint = new(1, 3);
        aStartPoint[1] = 3.0;
        aStartPoint[2] = 4.0;
        aStartPoint[3] = 5.0;

        anOptimizer.Perform(aFunc, aStartPoint);

        Assert.True(anOptimizer.IsDone, "BFGS optimization should succeed for 3D paraboloid");

        math_Vector aLocation = anOptimizer.Location();
        Assert.Equal(0.0, aLocation[1], 1.0e-6);
        Assert.Equal(0.0, aLocation[2], 1.0e-6);
        Assert.Equal(0.0, aLocation[3], 1.0e-6);

        Assert.Equal(0.0, anOptimizer.Minimum(), 1.0e-10);
    }

    [Fact]
    public void BoundaryConstraints()
    {
        QuadraticFunction2D aFunc = new();
        double aTolerance = 1.0e-8;
        int aMaxIterations = 100;

        Bfgs anOptimizer = new(2, aTolerance, aMaxIterations);

        // Set boundaries: x in [2, 4], y in [3, 5]
        // True minimum (1,2) is outside these bounds
        // Constrained minimum should be at (2,3)
        math_Vector aLowerBound = new(1, 2);
        aLowerBound[1] = 2.0;
        aLowerBound[2] = 3.0;

        math_Vector anUpperBound = new(1, 2);
        anUpperBound[1] = 4.0;
        anUpperBound[2] = 5.0;

        anOptimizer.SetBoundary(aLowerBound, anUpperBound);

        // Start from point (3, 4)
        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 3.0;
        aStartPoint[2] = 4.0;

        anOptimizer.Perform(aFunc, aStartPoint);

        Assert.True(anOptimizer.IsDone, "BFGS optimization with boundaries should succeed");

        math_Vector aLocation = anOptimizer.Location();
        Assert.True(aLocation[1] >= aLowerBound[1], "X should be within lower bound");
        Assert.True(aLocation[1] <= anUpperBound[1], "X should be within upper bound");
        Assert.True(aLocation[2] >= aLowerBound[2], "Y should be within lower bound");
        Assert.True(aLocation[2] <= anUpperBound[2], "Y should be within upper bound");

        // Should find constrained minimum at (2,3)
        Assert.Equal(2.0, aLocation[1], 1.0e-6);
        Assert.Equal(3.0, aLocation[2], 1.0e-6);
    }

    [Fact]
    public void LocationCopyMethod()
    {
        QuadraticFunction2D aFunc = new();
        Bfgs anOptimizer = new(2);

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 3.0;
        aStartPoint[2] = 4.0;

        anOptimizer.Perform(aFunc, aStartPoint);

        Assert.True(anOptimizer.IsDone, "Optimization should succeed");

        // Test Location copy method
        math_Vector aLocationCopy = new(1, 2);
        anOptimizer.Location(aLocationCopy);

        math_Vector aLocationRef = anOptimizer.Location();
        Assert.Equal(aLocationRef[1], aLocationCopy[1], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aLocationRef[2], aLocationCopy[2], global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void GradientCopyMethod()
    {
        QuadraticFunction2D aFunc = new();
        Bfgs anOptimizer = new(2);

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 3.0;
        aStartPoint[2] = 4.0;

        anOptimizer.Perform(aFunc, aStartPoint);

        Assert.True(anOptimizer.IsDone, "Optimization should succeed");

        // Test Gradient copy method
        math_Vector aGradientCopy = new(1, 2);
        anOptimizer.Gradient(aGradientCopy);

        math_Vector aGradientRef = anOptimizer.Gradient();
        Assert.Equal(aGradientRef[1], aGradientCopy[1], global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aGradientRef[2], aGradientCopy[2], global::Kerf.Engine.Foundation.Precision.Confusion());
    }

    [Fact]
    public void DifferentTolerances()
    {
        QuadraticFunction2D aFunc = new();

        // Test with very tight tolerance
        {
            double aTightTolerance = 1.0e-12;
            Bfgs anOptimizer = new(2, aTightTolerance);

            math_Vector aStartPoint = new(1, 2);
            aStartPoint[1] = 2.0;
            aStartPoint[2] = 3.0;

            anOptimizer.Perform(aFunc, aStartPoint);

            Assert.True(anOptimizer.IsDone, "High precision optimization should succeed");
            Assert.Equal(1.0, anOptimizer.Location()[1], aTightTolerance * 10);
            Assert.Equal(2.0, anOptimizer.Location()[2], aTightTolerance * 10);
        }

        // Test with loose tolerance
        {
            double aLooseTolerance = 1.0e-3;
            Bfgs anOptimizer = new(2, aLooseTolerance);

            math_Vector aStartPoint = new(1, 2);
            aStartPoint[1] = 10.0;
            aStartPoint[2] = 10.0;

            anOptimizer.Perform(aFunc, aStartPoint);

            Assert.True(anOptimizer.IsDone, "Low precision optimization should succeed");
            Assert.Equal(1.0, anOptimizer.Location()[1], aLooseTolerance * 10);
            Assert.Equal(2.0, anOptimizer.Location()[2], aLooseTolerance * 10);
        }
    }

    [Fact]
    public void MaxIterationsLimit()
    {
        RosenbrockFunction aFunc = new();                        // Challenging function
        double aTolerance = 1.0e-12; // Very tight tolerance
        int aVeryFewIterations = 5;       // Very few iterations

        Bfgs anOptimizer = new(2, aTolerance, aVeryFewIterations);

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = -2.0;
        aStartPoint[2] = 2.0;

        anOptimizer.Perform(aFunc, aStartPoint);

        // Either succeeds within few iterations (unlikely) or fails
        if (anOptimizer.IsDone)
        {
            Assert.True(anOptimizer.IterationCount <= aVeryFewIterations, "Should not exceed max iterations");
        }
        else
        {
            // Failure is acceptable with very few iterations
            Assert.True(aVeryFewIterations <= 10, "Failure expected with very few iterations");
        }
    }

    // Tests for exception handling
    [Fact]
    public void NotDoneState()
    {
        QuadraticFunction2D aFunc = new();
        Bfgs anOptimizer = new(2, 1.0e-12, 3); // Reasonable tolerance, few iterations

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 50.0; // Far from minimum but not extreme
        aStartPoint[2] = 50.0;

        // Wrap in try-catch to handle potential exceptions in debug mode
        // (EXPECT_NO_THROW: an exception fails the test.)
        anOptimizer.Perform(aFunc, aStartPoint);

        if (!anOptimizer.IsDone)
        {
            // OCCT's release build compiles out the NotDone check of NbIterations(); the port always checks.
            EngineAssert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => anOptimizer.IterationCount);
        }
        else
        {
            Assert.True(anOptimizer.IterationCount > 0, "Successful optimization should require at least one iteration");
        }
    }

    [Fact]
    public void DimensionCompatibility()
    {
        QuadraticFunction2D aFunc = new();
        Bfgs anOptimizer = new(2);

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 2.0;
        aStartPoint[2] = 3.0;

        anOptimizer.Perform(aFunc, aStartPoint);

        Assert.True(anOptimizer.IsDone, "Optimization should succeed for dimension compatibility tests");

        // Verify optimizer works correctly with properly dimensioned vectors
        math_Vector aCorrectSizeLocation = new(1, 2);
        math_Vector aCorrectSizeGradient = new(1, 2);

        anOptimizer.Location(aCorrectSizeLocation);
        anOptimizer.Gradient(aCorrectSizeGradient);

        // Verify the results make sense
        Assert.Equal(2, aCorrectSizeLocation.Length());
        Assert.Equal(2, aCorrectSizeGradient.Length());
    }

    [Fact]
    public void ConstructorParameters()
    {
        // Test different constructor parameter combinations
        {
            Bfgs anOptimizer1 = new(3); // Default tolerance, iterations, ZEPS
            QuadraticFunction2D aFunc = new(); // This is 2D, but optimizer is 3D - should handle gracefully

            // This might not work well due to dimension mismatch, but shouldn't crash
            {
                math_Vector aStart = new(1, 3);
                aStart.Init(1.0);
                // Don't perform - just test construction doesn't crash
            }
        }

        {
            Bfgs anOptimizer2 = new(2, 1.0e-6); // Custom tolerance, default iterations and ZEPS
            // Should construct successfully
        }

        {
            Bfgs anOptimizer3 = new(2, 1.0e-8, 50); // Custom tolerance and iterations
            // Should construct successfully
        }

        {
            Bfgs anOptimizer4 = new(2, 1.0e-8, 100, 1.0e-10); // All parameters custom
            // Should construct successfully
        }
    }

    [Fact]
    public void MultipleOptimizations()
    {
        QuadraticFunction2D aFunc = new();
        Bfgs anOptimizer = new(2);

        // Perform multiple optimizations with the same optimizer instance
        double[][] aStartPoints = [[5.0, 7.0], [-3.0, -4.0], [0.5, 1.5], [10.0, -5.0]];

        foreach (double[] aStart in aStartPoints)
        {
            math_Vector aStartPoint = new(1, 2);
            aStartPoint[1] = aStart[0];
            aStartPoint[2] = aStart[1];

            anOptimizer.Perform(aFunc, aStartPoint);

            Assert.True(anOptimizer.IsDone, "Each optimization should succeed");
            Assert.Equal(1.0, anOptimizer.Location()[1], 1.0e-6);
            Assert.Equal(2.0, anOptimizer.Location()[2], 1.0e-6);
        }
    }

    // Simple 1D square function: f(x) = x^2 (minimum at x=0, value = 0)
    private sealed class SquareFunction1D : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 1; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0; // C++ leaves it unset when returning false
            if (theX.Length() != 1)
            {
                return false;
            }
            double x = theX[1];
            theF = x * x;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            if (theX.Length() != 1 || theG.Length() != 1)
            {
                return false;
            }
            double x = theX[1];
            theG[1] = 2.0 * x;
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }
    }

    [Fact]
    public void OCC30492_StartingPointAtMinimum()
    {
        // Bug OCC30492: BFGS and FRPR fail if starting point is exactly the minimum
        // Test that both BFGS and FRPR work correctly when the starting point is at the minimum
        SquareFunction1D aFunc = new();
        math_Vector aStartPnt = new(1, 1);
        aStartPnt[1] = 0.0; // Start exactly at the minimum

        // Test FRPR
        Frpr aFRPR = new(aFunc, global::Kerf.Engine.Foundation.Precision.Confusion());
        aFRPR.Perform(aFunc, aStartPnt);
        Assert.True(aFRPR.IsDone, "FRPR optimization should succeed when starting at minimum");

        // Test BFGS
        Bfgs aBFGS = new(1, global::Kerf.Engine.Foundation.Precision.Confusion());
        aBFGS.Perform(aFunc, aStartPnt);
        Assert.True(aBFGS.IsDone, "BFGS optimization should succeed when starting at minimum");

        // Verify that both found the minimum (or stayed at it)
        if (aFRPR.IsDone)
        {
            Assert.Equal(0.0, aFRPR.Location()[1], global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.Equal(0.0, aFRPR.Minimum(), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        if (aBFGS.IsDone)
        {
            Assert.Equal(0.0, aBFGS.Location()[1], global::Kerf.Engine.Foundation.Precision.Confusion());
            Assert.Equal(0.0, aBFGS.Minimum(), global::Kerf.Engine.Foundation.Precision.Confusion());
        }
    }
}
