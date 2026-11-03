// Port of: src/FoundationClasses/TKMath/GTests/math_FRPR_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_FRPRTest
{
    // Quadratic bowl function: f(x,y) = (x-1)^2 + (y-2)^2, minimum at (1, 2) with value 0
    private sealed class QuadraticBowlFunction : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double dx = theX[1] - 1.0;
            double dy = theX[2] - 2.0;
            theF = dx * dx + dy * dy;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2.0 * (theX[1] - 1.0);
            theG[2] = 2.0 * (theX[2] - 2.0);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            return true;
        }
    }

    // Rosenbrock function: f(x,y) = (1-x)^2 + 100*(y-x^2)^2, minimum at (1, 1) with value 0
    private sealed class RosenbrockFunction : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double x = theX[1];
            double y = theX[2];
            double dx = 1.0 - x;
            double dy = y - x * x;
            theF = dx * dx + 100.0 * dy * dy;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            double x = theX[1];
            double y = theX[2];
            theG[1] = -2.0 * (1.0 - x) + 200.0 * (y - x * x) * (-2.0 * x);
            theG[2] = 200.0 * (y - x * x);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            return true;
        }
    }

    // 3D quadratic function: f(x,y,z) = (x-1)^2 + 2*(y-2)^2 + 3*(z-3)^2, minimum at (1,2,3)
    private sealed class Quadratic3DFunction : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 3; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double dx = theX[1] - 1.0;
            double dy = theX[2] - 2.0;
            double dz = theX[3] - 3.0;
            theF = dx * dx + 2.0 * dy * dy + 3.0 * dz * dz;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2.0 * (theX[1] - 1.0);
            theG[2] = 4.0 * (theX[2] - 2.0);
            theG[3] = 6.0 * (theX[3] - 3.0);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            return true;
        }
    }

    // Linear function: f(x,y) = 2*x + 3*y (unbounded, no minimum)
    private sealed class LinearFunction : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            theF = 2.0 * theX[1] + 3.0 * theX[2];
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2.0;
            theG[2] = 3.0;
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            return true;
        }
    }

    // Quartic function with flat minimum: f(x,y) = (x-1)^4 + (y-2)^4
    private sealed class QuarticFunction : MultipleVarFunctionWithGradient
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double dx = theX[1] - 1.0;
            double dy = theX[2] - 2.0;
            theF = dx * dx * dx * dx + dy * dy * dy * dy;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            double dx = theX[1] - 1.0;
            double dy = theX[2] - 2.0;
            theG[1] = 4.0 * dx * dx * dx;
            theG[2] = 4.0 * dy * dy * dy;
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            return true;
        }
    }



    [Fact]
    public void QuadraticBowlOptimization()
    {
        // Test FRPR on simple quadratic bowl function
        QuadraticBowlFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0; // Start at (0, 0)
        aStartPoint[2] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum for quadratic bowl function");

        math_Vector aLoc = aSolver.Location();
        Assert.Equal(1.0, aLoc[1], 1.0e-6);
        Assert.Equal(2.0, aLoc[2], 1.0e-6);
        Assert.Equal(0.0, aSolver.Minimum(), 1.0e-10);
        Assert.True(aSolver.IterationCount > 0, "Should have performed some iterations");
    }

    [Fact]
    public void RosenbrockOptimization()
    {
        // Test FRPR on the challenging Rosenbrock function
        RosenbrockFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0; // Start at (0, 0)
        aStartPoint[2] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-8, 500); // More iterations for challenging function
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum for Rosenbrock function");

        math_Vector aLoc = aSolver.Location();
        Assert.Equal(1.0, aLoc[1], 1.0e-3);
        Assert.Equal(1.0, aLoc[2], 1.0e-3);
        Assert.True(aSolver.Minimum() < 1.0e-4, "Should find a very small minimum");
    }

    [Fact]
    public void ThreeDimensionalOptimization()
    {
        // Test FRPR on 3D quadratic function
        Quadratic3DFunction aFunc = new();

        math_Vector aStartPoint = new(1, 3);
        aStartPoint[1] = 0.0; // Start at (0, 0, 0)
        aStartPoint[2] = 0.0;
        aStartPoint[3] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum for 3D function");

        math_Vector aLoc = aSolver.Location();
        Assert.Equal(1.0, aLoc[1], 1.0e-6);
        Assert.Equal(2.0, aLoc[2], 1.0e-6);
        Assert.Equal(3.0, aLoc[3], 1.0e-6);
        Assert.Equal(0.0, aSolver.Minimum(), 1.0e-10);
    }

    [Fact]
    public void CustomTolerance()
    {
        // Test with different tolerance values
        QuadraticBowlFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        // Loose tolerance
        Frpr aSolver1 = new(aFunc, 1.0e-3);
        aSolver1.Perform(aFunc, aStartPoint);

        Assert.True(aSolver1.IsDone, "Should converge with loose tolerance");
        Assert.Equal(1.0, aSolver1.Location()[1], 1.0e-2);
        Assert.Equal(2.0, aSolver1.Location()[2], 1.0e-2);

        // Tight tolerance
        Frpr aSolver2 = new(aFunc, 1.0e-12);
        aSolver2.Perform(aFunc, aStartPoint);

        Assert.True(aSolver2.IsDone, "Should converge with tight tolerance");
        Assert.Equal(1.0, aSolver2.Location()[1], 1.0e-8);
        Assert.Equal(2.0, aSolver2.Location()[2], 1.0e-8);
    }

    [Fact]
    public void CustomIterationLimit()
    {
        // Test with custom iteration limits
        RosenbrockFunction aFunc = new(); // More challenging function

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        // Few iterations
        Frpr aSolver1 = new(aFunc, 1.0e-10, 10);
        aSolver1.Perform(aFunc, aStartPoint);

        if (aSolver1.IsDone)
        {
            Assert.True(aSolver1.IterationCount <= 10, "Should respect iteration limit");
        }

        // Many iterations
        Frpr aSolver2 = new(aFunc, 1.0e-10, 1000);
        aSolver2.Perform(aFunc, aStartPoint);

        Assert.True(aSolver2.IsDone, "Should converge with many iterations allowed");
    }

    [Fact]
    public void GradientAccess()
    {
        // Test gradient vector access
        QuadraticBowlFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum");

        math_Vector aGrad = aSolver.Gradient();
        Assert.Equal(0.0, aGrad[1], 1.0e-8);
        Assert.Equal(0.0, aGrad[2], 1.0e-8);

        // Test gradient output method
        math_Vector aGradOut = new(1, 2);
        aSolver.Gradient(aGradOut);
        Assert.Equal(0.0, aGradOut[1], 1.0e-8);
        Assert.Equal(0.0, aGradOut[2], 1.0e-8);
    }

    [Fact]
    public void LocationAccess()
    {
        // Test location vector access methods
        QuadraticBowlFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum");

        // Test location output method
        math_Vector aLocOut = new(1, 2);
        aSolver.Location(aLocOut);
        Assert.Equal(1.0, aLocOut[1], 1.0e-6);
        Assert.Equal(2.0, aLocOut[2], 1.0e-6);
    }

    [Fact]
    public void CustomZEPS()
    {
        // Test with custom ZEPS (machine epsilon) parameter
        QuadraticBowlFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-10, 200, 1.0e-15);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should work with custom ZEPS");
        Assert.Equal(1.0, aSolver.Location()[1], 1.0e-6);
        Assert.Equal(2.0, aSolver.Location()[2], 1.0e-6);
    }

    [Fact]
    public void UnperformedState()
    {
        // Test state handling before Perform() is called
        QuadraticBowlFunction aFunc = new();
        Frpr aSolver = new(aFunc, 1.0e-10);

        // Before Perform() is called, solver should report not done
        Assert.False(aSolver.IsDone, "Solver should not be done before Perform()");

        // In release builds, verify the solver maintains consistent state
        if (!aSolver.IsDone)
        {
            Assert.False(aSolver.IsDone, "State should be consistent when not done");
        }
    }

    [Fact]
    public void DimensionCompatibility()
    {
        // Test dimension compatibility handling
        QuadraticBowlFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum");

        // Test with correctly dimensioned vectors
        math_Vector aCorrectVec = new(1, 2); // 2D vector for 2D function
        aSolver.Location(aCorrectVec);
        aSolver.Gradient(aCorrectVec);

        // Verify the results make sense
        Assert.Equal(2, aCorrectVec.Length());
    }

    [Fact]
    public void StartingNearMinimum()
    {
        // Test when starting point is already near the minimum
        QuadraticBowlFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 1.001; // Very close to minimum at (1, 2)
        aStartPoint[2] = 1.999;

        Frpr aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should succeed when starting near minimum");
        Assert.Equal(1.0, aSolver.Location()[1], 1.0e-6);
        Assert.Equal(2.0, aSolver.Location()[2], 1.0e-6);
        Assert.Equal(0.0, aSolver.Minimum(), 1.0e-10);
    }

    [Fact]
    public void QuarticFlatMinimum()
    {
        // Test with quartic function that has very flat minimum
        QuarticFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-8); // Slightly looser tolerance for flat minimum
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should handle quartic function with flat minimum");
        Assert.Equal(1.0, aSolver.Location()[1], 1.0e-4);
        Assert.Equal(2.0, aSolver.Location()[2], 1.0e-4);
    }

    [Fact]
    public void LinearFunctionUnbounded()
    {
        // Test with unbounded linear function
        LinearFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        Frpr aSolver = new(aFunc, 1.0e-10, 50); // Limited iterations
        aSolver.Perform(aFunc, aStartPoint);

        // The algorithm may or may not converge for unbounded functions
        // depending on implementation details and stopping criteria
        if (aSolver.IsDone)
        {
            // If it "converges", it should have done some work
            Assert.True(aSolver.IterationCount > 0, "Should have performed some iterations");
        }
    }

    [Fact]
    public void MultipleCalls()
    {
        // Test multiple calls to Perform with same instance
        QuadraticBowlFunction aFunc1 = new();
        Quadratic3DFunction aFunc2 = new();

        math_Vector aStartPoint2D = new(1, 2);
        aStartPoint2D[1] = 0.0;
        aStartPoint2D[2] = 0.0;

        math_Vector aStartPoint3D = new(1, 3);
        aStartPoint3D[1] = 0.0;
        aStartPoint3D[2] = 0.0;
        aStartPoint3D[3] = 0.0;

        Frpr aSolver = new(aFunc1, 1.0e-10);

        // First call with 2D function
        aSolver.Perform(aFunc1, aStartPoint2D);
        Assert.True(aSolver.IsDone, "First call should succeed");
        Assert.Equal(1.0, aSolver.Location()[1], 1.0e-6);

        // Second call with 3D function - need to create new solver with appropriate function
        Frpr aSolver2 = new(aFunc2, 1.0e-10);
        aSolver2.Perform(aFunc2, aStartPoint3D);
        Assert.True(aSolver2.IsDone, "Second call should succeed");
        Assert.Equal(1.0, aSolver2.Location()[1], 1.0e-6);
    }
}
