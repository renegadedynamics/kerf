// Port of: src/FoundationClasses/TKMath/GTests/math_NewtonMinimum_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Kerf.Kernel.Foundation.Tests;

public class math_NewtonMinimumTest
{
    // Quadratic bowl function: f(x,y) = (x-1)^2 + 2*(y-2)^2, minimum at (1, 2) with value 0
    private sealed class QuadraticBowlWithHessian : MultipleVarFunctionWithHessian
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double dx = theX[1] - 1.0;
            double dy = theX[2] - 2.0;
            theF = dx * dx + 2.0 * dy * dy;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2.0 * (theX[1] - 1.0);
            theG[2] = 4.0 * (theX[2] - 2.0);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG, Matrix theH)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            // Hessian matrix: [[2, 0], [0, 4]]
            theH[1, 1] = 2.0;
            theH[1, 2] = 0.0;
            theH[2, 1] = 0.0;
            theH[2, 2] = 4.0;
            return true;
        }
    }

    // Rosenbrock function: f(x,y) = (1-x)^2 + 100*(y-x^2)^2
    private sealed class RosenbrockWithHessian : MultipleVarFunctionWithHessian
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

        public override bool Values(math_Vector theX, out double theF, math_Vector theG, Matrix theH)
        {
            double x = theX[1];
            double y = theX[2];

            Value(theX, out theF);
            Gradient(theX, theG);

            // Hessian matrix computation
            theH[1, 1] = 2.0 + 1200.0 * x * x - 400.0 * (y - x * x);
            theH[1, 2] = -400.0 * x;
            theH[2, 1] = -400.0 * x;
            theH[2, 2] = 200.0;
            return true;
        }
    }

    // 3D quadratic function: f(x,y,z) = (x-1)^2 + 2*(y-2)^2 + 3*(z-3)^2
    private sealed class Quadratic3DWithHessian : MultipleVarFunctionWithHessian
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

        public override bool Values(math_Vector theX, out double theF, math_Vector theG, Matrix theH)
        {
            Value(theX, out theF);
            Gradient(theX, theG);

            // Diagonal Hessian matrix
            theH.Init(0.0);
            theH[1, 1] = 2.0;
            theH[2, 2] = 4.0;
            theH[3, 3] = 6.0;
            return true;
        }
    }

    // Non-convex function with saddle point: f(x,y) = x^2 - y^2
    private sealed class SaddleFunction : MultipleVarFunctionWithHessian
    {
        public override int VariableCount {
            get
            { return 2; }
        }

        public override bool Value(math_Vector theX, out double theF)
        {
            double x = theX[1];
            double y = theX[2];
            theF = x * x - y * y;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            theG[1] = 2.0 * theX[1];
            theG[2] = -2.0 * theX[2];
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG, Matrix theH)
        {
            Value(theX, out theF);
            Gradient(theX, theG);

            // Hessian matrix: [[2, 0], [0, -2]] (indefinite)
            theH[1, 1] = 2.0;
            theH[1, 2] = 0.0;
            theH[2, 1] = 0.0;
            theH[2, 2] = -2.0;
            return true;
        }
    }



    [Fact]
    public void QuadraticBowlOptimization()
    {
        // Test Newton minimum on simple quadratic bowl function
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0; // Start at (0, 0)
        aStartPoint[2] = 0.0;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum for quadratic bowl function");

        math_Vector aLoc = aSolver.Location();
        Assert.Equal(1.0, aLoc[1], 1.0e-8);
        Assert.Equal(2.0, aLoc[2], 1.0e-8);
        Assert.Equal(0.0, aSolver.Minimum(), 1.0e-12);
        Assert.True(aSolver.IterationCount > 0, "Should have performed some iterations");
        Assert.True(aSolver.IterationCount <= 10, "Should converge quickly for quadratic");
    }

    [Fact]
    public void RosenbrockOptimization()
    {
        // Test Newton minimum on challenging Rosenbrock function
        RosenbrockWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.8; // Start closer to minimum
        aStartPoint[2] = 0.8;

        NewtonMinimum aSolver = new(aFunc, 1.0e-6, 100); // More iterations for challenging function
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum for Rosenbrock function");

        math_Vector aLoc = aSolver.Location();
        Assert.Equal(1.0, aLoc[1], 2.0e-1);
        Assert.Equal(1.0, aLoc[2], 2.0e-1);
        Assert.True(aSolver.Minimum() < 2.0e-2, "Should find a reasonably small minimum");
    }

    [Fact]
    public void ThreeDimensionalOptimization()
    {
        // Test Newton minimum on 3D quadratic function
        Quadratic3DWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 3);
        aStartPoint[1] = 0.0; // Start at (0, 0, 0)
        aStartPoint[2] = 0.0;
        aStartPoint[3] = 0.0;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum for 3D function");

        math_Vector aLoc = aSolver.Location();
        Assert.Equal(1.0, aLoc[1], 1.0e-8);
        Assert.Equal(2.0, aLoc[2], 1.0e-8);
        Assert.Equal(3.0, aLoc[3], 1.0e-8);
        Assert.Equal(0.0, aSolver.Minimum(), 1.0e-12);
    }

    [Fact]
    public void BoundedOptimization()
    {
        // Test Newton minimum with bounds
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        math_Vector aLeftBound = new(1, 2);
        aLeftBound[1] = -0.5;
        aLeftBound[2] = -0.5;

        math_Vector aRightBound = new(1, 2);
        aRightBound[1] = 1.5;
        aRightBound[2] = 2.5;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10);
        aSolver.SetBoundary(aLeftBound, aRightBound);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum with bounds");

        math_Vector aLoc = aSolver.Location();
        Assert.Equal(1.0, aLoc[1], 1.0e-8);
        Assert.Equal(2.0, aLoc[2], 1.0e-8);
        Assert.True(aLoc[1] >= -0.5, "Should respect lower bound");
        Assert.True(aLoc[1] <= 1.5, "Should respect upper bound");
        Assert.True(aLoc[2] >= -0.5, "Should respect lower bound");
        Assert.True(aLoc[2] <= 2.5, "Should respect upper bound");
    }

    [Fact]
    public void CustomTolerance()
    {
        // Test with different tolerance values
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        // Loose tolerance
        NewtonMinimum aSolver1 = new(aFunc, 1.0e-3);
        aSolver1.Perform(aFunc, aStartPoint);

        Assert.True(aSolver1.IsDone, "Should converge with loose tolerance");
        Assert.Equal(1.0, aSolver1.Location()[1], 1.0e-2);
        Assert.Equal(2.0, aSolver1.Location()[2], 1.0e-2);

        // Tight tolerance
        NewtonMinimum aSolver2 = new(aFunc, 1.0e-12);
        aSolver2.Perform(aFunc, aStartPoint);

        Assert.True(aSolver2.IsDone, "Should converge with tight tolerance");
        Assert.Equal(1.0, aSolver2.Location()[1], 1.0e-10);
        Assert.Equal(2.0, aSolver2.Location()[2], 1.0e-10);
    }

    [Fact]
    public void CustomIterationLimit()
    {
        // Test with custom iteration limits
        RosenbrockWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.8;
        aStartPoint[2] = 0.8;

        // Few iterations
        NewtonMinimum aSolver1 = new(aFunc, 1.0e-6, 5);
        aSolver1.Perform(aFunc, aStartPoint);

        if (aSolver1.IsDone)
        {
            Assert.True(aSolver1.IterationCount <= 5, "Should respect iteration limit");
        }

        // Many iterations
        NewtonMinimum aSolver2 = new(aFunc, 1.0e-8, 200);
        aSolver2.Perform(aFunc, aStartPoint);

        Assert.True(aSolver2.IsDone, "Should converge with many iterations allowed");
    }

    [Fact]
    public void GradientAccess()
    {
        // Test gradient vector access
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10);
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
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum");

        // Test location output method
        math_Vector aLocOut = new(1, 2);
        aSolver.Location(aLocOut);
        Assert.Equal(1.0, aLocOut[1], 1.0e-8);
        Assert.Equal(2.0, aLocOut[2], 1.0e-8);
    }

    [Fact]
    public void CustomConvexity()
    {
        // Test with custom convexity parameter
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10, 40, 1.0e-8);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should work with custom convexity");
        Assert.Equal(1.0, aSolver.Location()[1], 1.0e-8);
        Assert.Equal(2.0, aSolver.Location()[2], 1.0e-8);
    }

    [Fact]
    public void WithSingularityTreatment()
    {
        // Test with singularity treatment enabled
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10, 40, 1.0e-6, true);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should work with singularity treatment");
        Assert.Equal(1.0, aSolver.Location()[1], 1.0e-8);
        Assert.Equal(2.0, aSolver.Location()[2], 1.0e-8);
    }

    [Fact]
    public void NonConvexFunction()
    {
        // Test with non-convex function (saddle point)
        SaddleFunction aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.1;
        aStartPoint[2] = 0.1;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10, 40, 1.0e-6, false);
        aSolver.Perform(aFunc, aStartPoint);

        // Function is not convex, Newton method may not converge
        // Testing that algorithm can handle non-convex functions
        { aSolver.Perform(aFunc, aStartPoint); }
    }

    [Fact]
    public void UnperformedState()
    {
        // Test state handling before Perform() is called
        QuadraticBowlWithHessian aFunc = new();
        NewtonMinimum aSolver = new(aFunc, 1.0e-10);

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
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10);
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
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 1.001; // Very close to minimum at (1, 2)
        aStartPoint[2] = 1.999;

        NewtonMinimum aSolver = new(aFunc, 1.0e-12);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should succeed when starting near minimum");
        Assert.Equal(1.0, aSolver.Location()[1], 1.0e-8);
        Assert.Equal(2.0, aSolver.Location()[2], 1.0e-8);
        Assert.Equal(0.0, aSolver.Minimum(), 1.0e-12);
        Assert.True(aSolver.IterationCount <= 5, "Should converge very quickly");
    }

    [Fact]
    public void StatusAccess()
    {
        // Test status access method
        QuadraticBowlWithHessian aFunc = new();

        math_Vector aStartPoint = new(1, 2);
        aStartPoint[1] = 0.0;
        aStartPoint[2] = 0.0;

        NewtonMinimum aSolver = new(aFunc, 1.0e-10);
        aSolver.Perform(aFunc, aStartPoint);

        Assert.True(aSolver.IsDone, "Should find minimum");

        // Test that we can access the status without exception
        { aSolver.GetStatus(); }
    }
}
