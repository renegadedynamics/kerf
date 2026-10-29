// Port of: src/FoundationClasses/TKMath/GTests/math_GlobOptMin_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

public class math_GlobOptMinTest
{
    // Simple quadratic function: f(x,y) = (x-1)^2 + (y-2)^2, minimum at (1, 2)
    private sealed class QuadraticFunction : math_MultipleVarFunction
    {
        public override int NbVariables() { return 2; }

        public override bool Value(math_Vector theX, out double theF)
        {
            double dx = theX[1] - 1.0;
            double dy = theX[2] - 2.0;
            theF = dx * dx + dy * dy;
            return true;
        }
    }

    // Multi-modal function with multiple local minima: f(x,y) = sin(x) + sin(y) + 0.1*(x^2 + y^2)
    private sealed class MultiModalFunction : math_MultipleVarFunction
    {
        public override int NbVariables() { return 2; }

        public override bool Value(math_Vector theX, out double theF)
        {
            double x = theX[1];
            double y = theX[2];
            theF = Math.Sin(x) + Math.Sin(y) + 0.1 * (x * x + y * y);
            return true;
        }
    }

    // 1D function: f(x) = sin(x) + 0.5*sin(3*x) with multiple local minima
    private sealed class MultiModal1DFunction : math_MultipleVarFunction
    {
        public override int NbVariables() { return 1; }

        public override bool Value(math_Vector theX, out double theF)
        {
            double x = theX[1];
            theF = Math.Sin(x) + 0.5 * Math.Sin(3.0 * x);
            return true;
        }
    }

    // Rosenbrock function in 2D: f(x,y) = (1-x)^2 + 100*(y-x^2)^2
    private sealed class RosenbrockFunction : math_MultipleVarFunction
    {
        public override int NbVariables() { return 2; }

        public override bool Value(math_Vector theX, out double theF)
        {
            double x = theX[1];
            double y = theX[2];
            double dx = 1.0 - x;
            double dy = y - x * x;
            theF = dx * dx + 100.0 * dy * dy;
            return true;
        }
    }

    // Simple linear function: f(x,y) = x + y (minimum at bounds)
    private sealed class LinearFunction : math_MultipleVarFunction
    {
        public override int NbVariables() { return 2; }

        public override bool Value(math_Vector theX, out double theF)
        {
            theF = theX[1] + theX[2];
            return true;
        }
    }

    // Branin function: standard global optimization benchmark with 3 global minima.
    // f(x,y) = a*(y - b*x^2 + c*x - r)^2 + s*(1-t)*cos(x) + s
    // Standard parameters: a=1, b=5.1/(4pi^2), c=5/pi, r=6, s=10, t=1/(8pi)
    // Global minimum value: ~0.397887 at 3 locations: (-pi,12.275), (pi,2.275), (9.42478,2.475)
    private sealed class BraninFunction : math_MultipleVarFunctionWithHessian
    {
        public BraninFunction()
        {
            a = 1.0;
            b = 5.1 / (4.0 * M_PI * M_PI);
            c = 5.0 / M_PI;
            r = 6.0;
            s = 10.0;
            t = 1.0 / (8.0 * M_PI);
        }

        public override int NbVariables() { return 2; }

        public override bool Value(math_Vector theX, out double theF)
        {
            double u = theX[1];
            double v = theX[2];
            double aSqPt = (v - b * u * u + c * u - r);
            double aLnPt = s * (1 - t) * Math.Cos(u);
            theF = a * aSqPt * aSqPt + aLnPt + s;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            double u = theX[1];
            double v = theX[2];
            double aSqPt = (v - b * u * u + c * u - r);
            theG[1] = 2 * a * aSqPt * (c - 2 * b * u) - s * (1 - t) * Math.Sin(u);
            theG[2] = 2 * a * aSqPt;
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG, math_Matrix theH)
        {
            Value(theX, out theF);
            Gradient(theX, theG);
            double u = theX[1];
            double v = theX[2];
            double aSqPt = (v - b * u * u + c * u - r);
            double aTmpPt = c - 2 * b * u;
            theH[1, 1] = 2 * a * aTmpPt * aTmpPt - 4 * a * b * aSqPt - s * (1 - t) * Math.Cos(u);
            theH[1, 2] = 2 * a * aTmpPt;
            theH[2, 1] = theH[1, 2];
            theH[2, 2] = 2 * a;
            return true;
        }
        double a, b, c, r, s, t;
    }



    [Fact]
    public void QuadraticFunctionOptimization()
    {
        // Test global optimization on simple quadratic function
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 3.0;
        aUpperBorder[2] = 4.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);
        aSolver.Perform(true); // Find single solution

        Assert.True(aSolver.isDone(), "Should successfully optimize quadratic function");
        Assert.True(aSolver.NbExtrema() > 0, "Should find at least one extremum");

        math_Vector aSol = new(1, 2);
        aSolver.Points(1, aSol);

        Assert.Equal(1.0, aSol[1], 1.0e-1);
        Assert.Equal(2.0, aSol[2], 1.0e-1);
        Assert.Equal(0.0, aSolver.GetF(), 1.0e-2);
    }

    [Fact]
    public void MultiModalFunctionOptimization()
    {
        // Test global optimization on multi-modal function
        MultiModalFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -5.0;
        aLowerBorder[2] = -5.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 5.0;
        aUpperBorder[2] = 5.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);
        aSolver.Perform(false); // Find all solutions

        Assert.True(aSolver.isDone(), "Should successfully optimize multi-modal function");
        Assert.True(aSolver.NbExtrema() >= 1, "Should find at least one extremum for multi-modal function");

        // Check that at least one solution exists
        math_Vector aSol = new(1, 2);
        aSolver.Points(1, aSol);
        Assert.True(aSol[1] >= -5.0 && aSol[1] <= 5.0, "Solution should be within bounds");
        Assert.True(aSol[2] >= -5.0 && aSol[2] <= 5.0, "Solution should be within bounds");
    }

    [Fact]
    public void OneDimensionalOptimization()
    {
        // Test global optimization on 1D function
        MultiModal1DFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 1);
        aLowerBorder[1] = 0.0;

        math_Vector aUpperBorder = new(1, 1);
        aUpperBorder[1] = 2.0 * M_PI;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);
        aSolver.Perform(false); // Find all solutions

        Assert.True(aSolver.isDone(), "Should successfully optimize 1D function");
        Assert.True(aSolver.NbExtrema() > 0, "Should find at least one extremum");

        math_Vector aSol = new(1, 1);
        aSolver.Points(1, aSol);
        Assert.True(aSol[1] >= 0.0 && aSol[1] <= 2.0 * M_PI, "Solution should be within bounds");
    }

    [Fact]
    public void SingleSolutionSearch()
    {
        // Test single solution search mode
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 3.0;
        aUpperBorder[2] = 4.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);
        aSolver.Perform(true); // Find single solution

        Assert.True(aSolver.isDone(), "Should find single solution");
        Assert.Equal(1, aSolver.NbExtrema());
    }

    [Fact]
    public void AllSolutionsSearch()
    {
        // Test all solutions search mode
        MultiModalFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -3.0;
        aLowerBorder[2] = -3.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 3.0;
        aUpperBorder[2] = 3.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);
        aSolver.Perform(false); // Find all solutions

        Assert.True(aSolver.isDone(), "Should find all solutions");
        Assert.True(aSolver.NbExtrema() >= 1, "Should find at least one extremum");
    }

    [Fact]
    public void CustomTolerances()
    {
        // Test with custom tolerances
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 3.0;
        aUpperBorder[2] = 4.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder, 9, 1.0e-3, 1.0e-8);

        // Test setting and getting tolerances
        double aDiscTol, aSameTol;
        aSolver.GetTol(out aDiscTol, out aSameTol);
        Assert.Equal(1.0e-3, aDiscTol, 1.0e-12);
        Assert.Equal(1.0e-8, aSameTol, 1.0e-12);

        // Update tolerances
        aSolver.SetTol(1.0e-2, 1.0e-6);
        aSolver.GetTol(out aDiscTol, out aSameTol);
        Assert.Equal(1.0e-2, aDiscTol, 1.0e-12);
        Assert.Equal(1.0e-6, aSameTol, 1.0e-12);

        aSolver.Perform(true);
        Assert.True(aSolver.isDone(), "Should work with custom tolerances");
    }

    [Fact]
    public void LocalParamsReduction()
    {
        // Test with local parameters (bounding box reduction)
        QuadraticFunction aFunc = new();

        math_Vector aGlobalLower = new(1, 2);
        aGlobalLower[1] = -5.0;
        aGlobalLower[2] = -5.0;

        math_Vector aGlobalUpper = new(1, 2);
        aGlobalUpper[1] = 5.0;
        aGlobalUpper[2] = 5.0;

        math_GlobOptMin aSolver = new(aFunc, aGlobalLower, aGlobalUpper);

        // Set local parameters (smaller search box)
        math_Vector aLocalLower = new(1, 2);
        aLocalLower[1] = 0.0;
        aLocalLower[2] = 1.0;

        math_Vector aLocalUpper = new(1, 2);
        aLocalUpper[1] = 2.0;
        aLocalUpper[2] = 3.0;

        aSolver.SetLocalParams(aLocalLower, aLocalUpper);
        aSolver.Perform(true);

        Assert.True(aSolver.isDone(), "Should work with local parameters");

        math_Vector aSol = new(1, 2);
        aSolver.Points(1, aSol);
        Assert.True(aSol[1] >= 0.0 && aSol[1] <= 2.0, "Solution should be within local bounds");
        Assert.True(aSol[2] >= 1.0 && aSol[2] <= 3.0, "Solution should be within local bounds");
    }

    [Fact]
    public void ContinuitySettings()
    {
        // Test continuity settings
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 3.0;
        aUpperBorder[2] = 4.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);

        // Test default continuity
        int aDefaultCont = aSolver.GetContinuity();
        Assert.True(aDefaultCont >= 0, "Default continuity should be non-negative");

        // Set and test different continuity values
        aSolver.SetContinuity(1);
        Assert.Equal(1, aSolver.GetContinuity());

        aSolver.SetContinuity(2);
        Assert.Equal(2, aSolver.GetContinuity());

        aSolver.Perform(true);
        Assert.True(aSolver.isDone(), "Should work with different continuity settings");
    }

    [Fact]
    public void FunctionalMinimalValue()
    {
        // Test functional minimal value setting
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 3.0;
        aUpperBorder[2] = 4.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);

        // Test default minimal value
        double aDefaultMin = aSolver.GetFunctionalMinimalValue();
        Assert.Equal(-Precision.Infinite(), aDefaultMin);

        // Set functional minimal value
        aSolver.SetFunctionalMinimalValue(-1.0);
        Assert.Equal(-1.0, aSolver.GetFunctionalMinimalValue(), 1.0e-12);

        aSolver.Perform(true);
        Assert.True(aSolver.isDone(), "Should work with functional minimal value");
    }

    [Fact]
    public void LipschitzConstantState()
    {
        // Test Lipschitz constant state management
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 3.0;
        aUpperBorder[2] = 4.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);

        // Test default state
        bool aDefaultState = aSolver.GetLipConstState();
        Assert.False(aDefaultState, "Default Lipschitz constant should be unlocked");

        // Lock Lipschitz constant
        aSolver.SetLipConstState(true);
        Assert.True(aSolver.GetLipConstState(), "Lipschitz constant should be locked");

        // Unlock Lipschitz constant
        aSolver.SetLipConstState(false);
        Assert.False(aSolver.GetLipConstState(), "Lipschitz constant should be unlocked");

        aSolver.Perform(true);
        Assert.True(aSolver.isDone(), "Should work with Lipschitz constant state management");
    }

    [Fact]
    public void RosenbrockOptimization()
    {
        // Test on challenging Rosenbrock function
        RosenbrockFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -1.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 2.0;
        aUpperBorder[2] = 3.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder, 50, 1.0e-2, 1.0e-6);
        aSolver.Perform(true);

        Assert.True(aSolver.isDone(), "Should handle Rosenbrock function");
        Assert.True(aSolver.NbExtrema() > 0, "Should find at least one extremum");

        math_Vector aSol = new(1, 2);
        aSolver.Points(1, aSol);
        // Rosenbrock minimum is at (1,1) but may not be found exactly due to discretization
        Assert.True(aSol[1] >= -2.0 && aSol[1] <= 2.0, "Solution should be within bounds");
        Assert.True(aSol[2] >= -1.0 && aSol[2] <= 3.0, "Solution should be within bounds");
    }

    [Fact]
    public void LinearFunctionOptimization()
    {
        // Test on linear function (minimum at boundary)
        LinearFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = 0.0;
        aLowerBorder[2] = 0.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 2.0;
        aUpperBorder[2] = 2.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);
        aSolver.Perform(true);

        Assert.True(aSolver.isDone(), "Should handle linear function");
        Assert.True(aSolver.NbExtrema() > 0, "Should find at least one extremum");

        math_Vector aSol = new(1, 2);
        aSolver.Points(1, aSol);
        // For linear function f(x,y) = x + y, minimum should be at (0, 0)
        Assert.Equal(0.0, aSol[1], 1.0e-1);
        Assert.Equal(0.0, aSol[2], 1.0e-1);
    }

    [Fact]
    public void SetGlobalParamsMethod()
    {
        // Test SetGlobalParams method
        QuadraticFunction aFunc1 = new();
        LinearFunction aFunc2 = new();

        math_Vector aLowerBorder1 = new(1, 2);
        aLowerBorder1[1] = -2.0;
        aLowerBorder1[2] = -1.0;

        math_Vector aUpperBorder1 = new(1, 2);
        aUpperBorder1[1] = 3.0;
        aUpperBorder1[2] = 4.0;

        // Create solver with first function
        math_GlobOptMin aSolver = new(aFunc1, aLowerBorder1, aUpperBorder1);
        aSolver.Perform(true);

        Assert.True(aSolver.isDone(), "Should work with first function");

        // Change to second function and different bounds
        math_Vector aLowerBorder2 = new(1, 2);
        aLowerBorder2[1] = 0.0;
        aLowerBorder2[2] = 0.0;

        math_Vector aUpperBorder2 = new(1, 2);
        aUpperBorder2[1] = 1.0;
        aUpperBorder2[2] = 1.0;

        aSolver.SetGlobalParams(aFunc2, aLowerBorder2, aUpperBorder2);
        aSolver.Perform(true);

        Assert.True(aSolver.isDone(), "Should work after changing global params");
    }

    [Fact]
    public void MultipleExtremaAccess()
    {
        // Test accessing multiple extrema
        MultiModalFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = -2.0;
        aLowerBorder[2] = -2.0;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 2.0;
        aUpperBorder[2] = 2.0;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder);
        aSolver.Perform(false); // Find all solutions

        Assert.True(aSolver.isDone(), "Should find multiple solutions");
        int aNbSol = aSolver.NbExtrema();
        Assert.True(aNbSol > 0, "Should have at least one solution");

        // Test accessing all solutions
        for (int i = 1; i <= aNbSol; ++i)
        {
            math_Vector aSol = new(1, 2);
            { aSolver.Points(i, aSol); }
            Assert.True(aSol[1] >= -2.0 && aSol[1] <= 2.0, $"Solution {i} should be within bounds");
            Assert.True(aSol[2] >= -2.0 && aSol[2] <= 2.0, $"Solution {i} should be within bounds");
        }
    }

    [Fact]
    public void SmallSearchSpace()
    {
        // Test with very small search space
        QuadraticFunction aFunc = new();

        math_Vector aLowerBorder = new(1, 2);
        aLowerBorder[1] = 0.99;
        aLowerBorder[2] = 1.99;

        math_Vector aUpperBorder = new(1, 2);
        aUpperBorder[1] = 1.01;
        aUpperBorder[2] = 2.01;

        math_GlobOptMin aSolver = new(aFunc, aLowerBorder, aUpperBorder, 9, 1.0e-3, 1.0e-8);
        aSolver.Perform(true);

        Assert.True(aSolver.isDone(), "Should handle small search space");

        math_Vector aSol = new(1, 2);
        aSolver.Points(1, aSol);
        Assert.Equal(1.0, aSol[1], 0.02);
        Assert.Equal(2.0, aSol[2], 0.02);
    }

    [Fact]
    public void OCC25004_BraninFunctionGlobalOptimum()
    {
        // OCC25004: Extrema_ExtCC incorrect result. Tests math_GlobOptMin on Branin benchmark.
        // The Branin function has exactly 3 global minima at ~(-pi,12.275), (pi,2.275), (9.42478,2.475).
        // Expected minimum value: ~0.39788735772.
        BraninFunction aFunc = new();

        math_Vector aLower = new(1, 2); math_Vector aUpper = new(1, 2);
        aLower[1] = -5;
        aLower[2] = 0;
        aUpper[1] = 10;
        aUpper[2] = 15;

        // Estimate Lipschitz constant on a regular 16x16 grid.
        int aGridOrder = 16;
        math_Vector aFuncValues = new(1, aGridOrder * aGridOrder);
        double aLipConst = 0;
        math_Vector aCurrPnt1 = new(1, 2); math_Vector aCurrPnt2 = new(1, 2);

        int idx = 1;
        for (int i = 1; i <= aGridOrder; i++)
        {
            for (int j = 1; j <= aGridOrder; j++)
            {
                aCurrPnt1[1] = aLower[1] + (aUpper[1] - aLower[1]) * (i - 1) / (aGridOrder - 1.0);
                aCurrPnt1[2] = aLower[2] + (aUpper[2] - aLower[2]) * (j - 1) / (aGridOrder - 1.0);
                aFunc.Value(aCurrPnt1, out aFuncValues[idx]);
                idx++;
            }
        }

        for (int i = 1; i <= aGridOrder; i++)
        {
            for (int j = 1; j <= aGridOrder; j++)
            {
                for (int k = 1; k <= aGridOrder; k++)
                {
                    for (int l = 1; l <= aGridOrder; l++)
                    {
                        if (i == k && j == l)
                        {
                            continue;
                        }
                        aCurrPnt1[1] = aLower[1] + (aUpper[1] - aLower[1]) * (i - 1) / (aGridOrder - 1.0);
                        aCurrPnt1[2] = aLower[2] + (aUpper[2] - aLower[2]) * (j - 1) / (aGridOrder - 1.0);
                        aCurrPnt2[1] = aLower[1] + (aUpper[1] - aLower[1]) * (k - 1) / (aGridOrder - 1.0);
                        aCurrPnt2[2] = aLower[2] + (aUpper[2] - aLower[2]) * (l - 1) / (aGridOrder - 1.0);
                        int idx1 = (i - 1) * aGridOrder + j;
                        int idx2 = (k - 1) * aGridOrder + l;
                        // Use subtracted vector norm for Lipschitz estimation
                        aCurrPnt1.Add(-aCurrPnt2);
                        double aDist = aCurrPnt1.Norm();
                        if (aDist > 0.0)
                        {
                            double aC = Math.Abs(aFuncValues[idx1] - aFuncValues[idx2]) / aDist;
                            if (aC > aLipConst)
                            {
                                aLipConst = aC;
                            }
                        }
                    }
                }
            }
        }

        math_GlobOptMin aFinder = new(aFunc, aLower, aUpper, aLipConst);
        aFinder.Perform();

        Assert.True(aFinder.isDone(), "Optimizer must converge on Branin function");
        Assert.Equal(0.39788735772, aFinder.GetF(), 0.1);
        Assert.Equal(3, aFinder.NbExtrema());
    }
}
