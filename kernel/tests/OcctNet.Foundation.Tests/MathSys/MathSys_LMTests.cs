// Port of: src/FoundationClasses/TKMath/GTests/MathSys_LM_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

// The test functions are named <C++ name>Func: C++ keeps them in a namespace apart from the test names.
public class MathSys_LM_Test
{

    private const double THE_TOLERANCE = 1.0e-6;
    private const double THE_LOOSE_TOL = 1.0e-3;

    // ============================================================================
    // Test function classes for new API
    // ============================================================================

    // Simple linear system: F(x) = A*x - b
    // System: x1 + x2 = 3, x1 - x2 = 1
    // Solution: x1 = 2, x2 = 1
    private sealed class LinearSystem2DFunc : MathSys.IFunctionSetWithDerivatives
    {
        public int NbVariables() { return 2; }

        public int NbEquations() { return 2; }

        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = theX[1] + theX[2] - 3.0;
            theF[2] = theX[1] - theX[2] - 1.0;
            return true;
        }

        public bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            theD[1, 1] = 1.0;
            theD[1, 2] = 1.0;
            theD[2, 1] = 1.0;
            theD[2, 2] = -1.0;
            return true;
        }

        public bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            return Value(theX, theF) && Derivatives(theX, theD);
        }
    }

    // Nonlinear system:
    // F1(x,y) = x^2 + y^2 - 4
    // F2(x,y) = x*y - 1
    // Solution near (1.93, 0.52) for starting point (1,1)
    private sealed class CircleHyperbolaFunc : MathSys.IFunctionSetWithDerivatives
    {
        public int NbVariables() { return 2; }

        public int NbEquations() { return 2; }

        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = theX[1] * theX[1] + theX[2] * theX[2] - 4.0;
            theF[2] = theX[1] * theX[2] - 1.0;
            return true;
        }

        public bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            theD[1, 1] = 2.0 * theX[1];
            theD[1, 2] = 2.0 * theX[2];
            theD[2, 1] = theX[2];
            theD[2, 2] = theX[1];
            return true;
        }

        public bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            return Value(theX, theF) && Derivatives(theX, theD);
        }
    }

    // Rosenbrock function in residual form for least squares
    // F1(x,y) = 10*(y - x^2)
    // F2(x,y) = 1 - x
    // Minimum at (1,1) with F = (0,0)
    private sealed class RosenbrockResidualFunc : MathSys.IFunctionSetWithDerivatives
    {
        public int NbVariables() { return 2; }

        public int NbEquations() { return 2; }

        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = 10.0 * (theX[2] - theX[1] * theX[1]);
            theF[2] = 1.0 - theX[1];
            return true;
        }

        public bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            theD[1, 1] = -20.0 * theX[1];
            theD[1, 2] = 10.0;
            theD[2, 1] = -1.0;
            theD[2, 2] = 0.0;
            return true;
        }

        public bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            return Value(theX, theF) && Derivatives(theX, theD);
        }
    }

    // Powell singular function (4D)
    // F1 = x1 + 10*x2
    // F2 = sqrt(5)*(x3 - x4)
    // F3 = (x2 - 2*x3)^2
    // F4 = sqrt(10)*(x1 - x4)^2
    // Minimum at origin with F = 0
    private sealed class PowellSingularFunc : MathSys.IFunctionSetWithDerivatives
    {
        public int NbVariables() { return 4; }

        public int NbEquations() { return 4; }

        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = theX[1] + 10.0 * theX[2];
            theF[2] = Math.Sqrt(5.0) * (theX[3] - theX[4]);
            theF[3] = (theX[2] - 2.0 * theX[3]) * (theX[2] - 2.0 * theX[3]);
            theF[4] = Math.Sqrt(10.0) * (theX[1] - theX[4]) * (theX[1] - theX[4]);
            return true;
        }

        public bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            theD.Init(0.0);
            // F1 = x1 + 10*x2
            theD[1, 1] = 1.0;
            theD[1, 2] = 10.0;
            // F2 = sqrt(5)*(x3 - x4)
            theD[2, 3] = Math.Sqrt(5.0);
            theD[2, 4] = -Math.Sqrt(5.0);
            // F3 = (x2 - 2*x3)^2
            double aT1 = theX[2] - 2.0 * theX[3];
            theD[3, 2] = 2.0 * aT1;
            theD[3, 3] = -4.0 * aT1;
            // F4 = sqrt(10)*(x1 - x4)^2
            double aT2 = theX[1] - theX[4];
            theD[4, 1] = 2.0 * Math.Sqrt(10.0) * aT2;
            theD[4, 4] = -2.0 * Math.Sqrt(10.0) * aT2;
            return true;
        }

        public bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            return Value(theX, theF) && Derivatives(theX, theD);
        }
    }

    // Exponential fitting problem
    // Data: (t_i, y_i), Model: f(t) = x1 * exp(x2 * t)
    // Residuals: F_i = y_i - x1 * exp(x2 * t_i)
    private sealed class ExponentialFitFunc : MathSys.IFunctionSetWithDerivatives
    {
        private readonly double[] myT = [0.0, 1.0, 2.0, 3.0, 4.0];
        private readonly double[] myY = [1.0, 2.7, 7.4, 20.1, 54.6]; // Approx e^t

        public int NbVariables() { return 2; }

        public int NbEquations() { return 5; }

        public bool Value(math_Vector theX, math_Vector theF)
        {
            for (int i = 0; i < 5; ++i)
            {
                theF[i + 1] = myY[i] - theX[1] * Math.Exp(theX[2] * myT[i]);
            }
            return true;
        }

        public bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            for (int i = 0; i < 5; ++i)
            {
                double aExp = Math.Exp(theX[2] * myT[i]);
                theD[i + 1, 1] = -aExp;
                theD[i + 1, 2] = -theX[1] * myT[i] * aExp;
            }
            return true;
        }

        public bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            return Value(theX, theF) && Derivatives(theX, theD);
        }
    }

    // Overdetermined system (more equations than variables)
    // F1 = x1 + x2 - 2
    // F2 = x1 - x2 - 0
    // F3 = 2*x1 + x2 - 3
    // Least squares solution: x1 = 1, x2 = 1
    private sealed class OverdeterminedSystemFunc : MathSys.IFunctionSetWithDerivatives
    {
        public int NbVariables() { return 2; }

        public int NbEquations() { return 3; }

        public bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = theX[1] + theX[2] - 2.0;
            theF[2] = theX[1] - theX[2];
            theF[3] = 2.0 * theX[1] + theX[2] - 3.0;
            return true;
        }

        public bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            theD[1, 1] = 1.0;
            theD[1, 2] = 1.0;
            theD[2, 1] = 1.0;
            theD[2, 2] = -1.0;
            theD[3, 1] = 2.0;
            theD[3, 2] = 1.0;
            return true;
        }

        public bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            return Value(theX, theF) && Derivatives(theX, theD);
        }
    }

    // ============================================================================
    // Old API adapters for comparison
    // ============================================================================

    private sealed class CircleHyperbolaOld : math_FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 2; }

        public override int NbEquations() { return 2; }

        public override bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = theX[1] * theX[1] + theX[2] * theX[2] - 4.0;
            theF[2] = theX[1] * theX[2] - 1.0;
            return true;
        }

        public override bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            theD[1, 1] = 2.0 * theX[1];
            theD[1, 2] = 2.0 * theX[2];
            theD[2, 1] = theX[2];
            theD[2, 2] = theX[1];
            return true;
        }

        public override bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            return Value(theX, theF) && Derivatives(theX, theD);
        }
    }

    private sealed class RosenbrockResidualOld : math_FunctionSetWithDerivatives
    {
        public override int NbVariables() { return 2; }

        public override int NbEquations() { return 2; }

        public override bool Value(math_Vector theX, math_Vector theF)
        {
            theF[1] = 10.0 * (theX[2] - theX[1] * theX[1]);
            theF[2] = 1.0 - theX[1];
            return true;
        }

        public override bool Derivatives(math_Vector theX, math_Matrix theD)
        {
            theD[1, 1] = -20.0 * theX[1];
            theD[1, 2] = 10.0;
            theD[2, 1] = -1.0;
            theD[2, 2] = 0.0;
            return true;
        }

        public override bool Values(math_Vector theX, math_Vector theF, math_Matrix theD)
        {
            return Value(theX, theF) && Derivatives(theX, theD);
        }
    }

    // ============================================================================
    // Basic Levenberg-Marquardt tests
    // ============================================================================

    [Fact]
    public void LinearSystem2D()
    {
        LinearSystem2DFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.0;
        aStart[2] = 0.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 100;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(2.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void CircleHyperbola()
    {
        CircleHyperbolaFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 1.5;
        aStart[2] = 0.5;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 100;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());

        // Verify solution satisfies both equations
        double aX = aResult.Solution![1];
        double aY = aResult.Solution![2];

        Assert.Equal(4.0, aX * aX + aY * aY, THE_TOLERANCE);
        Assert.Equal(1.0, aX * aY, THE_TOLERANCE);
    }

    [Fact]
    public void RosenbrockResidual()
    {
        RosenbrockResidualFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = -1.0;
        aStart[2] = 1.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 200;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_LOOSE_TOL);
        Assert.Equal(1.0, aResult.Solution![2], THE_LOOSE_TOL);
        Assert.True(aResult.Value!.Value < 0.01);
    }

    [Fact]
    public void PowellSingular()
    {
        PowellSingularFunc aFunc = new();

        math_Vector aStart = new(1, 4);
        aStart[1] = 3.0;
        aStart[2] = -1.0;
        aStart[3] = 0.0;
        aStart[4] = 1.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 500;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());

        // Powell singular function converges to origin
        for (int i = 1; i <= 4; ++i)
        {
            Assert.Equal(0.0, aResult.Solution![i], 0.1);
        }
        Assert.True(aResult.Value!.Value < 0.01);
    }

    // ============================================================================
    // Overdetermined system (nonlinear least squares)
    // ============================================================================

    [Fact]
    public void OverdeterminedSystem()
    {
        OverdeterminedSystemFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.0;
        aStart[2] = 0.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.MaxIterations = 100;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Solution![2], THE_TOLERANCE);
    }

    [Fact]
    public void ExponentialFit()
    {
        ExponentialFitFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.5; // Initial guess for amplitude
        aStart[2] = 0.5; // Initial guess for rate

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-8;
        aConfig.MaxIterations = 100;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());

        // The fit should be approximately f(t) = 1 * exp(1 * t)
        Assert.Equal(1.0, aResult.Solution![1], 0.1);
        Assert.Equal(1.0, aResult.Solution![2], 0.1);
    }

    // ============================================================================
    // Bounded Levenberg-Marquardt tests
    // ============================================================================

    [Fact]
    public void BoundedCircleHyperbola()
    {
        CircleHyperbolaFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 1.0;
        aStart[2] = 1.0;

        math_Vector aLower = new(1, 2);
        aLower[1] = 0.0;
        aLower[2] = 0.0;

        math_Vector aUpper = new(1, 2);
        aUpper[1] = 5.0;
        aUpper[2] = 5.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 100;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardtBounded(aFunc, aStart, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone());

        // Verify solution is within bounds
        double aX = aResult.Solution![1];
        double aY = aResult.Solution![2];

        Assert.True(aX >= 0.0);
        Assert.True(aX <= 5.0);
        Assert.True(aY >= 0.0);
        Assert.True(aY <= 5.0);

        // Verify solution satisfies both equations
        Assert.Equal(4.0, aX * aX + aY * aY, THE_TOLERANCE);
        Assert.Equal(1.0, aX * aY, THE_TOLERANCE);
    }

    [Fact]
    public void BoundedRosenbrock()
    {
        RosenbrockResidualFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = -1.0;
        aStart[2] = 1.0;

        math_Vector aLower = new(1, 2);
        aLower[1] = -5.0;
        aLower[2] = -5.0;

        math_Vector aUpper = new(1, 2);
        aUpper[1] = 5.0;
        aUpper[2] = 5.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 200;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardtBounded(aFunc, aStart, aLower, aUpper, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_LOOSE_TOL);
        Assert.Equal(1.0, aResult.Solution![2], THE_LOOSE_TOL);
    }

    [Fact]
    public void BoundedActiveConstraint()
    {
        // System where optimal solution would be outside bounds
        LinearSystem2DFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 1.0;
        aStart[2] = 1.0;

        // Constrain x1 <= 1.5, so solution (2, 1) can't be reached exactly
        math_Vector aLower = new(1, 2);
        aLower[1] = -10.0;
        aLower[2] = -10.0;

        math_Vector aUpper = new(1, 2);
        aUpper[1] = 1.5;
        aUpper[2] = 10.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.MaxIterations = 100;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardtBounded(aFunc, aStart, aLower, aUpper, aConfig);

        // Solution should be at or near the bound
        Assert.True(aResult.Solution![1] <= 1.5 + THE_TOLERANCE);
    }

    // ============================================================================
    // Damping parameter tests
    // ============================================================================

    [Fact]
    public void DampingParameters()
    {
        CircleHyperbolaFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 1.5;
        aStart[2] = 0.5;

        // Test with different initial lambda values
        double[] aLambdas = [1.0e-6, 1.0e-3, 1.0, 100.0];

        foreach (double aLambda in aLambdas)
        {
            MathSys.LMConfig aConfig = new();
            aConfig.Tolerance = 1.0e-10;
            aConfig.FTolerance = 1.0e-10;
            aConfig.MaxIterations = 200;
            aConfig.LambdaInit = aLambda;

            MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

            Assert.True(aResult.IsDone());

            double aX = aResult.Solution![1];
            double aY = aResult.Solution![2];

            Assert.Equal(4.0, aX * aX + aY * aY, THE_TOLERANCE);
            Assert.Equal(1.0, aX * aY, THE_TOLERANCE);
        }
    }

    [Fact]
    public void LambdaUpdateFactors()
    {
        RosenbrockResidualFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = -1.0;
        aStart[2] = 1.0;

        // Test with different lambda update factors
        (double myIncrease, double myDecrease)[] aFactors = [(2.0, 0.5), (5.0, 0.2), (10.0, 0.1), (20.0, 0.05)];

        foreach ((double myIncrease, double myDecrease) aFactor in aFactors)
        {
            MathSys.LMConfig aConfig = new();
            aConfig.Tolerance = 1.0e-10;
            aConfig.FTolerance = 1.0e-10;
            aConfig.MaxIterations = 300;
            aConfig.LambdaIncrease = aFactor.myIncrease;
            aConfig.LambdaDecrease = aFactor.myDecrease;

            MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

            Assert.True(aResult.IsDone());
            Assert.True(aResult.Value!.Value < 0.1);
        }
    }

    // ============================================================================
    // Comparison with old API
    // ============================================================================

    [Fact]
    public void CompareWithOldAPI_CircleHyperbola()
    {
        CircleHyperbolaOld anOldFunc = new();
        CircleHyperbolaFunc aNewFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 1.5;
        aStart[2] = 0.5;

        math_Vector aTol = new(1, 2, 1.0e-10);

        // Old API
        math_FunctionSetRoot anOldSolver = new(anOldFunc, aTol, 100);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 100;
        MathUtils.VectorResult aNewResult = MathSys.LevenbergMarquardt(aNewFunc, aStart, aConfig);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        // Both should find a valid solution
        math_Vector anOldSol = anOldSolver.Root();
        Assert.Equal(4.0, anOldSol[1] * anOldSol[1] + anOldSol[2] * anOldSol[2], THE_LOOSE_TOL);
        Assert.Equal(1.0, anOldSol[1] * anOldSol[2], THE_LOOSE_TOL);

        double aNewX = aNewResult.Solution![1];
        double aNewY = aNewResult.Solution![2];
        Assert.Equal(4.0, aNewX * aNewX + aNewY * aNewY, THE_TOLERANCE);
        Assert.Equal(1.0, aNewX * aNewY, THE_TOLERANCE);
    }

    [Fact]
    public void CompareWithOldAPI_Rosenbrock()
    {
        RosenbrockResidualOld anOldFunc = new();
        RosenbrockResidualFunc aNewFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = -1.0;
        aStart[2] = 1.0;

        math_Vector aTol = new(1, 2, 1.0e-10);

        // Old API
        math_FunctionSetRoot anOldSolver = new(anOldFunc, aTol, 200);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 200;
        MathUtils.VectorResult aNewResult = MathSys.LevenbergMarquardt(aNewFunc, aStart, aConfig);

        Assert.True(aNewResult.IsDone());

        // New API should find the optimal solution
        Assert.Equal(1.0, aNewResult.Solution![1], THE_LOOSE_TOL);
        Assert.Equal(1.0, aNewResult.Solution![2], THE_LOOSE_TOL);
    }

    [Fact]
    public void CompareWithOldAPI_Bounded()
    {
        CircleHyperbolaOld anOldFunc = new();
        CircleHyperbolaFunc aNewFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 1.0;
        aStart[2] = 1.0;

        math_Vector aLower = new(1, 2);
        aLower[1] = 0.0;
        aLower[2] = 0.0;

        math_Vector aUpper = new(1, 2);
        aUpper[1] = 5.0;
        aUpper[2] = 5.0;

        math_Vector aTol = new(1, 2, 1.0e-10);

        // Old API with bounds
        math_FunctionSetRoot anOldSolver = new(anOldFunc, aTol, 100);
        anOldSolver.Perform(anOldFunc, aStart, aLower, aUpper);

        // New API with bounds
        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-10;
        aConfig.FTolerance = 1.0e-10;
        aConfig.MaxIterations = 100;
        MathUtils.VectorResult aNewResult = MathSys.LevenbergMarquardtBounded(aNewFunc, aStart, aLower, aUpper, aConfig);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        // Both should find solutions within bounds
        math_Vector anOldSol = anOldSolver.Root();
        Assert.True(anOldSol[1] >= 0.0);
        Assert.True(anOldSol[1] <= 5.0);
        Assert.True(anOldSol[2] >= 0.0);
        Assert.True(anOldSol[2] <= 5.0);

        double aNewX = aNewResult.Solution![1];
        double aNewY = aNewResult.Solution![2];
        Assert.True(aNewX >= 0.0);
        Assert.True(aNewX <= 5.0);
        Assert.True(aNewY >= 0.0);
        Assert.True(aNewY <= 5.0);
    }

    // ============================================================================
    // Convergence and robustness tests
    // ============================================================================

    [Fact]
    public void DifferentStartingPoints()
    {
        CircleHyperbolaFunc aFunc = new();

        // Test different starting points
        double[][] aStarts = [[1.5, 0.5], [2.0, 0.5], [1.0, 1.0], [2.5, 0.5], [0.5, 2.0]];

        foreach (double[] aStartPt in aStarts)
        {
            math_Vector aStart = new(1, 2);
            aStart[1] = aStartPt[0];
            aStart[2] = aStartPt[1];

            MathSys.LMConfig aConfig = new();
            aConfig.Tolerance = 1.0e-10;
            aConfig.FTolerance = 1.0e-10;
            aConfig.MaxIterations = 100;

            MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

            Assert.True(aResult.IsDone());

            double aX = aResult.Solution![1];
            double aY = aResult.Solution![2];

            Assert.Equal(4.0, aX * aX + aY * aY, THE_TOLERANCE);
            Assert.Equal(1.0, aX * aY, THE_TOLERANCE);
        }
    }

    [Fact]
    public void ConvergenceIterations()
    {
        LinearSystem2DFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.0;
        aStart[2] = 0.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-12;
        aConfig.FTolerance = 1.0e-12;
        aConfig.MaxIterations = 100;

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());

        // Linear system should converge quickly
        Assert.True(aResult.NbIterations < 20);

        Assert.Equal(2.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(1.0, aResult.Solution![2], THE_TOLERANCE);
    }

    // ============================================================================
    // Error handling tests
    // ============================================================================

    [Fact]
    public void MaxIterationsReached()
    {
        RosenbrockResidualFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = -1.0;
        aStart[2] = 1.0;

        MathSys.LMConfig aConfig = new();
        aConfig.Tolerance = 1.0e-15; // Very tight tolerance
        aConfig.MaxIterations = 5;       // Very few iterations

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        // Should either succeed or reach max iterations (not crash)
        Assert.True(aResult.Solution is not null);
    }

    [Fact]
    public void InvalidInputDimensions()
    {
        LinearSystem2DFunc aFunc = new();

        // Wrong dimension for start vector
        math_Vector aStart = new(1, 3); // 3 elements but function expects 2
        aStart[1] = 0.0;
        aStart[2] = 0.0;
        aStart[3] = 0.0;

        MathSys.LMConfig aConfig = new();

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardt(aFunc, aStart, aConfig);

        Assert.False(aResult.IsDone());
        Assert.Equal(MathUtils.Status.InvalidInput, aResult.Status);
    }

    [Fact]
    public void BoundedInvalidDimensions()
    {
        LinearSystem2DFunc aFunc = new();

        math_Vector aStart = new(1, 2);
        aStart[1] = 0.0;
        aStart[2] = 0.0;

        math_Vector aLower = new(1, 3); // Wrong size
        aLower[1] = -10.0;
        aLower[2] = -10.0;
        aLower[3] = -10.0;

        math_Vector aUpper = new(1, 2);
        aUpper[1] = 10.0;
        aUpper[2] = 10.0;

        MathSys.LMConfig aConfig = new();

        MathUtils.VectorResult aResult = MathSys.LevenbergMarquardtBounded(aFunc, aStart, aLower, aUpper, aConfig);

        Assert.False(aResult.IsDone());
        Assert.Equal(MathUtils.Status.InvalidInput, aResult.Status);
    }
}
