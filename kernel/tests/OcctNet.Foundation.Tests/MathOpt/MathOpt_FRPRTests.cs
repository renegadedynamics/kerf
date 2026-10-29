// Port of: src/FoundationClasses/TKMath/GTests/MathOpt_FRPR_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace OcctNet.Foundation.Tests;

public class MathOpt_FRPRTest
{
    private const double THE_TOLERANCE = 1.0e-6;

    // Test function structs for new API

    // Quadratic function: f(x,y) = (x-1)^2 + (y-2)^2, minimum at (1, 2) with f = 0
    private struct QuadraticFunc : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aDx = theX[1] - 1.0;
            double aDy = theX[2] - 2.0;
            theF = aDx * aDx + aDy * aDy;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            theGrad[1] = 2.0 * (theX[1] - 1.0);
            theGrad[2] = 2.0 * (theX[2] - 2.0);
            return true;
        }
    }

    // Rosenbrock function: f(x,y) = 100*(y-x^2)^2 + (1-x)^2, minimum at (1, 1) with f = 0
    private struct RosenbrockFunc : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aY - aX * aX;
            double aT2 = 1.0 - aX;
            theF = 100.0 * aT1 * aT1 + aT2 * aT2;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            double aX = theX[1];
            double aY = theX[2];
            theGrad[1] = -400.0 * aX * (aY - aX * aX) - 2.0 * (1.0 - aX);
            theGrad[2] = 200.0 * (aY - aX * aX);
            return true;
        }
    }

    // Booth function: f(x,y) = (x + 2y - 7)^2 + (2x + y - 5)^2, minimum at (1, 3) with f = 0
    private struct BoothFunc : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aX + 2.0 * aY - 7.0;
            double aT2 = 2.0 * aX + aY - 5.0;
            theF = aT1 * aT1 + aT2 * aT2;
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aX + 2.0 * aY - 7.0;
            double aT2 = 2.0 * aX + aY - 5.0;
            theGrad[1] = 2.0 * aT1 + 4.0 * aT2;
            theGrad[2] = 4.0 * aT1 + 2.0 * aT2;
            return true;
        }
    }

    // Sphere function in N dimensions: f(x) = sum(x_i^2), minimum at origin with f = 0
    private struct SphereFunc : IVectorFunctionWithGradient
    {
        public bool Value(math_Vector theX, out double theF)
        {
            theF = 0.0;
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theF += theX[i] * theX[i];
            }
            return true;
        }

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            for (int i = theX.Lower(); i <= theX.Upper(); ++i)
            {
                theGrad[i] = 2.0 * theX[i];
            }
            return true;
        }
    }

    // Old API adapter classes

    private sealed class QuadraticFuncOld : math_MultipleVarFunctionWithGradient
    {
        public override int NbVariables() => 2;

        public override bool Value(math_Vector theX, out double theF)
        {
            double aDx = theX[1] - 1.0;
            double aDy = theX[2] - 2.0;
            theF = aDx * aDx + aDy * aDy;
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
            return Value(theX, out theF) && Gradient(theX, theG);
        }
    }

    private sealed class RosenbrockFuncOld : math_MultipleVarFunctionWithGradient
    {
        public override int NbVariables() => 2;

        public override bool Value(math_Vector theX, out double theF)
        {
            double aX = theX[1];
            double aY = theX[2];
            double aT1 = aY - aX * aX;
            double aT2 = 1.0 - aX;
            theF = 100.0 * aT1 * aT1 + aT2 * aT2;
            return true;
        }

        public override bool Gradient(math_Vector theX, math_Vector theG)
        {
            double aX = theX[1];
            double aY = theX[2];
            theG[1] = -400.0 * aX * (aY - aX * aX) - 2.0 * (1.0 - aX);
            theG[2] = 200.0 * (aY - aX * aX);
            return true;
        }

        public override bool Values(math_Vector theX, out double theF, math_Vector theG)
        {
            return Value(theX, out theF) && Gradient(theX, theG);
        }
    }

    private static math_Vector Vec(params double[] theValues)
    {
        math_Vector aVec = new(1, theValues.Length);
        for (int i = 0; i < theValues.Length; ++i)
        {
            aVec[i + 1] = theValues[i];
        }
        return aVec;
    }

    [Fact]
    public void Quadratic_FletcherReeves()
    {
        QuadraticFunc aFunc = new();
        math_Vector aStart = Vec(5.0, 7.0);

        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.Tolerance = 1.0e-8;
        aConfig.Formula = MathOpt.ConjugateGradientFormula.FletcherReeves;

        VectorResult aResult = MathOpt.FRPR(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Quadratic_PolakRibiere()
    {
        QuadraticFunc aFunc = new();
        math_Vector aStart = Vec(5.0, 7.0);

        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.Tolerance = 1.0e-8;
        aConfig.Formula = MathOpt.ConjugateGradientFormula.PolakRibiere;

        VectorResult aResult = MathOpt.FRPR(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(2.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Booth()
    {
        BoothFunc aFunc = new();
        math_Vector aStart = Vec(0.0, 0.0);

        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.Tolerance = 1.0e-8;

        VectorResult aResult = MathOpt.FRPR(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(3.0, aResult.Solution![2], THE_TOLERANCE);
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Rosenbrock()
    {
        RosenbrockFunc aFunc = new();
        math_Vector aStart = Vec(-1.0, 1.0);

        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 2000;
        aConfig.Tolerance = 1.0e-6;
        aConfig.Formula = MathOpt.ConjugateGradientFormula.PolakRibiere;

        VectorResult aResult = MathOpt.FRPR(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], 1.0e-3);
        Assert.Equal(1.0, aResult.Solution![2], 1.0e-3);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void Sphere3D()
    {
        SphereFunc aFunc = new();
        math_Vector aStart = Vec(3.0, -2.0, 4.0);

        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.Tolerance = 1.0e-8;

        VectorResult aResult = MathOpt.FRPR(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        for (int i = 1; i <= 3; ++i)
        {
            Assert.Equal(0.0, aResult.Solution![i], THE_TOLERANCE);
        }
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    [Fact]
    public void Sphere5D()
    {
        SphereFunc aFunc = new();

        math_Vector aStart = new(1, 5);
        for (int i = 1; i <= 5; ++i)
        {
            aStart[i] = (double)i;
        }

        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.Tolerance = 1.0e-8;

        VectorResult aResult = MathOpt.FRPR(aFunc, aStart, aConfig);

        Assert.True(aResult.IsDone());
        for (int i = 1; i <= 5; ++i)
        {
            Assert.Equal(0.0, aResult.Solution![i], THE_TOLERANCE);
        }
        Assert.Equal(0.0, aResult.Value!.Value, THE_TOLERANCE);
    }

    // Value-only function
    private struct QuadraticValueOnly : IVectorFunction
    {
        public bool Value(math_Vector theX, out double theF)
        {
            double aDx = theX[1] - 1.0;
            double aDy = theX[2] - 2.0;
            theF = aDx * aDx + aDy * aDy;
            return true;
        }
    }

    [Fact]
    public void NumericalGradient_Quadratic()
    {
        QuadraticValueOnly aFunc = new();
        math_Vector aStart = Vec(5.0, 7.0);

        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.Tolerance = 1.0e-6;

        VectorResult aResult = MathOpt.FRPRNumerical(aFunc, aStart, 1.0e-8, aConfig);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.Solution![1], 1.0e-4);
        Assert.Equal(2.0, aResult.Solution![2], 1.0e-4);
        Assert.Equal(0.0, aResult.Value!.Value, 1.0e-4);
    }

    [Fact]
    public void CompareWithOldAPI_Quadratic()
    {
        QuadraticFuncOld anOldFunc = new();
        QuadraticFunc aNewFunc = new();
        math_Vector aStart = Vec(5.0, 7.0);

        // Old API - construct with tolerance and max iterations, then call Perform
        math_FRPR anOldSolver = new(anOldFunc, 1.0e-8, 100);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.Tolerance = 1.0e-8;
        VectorResult aNewResult = MathOpt.FRPR(aNewFunc, aStart, aConfig);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        // Both should find the same minimum
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], THE_TOLERANCE);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], THE_TOLERANCE);
        Assert.Equal(aNewResult.Value!.Value, anOldSolver.Minimum(), THE_TOLERANCE);
    }

    [Fact]
    public void CompareWithOldAPI_Rosenbrock()
    {
        RosenbrockFuncOld anOldFunc = new();
        RosenbrockFunc aNewFunc = new();
        math_Vector aStart = Vec(-1.0, 1.0);

        // Old API - construct with tolerance and max iterations, then call Perform
        math_FRPR anOldSolver = new(anOldFunc, 1.0e-6, 2000);
        anOldSolver.Perform(anOldFunc, aStart);

        // New API
        MathOpt.FRPRConfig aConfig = new();
        aConfig.MaxIterations = 2000;
        aConfig.Tolerance = 1.0e-6;
        VectorResult aNewResult = MathOpt.FRPR(aNewFunc, aStart, aConfig);

        Assert.True(anOldSolver.IsDone());
        Assert.True(aNewResult.IsDone());

        // Both should find the minimum (with reasonable tolerance for Rosenbrock)
        Assert.Equal(aNewResult.Solution![1], anOldSolver.Location()[1], 0.01);
        Assert.Equal(aNewResult.Solution![2], anOldSolver.Location()[2], 0.01);
    }

    [Fact]
    public void CompareWithBFGS()
    {
        QuadraticFunc aFunc = new();
        math_Vector aStart = Vec(5.0, 7.0);

        Config aConfig = new();
        aConfig.MaxIterations = 100;
        aConfig.XTolerance = 1.0e-8;
        aConfig.FTolerance = 1.0e-10;

        MathOpt.FRPRConfig aFRPRConfig = new();
        aFRPRConfig.MaxIterations = 100;
        aFRPRConfig.Tolerance = 1.0e-8;

        VectorResult aFRPRResult = MathOpt.FRPR(aFunc, aStart, aFRPRConfig);
        VectorResult aBFGSResult = MathOpt.BFGS(aFunc, aStart, aConfig);

        Assert.True(aFRPRResult.IsDone());
        Assert.True(aBFGSResult.IsDone());

        // Both should find the same minimum
        Assert.Equal(aBFGSResult.Value!.Value, aFRPRResult.Value!.Value, THE_TOLERANCE);
        Assert.Equal(aBFGSResult.Solution![1], aFRPRResult.Solution![1], THE_TOLERANCE);
        Assert.Equal(aBFGSResult.Solution![2], aFRPRResult.Solution![2], THE_TOLERANCE);
    }

    [Fact]
    public void FormulaComparison()
    {
        QuadraticFunc aFunc = new();
        math_Vector aStart = Vec(5.0, 7.0);

        MathOpt.FRPRConfig aConfigFR = new();
        aConfigFR.MaxIterations = 100;
        aConfigFR.Tolerance = 1.0e-8;
        aConfigFR.Formula = MathOpt.ConjugateGradientFormula.FletcherReeves;

        MathOpt.FRPRConfig aConfigPR = new();
        aConfigPR.MaxIterations = 100;
        aConfigPR.Tolerance = 1.0e-8;
        aConfigPR.Formula = MathOpt.ConjugateGradientFormula.PolakRibiere;

        MathOpt.FRPRConfig aConfigHS = new();
        aConfigHS.MaxIterations = 100;
        aConfigHS.Tolerance = 1.0e-8;
        aConfigHS.Formula = MathOpt.ConjugateGradientFormula.HestenesStiefel;

        VectorResult aResultFR = MathOpt.FRPR(aFunc, aStart, aConfigFR);
        VectorResult aResultPR = MathOpt.FRPR(aFunc, aStart, aConfigPR);
        VectorResult aResultHS = MathOpt.FRPR(aFunc, aStart, aConfigHS);

        Assert.True(aResultFR.IsDone());
        Assert.True(aResultPR.IsDone());
        Assert.True(aResultHS.IsDone());

        // All should find the same minimum
        Assert.Equal(aResultPR.Value!.Value, aResultFR.Value!.Value, THE_TOLERANCE);
        Assert.Equal(aResultHS.Value!.Value, aResultFR.Value!.Value, THE_TOLERANCE);
        Assert.Equal(1.0, aResultFR.Solution![1], THE_TOLERANCE);
        Assert.Equal(2.0, aResultFR.Solution![2], THE_TOLERANCE);
    }
}
