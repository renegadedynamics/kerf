// Port of: src/FoundationClasses/TKMath/GTests/MathSys_Newton2D_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

namespace Kerf.Kernel.Foundation.Tests;

public class MathSys_Newton2DTest
{
    private struct QuadraticFunc : global::Kerf.Engine.Numerics.Systems.MathSys.INewton2DFunction
    {
        public bool Invoke(double theU, double theV, ref Array2<double> theF, ref Array2<Array2<double>> theJ)
        {
            theF[0] = 2.0 * theU;
            theF[1] = 2.0 * theV;
            theJ[0][0] = 2.0;
            theJ[0][1] = 0.0;
            theJ[1][0] = 0.0;
            theJ[1][1] = 2.0;
            return true;
        }
    }

    private struct GenericLinearExactStep : global::Kerf.Engine.Numerics.Systems.MathSys.INewton2DFunction
    {
        public bool Invoke(double theU, double theV, ref Array2<double> theF, ref Array2<Array2<double>> theJ)
        {
            theF[0] = theU - 1.0;
            theF[1] = theV - 2.0;
            theJ[0][0] = 1.0;
            theJ[0][1] = 0.0;
            theJ[1][0] = 0.0;
            theJ[1][1] = 1.0;
            return true;
        }
    }

    private struct GenericHugeJacobianConstantResidual : global::Kerf.Engine.Numerics.Systems.MathSys.INewton2DFunction
    {
        public bool Invoke(double theU, double theV, ref Array2<double> theF, ref Array2<Array2<double>> theJ)
        {
            theF[0] = 1.0;
            theF[1] = 1.0;
            theJ[0][0] = 1.0e20;
            theJ[0][1] = 0.0;
            theJ[1][0] = 0.0;
            theJ[1][1] = 1.0e20;
            return true;
        }
    }

    private sealed class SymmetricDistFunc : global::Kerf.Engine.Numerics.Systems.MathSys.INewton2DSymmetricFunction
    {
        private readonly double myTargetU;
        private readonly double myTargetV;

        public SymmetricDistFunc(double theTargetU, double theTargetV)
        {
            myTargetU = theTargetU;
            myTargetV = theTargetV;
        }

        public bool Value(double theU, double theV, out double theFu, out double theFv)
        {
            theFu = 2.0 * (theU - myTargetU);
            theFv = 2.0 * (theV - myTargetV);
            return true;
        }

        public bool ValueAndJacobian(double theU, double theV, out double theFu, out double theFv,
                                     out double theDFuu, out double theDFuv, out double theDFvv)
        {
            theFu = 2.0 * (theU - myTargetU);
            theFv = 2.0 * (theV - myTargetV);
            theDFuu = 2.0;
            theDFuv = 0.0;
            theDFvv = 2.0;
            return true;
        }
    }

    // Constant residual with huge Jacobian creates tiny Newton step.
    private sealed class HugeJacobianConstantResidual : global::Kerf.Engine.Numerics.Systems.MathSys.INewton2DSymmetricFunction
    {
        public bool Value(double theU, double theV, out double theF1, out double theF2)
        {
            theF1 = 1.0;
            theF2 = 1.0;
            return true;
        }

        public bool ValueAndJacobian(double theU, double theV, out double theF1, out double theF2,
                                     out double theJ11, out double theJ12, out double theJ22)
        {
            theF1 = 1.0;
            theF2 = 1.0;
            theJ11 = 1.0e20;
            theJ12 = 0.0;
            theJ22 = 1.0e20;
            return true;
        }
    }

    // Synthetic system for Armijo directional derivative regression.
    // Residual is constant and cannot decrease. For this system:
    // - F . d is positive (old condition could accept)
    // - (J^T F) . d is negative (correct Armijo rejects all trial steps)
    private sealed class ArmijoRegressionFunc : global::Kerf.Engine.Numerics.Systems.MathSys.INewton2DSymmetricFunction
    {
        public bool Value(double theU, double theV, out double theF1, out double theF2)
        {
            theF1 = 1.0;
            theF2 = 2.0;
            return true;
        }

        public bool ValueAndJacobian(double theU, double theV, out double theF1, out double theF2,
                                     out double theJ11, out double theJ12, out double theJ22)
        {
            theF1 = 1.0;
            theF2 = 2.0;
            theJ11 = 1.0;
            theJ12 = 0.0;
            theJ22 = -1.0;
            return true;
        }
    }

    private static global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 Bounds(double theMinU, double theMinV, double theMaxU, double theMaxV)
    {
        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = new();
        aBounds.Min = Array2<double>.Of(theMinU, theMinV);
        aBounds.Max = Array2<double>.Of(theMaxU, theMaxV);
        return aBounds;
    }

    [Fact]
    public void Solve2D_Quadratic_Converges()
    {
        QuadraticFunc aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(-10.0, -10.0, 10.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-10;
        aOptions.MaxIterations = 50;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult = global::Kerf.Engine.Numerics.Systems.MathSys.Solve2D(aFunc, Array2<double>.Of(5.0, 3.0), aBounds, aOptions);

        Assert.True(aResult.IsDone);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK, aResult.Status);
        Assert.Equal(0.0, aResult.X[0], 1.0e-12);
        Assert.Equal(0.0, aResult.X[1], 1.0e-12);
        Assert.True(aResult.ResidualNorm < 1.0e-10);
    }

    [Fact]
    public void Solve2D_SmallStepAtRoot_ReturnsOK()
    {
        GenericLinearExactStep aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(-10.0, -10.0, 10.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-12;
        aOptions.XTolerance = 100.0;
        aOptions.MaxIterations = 5;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult = global::Kerf.Engine.Numerics.Systems.MathSys.Solve2D(aFunc, Array2<double>.Of(0.0, 0.0), aBounds, aOptions);
        Assert.True(aResult.IsDone);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK, aResult.Status);
        Assert.Equal(1.0, aResult.X[0], 1.0e-14);
        Assert.Equal(2.0, aResult.X[1], 1.0e-14);
    }

    [Fact]
    public void Solve2D_TinyStepLargeResidual_ReturnsMaxIterations()
    {
        GenericHugeJacobianConstantResidual aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(-1.0, -1.0, 1.0, 1.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-8;
        aOptions.XTolerance = 1.0e-16;
        aOptions.MaxIterations = 10;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult = global::Kerf.Engine.Numerics.Systems.MathSys.Solve2D(aFunc, Array2<double>.Of(0.0, 0.0), aBounds, aOptions);
        Assert.False(aResult.IsDone);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations, aResult.Status);
        Assert.True(aResult.ResidualNorm > 1.0e-2);
    }

    [Fact]
    public void Solve2DSymmetric_Target_Converges()
    {
        SymmetricDistFunc aFunc = new(3.5, 7.2);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(-10.0, -10.0, 10.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-10;
        aOptions.MaxIterations = 50;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve2DSymmetric(aFunc, Array2<double>.Of(0.0, 0.0), aBounds, aOptions);

        Assert.True(aResult.IsDone);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK, aResult.Status);
        Assert.Equal(3.5, aResult.X[0], 1.0e-9);
        Assert.Equal(7.2, aResult.X[1], 1.0e-9);
        Assert.True(aResult.ResidualNorm < 1.0e-10);
    }

    [Fact]
    public void Regression_TinyStepDoesNotConvergeWithLargeResidual()
    {
        HugeJacobianConstantResidual aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(-1.0, -1.0, 1.0, 1.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-8;
        aOptions.XTolerance = 1.0e-16;
        aOptions.MaxIterations = 10;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve2DSymmetric(aFunc, Array2<double>.Of(0.0, 0.0), aBounds, aOptions);

        Assert.False(aResult.IsDone);
        Assert.NotEqual(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK, aResult.Status);
        Assert.True(aResult.ResidualNorm > 1.0e-2);
    }

    [Fact]
    public void Regression_ArmijoUsesJtFDirectionalDerivative()
    {
        ArmijoRegressionFunc aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(-10.0, -10.0, 10.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.EnableLineSearch = true;
        aOptions.MaxIterations = 10;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve2DSymmetric(aFunc, Array2<double>.Of(0.0, 0.0), aBounds, aOptions);

        Assert.False(aResult.IsDone);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NonDescentDirection, aResult.Status);
    }

    [Fact]
    public void Solve2DSymmetric_HighPrecision()
    {
        SymmetricDistFunc aFunc = new(5.123456789, 2.987654321);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(-10.0, -10.0, 10.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 5.0e-8;
        aOptions.MaxIterations = 100;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve2DSymmetric(aFunc, Array2<double>.Of(0.0, 0.0), aBounds, aOptions);

        Assert.True(aResult.IsDone);
        Assert.Equal(5.123456789, aResult.X[0], 1.0e-7);
        Assert.Equal(2.987654321, aResult.X[1], 1.0e-7);
        Assert.True(aResult.ResidualNorm < 5.0e-8);
    }

    [Fact]
    public void Solve2DSymmetric_InvalidInput()
    {
        SymmetricDistFunc aFunc = new(0.0, 0.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(10.0, 0.0, 0.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-10;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve2DSymmetric(aFunc, Array2<double>.Of(0.0, 0.0), aBounds, aOptions);

        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput, aResult.Status);
        Assert.False(aResult.IsDone);
    }

    [Fact]
    public void Solve2DSymmetric_ConsistentFromDifferentStarts()
    {
        SymmetricDistFunc aFunc = new(5.0, 5.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN2 aBounds = Bounds(0.0, 0.0, 10.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-10;
        aOptions.MaxIterations = 50;

        Array2<double>[] aStarts = [Array2<double>.Of(0.0, 0.0),
                                        Array2<double>.Of(10.0, 10.0),
                                        Array2<double>.Of(2.0, 8.0),
                                        Array2<double>.Of(8.0, 2.0)];

        foreach (Array2<double> aStart in aStarts)
        {
            global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN2 aResult =
              global::Kerf.Engine.Numerics.Systems.MathSys.Solve2DSymmetric(aFunc, aStart, aBounds, aOptions);
            Assert.True(aResult.IsDone);
            Assert.Equal(5.0, aResult.X[0], 1.0e-8);
            Assert.Equal(5.0, aResult.X[1], 1.0e-8);
        }
    }
}
