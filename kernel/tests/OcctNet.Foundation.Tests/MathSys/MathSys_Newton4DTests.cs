// Port of: src/FoundationClasses/TKMath/GTests/MathSys_Newton4D_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace OcctNet.Foundation.Tests;

// The C++ tests pass lambdas; here each lambda is a private struct implementing MathSys.INewton4DFunction.
public class MathSys_Newton4DTest
{
    private const double THE_TOL = 1.0e-10;

    private struct LinearSystemFunc : MathSys.INewton4DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3, double theX4,
                           ref std_array4<double> theF, ref std_array4<std_array4<double>> theJ)
        {
            theF[0] = theX1 - 1.0;
            theF[1] = theX2 - 2.0;
            theF[2] = theX3 - 3.0;
            theF[3] = theX4 - 4.0;

            theJ[0][0] = 1.0;
            theJ[0][1] = 0.0;
            theJ[0][2] = 0.0;
            theJ[0][3] = 0.0;
            theJ[1][0] = 0.0;
            theJ[1][1] = 1.0;
            theJ[1][2] = 0.0;
            theJ[1][3] = 0.0;
            theJ[2][0] = 0.0;
            theJ[2][1] = 0.0;
            theJ[2][2] = 1.0;
            theJ[2][3] = 0.0;
            theJ[3][0] = 0.0;
            theJ[3][1] = 0.0;
            theJ[3][2] = 0.0;
            theJ[3][3] = 1.0;
            return true;
        }
    }

    private struct ShiftedIdentityFunc : MathSys.INewton4DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3, double theX4,
                           ref std_array4<double> theF, ref std_array4<std_array4<double>> theJ)
        {
            theF[0] = theX1 - 1.0;
            theF[1] = theX2 - 2.0;
            theF[2] = theX3 - 3.0;
            theF[3] = theX4 - 4.0;

            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    theJ[r][c] = (r == c) ? 1.0 : 0.0;
                }
            }
            return true;
        }
    }

    private struct HugeJacobianConstantResidualFunc : MathSys.INewton4DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3, double theX4,
                           ref std_array4<double> theF, ref std_array4<std_array4<double>> theJ)
        {
            theF[0] = 1.0;
            theF[1] = 1.0;
            theF[2] = 1.0;
            theF[3] = 1.0;

            for (int r = 0; r < 4; ++r)
            {
                for (int c = 0; c < 4; ++c)
                {
                    theJ[r][c] = (r == c) ? 1.0e20 : 0.0;
                }
            }
            return true;
        }
    }

    private struct ZeroResidualFunc : MathSys.INewton4DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3, double theX4,
                           ref std_array4<double> theF, ref std_array4<std_array4<double>> theJ)
        {
            for (int i = 0; i < 4; ++i)
            {
                theF[i] = 0.0;
                for (int j = 0; j < 4; ++j)
                {
                    theJ[i][j] = (i == j) ? 1.0 : 0.0;
                }
            }
            return true;
        }
    }

    private static MathSys.NewtonBoundsN4 NoBounds()
    {
        MathSys.NewtonBoundsN4 aBounds = new();
        aBounds.HasBounds = false;
        return aBounds;
    }

    [Fact]
    public void Solve4D_LinearSystem()
    {
        LinearSystemFunc aFunc = new();

        MathSys.NewtonBoundsN4 aBounds = NoBounds();

        MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = THE_TOL;

        MathSys.NewtonResultN4 aResult =
          MathSys.Solve4D(aFunc, std_array4<double>.Of(0.0, 0.0, 0.0, 0.0), aBounds, aOptions);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.X[0], 1.0e-12);
        Assert.Equal(2.0, aResult.X[1], 1.0e-12);
        Assert.Equal(3.0, aResult.X[2], 1.0e-12);
        Assert.Equal(4.0, aResult.X[3], 1.0e-12);
    }

    [Fact]
    public void Solve4D_Bounded()
    {
        ShiftedIdentityFunc aFunc = new();

        MathSys.NewtonBoundsN4 aBounds = new();
        aBounds.Min = std_array4<double>.Of(-10.0, -10.0, -10.0, -10.0);
        aBounds.Max = std_array4<double>.Of(10.0, 10.0, 10.0, 10.0);

        MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = THE_TOL;

        MathSys.NewtonResultN4 aResult =
          MathSys.Solve4D(aFunc, std_array4<double>.Of(9.0, 8.0, 7.0, 6.0), aBounds, aOptions);

        Assert.True(aResult.IsDone());
        Assert.Equal(1.0, aResult.X[0], 1.0e-12);
        Assert.Equal(2.0, aResult.X[1], 1.0e-12);
        Assert.Equal(3.0, aResult.X[2], 1.0e-12);
        Assert.Equal(4.0, aResult.X[3], 1.0e-12);
    }

    [Fact]
    public void Solve4D_SmallStepAtRoot_ReturnsOK()
    {
        ShiftedIdentityFunc aFunc = new();

        MathSys.NewtonBoundsN4 aBounds = NoBounds();

        MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-12;
        aOptions.XTolerance = 100.0;
        aOptions.MaxIterations = 5;
        aOptions.MaxStepRatio = 100.0;

        MathSys.NewtonResultN4 aResult =
          MathSys.Solve4D(aFunc, std_array4<double>.Of(0.0, 0.0, 0.0, 0.0), aBounds, aOptions);
        Assert.True(aResult.IsDone());
        Assert.Equal(MathUtils.Status.OK, aResult.Status);
        Assert.Equal(1.0, aResult.X[0], 1.0e-14);
        Assert.Equal(2.0, aResult.X[1], 1.0e-14);
        Assert.Equal(3.0, aResult.X[2], 1.0e-14);
        Assert.Equal(4.0, aResult.X[3], 1.0e-14);
    }

    [Fact]
    public void Solve4D_TinyStepLargeResidual_ReturnsMaxIterations()
    {
        HugeJacobianConstantResidualFunc aFunc = new();

        MathSys.NewtonBoundsN4 aBounds = NoBounds();

        MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-8;
        aOptions.XTolerance = 1.0e-16;
        aOptions.MaxIterations = 10;

        MathSys.NewtonResultN4 aResult =
          MathSys.Solve4D(aFunc, std_array4<double>.Of(0.0, 0.0, 0.0, 0.0), aBounds, aOptions);
        Assert.False(aResult.IsDone());
        Assert.Equal(MathUtils.Status.MaxIterations, aResult.Status);
        Assert.True(aResult.ResidualNorm > 1.0e-2);
    }

    [Fact]
    public void Solve4D_InvalidInput()
    {
        ZeroResidualFunc aFunc = new();

        MathSys.NewtonBoundsN4 aBounds = new();
        aBounds.Min = std_array4<double>.Of(1.0, 0.0, 0.0, 0.0);
        aBounds.Max = std_array4<double>.Of(0.0, 1.0, 1.0, 1.0);

        MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = THE_TOL;

        MathSys.NewtonResultN4 aResult =
          MathSys.Solve4D(aFunc, std_array4<double>.Of(0.0, 0.0, 0.0, 0.0), aBounds, aOptions);

        Assert.Equal(MathUtils.Status.InvalidInput, aResult.Status);
        Assert.False(aResult.IsDone());
    }

    private sealed class PlaneSurfaceEval : MathSys.ISurfaceD2Evaluator
    {
        public void D2(double theU, double theV, out gp_Pnt theP, out gp_Vec theD1U, out gp_Vec theD1V,
                       out gp_Vec theD2UU, out gp_Vec theD2VV, out gp_Vec theD2UV)
        {
            theP = new gp_Pnt(theU, theV, 0.0);
            theD1U = new gp_Vec(1.0, 0.0, 0.0);
            theD1V = new gp_Vec(0.0, 1.0, 0.0);
            theD2UU = new gp_Vec(0.0, 0.0, 0.0);
            theD2VV = new gp_Vec(0.0, 0.0, 0.0);
            theD2UV = new gp_Vec(0.0, 0.0, 0.0);
        }
    }

    [Fact]
    public void SolveSurfaceSurfaceExtrema4D_Smoke()
    {
        PlaneSurfaceEval aSurf1 = new();
        PlaneSurfaceEval aSurf2 = new();

        MathSys.NewtonBoundsN4 aBounds = new();
        aBounds.Min = std_array4<double>.Of(-10.0, -10.0, -10.0, -10.0);
        aBounds.Max = std_array4<double>.Of(10.0, 10.0, 10.0, 10.0);

        MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = THE_TOL;

        MathSys.NewtonResultN4 aResult =
          MathSys.SolveSurfaceSurfaceExtrema4D(aSurf1,
                                               aSurf2,
                                               std_array4<double>.Of(1.0, 2.0, 1.0, 2.0),
                                               aBounds,
                                               aOptions);

        Assert.True(aResult.IsDone());
        Assert.Equal(0.0, aResult.ResidualNorm, 1.0e-12);
    }
}
