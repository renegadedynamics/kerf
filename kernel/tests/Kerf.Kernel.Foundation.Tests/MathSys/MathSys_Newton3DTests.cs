// Port of: src/FoundationClasses/TKMath/GTests/MathSys_Newton3D_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation.Std;

using Kerf.Geometry;

namespace Kerf.Kernel.Foundation.Tests;

// The C++ tests pass lambdas; here each lambda is a private struct implementing MathSys.INewton3DFunction.
public class MathSys_Newton3DTest
{
    private const double THE_TOL = 1.0e-10;

    private struct LinearSystemFunc : global::Kerf.Engine.Numerics.Systems.MathSys.INewton3DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3,
                           ref Array3<double> theF, ref Array3<Array3<double>> theJ)
        {
            theF[0] = 2.0 * theX1 + theX2 + theX3 - 4.0;
            theF[1] = theX1 + 3.0 * theX2 + theX3 - 5.0;
            theF[2] = theX1 + theX2 + 4.0 * theX3 - 6.0;

            theJ[0][0] = 2.0;
            theJ[0][1] = 1.0;
            theJ[0][2] = 1.0;
            theJ[1][0] = 1.0;
            theJ[1][1] = 3.0;
            theJ[1][2] = 1.0;
            theJ[2][0] = 1.0;
            theJ[2][1] = 1.0;
            theJ[2][2] = 4.0;
            return true;
        }
    }

    private struct NonlinearSystemFunc : global::Kerf.Engine.Numerics.Systems.MathSys.INewton3DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3,
                           ref Array3<double> theF, ref Array3<Array3<double>> theJ)
        {
            theF[0] = theX1 * theX1 + theX2 * theX2 + theX3 * theX3 - 3.0;
            theF[1] = theX1 + theX2 + theX3 - 3.0;
            theF[2] = theX1 - theX2;

            theJ[0][0] = 2.0 * theX1;
            theJ[0][1] = 2.0 * theX2;
            theJ[0][2] = 2.0 * theX3;
            theJ[1][0] = 1.0;
            theJ[1][1] = 1.0;
            theJ[1][2] = 1.0;
            theJ[2][0] = 1.0;
            theJ[2][1] = -1.0;
            theJ[2][2] = 0.0;
            return true;
        }
    }

    private struct ShiftedIdentityFunc : global::Kerf.Engine.Numerics.Systems.MathSys.INewton3DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3,
                           ref Array3<double> theF, ref Array3<Array3<double>> theJ)
        {
            theF[0] = theX1 - 1.0;
            theF[1] = theX2 - 2.0;
            theF[2] = theX3 - 3.0;

            theJ[0][0] = 1.0;
            theJ[0][1] = 0.0;
            theJ[0][2] = 0.0;
            theJ[1][0] = 0.0;
            theJ[1][1] = 1.0;
            theJ[1][2] = 0.0;
            theJ[2][0] = 0.0;
            theJ[2][1] = 0.0;
            theJ[2][2] = 1.0;
            return true;
        }
    }

    private struct HugeJacobianConstantResidualFunc : global::Kerf.Engine.Numerics.Systems.MathSys.INewton3DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3,
                           ref Array3<double> theF, ref Array3<Array3<double>> theJ)
        {
            theF[0] = 1.0;
            theF[1] = 1.0;
            theF[2] = 1.0;
            theJ[0][0] = 1.0e20;
            theJ[0][1] = 0.0;
            theJ[0][2] = 0.0;
            theJ[1][0] = 0.0;
            theJ[1][1] = 1.0e20;
            theJ[1][2] = 0.0;
            theJ[2][0] = 0.0;
            theJ[2][1] = 0.0;
            theJ[2][2] = 1.0e20;
            return true;
        }
    }

    private struct ZeroResidualFunc : global::Kerf.Engine.Numerics.Systems.MathSys.INewton3DFunction
    {
        public bool Invoke(double theX1, double theX2, double theX3,
                           ref Array3<double> theF, ref Array3<Array3<double>> theJ)
        {
            theF[0] = 0.0;
            theF[1] = 0.0;
            theF[2] = 0.0;
            theJ[0][0] = 1.0;
            theJ[0][1] = 0.0;
            theJ[0][2] = 0.0;
            theJ[1][0] = 0.0;
            theJ[1][1] = 1.0;
            theJ[1][2] = 0.0;
            theJ[2][0] = 0.0;
            theJ[2][1] = 0.0;
            theJ[2][2] = 1.0;
            return true;
        }
    }

    private static global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 NoBounds()
    {
        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 aBounds = new();
        aBounds.HasBounds = false;
        return aBounds;
    }

    [Fact]
    public void Solve3D_LinearSystem()
    {
        LinearSystemFunc aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 aBounds = NoBounds();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = THE_TOL;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN3 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve3D(aFunc, Array3<double>.Of(0.0, 0.0, 0.0), aBounds, aOptions);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.X[0], 1.0e-8);
        Assert.Equal(1.0, aResult.X[1], 1.0e-8);
        Assert.Equal(1.0, aResult.X[2], 1.0e-8);
    }

    [Fact]
    public void Solve3D_NonlinearSystem()
    {
        NonlinearSystemFunc aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 aBounds = NoBounds();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-10;
        aOptions.MaxIterations = 50;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN3 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve3D(aFunc, Array3<double>.Of(0.8, 0.8, 1.4), aBounds, aOptions);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.X[0], 1.0e-5);
        Assert.Equal(1.0, aResult.X[1], 1.0e-5);
        Assert.Equal(1.0, aResult.X[2], 1.0e-5);
    }

    [Fact]
    public void Solve3D_SmallStepAtRoot_ReturnsOK()
    {
        ShiftedIdentityFunc aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 aBounds = NoBounds();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-12;
        aOptions.XTolerance = 100.0;
        aOptions.MaxIterations = 5;
        aOptions.MaxStepRatio = 100.0;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN3 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve3D(aFunc, Array3<double>.Of(0.0, 0.0, 0.0), aBounds, aOptions);
        Assert.True(aResult.IsDone);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK, aResult.Status);
        Assert.Equal(1.0, aResult.X[0], 1.0e-14);
        Assert.Equal(2.0, aResult.X[1], 1.0e-14);
        Assert.Equal(3.0, aResult.X[2], 1.0e-14);
    }

    [Fact]
    public void Solve3D_TinyStepLargeResidual_ReturnsMaxIterations()
    {
        HugeJacobianConstantResidualFunc aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 aBounds = NoBounds();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = 1.0e-8;
        aOptions.XTolerance = 1.0e-16;
        aOptions.MaxIterations = 10;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN3 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve3D(aFunc, Array3<double>.Of(0.0, 0.0, 0.0), aBounds, aOptions);
        Assert.False(aResult.IsDone);
        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.MaxIterations, aResult.Status);
        Assert.True(aResult.ResidualNorm > 1.0e-2);
    }

    [Fact]
    public void Solve3D_Bounded()
    {
        LinearSystemFunc aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 aBounds = new();
        aBounds.Min = Array3<double>.Of(-10.0, -10.0, -10.0);
        aBounds.Max = Array3<double>.Of(10.0, 10.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = THE_TOL;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN3 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve3D(aFunc, Array3<double>.Of(0.0, 0.0, 0.0), aBounds, aOptions);

        Assert.True(aResult.IsDone);
        Assert.Equal(1.0, aResult.X[0], 1.0e-8);
        Assert.Equal(1.0, aResult.X[1], 1.0e-8);
        Assert.Equal(1.0, aResult.X[2], 1.0e-8);
    }

    [Fact]
    public void Solve3D_InvalidInput()
    {
        ZeroResidualFunc aFunc = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 aBounds = new();
        aBounds.Min = Array3<double>.Of(1.0, 0.0, 0.0);
        aBounds.Max = Array3<double>.Of(0.0, 1.0, 1.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = THE_TOL;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN3 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.Solve3D(aFunc, Array3<double>.Of(0.0, 0.0, 0.0), aBounds, aOptions);

        Assert.Equal(global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InvalidInput, aResult.Status);
        Assert.False(aResult.IsDone);
    }

    private sealed class LineCurveEval : global::Kerf.Engine.Numerics.Systems.MathSys.ICurveD2Evaluator
    {
        public void D2(double theT, out Point3D theP, out Vector3D theD1, out Vector3D theD2)
        {
            theP = new Point3D(theT, 0.0, 0.0);
            theD1 = new Vector3D(1.0, 0.0, 0.0);
            theD2 = new Vector3D(0.0, 0.0, 0.0);
        }
    }

    private sealed class PlaneSurfaceEval : global::Kerf.Engine.Numerics.Systems.MathSys.ISurfaceD2Evaluator
    {
        public void D2(double theU, double theV, out Point3D theP, out Vector3D theD1U, out Vector3D theD1V,
                       out Vector3D theD2UU, out Vector3D theD2VV, out Vector3D theD2UV)
        {
            theP = new Point3D(theU, theV, 0.0);
            theD1U = new Vector3D(1.0, 0.0, 0.0);
            theD1V = new Vector3D(0.0, 1.0, 0.0);
            theD2UU = new Vector3D(0.0, 0.0, 0.0);
            theD2VV = new Vector3D(0.0, 0.0, 0.0);
            theD2UV = new Vector3D(0.0, 0.0, 0.0);
        }
    }

    [Fact]
    public void SolveCurveSurfaceExtrema3D_Smoke()
    {
        LineCurveEval aCurve = new();
        PlaneSurfaceEval aSurface = new();

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonBoundsN3 aBounds = new();
        aBounds.Min = Array3<double>.Of(-10.0, -10.0, -10.0);
        aBounds.Max = Array3<double>.Of(10.0, 10.0, 10.0);

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonOptions aOptions = new();
        aOptions.FTolerance = THE_TOL;
        aOptions.MaxIterations = 20;

        global::Kerf.Engine.Numerics.Systems.MathSys.NewtonResultN3 aResult =
          global::Kerf.Engine.Numerics.Systems.MathSys.SolveCurveSurfaceExtrema3D(aCurve,
                                             aSurface,
                                             Array3<double>.Of(1.0, 1.0, 0.0),
                                             aBounds,
                                             aOptions);

        Assert.True(aResult.IsDone);
        Assert.Equal(0.0, aResult.ResidualNorm, 1.0e-12);
    }
}
