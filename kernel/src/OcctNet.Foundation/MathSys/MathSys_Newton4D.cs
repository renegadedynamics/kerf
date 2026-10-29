// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_Newton4D.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

// The function contract `bool operator()(double x1, ..., double x4, double f[4], double j[4][4]) const` is
// INewton4DFunction (see MathSys_Newton2D.cs); SolveSurfaceSurfaceExtrema4D's lambda is a private struct functor
// over two ISurfaceD2Evaluator (MathSys_Newton3D.cs).

/// <summary>Optimized 4D Newton-Raphson solver for systems of 4 equations in 4 unknowns.</summary>
public static partial class MathSys
{
    /// <summary>A 4x4 system with its Jacobian, for <see cref="Solve4D"/> (C++'s operator() contract).</summary>
    public interface INewton4DFunction
    {
        /// <summary>Computes F into theF and the Jacobian into theJ (theJ[i][j] = dF_i/dx_j); false on failure.</summary>
        bool Invoke(double theX1, double theX2, double theX3, double theX4,
                    ref std_array4<double> theF, ref std_array4<std_array4<double>> theJ);
    }

    internal static partial class detail
    {
        /// <summary>A row of Solve4x4's augmented matrix: C++'s <c>double[5]</c>, held inline.</summary>
        [InlineArray(5)]
        internal struct Array5
        {
            private double myElement0;
        }

        /// <summary>Check that the bounds have valid (min &lt;= max) ranges.</summary>
        internal static bool IsBoundsValid4D(in NewtonBoundsN4 theBounds)
        {
            if (!theBounds.HasBounds)
            {
                return true;
            }

            return theBounds.Min[0] <= theBounds.Max[0] && theBounds.Min[1] <= theBounds.Max[1]
                   && theBounds.Min[2] <= theBounds.Max[2] && theBounds.Min[3] <= theBounds.Max[3];
        }

        /// <summary>Check that NewtonOptions fields are positive and valid.</summary>
        internal static bool IsOptionsValid4D(NewtonOptions theOptions)
        {
            return theOptions.FTolerance > 0.0 && theOptions.XTolerance > 0.0 && theOptions.MaxIterations > 0
                   && theOptions.MaxStepRatio > 0.0 && theOptions.SoftBoundsExtension >= 0.0;
        }

        /// <summary>Return the largest domain extent across all 4 dimensions (min 1.0).</summary>
        internal static double MaxDomainSize4D(in NewtonBoundsN4 theBounds)
        {
            if (!theBounds.HasBounds)
            {
                return 1.0;
            }

            double aD0 = theBounds.Max[0] - theBounds.Min[0];
            double aD1 = theBounds.Max[1] - theBounds.Min[1];
            double aD2 = theBounds.Max[2] - theBounds.Min[2];
            double aD3 = theBounds.Max[3] - theBounds.Min[3];
            return Max(1.0, Max(aD0, Max(aD1, Max(aD2, aD3))));
        }

        /// <summary>Clamp solution array to bounds, optionally extending by soft-bounds ratio.</summary>
        internal static void Clamp4D(ref std_array4<double> theX,
                                     in NewtonBoundsN4 theBounds,
                                     bool theUseSoftBounds,
                                     double theSoftExtRatio)
        {
            if (!theBounds.HasBounds)
            {
                return;
            }

            for (int i = 0; i < 4; ++i)
            {
                double aExt =
                  theUseSoftBounds ? (theBounds.Max[i] - theBounds.Min[i]) * theSoftExtRatio : 0.0;
                theX[i] = std_clamp(theX[i], theBounds.Min[i] - aExt, theBounds.Max[i] + aExt);
            }
        }

        /// <summary>
        /// Solve the 4x4 linear system J*x = -F by Gaussian elimination with partial pivoting; theDelta is written
        /// only when the system is non-singular (true).
        /// </summary>
        internal static bool Solve4x4(in std_array4<std_array4<double>> theJ, in std_array4<double> theF, ref std_array4<double> theDelta)
        {
            // Augmented matrix [J | -F]
            std_array4<Array5> A = default;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    A[i][j] = theJ[i][j];
                }
                A[i][4] = -theF[i];
            }

            // Forward elimination with partial pivoting
            for (int k = 0; k < 4; ++k)
            {
                // Find pivot
                int aMaxRow = k;
                double aMaxVal = Math.Abs(A[k][k]);
                for (int i = k + 1; i < 4; ++i)
                {
                    double aVal = Math.Abs(A[i][k]);
                    if (aVal > aMaxVal)
                    {
                        aMaxVal = aVal;
                        aMaxRow = i;
                    }
                }

                // Check for singularity
                if (aMaxVal < 1.0e-30)
                {
                    return false;
                }

                // Swap rows if needed
                if (aMaxRow != k)
                {
                    for (int j = k; j <= 4; ++j)
                    {
                        (A[k][j], A[aMaxRow][j]) = (A[aMaxRow][j], A[k][j]);
                    }
                }

                // Eliminate column
                double aInvPivot = 1.0 / A[k][k];
                for (int i = k + 1; i < 4; ++i)
                {
                    double aFactor = A[i][k] * aInvPivot;
                    for (int j = k + 1; j <= 4; ++j)
                    {
                        A[i][j] -= aFactor * A[k][j];
                    }
                    A[i][k] = 0.0;
                }
            }

            // Back substitution
            for (int i = 3; i >= 0; --i)
            {
                double aSum = A[i][4];
                for (int j = i + 1; j < 4; ++j)
                {
                    aSum -= A[i][j] * theDelta[j];
                }
                theDelta[i] = aSum / A[i][i];
            }

            return true;
        }
    }

    /// <summary>
    /// Solve a 4x4 nonlinear system [F1, F2, F3, F4] = 0 by Newton iteration with bounds, using Gaussian
    /// elimination for the linear system at each step and steepest descent when the Jacobian is singular.
    /// </summary>
    public static NewtonResultN4 Solve4D<Function>(Function theFunc,
                                                   in std_array4<double> theX0,
                                                   in NewtonBoundsN4 theBounds,
                                                   NewtonOptions? theOptions = null)
        where Function : INewton4DFunction
    {
        theOptions ??= new NewtonOptions();
        NewtonResultN4 aRes = new();
        aRes.X = theX0;

        if (!detail.IsOptionsValid4D(theOptions) || !detail.IsBoundsValid4D(theBounds))
        {
            aRes.Status = MathUtils.Status.InvalidInput;
            return aRes;
        }

        detail.Clamp4D(ref aRes.X, theBounds, theOptions.AllowSoftBounds, theOptions.SoftBoundsExtension);

        double aTolSq = theOptions.FTolerance * theOptions.FTolerance;
        double aMaxStep = theOptions.MaxStepRatio * detail.MaxDomainSize4D(theBounds);

        for (int anIter = 0; anIter < theOptions.MaxIterations; ++anIter)
        {
            aRes.NbIterations = anIter + 1;

            std_array4<double> aF = default;
            std_array4<std_array4<double>> aJ = default;
            if (!theFunc.Invoke(aRes.X[0], aRes.X[1], aRes.X[2], aRes.X[3], ref aF, ref aJ))
            {
                aRes.Status = MathUtils.Status.NumericalError;
                return aRes;
            }

            // Check convergence using squared norm (avoid sqrt)
            double aFNormSq = aF[0] * aF[0] + aF[1] * aF[1] + aF[2] * aF[2] + aF[3] * aF[3];
            aRes.ResidualNorm = Math.Sqrt(aFNormSq);
            if (aFNormSq <= aTolSq)
            {
                aRes.Status = MathUtils.Status.OK;
                return aRes;
            }

            // Solve 4x4 linear system: J * delta = -F
            std_array4<double> aDelta = default;
            if (!detail.Solve4x4(aJ, aF, ref aDelta))
            {
                // Singular Jacobian - try steepest descent direction: -J^T * F
                std_array4<double> aGrad = std_array4<double>.Of(0.0, 0.0, 0.0, 0.0);
                for (int c = 0; c < 4; ++c)
                {
                    for (int r = 0; r < 4; ++r)
                    {
                        aGrad[c] += aJ[r][c] * aF[r];
                    }
                }

                double aGradSq =
                  aGrad[0] * aGrad[0] + aGrad[1] * aGrad[1] + aGrad[2] * aGrad[2] + aGrad[3] * aGrad[3];
                if (aGradSq < 1.0e-60)
                {
                    aRes.Status = MathUtils.Status.Singular;
                    return aRes;
                }

                double aAlpha = Min(1.0, Math.Sqrt(aFNormSq / aGradSq) * 0.1);
                for (int i = 0; i < 4; ++i)
                {
                    aDelta[i] = -aAlpha * aGrad[i];
                }
            }

            // Limit step size to prevent wild oscillations
            double aStepNormSq =
              aDelta[0] * aDelta[0] + aDelta[1] * aDelta[1] + aDelta[2] * aDelta[2] + aDelta[3] * aDelta[3];
            double aStepNorm = Math.Sqrt(aStepNormSq);
            if (aStepNorm > aMaxStep)
            {
                double aScale = aMaxStep / aStepNorm;
                for (int i = 0; i < 4; ++i)
                {
                    aDelta[i] *= aScale;
                }
            }

            // Update and clamp to bounds
            std_array4<double> aNewX = std_array4<double>.Of(aRes.X[0] + aDelta[0],
                                                             aRes.X[1] + aDelta[1],
                                                             aRes.X[2] + aDelta[2],
                                                             aRes.X[3] + aDelta[3]);
            detail.Clamp4D(ref aNewX, theBounds, theOptions.AllowSoftBounds, theOptions.SoftBoundsExtension);

            aRes.StepNorm = Math.Sqrt((aNewX[0] - aRes.X[0]) * (aNewX[0] - aRes.X[0])
                                      + (aNewX[1] - aRes.X[1]) * (aNewX[1] - aRes.X[1])
                                      + (aNewX[2] - aRes.X[2]) * (aNewX[2] - aRes.X[2])
                                      + (aNewX[3] - aRes.X[3]) * (aNewX[3] - aRes.X[3]));
            aRes.X = aNewX;

            double aScaleRef = Max(
              1.0,
              Max(Math.Abs(aRes.X[0]),
                  Max(Math.Abs(aRes.X[1]), Max(Math.Abs(aRes.X[2]), Math.Abs(aRes.X[3])))));
            if (aRes.StepNorm <= theOptions.XTolerance * aScaleRef)
            {
                std_array4<double> aCheckF = default;
                std_array4<std_array4<double>> aCheckJ = default;
                if (!theFunc.Invoke(aRes.X[0], aRes.X[1], aRes.X[2], aRes.X[3], ref aCheckF, ref aCheckJ))
                {
                    aRes.Status = MathUtils.Status.NumericalError;
                    return aRes;
                }

                aRes.ResidualNorm = Math.Sqrt(aCheckF[0] * aCheckF[0] + aCheckF[1] * aCheckF[1]
                                              + aCheckF[2] * aCheckF[2] + aCheckF[3] * aCheckF[3]);
                aRes.Status = (aRes.ResidualNorm <= theOptions.FTolerance) ? MathUtils.Status.OK
                                                                           : MathUtils.Status.MaxIterations;
                return aRes;
            }
        }

        // Final convergence check after max iterations
        // (braced: C# doesn't allow the loop's local names again in the enclosing block)
        {
            std_array4<double> aF = default;
            std_array4<std_array4<double>> aJ = default;
            if (!theFunc.Invoke(aRes.X[0], aRes.X[1], aRes.X[2], aRes.X[3], ref aF, ref aJ))
            {
                aRes.Status = MathUtils.Status.NumericalError;
                return aRes;
            }

            aRes.ResidualNorm = Math.Sqrt(aF[0] * aF[0] + aF[1] * aF[1] + aF[2] * aF[2] + aF[3] * aF[3]);
            aRes.Status = (aRes.ResidualNorm <= theOptions.FTolerance) ? MathUtils.Status.OK
                                                                       : MathUtils.Status.MaxIterations;
            return aRes;
        }
    }

    /// <summary>The lambda of SolveSurfaceSurfaceExtrema4D: the gradient of ||S1 - S2||^2 / 2 and its Hessian.</summary>
    private struct SurfaceSurfaceExtrema4DFunction<SurfaceEvaluator1, SurfaceEvaluator2> : INewton4DFunction
        where SurfaceEvaluator1 : ISurfaceD2Evaluator
        where SurfaceEvaluator2 : ISurfaceD2Evaluator
    {
        internal SurfaceEvaluator1 theSurf1;
        internal SurfaceEvaluator2 theSurf2;

        public bool Invoke(double theU1, double theV1, double theU2, double theV2,
                           ref std_array4<double> theF, ref std_array4<std_array4<double>> theJ)
        {
            theSurf1.D2(theU1, theV1, out gp_Pnt aPt1, out gp_Vec aD1U1, out gp_Vec aD1V1, out gp_Vec aD2UU1, out gp_Vec aD2VV1, out gp_Vec aD2UV1);
            theSurf2.D2(theU2, theV2, out gp_Pnt aPt2, out gp_Vec aD1U2, out gp_Vec aD1V2, out gp_Vec aD2UU2, out gp_Vec aD2VV2, out gp_Vec aD2UV2);

            // D = P1 - P2
            gp_Vec aD = new(aPt2, aPt1);

            // Function values: gradient of ||S1 - S2||^2
            theF[0] = aD.Dot(aD1U1);
            theF[1] = aD.Dot(aD1V1);
            theF[2] = -aD.Dot(aD1U2);
            theF[3] = -aD.Dot(aD1V2);

            // Jacobian: Hessian of ||S1 - S2||^2
            // Block [0:2, 0:2]: derivatives w.r.t. (u1, v1)
            theJ[0][0] = aD1U1.Dot(aD1U1) + aD.Dot(aD2UU1);
            theJ[0][1] = aD1V1.Dot(aD1U1) + aD.Dot(aD2UV1);
            theJ[1][0] = theJ[0][1]; // Symmetric
            theJ[1][1] = aD1V1.Dot(aD1V1) + aD.Dot(aD2VV1);

            // Block [0:2, 2:4]: cross derivatives
            theJ[0][2] = -aD1U2.Dot(aD1U1);
            theJ[0][3] = -aD1V2.Dot(aD1U1);
            theJ[1][2] = -aD1U2.Dot(aD1V1);
            theJ[1][3] = -aD1V2.Dot(aD1V1);

            // Block [2:4, 0:2]: cross derivatives (symmetric to above)
            theJ[2][0] = theJ[0][2];
            theJ[2][1] = theJ[1][2];
            theJ[3][0] = theJ[0][3];
            theJ[3][1] = theJ[1][3];

            // Block [2:4, 2:4]: derivatives w.r.t. (u2, v2)
            theJ[2][2] = aD1U2.Dot(aD1U2) - aD.Dot(aD2UU2);
            theJ[2][3] = aD1V2.Dot(aD1U2) - aD.Dot(aD2UV2);
            theJ[3][2] = theJ[2][3]; // Symmetric
            theJ[3][3] = aD1V2.Dot(aD1V2) - aD.Dot(aD2VV2);
            return true;
        }
    }

    /// <summary>
    /// 4D Newton solver for extrema between two surfaces S1(u1, v1) and S2(u2, v2). The result holds u1, v1, u2,
    /// v2 in X[0..3].
    /// </summary>
    public static NewtonResultN4 SolveSurfaceSurfaceExtrema4D<SurfaceEvaluator1, SurfaceEvaluator2>(SurfaceEvaluator1 theSurf1,
                                                                                                    SurfaceEvaluator2 theSurf2,
                                                                                                    in std_array4<double> theX0,
                                                                                                    in NewtonBoundsN4 theBounds,
                                                                                                    NewtonOptions? theOptions = null)
        where SurfaceEvaluator1 : ISurfaceD2Evaluator
        where SurfaceEvaluator2 : ISurfaceD2Evaluator
    {
        SurfaceSurfaceExtrema4DFunction<SurfaceEvaluator1, SurfaceEvaluator2> aFunc = new() { theSurf1 = theSurf1, theSurf2 = theSurf2 };

        return Solve4D(aFunc, theX0, theBounds, theOptions);
    }
}
