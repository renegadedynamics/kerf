// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_Newton3D.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The function contract `bool operator()(double x1, double x2, double x3, double f[3], double j[3][3]) const` is
// INewton3DFunction (see MathSys_Newton2D.cs). The curve and surface evaluators of SolveCurveSurfaceExtrema3D,
// duck-typed in C++, are ICurveD2Evaluator and ISurfaceD2Evaluator; its lambda is a private struct functor.

/// <summary>Optimized 3D Newton-Raphson solver for systems of 3 equations in 3 unknowns.</summary>
public static partial class MathSys
{
    /// <summary>A 3x3 system with its Jacobian, for <see cref="Solve3D"/> (C++'s operator() contract).</summary>
    public interface INewton3DFunction
    {
        /// <summary>Computes F into theF and the Jacobian into theJ (theJ[i][j] = dF_i/dx_j); false on failure.</summary>
        bool Invoke(double theX1, double theX2, double theX3,
                    ref std_array3<double> theF, ref std_array3<std_array3<double>> theJ);
    }

    /// <summary>A curve evaluator with second derivatives: C++'s "type providing D2(t, P, D1, D2)".</summary>
    public interface ICurveD2Evaluator
    {
        void D2(double theT, out gp_Pnt theP, out gp_Vec theD1, out gp_Vec theD2);
    }

    /// <summary>A surface evaluator with second derivatives: C++'s "type providing D2(u, v, P, D1U, D1V, D2UU, D2VV, D2UV)".</summary>
    public interface ISurfaceD2Evaluator
    {
        void D2(double theU, double theV, out gp_Pnt theP, out gp_Vec theD1U, out gp_Vec theD1V,
                out gp_Vec theD2UU, out gp_Vec theD2VV, out gp_Vec theD2UV);
    }

    internal static partial class detail
    {
        /// <summary>Check that the bounds have valid (min &lt;= max) ranges.</summary>
        internal static bool IsBoundsValid3D(in NewtonBoundsN3 theBounds)
        {
            if (!theBounds.HasBounds)
            {
                return true;
            }

            return theBounds.Min[0] <= theBounds.Max[0] && theBounds.Min[1] <= theBounds.Max[1]
                   && theBounds.Min[2] <= theBounds.Max[2];
        }

        /// <summary>Check that NewtonOptions fields are positive and valid.</summary>
        internal static bool IsOptionsValid3D(NewtonOptions theOptions)
        {
            return theOptions.FTolerance > 0.0 && theOptions.XTolerance > 0.0 && theOptions.MaxIterations > 0
                   && theOptions.MaxStepRatio > 0.0 && theOptions.SoftBoundsExtension >= 0.0;
        }

        /// <summary>Return the largest domain extent across all 3 dimensions (min 1.0).</summary>
        internal static double MaxDomainSize3D(in NewtonBoundsN3 theBounds)
        {
            if (!theBounds.HasBounds)
            {
                return 1.0;
            }

            double aDX = theBounds.Max[0] - theBounds.Min[0];
            double aDY = theBounds.Max[1] - theBounds.Min[1];
            double aDZ = theBounds.Max[2] - theBounds.Min[2];
            return Max(1.0, Max(aDX, Max(aDY, aDZ)));
        }

        /// <summary>Clamp solution array to bounds, optionally extending by soft-bounds ratio.</summary>
        internal static void Clamp3D(ref std_array3<double> theX,
                                     in NewtonBoundsN3 theBounds,
                                     bool theUseSoftBounds,
                                     double theSoftExtRatio)
        {
            if (!theBounds.HasBounds)
            {
                return;
            }

            double aExtX =
              theUseSoftBounds ? (theBounds.Max[0] - theBounds.Min[0]) * theSoftExtRatio : 0.0;
            double aExtY =
              theUseSoftBounds ? (theBounds.Max[1] - theBounds.Min[1]) * theSoftExtRatio : 0.0;
            double aExtZ =
              theUseSoftBounds ? (theBounds.Max[2] - theBounds.Min[2]) * theSoftExtRatio : 0.0;

            theX[0] = std_clamp(theX[0], theBounds.Min[0] - aExtX, theBounds.Max[0] + aExtX);
            theX[1] = std_clamp(theX[1], theBounds.Min[1] - aExtY, theBounds.Max[1] + aExtY);
            theX[2] = std_clamp(theX[2], theBounds.Min[2] - aExtZ, theBounds.Max[2] + aExtZ);
        }

        /// <summary>
        /// Solve the 3x3 linear system J*x = -F by Cramer's rule with cofactor expansion; theDelta is written only
        /// when the system is non-singular (true).
        /// </summary>
        internal static bool Solve3x3(in std_array3<std_array3<double>> theJ, in std_array3<double> theF, ref std_array3<double> theDelta)
        {
            // Compute cofactors for first row (used for determinant and inverse)
            double aCof00 = theJ[1][1] * theJ[2][2] - theJ[1][2] * theJ[2][1];
            double aCof01 = theJ[1][2] * theJ[2][0] - theJ[1][0] * theJ[2][2];
            double aCof02 = theJ[1][0] * theJ[2][1] - theJ[1][1] * theJ[2][0];

            // Determinant by first row expansion
            double aDet = theJ[0][0] * aCof00 + theJ[0][1] * aCof01 + theJ[0][2] * aCof02;
            // Check for singularity
            if (Math.Abs(aDet) < 1.0e-30)
            {
                return false;
            }

            double aInvDet = 1.0 / aDet;

            // Remaining cofactors
            double aCof10 = theJ[0][2] * theJ[2][1] - theJ[0][1] * theJ[2][2];
            double aCof11 = theJ[0][0] * theJ[2][2] - theJ[0][2] * theJ[2][0];
            double aCof12 = theJ[0][1] * theJ[2][0] - theJ[0][0] * theJ[2][1];

            double aCof20 = theJ[0][1] * theJ[1][2] - theJ[0][2] * theJ[1][1];
            double aCof21 = theJ[0][2] * theJ[1][0] - theJ[0][0] * theJ[1][2];
            double aCof22 = theJ[0][0] * theJ[1][1] - theJ[0][1] * theJ[1][0];

            // Solve: delta = -J^(-1) * F = -adjugate(J)^T * F / det
            theDelta[0] = -(aCof00 * theF[0] + aCof10 * theF[1] + aCof20 * theF[2]) * aInvDet;
            theDelta[1] = -(aCof01 * theF[0] + aCof11 * theF[1] + aCof21 * theF[2]) * aInvDet;
            theDelta[2] = -(aCof02 * theF[0] + aCof12 * theF[1] + aCof22 * theF[2]) * aInvDet;
            return true;
        }
    }

    /// <summary>
    /// Solve a 3x3 nonlinear system [F1, F2, F3] = 0 by Newton iteration with bounds, using Cramer's rule for the
    /// linear system at each step and steepest descent when the Jacobian is singular.
    /// </summary>
    public static NewtonResultN3 Solve3D<Function>(Function theFunc,
                                                   in std_array3<double> theX0,
                                                   in NewtonBoundsN3 theBounds,
                                                   NewtonOptions? theOptions = null)
        where Function : INewton3DFunction
    {
        theOptions ??= new NewtonOptions();
        NewtonResultN3 aRes = new();
        aRes.X = theX0;

        if (!detail.IsOptionsValid3D(theOptions) || !detail.IsBoundsValid3D(theBounds))
        {
            aRes.Status = MathUtils.Status.InvalidInput;
            return aRes;
        }

        detail.Clamp3D(ref aRes.X, theBounds, theOptions.AllowSoftBounds, theOptions.SoftBoundsExtension);

        double aTolSq = theOptions.FTolerance * theOptions.FTolerance;
        double aMaxStep = theOptions.MaxStepRatio * detail.MaxDomainSize3D(theBounds);

        for (int anIter = 0; anIter < theOptions.MaxIterations; ++anIter)
        {
            aRes.NbIterations = anIter + 1;

            std_array3<double> aF = default;
            std_array3<std_array3<double>> aJ = default;
            if (!theFunc.Invoke(aRes.X[0], aRes.X[1], aRes.X[2], ref aF, ref aJ))
            {
                aRes.Status = MathUtils.Status.NumericalError;
                return aRes;
            }

            // Check convergence using squared norm (avoid sqrt)
            double aFNormSq = aF[0] * aF[0] + aF[1] * aF[1] + aF[2] * aF[2];
            aRes.ResidualNorm = Math.Sqrt(aFNormSq);
            if (aFNormSq <= aTolSq)
            {
                aRes.Status = MathUtils.Status.OK;
                return aRes;
            }

            // Solve 3x3 linear system: J * delta = -F
            std_array3<double> aDelta = default;
            if (!detail.Solve3x3(aJ, aF, ref aDelta))
            {
                // Singular Jacobian - try steepest descent direction: -J^T * F
                double aGradX = aJ[0][0] * aF[0] + aJ[1][0] * aF[1] + aJ[2][0] * aF[2];
                double aGradY = aJ[0][1] * aF[0] + aJ[1][1] * aF[1] + aJ[2][1] * aF[2];
                double aGradZ = aJ[0][2] * aF[0] + aJ[1][2] * aF[1] + aJ[2][2] * aF[2];
                double aGradSq = aGradX * aGradX + aGradY * aGradY + aGradZ * aGradZ;
                if (aGradSq < 1.0e-60)
                {
                    aRes.Status = MathUtils.Status.Singular;
                    return aRes;
                }

                double aAlpha = Min(1.0, Math.Sqrt(aFNormSq / aGradSq) * 0.1);
                aDelta[0] = -aAlpha * aGradX;
                aDelta[1] = -aAlpha * aGradY;
                aDelta[2] = -aAlpha * aGradZ;
            }

            // Limit step size to prevent wild oscillations
            double aStepNormSq =
              aDelta[0] * aDelta[0] + aDelta[1] * aDelta[1] + aDelta[2] * aDelta[2];
            double aStepNorm = Math.Sqrt(aStepNormSq);
            if (aStepNorm > aMaxStep)
            {
                double aScale = aMaxStep / aStepNorm;
                aDelta[0] *= aScale;
                aDelta[1] *= aScale;
                aDelta[2] *= aScale;
            }

            // Update and clamp to bounds
            std_array3<double> aNewX = std_array3<double>.Of(aRes.X[0] + aDelta[0],
                                                             aRes.X[1] + aDelta[1],
                                                             aRes.X[2] + aDelta[2]);
            detail.Clamp3D(ref aNewX, theBounds, theOptions.AllowSoftBounds, theOptions.SoftBoundsExtension);

            aRes.StepNorm = Math.Sqrt((aNewX[0] - aRes.X[0]) * (aNewX[0] - aRes.X[0])
                                      + (aNewX[1] - aRes.X[1]) * (aNewX[1] - aRes.X[1])
                                      + (aNewX[2] - aRes.X[2]) * (aNewX[2] - aRes.X[2]));
            aRes.X = aNewX;

            double aScaleRef =
              Max(1.0, Max(Math.Abs(aRes.X[0]), Max(Math.Abs(aRes.X[1]), Math.Abs(aRes.X[2]))));
            if (aRes.StepNorm <= theOptions.XTolerance * aScaleRef)
            {
                std_array3<double> aCheckF = default;
                std_array3<std_array3<double>> aCheckJ = default;
                if (!theFunc.Invoke(aRes.X[0], aRes.X[1], aRes.X[2], ref aCheckF, ref aCheckJ))
                {
                    aRes.Status = MathUtils.Status.NumericalError;
                    return aRes;
                }

                aRes.ResidualNorm =
                  Math.Sqrt(aCheckF[0] * aCheckF[0] + aCheckF[1] * aCheckF[1] + aCheckF[2] * aCheckF[2]);
                aRes.Status = (aRes.ResidualNorm <= theOptions.FTolerance) ? MathUtils.Status.OK
                                                                           : MathUtils.Status.MaxIterations;
                return aRes;
            }
        }

        // Final convergence check after max iterations
        // (braced: C# doesn't allow the loop's local names again in the enclosing block)
        {
            std_array3<double> aF = default;
            std_array3<std_array3<double>> aJ = default;
            if (!theFunc.Invoke(aRes.X[0], aRes.X[1], aRes.X[2], ref aF, ref aJ))
            {
                aRes.Status = MathUtils.Status.NumericalError;
                return aRes;
            }

            aRes.ResidualNorm = Math.Sqrt(aF[0] * aF[0] + aF[1] * aF[1] + aF[2] * aF[2]);
            aRes.Status = (aRes.ResidualNorm <= theOptions.FTolerance) ? MathUtils.Status.OK
                                                                       : MathUtils.Status.MaxIterations;
            return aRes;
        }
    }

    /// <summary>The lambda of SolveCurveSurfaceExtrema3D: the gradient of ||C(t) - S(u, v)||^2 / 2 and its Hessian.</summary>
    private struct CurveSurfaceExtrema3DFunction<CurveEvaluator, SurfaceEvaluator> : INewton3DFunction
        where CurveEvaluator : ICurveD2Evaluator
        where SurfaceEvaluator : ISurfaceD2Evaluator
    {
        internal CurveEvaluator theCurve;
        internal SurfaceEvaluator theSurface;

        public bool Invoke(double theT, double theU, double theV,
                           ref std_array3<double> theF, ref std_array3<std_array3<double>> theJ)
        {
            theCurve.D2(theT, out gp_Pnt aPtC, out gp_Vec aD1C, out gp_Vec aD2C);
            theSurface.D2(theU, theV, out gp_Pnt aPtS, out gp_Vec aD1U, out gp_Vec aD1V, out gp_Vec aD2UU, out gp_Vec aD2VV, out gp_Vec aD2UV);

            // D = C - S (vector from surface point to curve point)
            gp_Vec aD = new(aPtS, aPtC);

            // Function values: gradient of ||C - S||^2 (factor 2 dropped)
            theF[0] = aD.Dot(aD1C);
            theF[1] = -aD.Dot(aD1U);
            theF[2] = -aD.Dot(aD1V);

            // Jacobian: Hessian of ||C - S||^2 (without factor 2)
            // J[0][0] = dF1/dt = dC/dt . dC/dt + D . d2C/dt2
            theJ[0][0] = aD1C.Dot(aD1C) + aD.Dot(aD2C);
            // J[0][1] = dF1/du = -dS/du . dC/dt
            theJ[0][1] = -aD1U.Dot(aD1C);
            // J[0][2] = dF1/dv = -dS/dv . dC/dt
            theJ[0][2] = -aD1V.Dot(aD1C);

            // J[1][0] = dF2/dt (symmetric to J[0][1])
            theJ[1][0] = theJ[0][1];
            // J[1][1] = dF2/du = dS/du . dS/du - D . d2S/du2
            theJ[1][1] = aD1U.Dot(aD1U) - aD.Dot(aD2UU);
            // J[1][2] = dF2/dv = dS/dv . dS/du - D . d2S/dudv
            theJ[1][2] = aD1V.Dot(aD1U) - aD.Dot(aD2UV);

            // J[2][0] = dF3/dt (symmetric to J[0][2])
            theJ[2][0] = theJ[0][2];
            // J[2][1] = dF3/du (symmetric to J[1][2])
            theJ[2][1] = theJ[1][2];
            // J[2][2] = dF3/dv = dS/dv . dS/dv - D . d2S/dv2
            theJ[2][2] = aD1V.Dot(aD1V) - aD.Dot(aD2VV);
            return true;
        }
    }

    /// <summary>
    /// 3D Newton solver for extrema between a curve C(t) and a surface S(u, v): F = ((C-S).dC/dt, (S-C).dS/du,
    /// (S-C).dS/dv). The result holds t, u, v in X[0], X[1], X[2].
    /// </summary>
    public static NewtonResultN3 SolveCurveSurfaceExtrema3D<CurveEvaluator, SurfaceEvaluator>(CurveEvaluator theCurve,
                                                                                              SurfaceEvaluator theSurface,
                                                                                              in std_array3<double> theX0,
                                                                                              in NewtonBoundsN3 theBounds,
                                                                                              NewtonOptions? theOptions = null)
        where CurveEvaluator : ICurveD2Evaluator
        where SurfaceEvaluator : ISurfaceD2Evaluator
    {
        CurveSurfaceExtrema3DFunction<CurveEvaluator, SurfaceEvaluator> aFunc = new() { theCurve = theCurve, theSurface = theSurface };

        return Solve3D(aFunc, theX0, theBounds, theOptions);
    }
}
