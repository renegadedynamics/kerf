// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_Newton2D.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The C++ function contract `bool operator()(double u, double v, double f[2], double j[2][2]) const` is the
// interface INewton2DFunction, whose Invoke takes the arrays by `ref` as std_array2 values, so a functor writes
// `theF[0] = ...; theJ[0][1] = ...;` as in C++. Solve2D is generic over it: a struct functor gets JIT-specialised
// code with the call inlined, as the C++ template does.

/// <summary>Optimized 2D Newton-Raphson solvers with strict convergence criteria.</summary>
public static partial class MathSys
{
    /// <summary>A general 2x2 system with its Jacobian, for <see cref="Solve2D"/> (C++'s operator() contract).</summary>
    public interface INewton2DFunction
    {
        /// <summary>Computes F(u, v) into theF and the Jacobian into theJ (theJ[i][j] = dF_i/dx_j); false on failure.</summary>
        bool Invoke(double theU, double theV, ref std_array2<double> theF, ref std_array2<std_array2<double>> theJ);
    }

    /// <summary>A 2x2 system with a symmetric Jacobian, for <see cref="Solve2DSymmetric"/>.</summary>
    public interface INewton2DSymmetricFunction
    {
        /// <summary>Computes F and the Jacobian's elements J11, J12 (= J21) and J22; false on failure.</summary>
        bool ValueAndJacobian(double theU, double theV, out double theF1, out double theF2,
                              out double theJ11, out double theJ12, out double theJ22);

        /// <summary>Computes F only; false on failure.</summary>
        bool Value(double theU, double theV, out double theF1, out double theF2);
    }

    internal static partial class detail
    {
        internal const double THE_SINGULAR_DET_TOL = 1.0e-25;
        internal const double THE_CRITICAL_GRAD_SQ = 1.0e-60;
        internal const int THE_LINE_SEARCH_MAX = 8;
        internal const double THE_ARMIJO_C1 = 1.0e-4;

        /// <summary>C++'s std::clamp: theLo if theV &lt; theLo, theHi if theHi &lt; theV, else theV.</summary>
        internal static double std_clamp(double theV, double theLo, double theHi)
        {
            return theV < theLo ? theLo : theHi < theV ? theHi : theV;
        }

        /// <summary>Check that NewtonOptions fields are positive and valid.</summary>
        internal static bool IsOptionsValid(NewtonOptions theOptions)
        {
            return theOptions.FTolerance > 0.0 && theOptions.XTolerance > 0.0 && theOptions.MaxIterations > 0
                   && theOptions.MaxStepRatio > 0.0 && theOptions.SoftBoundsExtension >= 0.0;
        }

        /// <summary>Check that the bounds have valid (min &lt;= max) ranges.</summary>
        internal static bool IsBoundsValid2D(in NewtonBoundsN2 theBounds)
        {
            if (!theBounds.HasBounds)
            {
                return true;
            }
            return theBounds.Min[0] <= theBounds.Max[0] && theBounds.Min[1] <= theBounds.Max[1];
        }

        /// <summary>Return the largest domain extent across both dimensions (min 1.0).</summary>
        internal static double MaxDomainSize2D(in NewtonBoundsN2 theBounds)
        {
            if (!theBounds.HasBounds)
            {
                return 1.0;
            }

            double aDU = theBounds.Max[0] - theBounds.Min[0];
            double aDV = theBounds.Max[1] - theBounds.Min[1];
            return Max(1.0, Max(aDU, aDV));
        }

        /// <summary>Clamp solution array to bounds, optionally extending by soft-bounds ratio.</summary>
        internal static void Clamp2D(ref std_array2<double> theX,
                                     in NewtonBoundsN2 theBounds,
                                     bool theUseSoftBounds,
                                     double theSoftExtRatio)
        {
            if (!theBounds.HasBounds)
            {
                return;
            }

            double aExtU =
              theUseSoftBounds ? (theBounds.Max[0] - theBounds.Min[0]) * theSoftExtRatio : 0.0;
            double aExtV =
              theUseSoftBounds ? (theBounds.Max[1] - theBounds.Min[1]) * theSoftExtRatio : 0.0;

            double aUMin = theBounds.Min[0] - aExtU;
            double aUMax = theBounds.Max[0] + aExtU;
            double aVMin = theBounds.Min[1] - aExtV;
            double aVMax = theBounds.Max[1] + aExtV;

            theX[0] = std_clamp(theX[0], aUMin, aUMax);
            theX[1] = std_clamp(theX[1], aVMin, aVMax);
        }

        /// <summary>
        /// Solve a 2x2 symmetric system by eigenvalue decomposition (SVD fallback); more robust than Cramer's rule
        /// for ill-conditioned matrices. theDU and theDV are written only on success.
        /// </summary>
        internal static bool SolveSymmetric2x2SVD(double theJ11,
                                                  double theJ12,
                                                  double theJ22,
                                                  double theF1,
                                                  double theF2,
                                                  ref double theDU,
                                                  ref double theDV,
                                                  double theTol = 1.0e-15)
        {
            double aTrace = theJ11 + theJ22;
            double aDiff = (theJ11 - theJ22) * 0.5;
            double aDiscrim = Math.Sqrt(aDiff * aDiff + theJ12 * theJ12);

            double aLambda1 = aTrace * 0.5 + aDiscrim;
            double aLambda2 = aTrace * 0.5 - aDiscrim;

            double aMinLambda = Max(Math.Abs(aLambda1), Math.Abs(aLambda2)) * theTol;
            if (Math.Abs(aLambda1) < aMinLambda && Math.Abs(aLambda2) < aMinLambda)
            {
                return false;
            }

            double aV1x = 0.0;
            double aV1y = 0.0;
            if (Math.Abs(theJ12) > Math.Abs(aDiff))
            {
                double aLen1 = Math.Sqrt(theJ12 * theJ12 + (aLambda1 - theJ11) * (aLambda1 - theJ11));
                if (aLen1 < theTol)
                {
                    return false;
                }

                aV1x = theJ12 / aLen1;
                aV1y = (aLambda1 - theJ11) / aLen1;
            }
            else
            {
                double aLen1 = Math.Sqrt((aLambda1 - theJ22) * (aLambda1 - theJ22) + theJ12 * theJ12);
                if (aLen1 < theTol)
                {
                    return false;
                }

                aV1x = (aLambda1 - theJ22) / aLen1;
                aV1y = theJ12 / aLen1;
            }

            double aV2x = -aV1y;
            double aV2y = aV1x;

            double aB1 = aV1x * (-theF1) + aV1y * (-theF2);
            double aB2 = aV2x * (-theF1) + aV2y * (-theF2);

            double aX1 = (Math.Abs(aLambda1) > aMinLambda) ? aB1 / aLambda1 : 0.0;
            double aX2 = (Math.Abs(aLambda2) > aMinLambda) ? aB2 / aLambda2 : 0.0;

            theDU = aV1x * aX1 + aV2x * aX2;
            theDV = aV1y * aX1 + aV2y * aX2;
            return true;
        }
    }

    /// <summary>Solve a general 2x2 nonlinear system by Newton iteration.</summary>
    public static NewtonResultN2 Solve2D<Function>(Function theFunc,
                                                   in std_array2<double> theX0,
                                                   in NewtonBoundsN2 theBounds,
                                                   NewtonOptions? theOptions = null)
        where Function : INewton2DFunction
    {
        theOptions ??= new NewtonOptions();
        NewtonResultN2 aRes = new();
        aRes.X = theX0;

        if (!detail.IsOptionsValid(theOptions) || !detail.IsBoundsValid2D(theBounds))
        {
            aRes.Status = MathUtils.Status.InvalidInput;
            return aRes;
        }

        detail.Clamp2D(ref aRes.X, theBounds, false, 0.0);

        double aTolSq = theOptions.FTolerance * theOptions.FTolerance;
        double aMaxStep = theOptions.MaxStepRatio * detail.MaxDomainSize2D(theBounds);

        for (int anIter = 0; anIter < theOptions.MaxIterations; ++anIter)
        {
            aRes.NbIterations = anIter + 1;

            std_array2<double> aF = default;
            std_array2<std_array2<double>> aJ = default;
            if (!theFunc.Invoke(aRes.X[0], aRes.X[1], ref aF, ref aJ))
            {
                aRes.Status = MathUtils.Status.NumericalError;
                return aRes;
            }

            double aFNormSq = aF[0] * aF[0] + aF[1] * aF[1];
            aRes.ResidualNorm = Math.Sqrt(aFNormSq);

            if (aFNormSq <= aTolSq)
            {
                aRes.Status = MathUtils.Status.OK;
                return aRes;
            }

            double aDU = 0.0;
            double aDV = 0.0;

            double aDet = aJ[0][0] * aJ[1][1] - aJ[0][1] * aJ[1][0];
            if (Math.Abs(aDet) < detail.THE_SINGULAR_DET_TOL)
            {
                double aGradU = aJ[0][0] * aF[0] + aJ[1][0] * aF[1];
                double aGradV = aJ[0][1] * aF[0] + aJ[1][1] * aF[1];
                double aGradSq = aGradU * aGradU + aGradV * aGradV;
                if (aGradSq < detail.THE_CRITICAL_GRAD_SQ)
                {
                    aRes.Status = MathUtils.Status.Singular;
                    return aRes;
                }

                double aAlpha = Min(1.0, Math.Sqrt(aFNormSq / aGradSq) * 0.1);
                aDU = -aAlpha * aGradU;
                aDV = -aAlpha * aGradV;
            }
            else
            {
                double aInvDet = 1.0 / aDet;
                aDU = (-aF[0] * aJ[1][1] + aF[1] * aJ[0][1]) * aInvDet;
                aDV = (-aF[1] * aJ[0][0] + aF[0] * aJ[1][0]) * aInvDet;
            }

            double aStepNormSq = aDU * aDU + aDV * aDV;
            double aStepNorm = Math.Sqrt(aStepNormSq);
            if (aStepNorm > aMaxStep)
            {
                double aScale = aMaxStep / aStepNorm;
                aDU *= aScale;
                aDV *= aScale;
            }

            std_array2<double> aNewX = std_array2<double>.Of(aRes.X[0] + aDU, aRes.X[1] + aDV);
            detail.Clamp2D(ref aNewX, theBounds, false, 0.0);

            aRes.StepNorm = Math.Sqrt((aNewX[0] - aRes.X[0]) * (aNewX[0] - aRes.X[0])
                                      + (aNewX[1] - aRes.X[1]) * (aNewX[1] - aRes.X[1]));
            aRes.X = aNewX;

            double aScaleRef = Max(1.0, Max(Math.Abs(aRes.X[0]), Math.Abs(aRes.X[1])));
            if (aRes.StepNorm <= theOptions.XTolerance * aScaleRef)
            {
                std_array2<double> aCheckF = default;
                std_array2<std_array2<double>> aCheckJ = default;
                if (!theFunc.Invoke(aRes.X[0], aRes.X[1], ref aCheckF, ref aCheckJ))
                {
                    aRes.Status = MathUtils.Status.NumericalError;
                    return aRes;
                }

                aRes.ResidualNorm = Math.Sqrt(aCheckF[0] * aCheckF[0] + aCheckF[1] * aCheckF[1]);
                aRes.Status = (aRes.ResidualNorm <= theOptions.FTolerance) ? MathUtils.Status.OK
                                                                           : MathUtils.Status.MaxIterations;
                return aRes;
            }
        }

        // Braced: C# doesn't allow the loop's local names again in the enclosing block.
        {
            std_array2<double> aF = default;
            std_array2<std_array2<double>> aJ = default;
            if (!theFunc.Invoke(aRes.X[0], aRes.X[1], ref aF, ref aJ))
            {
                aRes.Status = MathUtils.Status.NumericalError;
                return aRes;
            }

            aRes.ResidualNorm = Math.Sqrt(aF[0] * aF[0] + aF[1] * aF[1]);
            aRes.Status = (aRes.ResidualNorm <= theOptions.FTolerance) ? MathUtils.Status.OK
                                                                       : MathUtils.Status.MaxIterations;
            return aRes;
        }
    }

    /// <summary>
    /// Solve a 2x2 system with symmetric Jacobian by robust Newton iteration (SVD fallback for a singular
    /// Jacobian, Armijo line search on 0.5*||F||^2 when enabled).
    /// </summary>
    public static NewtonResultN2 Solve2DSymmetric<Function>(Function theFunc,
                                                            in std_array2<double> theX0,
                                                            in NewtonBoundsN2 theBounds,
                                                            NewtonOptions? theOptions = null)
        where Function : INewton2DSymmetricFunction
    {
        theOptions ??= new NewtonOptions();
        NewtonResultN2 aRes = new();
        aRes.X = theX0;

        if (!detail.IsOptionsValid(theOptions) || !detail.IsBoundsValid2D(theBounds))
        {
            aRes.Status = MathUtils.Status.InvalidInput;
            return aRes;
        }

        detail.Clamp2D(ref aRes.X, theBounds, theOptions.AllowSoftBounds, theOptions.SoftBoundsExtension);

        double aTolSq = theOptions.FTolerance * theOptions.FTolerance;
        double aMaxStep = theOptions.MaxStepRatio * detail.MaxDomainSize2D(theBounds);

        for (int anIter = 0; anIter < theOptions.MaxIterations; ++anIter)
        {
            aRes.NbIterations = anIter + 1;

            if (!theFunc.ValueAndJacobian(aRes.X[0], aRes.X[1], out double aF1, out double aF2, out double aJ11, out double aJ12, out double aJ22))
            {
                aRes.Status = MathUtils.Status.NumericalError;
                return aRes;
            }

            double aFNormSq = aF1 * aF1 + aF2 * aF2;
            aRes.ResidualNorm = Math.Sqrt(aFNormSq);

            if (aFNormSq <= aTolSq)
            {
                aRes.Status = MathUtils.Status.OK;
                return aRes;
            }

            double aDU = 0.0;
            double aDV = 0.0;

            double aDet = aJ11 * aJ22 - aJ12 * aJ12;
            if (Math.Abs(aDet) < detail.THE_SINGULAR_DET_TOL)
            {
                if (!detail.SolveSymmetric2x2SVD(aJ11, aJ12, aJ22, aF1, aF2, ref aDU, ref aDV))
                {
                    double aGradU = aJ11 * aF1 + aJ12 * aF2;
                    double aGradV = aJ12 * aF1 + aJ22 * aF2;
                    double aGradSq = aGradU * aGradU + aGradV * aGradV;
                    if (aGradSq < detail.THE_CRITICAL_GRAD_SQ)
                    {
                        aRes.Status = MathUtils.Status.Singular;
                        return aRes;
                    }

                    double aAlpha = Min(1.0, Math.Sqrt(aFNormSq / aGradSq) * 0.1);
                    aDU = -aAlpha * aGradU;
                    aDV = -aAlpha * aGradV;
                }
            }
            else
            {
                double aInvDet = 1.0 / aDet;
                aDU = (-aF1 * aJ22 + aF2 * aJ12) * aInvDet;
                aDV = (-aF2 * aJ11 + aF1 * aJ12) * aInvDet;
            }

            double aStepNormSq = aDU * aDU + aDV * aDV;
            double aStepNorm = Math.Sqrt(aStepNormSq);
            if (aStepNorm > aMaxStep)
            {
                double aScale = aMaxStep / aStepNorm;
                aDU *= aScale;
                aDV *= aScale;
            }

            // Merit function for line search is phi = 0.5 * ||F||^2.
            // Its gradient is grad(phi) = J^T * F, so directional derivative is grad(phi) . dX.
            double aGradPhiU = aJ11 * aF1 + aJ12 * aF2;
            double aGradPhiV = aJ12 * aF1 + aJ22 * aF2;
            double aDirDeriv = aGradPhiU * aDU + aGradPhiV * aDV;

            std_array2<double> aNewX = aRes.X;
            double aNewResidualNorm = -1.0;

            if (theOptions.EnableLineSearch)
            {
                if (aDirDeriv >= 0.0)
                {
                    // Armijo backtracking requires a descent direction for phi; non-negative derivative cannot
                    // provide sufficient decrease.
                    aRes.Status = MathUtils.Status.NonDescentDirection;
                    return aRes;
                }

                double aPhi0 = 0.5 * aFNormSq;
                double aAlpha = 1.0;
                bool isAccepted = false;

                for (int k = 0; k < detail.THE_LINE_SEARCH_MAX; ++k)
                {
                    aNewX[0] = aRes.X[0] + aAlpha * aDU;
                    aNewX[1] = aRes.X[1] + aAlpha * aDV;
                    detail.Clamp2D(ref aNewX,
                                   theBounds,
                                   theOptions.AllowSoftBounds,
                                   theOptions.AllowSoftBounds ? theOptions.SoftBoundsExtension : 0.0);

                    if (!theFunc.Value(aNewX[0], aNewX[1], out double aTryF1, out double aTryF2))
                    {
                        aAlpha *= 0.5;
                        continue;
                    }

                    double aTryFNormSq = aTryF1 * aTryF1 + aTryF2 * aTryF2;
                    double aTryPhi = 0.5 * aTryFNormSq;
                    double aArmijoBound = aPhi0 + detail.THE_ARMIJO_C1 * aAlpha * aDirDeriv;
                    if (aTryPhi <= aArmijoBound)
                    {
                        isAccepted = true;
                        aNewResidualNorm = Math.Sqrt(aTryFNormSq);
                        break;
                    }

                    aAlpha *= 0.5;
                }

                if (!isAccepted)
                {
                    aRes.Status = MathUtils.Status.NonDescentDirection;
                    return aRes;
                }
            }
            else
            {
                aNewX[0] += aDU;
                aNewX[1] += aDV;
                detail.Clamp2D(ref aNewX,
                               theBounds,
                               theOptions.AllowSoftBounds,
                               theOptions.AllowSoftBounds ? theOptions.SoftBoundsExtension : 0.0);
            }

            aRes.StepNorm = Math.Sqrt((aNewX[0] - aRes.X[0]) * (aNewX[0] - aRes.X[0])
                                      + (aNewX[1] - aRes.X[1]) * (aNewX[1] - aRes.X[1]));
            aRes.X = aNewX;

            if (aNewResidualNorm >= 0.0)
            {
                aRes.ResidualNorm = aNewResidualNorm;
            }

            double aScaleRef = Max(1.0, Max(Math.Abs(aRes.X[0]), Math.Abs(aRes.X[1])));
            if (aRes.StepNorm <= theOptions.XTolerance * aScaleRef)
            {
                if (!theFunc.Value(aRes.X[0], aRes.X[1], out double aCheckF1, out double aCheckF2))
                {
                    aRes.Status = MathUtils.Status.NumericalError;
                    return aRes;
                }

                aRes.ResidualNorm = Math.Sqrt(aCheckF1 * aCheckF1 + aCheckF2 * aCheckF2);
                aRes.Status = (aRes.ResidualNorm <= theOptions.FTolerance) ? MathUtils.Status.OK
                                                                           : MathUtils.Status.MaxIterations;
                return aRes;
            }
        }

        // Braced: C# doesn't allow the loop's local names again in the enclosing block.
        {
            if (!theFunc.Value(aRes.X[0], aRes.X[1], out double aF1, out double aF2))
            {
                aRes.Status = MathUtils.Status.NumericalError;
                return aRes;
            }

            aRes.ResidualNorm = Math.Sqrt(aF1 * aF1 + aF2 * aF2);
            aRes.Status = (aRes.ResidualNorm <= theOptions.FTolerance) ? MathUtils.Status.OK
                                                                       : MathUtils.Status.MaxIterations;
            return aRes;
        }
    }
}
