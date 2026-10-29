// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_LevenbergMarquardt.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

public static partial class MathSys
{
    /// <summary>Configuration for the Levenberg-Marquardt algorithm: Config plus the damping parameters.</summary>
    public class LMConfig : MathUtils.Config
    {
        /// <summary>Initial damping parameter.</summary>
        public double LambdaInit = 1.0e-3;
        /// <summary>Factor to increase lambda on rejected step.</summary>
        public double LambdaIncrease = 10.0;
        /// <summary>Factor to decrease lambda on accepted step.</summary>
        public double LambdaDecrease = 0.1;
        /// <summary>Maximum lambda value before failing.</summary>
        public double LambdaMax = 1.0e10;
        /// <summary>Minimum lambda value.</summary>
        public double LambdaMin = 1.0e-12;

        public LMConfig()
        {
        }

        public LMConfig(double theTolerance, int theMaxIter = 100)
            : base(theTolerance, theMaxIter)
        {
        }

        /// <summary>Copy, as C++'s copy construction.</summary>
        public LMConfig(LMConfig theOther)
            : base(theOther)
        {
            LambdaInit = theOther.LambdaInit;
            LambdaIncrease = theOther.LambdaIncrease;
            LambdaDecrease = theOther.LambdaDecrease;
            LambdaMax = theOther.LambdaMax;
            LambdaMin = theOther.LambdaMin;
        }
    }

    /// <summary>
    /// Levenberg-Marquardt algorithm for nonlinear least squares: minimizes ||F(X)||^2, solving
    /// (J^T*J + lambda*I) * dX = -J^T*F with adaptive damping lambda.
    /// </summary>
    public static VectorResult LevenbergMarquardt<FuncSetType>(FuncSetType theFunc,
                                                           math_Vector theStart,
                                                           LMConfig? theConfig = null)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        theConfig ??= new LMConfig();
        VectorResult aResult = new();

        int aNbVars = theFunc.NbVariables();
        int aNbEqs = theFunc.NbEquations();

        // Check dimensions
        if (theStart.Length() != aNbVars)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aVarLower = theStart.Lower();
        int aVarUpper = theStart.Upper();

        // Working vectors and matrices
        math_Vector aSol = new(theStart);
        math_Vector aF = new(1, aNbEqs);
        math_Vector aFNew = new(1, aNbEqs);
        math_Vector aDeltaX = new(aVarLower, aVarUpper);
        math_Vector aGrad = new(aVarLower, aVarUpper);
        math_Matrix aJac = new(1, aNbEqs, aVarLower, aVarUpper);
        math_Matrix aJtJ = new(aVarLower, aVarUpper, aVarLower, aVarUpper);
        math_Vector aJtF = new(aVarLower, aVarUpper);

        double aLambda = theConfig.LambdaInit;

        // Evaluate initial residual
        if (!theFunc.Value(aSol, aF))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        // Compute initial ||F||^2
        double aChi2 = 0.0;
        for (int i = 1; i <= aNbEqs; ++i)
        {
            aChi2 += aF[i] * aF[i];
        }

        // Main iteration loop
        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Check F convergence
            bool aFConverged = true;
            for (int i = 1; i <= aNbEqs; ++i)
            {
                if (Math.Abs(aF[i]) > theConfig.FTolerance)
                {
                    aFConverged = false;
                    break;
                }
            }

            if (aFConverged)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aSol;
                aResult.Value = aChi2;
                return aResult;
            }

            // Compute Jacobian
            if (!theFunc.Derivatives(aSol, aJac))
            {
                aResult.Status = Status.NumericalError;
                return aResult;
            }

            // Compute J^T * J
            for (int i = aVarLower; i <= aVarUpper; ++i)
            {
                for (int j = aVarLower; j <= aVarUpper; ++j)
                {
                    double aSum = 0.0;
                    for (int k = 1; k <= aNbEqs; ++k)
                    {
                        aSum += aJac[k, i] * aJac[k, j];
                    }
                    aJtJ[i, j] = aSum;
                }
            }

            // Compute J^T * F (negative gradient of chi^2)
            for (int i = aVarLower; i <= aVarUpper; ++i)
            {
                double aSum = 0.0;
                for (int k = 1; k <= aNbEqs; ++k)
                {
                    aSum += aJac[k, i] * aF[k];
                }
                aJtF[i] = aSum;
                aGrad[i] = 2.0 * aSum; // Gradient of chi^2 = 2 * J^T * F
            }

            // Check gradient convergence
            double aGradNorm = 0.0;
            for (int i = aVarLower; i <= aVarUpper; ++i)
            {
                aGradNorm += aGrad[i] * aGrad[i];
            }
            aGradNorm = Math.Sqrt(aGradNorm);

            if (aGradNorm < theConfig.Tolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aSol;
                aResult.Value = aChi2;
                aResult.Gradient = aGrad;
                return aResult;
            }

            // Inner loop: try to find an acceptable step
            bool aStepAccepted = false;
            for (int aLamIter = 0; aLamIter < 20 && !aStepAccepted; ++aLamIter)
            {
                // Add damping: J^T*J + lambda*I
                math_Matrix aDamped = new(aJtJ);
                for (int i = aVarLower; i <= aVarUpper; ++i)
                {
                    aDamped[i, i] += aLambda;
                }

                // Solve (J^T*J + lambda*I) * dX = -J^T*F
                math_Vector aNegJtF = new(aVarLower, aVarUpper);
                for (int i = aVarLower; i <= aVarUpper; ++i)
                {
                    aNegJtF[i] = -aJtF[i];
                }

                LinearResult aLinResult = MathLin.Solve(aDamped, aNegJtF);
                if (!aLinResult.IsDone())
                {
                    // Matrix is singular, increase damping
                    aLambda *= theConfig.LambdaIncrease;
                    if (aLambda > theConfig.LambdaMax)
                    {
                        aResult.Status = Status.Singular;
                        aResult.Solution = aSol;
                        aResult.Value = aChi2;
                        return aResult;
                    }
                    continue;
                }

                aDeltaX.Initialized(aLinResult.Solution!);

                // Compute new solution
                math_Vector aSolNew = new(aVarLower, aVarUpper);
                for (int i = aVarLower; i <= aVarUpper; ++i)
                {
                    aSolNew[i] = aSol[i] + aDeltaX[i];
                }

                // Evaluate new residual
                if (!theFunc.Value(aSolNew, aFNew))
                {
                    // Function evaluation failed, increase damping
                    aLambda *= theConfig.LambdaIncrease;
                    if (aLambda > theConfig.LambdaMax)
                    {
                        aResult.Status = Status.NumericalError;
                        aResult.Solution = aSol;
                        aResult.Value = aChi2;
                        return aResult;
                    }
                    continue;
                }

                // Compute new ||F||^2
                double aChi2New = 0.0;
                for (int i = 1; i <= aNbEqs; ++i)
                {
                    aChi2New += aFNew[i] * aFNew[i];
                }

                // Accept or reject step
                if (aChi2New < aChi2)
                {
                    // Step accepted
                    aSol.Initialized(aSolNew);
                    aF.Initialized(aFNew);
                    aChi2 = aChi2New;
                    aLambda *= theConfig.LambdaDecrease;
                    if (aLambda < theConfig.LambdaMin)
                    {
                        aLambda = theConfig.LambdaMin;
                    }
                    aStepAccepted = true;

                    // Check X convergence
                    bool aXConverged = true;
                    for (int i = aVarLower; i <= aVarUpper; ++i)
                    {
                        if (Math.Abs(aDeltaX[i]) > theConfig.XTolerance * (1.0 + Math.Abs(aSol[i])))
                        {
                            aXConverged = false;
                            break;
                        }
                    }

                    if (aXConverged)
                    {
                        aResult.Status = Status.OK;
                        aResult.Solution = aSol;
                        aResult.Value = aChi2;
                        return aResult;
                    }
                }
                else
                {
                    // Step rejected, increase damping
                    aLambda *= theConfig.LambdaIncrease;
                    if (aLambda > theConfig.LambdaMax)
                    {
                        aResult.Status = Status.NotConverged;
                        aResult.Solution = aSol;
                        aResult.Value = aChi2;
                        return aResult;
                    }
                }
            }

            if (!aStepAccepted)
            {
                // Failed to find acceptable step
                aResult.Status = Status.NotConverged;
                aResult.Solution = aSol;
                aResult.Value = aChi2;
                return aResult;
            }
        }

        // Max iterations reached
        aResult.Status = Status.MaxIterations;
        aResult.Solution = aSol;
        aResult.Value = aChi2;
        return aResult;
    }

    /// <summary>
    /// Levenberg-Marquardt with bounds: minimizes ||F(X)||^2 subject to theInfBound &lt;= X &lt;= theSupBound,
    /// clamping the start and every step.
    /// </summary>
    public static VectorResult LevenbergMarquardtBounded<FuncSetType>(FuncSetType theFunc,
                                                                  math_Vector theStart,
                                                                  math_Vector theInfBound,
                                                                  math_Vector theSupBound,
                                                                  LMConfig? theConfig = null)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        theConfig ??= new LMConfig();
        VectorResult aResult = new();

        int aNbVars = theFunc.NbVariables();
        int aNbEqs = theFunc.NbEquations();

        // Check dimensions
        if (theStart.Length() != aNbVars || theInfBound.Length() != aNbVars
                || theSupBound.Length() != aNbVars)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aVarLower = theStart.Lower();
        int aVarUpper = theStart.Upper();

        // Working vectors and matrices
        math_Vector aSol = new(theStart);
        math_Vector aF = new(1, aNbEqs);
        math_Vector aFNew = new(1, aNbEqs);
        math_Vector aDeltaX = new(aVarLower, aVarUpper);
        math_Vector aGrad = new(aVarLower, aVarUpper);
        math_Matrix aJac = new(1, aNbEqs, aVarLower, aVarUpper);
        math_Matrix aJtJ = new(aVarLower, aVarUpper, aVarLower, aVarUpper);
        math_Vector aJtF = new(aVarLower, aVarUpper);

        // Clamp initial solution to bounds
        for (int i = aVarLower; i <= aVarUpper; ++i)
        {
            aSol[i] = MathUtils.Clamp(aSol[i], theInfBound[i], theSupBound[i]);
        }

        double aLambda = theConfig.LambdaInit;

        // Evaluate initial residual
        if (!theFunc.Value(aSol, aF))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        // Compute initial ||F||^2
        double aChi2 = 0.0;
        for (int i = 1; i <= aNbEqs; ++i)
        {
            aChi2 += aF[i] * aF[i];
        }

        // Main iteration loop
        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Check F convergence
            bool aFConverged = true;
            for (int i = 1; i <= aNbEqs; ++i)
            {
                if (Math.Abs(aF[i]) > theConfig.FTolerance)
                {
                    aFConverged = false;
                    break;
                }
            }

            if (aFConverged)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aSol;
                aResult.Value = aChi2;
                return aResult;
            }

            // Compute Jacobian
            if (!theFunc.Derivatives(aSol, aJac))
            {
                aResult.Status = Status.NumericalError;
                return aResult;
            }

            // Compute J^T * J
            for (int i = aVarLower; i <= aVarUpper; ++i)
            {
                for (int j = aVarLower; j <= aVarUpper; ++j)
                {
                    double aSum = 0.0;
                    for (int k = 1; k <= aNbEqs; ++k)
                    {
                        aSum += aJac[k, i] * aJac[k, j];
                    }
                    aJtJ[i, j] = aSum;
                }
            }

            // Compute J^T * F
            for (int i = aVarLower; i <= aVarUpper; ++i)
            {
                double aSum = 0.0;
                for (int k = 1; k <= aNbEqs; ++k)
                {
                    aSum += aJac[k, i] * aF[k];
                }
                aJtF[i] = aSum;
                aGrad[i] = 2.0 * aSum;
            }

            // Check gradient convergence
            double aGradNorm = 0.0;
            for (int i = aVarLower; i <= aVarUpper; ++i)
            {
                aGradNorm += aGrad[i] * aGrad[i];
            }
            aGradNorm = Math.Sqrt(aGradNorm);

            if (aGradNorm < theConfig.Tolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aSol;
                aResult.Value = aChi2;
                aResult.Gradient = aGrad;
                return aResult;
            }

            // Inner loop: try to find an acceptable step
            bool aStepAccepted = false;
            for (int aLamIter = 0; aLamIter < 20 && !aStepAccepted; ++aLamIter)
            {
                // Add damping
                math_Matrix aDamped = new(aJtJ);
                for (int i = aVarLower; i <= aVarUpper; ++i)
                {
                    aDamped[i, i] += aLambda;
                }

                // Solve (J^T*J + lambda*I) * dX = -J^T*F
                math_Vector aNegJtF = new(aVarLower, aVarUpper);
                for (int i = aVarLower; i <= aVarUpper; ++i)
                {
                    aNegJtF[i] = -aJtF[i];
                }

                LinearResult aLinResult = MathLin.Solve(aDamped, aNegJtF);
                if (!aLinResult.IsDone())
                {
                    aLambda *= theConfig.LambdaIncrease;
                    if (aLambda > theConfig.LambdaMax)
                    {
                        aResult.Status = Status.Singular;
                        aResult.Solution = aSol;
                        aResult.Value = aChi2;
                        return aResult;
                    }
                    continue;
                }

                aDeltaX.Initialized(aLinResult.Solution!);

                // Compute new solution with bounds clamping
                math_Vector aSolNew = new(aVarLower, aVarUpper);
                for (int i = aVarLower; i <= aVarUpper; ++i)
                {
                    aSolNew[i] = MathUtils.Clamp(aSol[i] + aDeltaX[i], theInfBound[i], theSupBound[i]);
                }

                // Evaluate new residual
                if (!theFunc.Value(aSolNew, aFNew))
                {
                    aLambda *= theConfig.LambdaIncrease;
                    if (aLambda > theConfig.LambdaMax)
                    {
                        aResult.Status = Status.NumericalError;
                        aResult.Solution = aSol;
                        aResult.Value = aChi2;
                        return aResult;
                    }
                    continue;
                }

                // Compute new ||F||^2
                double aChi2New = 0.0;
                for (int i = 1; i <= aNbEqs; ++i)
                {
                    aChi2New += aFNew[i] * aFNew[i];
                }

                // Accept or reject step
                if (aChi2New < aChi2)
                {
                    aSol.Initialized(aSolNew);
                    aF.Initialized(aFNew);
                    aChi2 = aChi2New;
                    aLambda *= theConfig.LambdaDecrease;
                    if (aLambda < theConfig.LambdaMin)
                    {
                        aLambda = theConfig.LambdaMin;
                    }
                    aStepAccepted = true;

                    // Check X convergence
                    // OCCT: tests the unclamped step aDeltaX, not the step actually taken, so at an active bound the
                    // X criterion fails however small the real step is.
                    bool aXConverged = true;
                    for (int i = aVarLower; i <= aVarUpper; ++i)
                    {
                        if (Math.Abs(aDeltaX[i]) > theConfig.XTolerance * (1.0 + Math.Abs(aSol[i])))
                        {
                            aXConverged = false;
                            break;
                        }
                    }

                    if (aXConverged)
                    {
                        aResult.Status = Status.OK;
                        aResult.Solution = aSol;
                        aResult.Value = aChi2;
                        return aResult;
                    }
                }
                else
                {
                    aLambda *= theConfig.LambdaIncrease;
                    if (aLambda > theConfig.LambdaMax)
                    {
                        aResult.Status = Status.NotConverged;
                        aResult.Solution = aSol;
                        aResult.Value = aChi2;
                        return aResult;
                    }
                }
            }

            if (!aStepAccepted)
            {
                aResult.Status = Status.NotConverged;
                aResult.Solution = aSol;
                aResult.Value = aChi2;
                return aResult;
            }
        }

        // Max iterations reached
        aResult.Status = Status.MaxIterations;
        aResult.Solution = aSol;
        aResult.Value = aChi2;
        return aResult;
    }
}
