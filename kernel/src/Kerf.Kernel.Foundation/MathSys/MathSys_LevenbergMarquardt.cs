// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_LevenbergMarquardt.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Systems;

internal static partial class MathSys
{
    /// <summary>Configuration for the Levenberg-Marquardt algorithm: Config plus the damping parameters.</summary>
    public class LMConfig : global::Kerf.Engine.Numerics.Utilities.MathUtils.Config
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

        public LMConfig(double tolerance, int maxIter = 100)
            : base(tolerance, maxIter)
        {
        }

        /// <summary>Copy, as C++'s copy construction.</summary>
        public LMConfig(LMConfig other)
            : base(other)
        {
            LambdaInit = other.LambdaInit;
            LambdaIncrease = other.LambdaIncrease;
            LambdaDecrease = other.LambdaDecrease;
            LambdaMax = other.LambdaMax;
            LambdaMin = other.LambdaMin;
        }
    }

    /// <summary>
    /// Levenberg-Marquardt algorithm for nonlinear least squares: minimizes ||F(X)||^2, solving
    /// (J^T*J + lambda*I) * dX = -J^T*F with adaptive damping lambda.
    /// </summary>
    public static VectorResult LevenbergMarquardt<FuncSetType>(FuncSetType func,
                                                           math_Vector start,
                                                           LMConfig? config = null)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        config ??= new LMConfig();
        VectorResult result = new();

        int nbVars = func.NbVariables();
        int nbEqs = func.NbEquations();

        // Check dimensions
        if (start.Length() != nbVars)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int varLower = start.Lower();
        int varUpper = start.Upper();

        // Working vectors and matrices
        math_Vector sol = new(start);
        math_Vector f = new(1, nbEqs);
        math_Vector fNew = new(1, nbEqs);
        math_Vector deltaX = new(varLower, varUpper);
        math_Vector grad = new(varLower, varUpper);
        Matrix jac = new(1, nbEqs, varLower, varUpper);
        Matrix jtJ = new(varLower, varUpper, varLower, varUpper);
        math_Vector jtF = new(varLower, varUpper);

        double lambda = config.LambdaInit;

        // Evaluate initial residual
        if (!func.Value(sol, f))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Compute initial ||F||^2
        double chi2 = 0.0;
        for (int i = 1; i <= nbEqs; ++i)
        {
            chi2 += f[i] * f[i];
        }

        // Main iteration loop
        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Check F convergence
            bool fConverged = true;
            for (int i = 1; i <= nbEqs; ++i)
            {
                if (Math.Abs(f[i]) > config.FTolerance)
                {
                    fConverged = false;
                    break;
                }
            }

            if (fConverged)
            {
                result.Status = Status.OK;
                result.Solution = sol;
                result.Value = chi2;
                return result;
            }

            // Compute Jacobian
            if (!func.Derivatives(sol, jac))
            {
                result.Status = Status.NumericalError;
                return result;
            }

            // Compute J^T * J
            for (int i = varLower; i <= varUpper; ++i)
            {
                for (int j = varLower; j <= varUpper; ++j)
                {
                    double sum = 0.0;
                    for (int k = 1; k <= nbEqs; ++k)
                    {
                        sum += jac[k, i] * jac[k, j];
                    }
                    jtJ[i, j] = sum;
                }
            }

            // Compute J^T * F (negative gradient of chi^2)
            for (int i = varLower; i <= varUpper; ++i)
            {
                double sum = 0.0;
                for (int k = 1; k <= nbEqs; ++k)
                {
                    sum += jac[k, i] * f[k];
                }
                jtF[i] = sum;
                grad[i] = 2.0 * sum; // Gradient of chi^2 = 2 * J^T * F
            }

            // Check gradient convergence
            double gradNorm = 0.0;
            for (int i = varLower; i <= varUpper; ++i)
            {
                gradNorm += grad[i] * grad[i];
            }
            gradNorm = Math.Sqrt(gradNorm);

            if (gradNorm < config.Tolerance)
            {
                result.Status = Status.OK;
                result.Solution = sol;
                result.Value = chi2;
                result.Gradient = grad;
                return result;
            }

            // Inner loop: try to find an acceptable step
            bool stepAccepted = false;
            for (int lamIter = 0; lamIter < 20 && !stepAccepted; ++lamIter)
            {
                // Add damping: J^T*J + lambda*I
                Matrix damped = new(jtJ);
                for (int i = varLower; i <= varUpper; ++i)
                {
                    damped[i, i] += lambda;
                }

                // Solve (J^T*J + lambda*I) * dX = -J^T*F
                math_Vector negJtF = new(varLower, varUpper);
                for (int i = varLower; i <= varUpper; ++i)
                {
                    negJtF[i] = -jtF[i];
                }

                LinearResult linResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(damped, negJtF);
                if (!linResult.IsDone)
                {
                    // Matrix is singular, increase damping
                    lambda *= config.LambdaIncrease;
                    if (lambda > config.LambdaMax)
                    {
                        result.Status = Status.Singular;
                        result.Solution = sol;
                        result.Value = chi2;
                        return result;
                    }
                    continue;
                }

                deltaX.Initialized(linResult.Solution!);

                // Compute new solution
                math_Vector solNew = new(varLower, varUpper);
                for (int i = varLower; i <= varUpper; ++i)
                {
                    solNew[i] = sol[i] + deltaX[i];
                }

                // Evaluate new residual
                if (!func.Value(solNew, fNew))
                {
                    // Function evaluation failed, increase damping
                    lambda *= config.LambdaIncrease;
                    if (lambda > config.LambdaMax)
                    {
                        result.Status = Status.NumericalError;
                        result.Solution = sol;
                        result.Value = chi2;
                        return result;
                    }
                    continue;
                }

                // Compute new ||F||^2
                double chi2New = 0.0;
                for (int i = 1; i <= nbEqs; ++i)
                {
                    chi2New += fNew[i] * fNew[i];
                }

                // Accept or reject step
                if (chi2New < chi2)
                {
                    // Step accepted
                    sol.Initialized(solNew);
                    f.Initialized(fNew);
                    chi2 = chi2New;
                    lambda *= config.LambdaDecrease;
                    if (lambda < config.LambdaMin)
                    {
                        lambda = config.LambdaMin;
                    }
                    stepAccepted = true;

                    // Check X convergence
                    bool xConverged = true;
                    for (int i = varLower; i <= varUpper; ++i)
                    {
                        if (Math.Abs(deltaX[i]) > config.XTolerance * (1.0 + Math.Abs(sol[i])))
                        {
                            xConverged = false;
                            break;
                        }
                    }

                    if (xConverged)
                    {
                        result.Status = Status.OK;
                        result.Solution = sol;
                        result.Value = chi2;
                        return result;
                    }
                }
                else
                {
                    // Step rejected, increase damping
                    lambda *= config.LambdaIncrease;
                    if (lambda > config.LambdaMax)
                    {
                        result.Status = Status.NotConverged;
                        result.Solution = sol;
                        result.Value = chi2;
                        return result;
                    }
                }
            }

            if (!stepAccepted)
            {
                // Failed to find acceptable step
                result.Status = Status.NotConverged;
                result.Solution = sol;
                result.Value = chi2;
                return result;
            }
        }

        // Max iterations reached
        result.Status = Status.MaxIterations;
        result.Solution = sol;
        result.Value = chi2;
        return result;
    }

    /// <summary>
    /// Levenberg-Marquardt with bounds: minimizes ||F(X)||^2 subject to theInfBound &lt;= X &lt;= theSupBound,
    /// clamping the start and every step.
    /// </summary>
    public static VectorResult LevenbergMarquardtBounded<FuncSetType>(FuncSetType func,
                                                                  math_Vector start,
                                                                  math_Vector infBound,
                                                                  math_Vector supBound,
                                                                  LMConfig? config = null)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        config ??= new LMConfig();
        VectorResult result = new();

        int nbVars = func.NbVariables();
        int nbEqs = func.NbEquations();

        // Check dimensions
        if (start.Length() != nbVars || infBound.Length() != nbVars
                || supBound.Length() != nbVars)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int varLower = start.Lower();
        int varUpper = start.Upper();

        // Working vectors and matrices
        math_Vector sol = new(start);
        math_Vector f = new(1, nbEqs);
        math_Vector fNew = new(1, nbEqs);
        math_Vector deltaX = new(varLower, varUpper);
        math_Vector grad = new(varLower, varUpper);
        Matrix jac = new(1, nbEqs, varLower, varUpper);
        Matrix jtJ = new(varLower, varUpper, varLower, varUpper);
        math_Vector jtF = new(varLower, varUpper);

        // Clamp initial solution to bounds
        for (int i = varLower; i <= varUpper; ++i)
        {
            sol[i] = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(sol[i], infBound[i], supBound[i]);
        }

        double lambda = config.LambdaInit;

        // Evaluate initial residual
        if (!func.Value(sol, f))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        // Compute initial ||F||^2
        double chi2 = 0.0;
        for (int i = 1; i <= nbEqs; ++i)
        {
            chi2 += f[i] * f[i];
        }

        // Main iteration loop
        for (int iter = 0; iter < config.MaxIterations; ++iter)
        {
            result.NbIterations = iter + 1;

            // Check F convergence
            bool fConverged = true;
            for (int i = 1; i <= nbEqs; ++i)
            {
                if (Math.Abs(f[i]) > config.FTolerance)
                {
                    fConverged = false;
                    break;
                }
            }

            if (fConverged)
            {
                result.Status = Status.OK;
                result.Solution = sol;
                result.Value = chi2;
                return result;
            }

            // Compute Jacobian
            if (!func.Derivatives(sol, jac))
            {
                result.Status = Status.NumericalError;
                return result;
            }

            // Compute J^T * J
            for (int i = varLower; i <= varUpper; ++i)
            {
                for (int j = varLower; j <= varUpper; ++j)
                {
                    double sum = 0.0;
                    for (int k = 1; k <= nbEqs; ++k)
                    {
                        sum += jac[k, i] * jac[k, j];
                    }
                    jtJ[i, j] = sum;
                }
            }

            // Compute J^T * F
            for (int i = varLower; i <= varUpper; ++i)
            {
                double sum = 0.0;
                for (int k = 1; k <= nbEqs; ++k)
                {
                    sum += jac[k, i] * f[k];
                }
                jtF[i] = sum;
                grad[i] = 2.0 * sum;
            }

            // Check gradient convergence
            double gradNorm = 0.0;
            for (int i = varLower; i <= varUpper; ++i)
            {
                gradNorm += grad[i] * grad[i];
            }
            gradNorm = Math.Sqrt(gradNorm);

            if (gradNorm < config.Tolerance)
            {
                result.Status = Status.OK;
                result.Solution = sol;
                result.Value = chi2;
                result.Gradient = grad;
                return result;
            }

            // Inner loop: try to find an acceptable step
            bool stepAccepted = false;
            for (int lamIter = 0; lamIter < 20 && !stepAccepted; ++lamIter)
            {
                // Add damping
                Matrix damped = new(jtJ);
                for (int i = varLower; i <= varUpper; ++i)
                {
                    damped[i, i] += lambda;
                }

                // Solve (J^T*J + lambda*I) * dX = -J^T*F
                math_Vector negJtF = new(varLower, varUpper);
                for (int i = varLower; i <= varUpper; ++i)
                {
                    negJtF[i] = -jtF[i];
                }

                LinearResult linResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(damped, negJtF);
                if (!linResult.IsDone)
                {
                    lambda *= config.LambdaIncrease;
                    if (lambda > config.LambdaMax)
                    {
                        result.Status = Status.Singular;
                        result.Solution = sol;
                        result.Value = chi2;
                        return result;
                    }
                    continue;
                }

                deltaX.Initialized(linResult.Solution!);

                // Compute new solution with bounds clamping
                math_Vector solNew = new(varLower, varUpper);
                for (int i = varLower; i <= varUpper; ++i)
                {
                    solNew[i] = global::Kerf.Engine.Numerics.Utilities.MathUtils.Clamp(sol[i] + deltaX[i], infBound[i], supBound[i]);
                }

                // Evaluate new residual
                if (!func.Value(solNew, fNew))
                {
                    lambda *= config.LambdaIncrease;
                    if (lambda > config.LambdaMax)
                    {
                        result.Status = Status.NumericalError;
                        result.Solution = sol;
                        result.Value = chi2;
                        return result;
                    }
                    continue;
                }

                // Compute new ||F||^2
                double chi2New = 0.0;
                for (int i = 1; i <= nbEqs; ++i)
                {
                    chi2New += fNew[i] * fNew[i];
                }

                // Accept or reject step
                if (chi2New < chi2)
                {
                    sol.Initialized(solNew);
                    f.Initialized(fNew);
                    chi2 = chi2New;
                    lambda *= config.LambdaDecrease;
                    if (lambda < config.LambdaMin)
                    {
                        lambda = config.LambdaMin;
                    }
                    stepAccepted = true;

                    // Check X convergence
                    // OCCT: tests the unclamped step aDeltaX, not the step actually taken, so at an active bound the
                    // X criterion fails however small the real step is.
                    bool xConverged = true;
                    for (int i = varLower; i <= varUpper; ++i)
                    {
                        if (Math.Abs(deltaX[i]) > config.XTolerance * (1.0 + Math.Abs(sol[i])))
                        {
                            xConverged = false;
                            break;
                        }
                    }

                    if (xConverged)
                    {
                        result.Status = Status.OK;
                        result.Solution = sol;
                        result.Value = chi2;
                        return result;
                    }
                }
                else
                {
                    lambda *= config.LambdaIncrease;
                    if (lambda > config.LambdaMax)
                    {
                        result.Status = Status.NotConverged;
                        result.Solution = sol;
                        result.Value = chi2;
                        return result;
                    }
                }
            }

            if (!stepAccepted)
            {
                result.Status = Status.NotConverged;
                result.Solution = sol;
                result.Value = chi2;
                return result;
            }
        }

        // Max iterations reached
        result.Status = Status.MaxIterations;
        result.Solution = sol;
        result.Value = chi2;
        return result;
    }
}
