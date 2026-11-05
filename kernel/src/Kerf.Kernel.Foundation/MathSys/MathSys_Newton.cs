// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_Newton.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Systems;

// OCCT's MathSys::Newton and LevenbergMarquardt are templates over any "FuncSetType" with the members of
// math_FunctionSetWithDerivatives. Here those members are the interfaces below, with the same signatures as the
// classic abstract classes, so a class can derive from math_FunctionSetWithDerivatives and also implement
// MathSys.IFunctionSetWithDerivatives with the same overrides (as the comparison tests do).

internal static partial class MathSys
{
    /// <summary>A system of NbEquations() functions of NbVariables() variables (math_FunctionSet's members).</summary>
    public interface IFunctionSet : global::Kerf.Engine.Numerics.Utilities.MathUtils.ISystemFunction
    {
        /// <summary>Number of variables of the function.</summary>
        int NbVariables();

        /// <summary>Number of equations of the function.</summary>
        int NbEquations();
    }

    /// <summary>A system of functions with its Jacobian (math_FunctionSetWithDerivatives's members).</summary>
    public interface IFunctionSetWithDerivatives : IFunctionSet
    {
        /// <summary>Computes the Jacobian at theX into theD; false if it could not be computed.</summary>
        bool Derivatives(math_Vector x, Matrix d);

        /// <summary>Computes the values and the Jacobian at theX; false if they could not be computed.</summary>
        bool Values(math_Vector x, math_Vector f, Matrix d);
    }

    /// <summary>
    /// Newton-Raphson method for solving systems of nonlinear equations F(X) = 0: solves J(X)*dX = -F(X) at each
    /// step. Converged when every |dX(i)| &lt;= theTolX(i) and every |F(i)| &lt;= theTolF.
    /// </summary>
    public static VectorResult Newton<FuncSetType>(FuncSetType func,
                                                   math_Vector start,
                                                   math_Vector tolX,
                                                   double tolF,
                                                   int maxIter = 100)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        VectorResult result = new();

        int n = func.NbVariables();
        int m = func.NbEquations();

        // Check dimensions
        if (n != m || start.Length() != n || tolX.Length() != n)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int lower = start.Lower();
        int upper = start.Upper();

        // Working vectors
        math_Vector sol = new(start);
        math_Vector f = new(lower, upper);
        math_Vector deltaX = new(lower, upper);
        Matrix jacobian = new(lower, upper, lower, upper);

        // Newton iteration
        for (int iter = 0; iter < maxIter; ++iter)
        {
            // Evaluate function and Jacobian
            if (!func.Values(sol, f, jacobian))
            {
                result.Status = Status.NumericalError;
                result.NbIterations = iter;
                return result;
            }

            // Solve J * dX = -F
            // Negate F for the right-hand side
            math_Vector negF = new(lower, upper);
            for (int i = lower; i <= upper; ++i)
            {
                negF[i] = -f[i];
            }

            LinearResult linResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(jacobian, negF);
            if (!linResult.IsDone)
            {
                result.Status = Status.Singular;
                result.NbIterations = iter;
                return result;
            }

            deltaX.Initialized(linResult.Solution!);

            // Check X convergence before update
            bool xConverged = true;
            for (int i = lower; i <= upper; ++i)
            {
                if (Math.Abs(deltaX[i]) > tolX[i])
                {
                    xConverged = false;
                    break;
                }
            }

            // Update solution
            for (int i = lower; i <= upper; ++i)
            {
                sol[i] += deltaX[i];
            }

            // Re-evaluate function at new solution to check F convergence
            if (!func.Value(sol, f))
            {
                result.Status = Status.NumericalError;
                result.NbIterations = iter + 1;
                return result;
            }

            // Check F convergence with updated function values
            bool fConverged = true;
            for (int i = lower; i <= upper; ++i)
            {
                if (Math.Abs(f[i]) > tolF)
                {
                    fConverged = false;
                    break;
                }
            }

            if (xConverged && fConverged)
            {
                // OCCT: the Jacobian returned is the one evaluated before the last step, not at the returned Solution.
                result.Status = Status.OK;
                result.NbIterations = iter + 1;
                result.Solution = sol;
                result.Jacobian = jacobian;
                return result;
            }

            result.NbIterations = iter + 1;
        }

        // Max iterations reached - return last solution
        result.Status = Status.MaxIterations;
        result.Solution = sol;
        result.Jacobian = jacobian;
        return result;
    }

    /// <summary>
    /// Newton-Raphson method with bounds: the start and every step are clamped to [theInfBound, theSupBound].
    /// </summary>
    public static VectorResult NewtonBounded<FuncSetType>(FuncSetType func,
                                                          math_Vector start,
                                                          math_Vector infBound,
                                                          math_Vector supBound,
                                                          math_Vector tolX,
                                                          double tolF,
                                                          int maxIter = 100)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        VectorResult result = new();

        int n = func.NbVariables();
        int m = func.NbEquations();

        // Check dimensions
        if (n != m || start.Length() != n || tolX.Length() != n || infBound.Length() != n
            || supBound.Length() != n)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int lower = start.Lower();
        int upper = start.Upper();

        // Working vectors
        math_Vector sol = new(start);
        math_Vector f = new(lower, upper);
        math_Vector deltaX = new(lower, upper);
        Matrix jacobian = new(lower, upper, lower, upper);

        // Clamp initial solution to bounds
        for (int i = lower; i <= upper; ++i)
        {
            if (sol[i] < infBound[i])
            {
                sol[i] = infBound[i];
            }
            if (sol[i] > supBound[i])
            {
                sol[i] = supBound[i];
            }
        }

        // Newton iteration
        for (int iter = 0; iter < maxIter; ++iter)
        {
            // Evaluate function and Jacobian
            if (!func.Values(sol, f, jacobian))
            {
                result.Status = Status.NumericalError;
                result.NbIterations = iter;
                return result;
            }

            // Solve J * dX = -F
            math_Vector negF = new(lower, upper);
            for (int i = lower; i <= upper; ++i)
            {
                negF[i] = -f[i];
            }

            LinearResult linResult = global::Kerf.Engine.Numerics.Linear.MathLin.Solve(jacobian, negF);
            if (!linResult.IsDone)
            {
                result.Status = Status.Singular;
                result.NbIterations = iter;
                return result;
            }

            deltaX.Initialized(linResult.Solution!);

            // Check X convergence before update
            bool xConverged = true;
            for (int i = lower; i <= upper; ++i)
            {
                if (Math.Abs(deltaX[i]) > tolX[i])
                {
                    xConverged = false;
                    break;
                }
            }

            // Update solution with bounds clamping
            for (int i = lower; i <= upper; ++i)
            {
                sol[i] += deltaX[i];
                if (sol[i] < infBound[i])
                {
                    sol[i] = infBound[i];
                }
                if (sol[i] > supBound[i])
                {
                    sol[i] = supBound[i];
                }
            }

            // Re-evaluate function at new solution to check F convergence
            if (!func.Value(sol, f))
            {
                result.Status = Status.NumericalError;
                result.NbIterations = iter + 1;
                return result;
            }

            // Check F convergence with updated function values
            bool fConverged = true;
            for (int i = lower; i <= upper; ++i)
            {
                if (Math.Abs(f[i]) > tolF)
                {
                    fConverged = false;
                    break;
                }
            }

            if (xConverged && fConverged)
            {
                // OCCT: the Jacobian returned is the one evaluated before the last step, not at the returned Solution.
                result.Status = Status.OK;
                result.NbIterations = iter + 1;
                result.Solution = sol;
                result.Jacobian = jacobian;
                return result;
            }

            result.NbIterations = iter + 1;
        }

        // Max iterations reached
        result.Status = Status.MaxIterations;
        result.Solution = sol;
        result.Jacobian = jacobian;
        return result;
    }

    /// <summary>Simplified Newton method with a uniform X tolerance for all variables.</summary>
    public static VectorResult Newton<FuncSetType>(FuncSetType func,
                                                   math_Vector start,
                                                   double tolX,
                                                   double tolF,
                                                   int maxIter = 100)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        math_Vector tolXVec = new(start.Lower(), start.Upper(), tolX);
        return Newton(func, start, tolXVec, tolF, maxIter);
    }
}
