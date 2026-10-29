// Port of: src/FoundationClasses/TKMath/MathSys/MathSys_Newton.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

// OCCT's MathSys::Newton and LevenbergMarquardt are templates over any "FuncSetType" with the members of
// math_FunctionSetWithDerivatives. Here those members are the interfaces below, with the same signatures as the
// classic abstract classes, so a class can derive from math_FunctionSetWithDerivatives and also implement
// MathSys.IFunctionSetWithDerivatives with the same overrides (as the comparison tests do).

public static partial class MathSys
{
    /// <summary>A system of NbEquations() functions of NbVariables() variables (math_FunctionSet's members).</summary>
    public interface IFunctionSet : MathUtils.ISystemFunction
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
        bool Derivatives(math_Vector theX, math_Matrix theD);

        /// <summary>Computes the values and the Jacobian at theX; false if they could not be computed.</summary>
        bool Values(math_Vector theX, math_Vector theF, math_Matrix theD);
    }

    /// <summary>
    /// Newton-Raphson method for solving systems of nonlinear equations F(X) = 0: solves J(X)*dX = -F(X) at each
    /// step. Converged when every |dX(i)| &lt;= theTolX(i) and every |F(i)| &lt;= theTolF.
    /// </summary>
    public static VectorResult Newton<FuncSetType>(FuncSetType theFunc,
                                                   math_Vector theStart,
                                                   math_Vector theTolX,
                                                   double theTolF,
                                                   int theMaxIter = 100)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        VectorResult aResult = new();

        int aN = theFunc.NbVariables();
        int aM = theFunc.NbEquations();

        // Check dimensions
        if (aN != aM || theStart.Length() != aN || theTolX.Length() != aN)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aLower = theStart.Lower();
        int aUpper = theStart.Upper();

        // Working vectors
        math_Vector aSol = new(theStart);
        math_Vector aF = new(aLower, aUpper);
        math_Vector aDeltaX = new(aLower, aUpper);
        math_Matrix aJacobian = new(aLower, aUpper, aLower, aUpper);

        // Newton iteration
        for (int anIter = 0; anIter < theMaxIter; ++anIter)
        {
            // Evaluate function and Jacobian
            if (!theFunc.Values(aSol, aF, aJacobian))
            {
                aResult.Status = Status.NumericalError;
                aResult.NbIterations = anIter;
                return aResult;
            }

            // Solve J * dX = -F
            // Negate F for the right-hand side
            math_Vector aNegF = new(aLower, aUpper);
            for (int i = aLower; i <= aUpper; ++i)
            {
                aNegF[i] = -aF[i];
            }

            LinearResult aLinResult = MathLin.Solve(aJacobian, aNegF);
            if (!aLinResult.IsDone())
            {
                aResult.Status = Status.Singular;
                aResult.NbIterations = anIter;
                return aResult;
            }

            aDeltaX.Initialized(aLinResult.Solution!);

            // Check X convergence before update
            bool aXConverged = true;
            for (int i = aLower; i <= aUpper; ++i)
            {
                if (Math.Abs(aDeltaX[i]) > theTolX[i])
                {
                    aXConverged = false;
                    break;
                }
            }

            // Update solution
            for (int i = aLower; i <= aUpper; ++i)
            {
                aSol[i] += aDeltaX[i];
            }

            // Re-evaluate function at new solution to check F convergence
            if (!theFunc.Value(aSol, aF))
            {
                aResult.Status = Status.NumericalError;
                aResult.NbIterations = anIter + 1;
                return aResult;
            }

            // Check F convergence with updated function values
            bool aFConverged = true;
            for (int i = aLower; i <= aUpper; ++i)
            {
                if (Math.Abs(aF[i]) > theTolF)
                {
                    aFConverged = false;
                    break;
                }
            }

            if (aXConverged && aFConverged)
            {
                aResult.Status = Status.OK;
                aResult.NbIterations = anIter + 1;
                aResult.Solution = aSol;
                aResult.Jacobian = aJacobian;
                return aResult;
            }

            aResult.NbIterations = anIter + 1;
        }

        // Max iterations reached - return last solution
        aResult.Status = Status.MaxIterations;
        aResult.Solution = aSol;
        aResult.Jacobian = aJacobian;
        return aResult;
    }

    /// <summary>
    /// Newton-Raphson method with bounds: the start and every step are clamped to [theInfBound, theSupBound].
    /// </summary>
    public static VectorResult NewtonBounded<FuncSetType>(FuncSetType theFunc,
                                                          math_Vector theStart,
                                                          math_Vector theInfBound,
                                                          math_Vector theSupBound,
                                                          math_Vector theTolX,
                                                          double theTolF,
                                                          int theMaxIter = 100)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        VectorResult aResult = new();

        int aN = theFunc.NbVariables();
        int aM = theFunc.NbEquations();

        // Check dimensions
        if (aN != aM || theStart.Length() != aN || theTolX.Length() != aN || theInfBound.Length() != aN
            || theSupBound.Length() != aN)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aLower = theStart.Lower();
        int aUpper = theStart.Upper();

        // Working vectors
        math_Vector aSol = new(theStart);
        math_Vector aF = new(aLower, aUpper);
        math_Vector aDeltaX = new(aLower, aUpper);
        math_Matrix aJacobian = new(aLower, aUpper, aLower, aUpper);

        // Clamp initial solution to bounds
        for (int i = aLower; i <= aUpper; ++i)
        {
            if (aSol[i] < theInfBound[i])
            {
                aSol[i] = theInfBound[i];
            }
            if (aSol[i] > theSupBound[i])
            {
                aSol[i] = theSupBound[i];
            }
        }

        // Newton iteration
        for (int anIter = 0; anIter < theMaxIter; ++anIter)
        {
            // Evaluate function and Jacobian
            if (!theFunc.Values(aSol, aF, aJacobian))
            {
                aResult.Status = Status.NumericalError;
                aResult.NbIterations = anIter;
                return aResult;
            }

            // Solve J * dX = -F
            math_Vector aNegF = new(aLower, aUpper);
            for (int i = aLower; i <= aUpper; ++i)
            {
                aNegF[i] = -aF[i];
            }

            LinearResult aLinResult = MathLin.Solve(aJacobian, aNegF);
            if (!aLinResult.IsDone())
            {
                aResult.Status = Status.Singular;
                aResult.NbIterations = anIter;
                return aResult;
            }

            aDeltaX.Initialized(aLinResult.Solution!);

            // Check X convergence before update
            bool aXConverged = true;
            for (int i = aLower; i <= aUpper; ++i)
            {
                if (Math.Abs(aDeltaX[i]) > theTolX[i])
                {
                    aXConverged = false;
                    break;
                }
            }

            // Update solution with bounds clamping
            for (int i = aLower; i <= aUpper; ++i)
            {
                aSol[i] += aDeltaX[i];
                if (aSol[i] < theInfBound[i])
                {
                    aSol[i] = theInfBound[i];
                }
                if (aSol[i] > theSupBound[i])
                {
                    aSol[i] = theSupBound[i];
                }
            }

            // Re-evaluate function at new solution to check F convergence
            if (!theFunc.Value(aSol, aF))
            {
                aResult.Status = Status.NumericalError;
                aResult.NbIterations = anIter + 1;
                return aResult;
            }

            // Check F convergence with updated function values
            bool aFConverged = true;
            for (int i = aLower; i <= aUpper; ++i)
            {
                if (Math.Abs(aF[i]) > theTolF)
                {
                    aFConverged = false;
                    break;
                }
            }

            if (aXConverged && aFConverged)
            {
                aResult.Status = Status.OK;
                aResult.NbIterations = anIter + 1;
                aResult.Solution = aSol;
                aResult.Jacobian = aJacobian;
                return aResult;
            }

            aResult.NbIterations = anIter + 1;
        }

        // Max iterations reached
        aResult.Status = Status.MaxIterations;
        aResult.Solution = aSol;
        aResult.Jacobian = aJacobian;
        return aResult;
    }

    /// <summary>Simplified Newton method with a uniform X tolerance for all variables.</summary>
    public static VectorResult Newton<FuncSetType>(FuncSetType theFunc,
                                                   math_Vector theStart,
                                                   double theTolX,
                                                   double theTolF,
                                                   int theMaxIter = 100)
        where FuncSetType : IFunctionSetWithDerivatives
    {
        math_Vector aTolXVec = new(theStart.Lower(), theStart.Upper(), theTolX);
        return Newton(theFunc, theStart, aTolXVec, theTolF, theMaxIter);
    }
}
