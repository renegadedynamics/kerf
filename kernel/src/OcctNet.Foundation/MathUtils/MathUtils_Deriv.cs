// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Deriv.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The derivative outputs are written only on success and left unchanged on failure, as in C++, so they are `ref`.

/// <summary>Numerical differentiation utilities.</summary>
public static partial class MathUtils
{
    /// <summary>Central difference derivative: f'(x) ~= (f(x+h) - f(x-h)) / (2h), O(h^2).</summary>
    public static bool CentralDifference<Function>(Function theFunc, double theX, ref double theDeriv, double theStep = 1.0e-8)
        where Function : IScalarFunction
    {
        if (!theFunc.Value(theX + theStep, out double aFPlus))
        {
            return false;
        }
        if (!theFunc.Value(theX - theStep, out double aFMinus))
        {
            return false;
        }

        theDeriv = (aFPlus - aFMinus) / (2.0 * theStep);
        return true;
    }

    /// <summary>Forward difference derivative: f'(x) ~= (f(x+h) - f(x)) / h, O(h).</summary>
    public static bool ForwardDifference<Function>(Function theFunc, double theX, double theFx, ref double theDeriv, double theStep = 1.0e-8)
        where Function : IScalarFunction
    {
        if (!theFunc.Value(theX + theStep, out double aFPlus))
        {
            return false;
        }

        theDeriv = (aFPlus - theFx) / theStep;
        return true;
    }

    /// <summary>Numerical gradient using central differences (theX is temporarily modified, then restored).</summary>
    public static bool NumericalGradient<Function>(Function theFunc, math_Vector theX, math_Vector theGrad, double theStep = 1.0e-8)
        where Function : IVectorFunction
    {
        int aLower = theX.Lower();
        int aUpper = theX.Upper();

        for (int i = aLower; i <= aUpper; ++i)
        {
            double aXi = theX[i];

            // Forward perturbation
            theX[i] = aXi + theStep;
            if (!theFunc.Value(theX, out double aFPlus))
            {
                theX[i] = aXi;
                return false;
            }

            // Backward perturbation
            theX[i] = aXi - theStep;
            if (!theFunc.Value(theX, out double aFMinus))
            {
                theX[i] = aXi;
                return false;
            }

            // Restore original value
            theX[i] = aXi;

            // Central difference
            theGrad[i] = (aFPlus - aFMinus) / (2.0 * theStep);
        }

        return true;
    }

    /// <summary>Numerical gradient with a step proportional to |x[i]| (at least theRelStep).</summary>
    public static bool NumericalGradientAdaptive<Function>(Function theFunc, math_Vector theX, math_Vector theGrad, double theRelStep = 1.0e-8)
        where Function : IVectorFunction
    {
        int aLower = theX.Lower();
        int aUpper = theX.Upper();

        for (int i = aLower; i <= aUpper; ++i)
        {
            double aXi = theX[i];
            // Adaptive step: larger for larger |x|, with minimum floor
            double aStep = theRelStep * Max(1.0, Math.Abs(aXi));

            theX[i] = aXi + aStep;
            if (!theFunc.Value(theX, out double aFPlus))
            {
                theX[i] = aXi;
                return false;
            }

            theX[i] = aXi - aStep;
            if (!theFunc.Value(theX, out double aFMinus))
            {
                theX[i] = aXi;
                return false;
            }

            theX[i] = aXi;
            theGrad[i] = (aFPlus - aFMinus) / (2.0 * aStep);
        }

        return true;
    }

    /// <summary>Numerical Jacobian (m x n) of a vector-valued function by central differences.</summary>
    public static bool NumericalJacobian<Function>(Function theFunc, math_Vector theX, math_Matrix theJac, double theStep = 1.0e-8)
        where Function : ISystemFunction
    {
        int aNbRows = theJac.RowNumber();
        int aNbCols = theJac.ColNumber();

        math_Vector aFPlus = new(1, aNbRows);
        math_Vector aFMinus = new(1, aNbRows);

        for (int j = 1; j <= aNbCols; ++j)
        {
            int aIdx = theX.Lower() + j - 1;
            double aXj = theX[aIdx];

            // Forward perturbation
            theX[aIdx] = aXj + theStep;
            if (!theFunc.Value(theX, aFPlus))
            {
                theX[aIdx] = aXj;
                return false;
            }

            // Backward perturbation
            theX[aIdx] = aXj - theStep;
            if (!theFunc.Value(theX, aFMinus))
            {
                theX[aIdx] = aXj;
                return false;
            }

            // Restore
            theX[aIdx] = aXj;

            // Fill column of Jacobian
            for (int i = 1; i <= aNbRows; ++i)
            {
                theJac[i, j] = (aFPlus[i] - aFMinus[i]) / (2.0 * theStep);
            }
        }

        return true;
    }

    /// <summary>Numerical Hessian (symmetric) by central differences.</summary>
    public static bool NumericalHessian<Function>(Function theFunc, math_Vector theX, math_Matrix theHess, double theStep = 1.0e-5)
        where Function : IVectorFunction
    {
        int aLower = theX.Lower();
        int aUpper = theX.Upper();

        if (!theFunc.Value(theX, out double aFx))
        {
            return false;
        }

        // Diagonal elements: d^2f/dx[i]^2 ~= (f(x+h[i]*e[i]) - 2f(x) + f(x-h[i]*e[i])) / h^2
        for (int i = aLower; i <= aUpper; ++i)
        {
            double aXi = theX[i];

            theX[i] = aXi + theStep;
            if (!theFunc.Value(theX, out double aFPlus))
            {
                theX[i] = aXi;
                return false;
            }

            theX[i] = aXi - theStep;
            if (!theFunc.Value(theX, out double aFMinus))
            {
                theX[i] = aXi;
                return false;
            }

            theX[i] = aXi;

            int aMatIdx = i - aLower + 1;
            theHess[aMatIdx, aMatIdx] = (aFPlus - 2.0 * aFx + aFMinus) / (theStep * theStep);
        }

        // Off-diagonal elements: d^2f/dx[i]dx[j]
        // ~= (f(x+h[i]*e[i]+h[j]*e[j]) - f(x+h[i]*e[i]-h[j]*e[j])
        //    - f(x-h[i]*e[i]+h[j]*e[j]) + f(x-h[i]*e[i]-h[j]*e[j])) / (4h^2)
        for (int i = aLower; i <= aUpper; ++i)
        {
            for (int j = i + 1; j <= aUpper; ++j)
            {
                double aXi = theX[i];
                double aXj = theX[j];

                // f(x + h[i]*e[i] + h[j]*e[j])
                theX[i] = aXi + theStep;
                theX[j] = aXj + theStep;
                if (!theFunc.Value(theX, out double aFpp))
                {
                    theX[i] = aXi;
                    theX[j] = aXj;
                    return false;
                }

                // f(x + h[i]*e[i] - h[j]*e[j])
                theX[j] = aXj - theStep;
                if (!theFunc.Value(theX, out double aFpm))
                {
                    theX[i] = aXi;
                    theX[j] = aXj;
                    return false;
                }

                // f(x - h[i]*e[i] - h[j]*e[j])
                theX[i] = aXi - theStep;
                if (!theFunc.Value(theX, out double aFmm))
                {
                    theX[i] = aXi;
                    theX[j] = aXj;
                    return false;
                }

                // f(x - h[i]*e[i] + h[j]*e[j])
                theX[j] = aXj + theStep;
                if (!theFunc.Value(theX, out double aFmp))
                {
                    theX[i] = aXi;
                    theX[j] = aXj;
                    return false;
                }

                // Restore
                theX[i] = aXi;
                theX[j] = aXj;

                int aMatI = i - aLower + 1;
                int aMatJ = j - aLower + 1;
                double aHij = (aFpp - aFpm - aFmp + aFmm) / (4.0 * theStep * theStep);

                // Symmetric
                theHess[aMatI, aMatJ] = aHij;
                theHess[aMatJ, aMatI] = aHij;
            }
        }

        return true;
    }

    /// <summary>Second derivative by central difference: f''(x) ~= (f(x+h) - 2f(x) + f(x-h)) / h^2.</summary>
    public static bool SecondDerivative<Function>(Function theFunc, double theX, double theFx, ref double theD2f, double theStep = 1.0e-5)
        where Function : IScalarFunction
    {
        if (!theFunc.Value(theX + theStep, out double aFPlus))
        {
            return false;
        }
        if (!theFunc.Value(theX - theStep, out double aFMinus))
        {
            return false;
        }

        theD2f = (aFPlus - 2.0 * theFx + aFMinus) / (theStep * theStep);
        return true;
    }
}
