// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Deriv.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Utilities;

// The derivative outputs are written only on success and left unchanged on failure, as in C++, so they are `ref`.

/// <summary>Numerical differentiation utilities.</summary>
internal static partial class MathUtils
{
    /// <summary>Central difference derivative: f'(x) ~= (f(x+h) - f(x-h)) / (2h), O(h^2).</summary>
    public static bool CentralDifference<Function>(Function func, double x, ref double deriv, double step = 1.0e-8)
        where Function : IScalarFunction
    {
        if (!func.Value(x + step, out double fPlus))
        {
            return false;
        }
        if (!func.Value(x - step, out double fMinus))
        {
            return false;
        }

        deriv = (fPlus - fMinus) / (2.0 * step);
        return true;
    }

    /// <summary>Forward difference derivative: f'(x) ~= (f(x+h) - f(x)) / h, O(h).</summary>
    public static bool ForwardDifference<Function>(Function func, double x, double fx, ref double deriv, double step = 1.0e-8)
        where Function : IScalarFunction
    {
        if (!func.Value(x + step, out double fPlus))
        {
            return false;
        }

        deriv = (fPlus - fx) / step;
        return true;
    }

    /// <summary>Numerical gradient using central differences (theX is temporarily modified, then restored).</summary>
    public static bool NumericalGradient<Function>(Function func, math_Vector x, math_Vector grad, double step = 1.0e-8)
        where Function : IVectorFunction
    {
        int lower = x.Lower();
        int upper = x.Upper();

        for (int i = lower; i <= upper; ++i)
        {
            double xi = x[i];

            // Forward perturbation
            x[i] = xi + step;
            if (!func.Value(x, out double fPlus))
            {
                x[i] = xi;
                return false;
            }

            // Backward perturbation
            x[i] = xi - step;
            if (!func.Value(x, out double fMinus))
            {
                x[i] = xi;
                return false;
            }

            // Restore original value
            x[i] = xi;

            // Central difference
            grad[i] = (fPlus - fMinus) / (2.0 * step);
        }

        return true;
    }

    /// <summary>Numerical gradient with a step proportional to |x[i]| (at least theRelStep).</summary>
    public static bool NumericalGradientAdaptive<Function>(Function func, math_Vector x, math_Vector grad, double relStep = 1.0e-8)
        where Function : IVectorFunction
    {
        int lower = x.Lower();
        int upper = x.Upper();

        for (int i = lower; i <= upper; ++i)
        {
            double xi = x[i];
            // Adaptive step: larger for larger |x|, with minimum floor
            double step = relStep * Max(1.0, Math.Abs(xi));

            x[i] = xi + step;
            if (!func.Value(x, out double fPlus))
            {
                x[i] = xi;
                return false;
            }

            x[i] = xi - step;
            if (!func.Value(x, out double fMinus))
            {
                x[i] = xi;
                return false;
            }

            x[i] = xi;
            grad[i] = (fPlus - fMinus) / (2.0 * step);
        }

        return true;
    }

    /// <summary>Numerical Jacobian (m x n) of a vector-valued function by central differences.</summary>
    public static bool NumericalJacobian<Function>(Function func, math_Vector x, Matrix jac, double step = 1.0e-8)
        where Function : ISystemFunction
    {
        int nbRows = jac.RowNumber();
        int nbCols = jac.ColNumber();

        math_Vector fPlus = new(1, nbRows);
        math_Vector fMinus = new(1, nbRows);

        for (int j = 1; j <= nbCols; ++j)
        {
            int idx = x.Lower() + j - 1;
            double xj = x[idx];

            // Forward perturbation
            x[idx] = xj + step;
            if (!func.Value(x, fPlus))
            {
                x[idx] = xj;
                return false;
            }

            // Backward perturbation
            x[idx] = xj - step;
            if (!func.Value(x, fMinus))
            {
                x[idx] = xj;
                return false;
            }

            // Restore
            x[idx] = xj;

            // Fill column of Jacobian
            for (int i = 1; i <= nbRows; ++i)
            {
                jac[i, j] = (fPlus[i] - fMinus[i]) / (2.0 * step);
            }
        }

        return true;
    }

    /// <summary>Numerical Hessian (symmetric) by central differences.</summary>
    public static bool NumericalHessian<Function>(Function func, math_Vector x, Matrix hess, double step = 1.0e-5)
        where Function : IVectorFunction
    {
        int lower = x.Lower();
        int upper = x.Upper();

        if (!func.Value(x, out double fx))
        {
            return false;
        }

        // Diagonal elements: d^2f/dx[i]^2 ~= (f(x+h[i]*e[i]) - 2f(x) + f(x-h[i]*e[i])) / h^2
        for (int i = lower; i <= upper; ++i)
        {
            double xi = x[i];

            x[i] = xi + step;
            if (!func.Value(x, out double fPlus))
            {
                x[i] = xi;
                return false;
            }

            x[i] = xi - step;
            if (!func.Value(x, out double fMinus))
            {
                x[i] = xi;
                return false;
            }

            x[i] = xi;

            int matIdx = i - lower + 1;
            hess[matIdx, matIdx] = (fPlus - 2.0 * fx + fMinus) / (step * step);
        }

        // Off-diagonal elements: d^2f/dx[i]dx[j]
        // ~= (f(x+h[i]*e[i]+h[j]*e[j]) - f(x+h[i]*e[i]-h[j]*e[j])
        //    - f(x-h[i]*e[i]+h[j]*e[j]) + f(x-h[i]*e[i]-h[j]*e[j])) / (4h^2)
        for (int i = lower; i <= upper; ++i)
        {
            for (int j = i + 1; j <= upper; ++j)
            {
                double xi = x[i];
                double xj = x[j];

                // f(x + h[i]*e[i] + h[j]*e[j])
                x[i] = xi + step;
                x[j] = xj + step;
                if (!func.Value(x, out double fpp))
                {
                    x[i] = xi;
                    x[j] = xj;
                    return false;
                }

                // f(x + h[i]*e[i] - h[j]*e[j])
                x[j] = xj - step;
                if (!func.Value(x, out double fpm))
                {
                    x[i] = xi;
                    x[j] = xj;
                    return false;
                }

                // f(x - h[i]*e[i] - h[j]*e[j])
                x[i] = xi - step;
                if (!func.Value(x, out double fmm))
                {
                    x[i] = xi;
                    x[j] = xj;
                    return false;
                }

                // f(x - h[i]*e[i] + h[j]*e[j])
                x[j] = xj + step;
                if (!func.Value(x, out double fmp))
                {
                    x[i] = xi;
                    x[j] = xj;
                    return false;
                }

                // Restore
                x[i] = xi;
                x[j] = xj;

                int matI = i - lower + 1;
                int matJ = j - lower + 1;
                double hij = (fpp - fpm - fmp + fmm) / (4.0 * step * step);

                // Symmetric
                hess[matI, matJ] = hij;
                hess[matJ, matI] = hij;
            }
        }

        return true;
    }

    /// <summary>Second derivative by central difference: f''(x) ~= (f(x+h) - 2f(x) + f(x-h)) / h^2.</summary>
    public static bool SecondDerivative<Function>(Function func, double x, double fx, ref double d2f, double step = 1.0e-5)
        where Function : IScalarFunction
    {
        if (!func.Value(x + step, out double fPlus))
        {
            return false;
        }
        if (!func.Value(x - step, out double fMinus))
        {
            return false;
        }

        d2f = (fPlus - 2.0 * fx + fMinus) / (step * step);
        return true;
    }
}
