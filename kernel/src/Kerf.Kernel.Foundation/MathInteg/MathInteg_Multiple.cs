// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_Multiple.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Integration;

/// <summary>Multi-dimensional Gauss-Legendre integration.</summary>
public static partial class MathInteg
{
    /// <summary>Configuration for multi-dimensional Gauss integration.</summary>
    public class MultipleConfig
    {
        /// <summary>Maximum integration order per dimension.</summary>
        public int MaxOrder = 61;

        public MultipleConfig()
        {
        }

        public MultipleConfig(MultipleConfig other)
        {
            MaxOrder = other.MaxOrder;
        }
    }

    /// <summary>
    /// Gauss-Legendre integration of a function of theNVars variables over the box [theLower, theUpper], with
    /// theOrder points per variable (clamped to [1, MaxOrder]): tensor product of 1D rules.
    /// </summary>
    public static IntegResult GaussMultiple<Func>(Func func,
                                                  int nVars,
                                                  math_Vector lower,
                                                  math_Vector upper,
                                                  math_IntegerVector order,
                                                  MultipleConfig? config = null)
        where Func : IVectorFunction
    {
        config ??= new MultipleConfig();
        IntegResult result = new();

        // Validate inputs
        if (nVars <= 0 || lower.Length() != nVars || upper.Length() != nVars
            || order.Length() != nVars)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int lowerL = lower.Lower();
        int lowerU = upper.Lower();
        int lowerOr = order.Lower();

        // Find maximum order and clamp orders
        math_IntegerVector ord = new(0, nVars - 1);
        int maxOrder = 0;
        for (int i = 0; i < nVars; ++i)
        {
            ord[i] = Min(order[i + lowerOr], config.MaxOrder);
            ord[i] = Max(ord[i], 1);
            if (ord[i] > maxOrder)
            {
                maxOrder = ord[i];
            }
        }

        // Compute midpoints and half-widths for coordinate transformation
        math_Vector xm = new(0, nVars - 1);
        math_Vector xr = new(0, nVars - 1);
        for (int i = 0; i < nVars; ++i)
        {
            xm[i] = 0.5 * (lower[i + lowerL] + upper[i + lowerU]);
            xr[i] = 0.5 * (upper[i + lowerU] - lower[i + lowerL]);
        }

        // Get Gauss points and weights for each variable
        NCollection_DynamicArray<math_Vector> gaussPoints = new();
        NCollection_DynamicArray<math_Vector> gaussWeights = new();

        for (int i = 0; i < nVars; ++i)
        {
            gaussPoints.Append(new math_Vector(0, ord[i] - 1));
            gaussWeights.Append(new math_Vector(0, ord[i] - 1));

            math_Vector gp = new(1, ord[i]);
            math_Vector gw = new(1, ord[i]);
            if (!GetOrderedGaussPointsAndWeights(ord[i], gp, gw))
            {
                result.Status = Status.InvalidInput;
                return result;
            }

            for (int k = 0; k < ord[i]; ++k)
            {
                gaussPoints.ChangeValue(i)[k] = gp[k + 1];
                gaussWeights.ChangeValue(i)[k] = gw[k + 1];
            }
        }

        // Recursive integration
        double val = 0.0;
        math_Vector x = new(1, nVars);
        math_Vector dx = new(1, nVars);

        // Index array for iteration
        math_IntegerVector inc = new(0, nVars - 1, 0);

        // C++'s recursive std::function lambda is a recursive local function
        bool aRecurse(int n)
        {
            if (n == nVars)
            {
                // Compute function value at current Gauss point
                for (int j = 0; j < nVars; ++j)
                {
                    dx[j + 1] = xr[j] * gaussPoints.Value(j)[inc[j]];
                    x[j + 1] = xm[j] + dx[j + 1];
                }

                if (!func.Value(x, out double f1))
                {
                    return false;
                }

                // Compute product of weights
                double weight = 1.0;
                for (int j = 0; j < nVars; ++j)
                {
                    weight *= gaussWeights.Value(j)[inc[j]];
                }

                val += weight * f1;
                return true;
            }

            // Iterate over Gauss points for variable theN
            for (inc[n] = 0; inc[n] < ord[n]; ++inc[n])
            {
                if (!aRecurse(n + 1))
                {
                    return false;
                }
            }
            return true;
        }

        if (!aRecurse(0))
        {
            result.Status = Status.NotConverged;
            return result;
        }

        // Scale by half-widths
        for (int i = 0; i < nVars; ++i)
        {
            val *= xr[i];
        }

        result.Value = val;
        result.Status = Status.OK;
        return result;
    }

    /// <summary>Gauss-Legendre multiple integration with the same order for all variables.</summary>
    public static IntegResult GaussMultipleUniform<Func>(Func func, int nVars, math_Vector lower, math_Vector upper, int order)
        where Func : IVectorFunction
    {
        math_IntegerVector orders = new(0, nVars - 1, order);
        return GaussMultiple(func, nVars, lower, upper, orders);
    }
}
