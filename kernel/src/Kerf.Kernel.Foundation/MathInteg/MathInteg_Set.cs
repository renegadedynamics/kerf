// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_Set.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Integration;

/// <summary>Integration of vector-valued functions.</summary>
internal static partial class MathInteg
{
    /// <summary>
    /// A vector-valued function F: R -> R^N for GaussSet: C++'s "type with <c>int NbEquations()</c> and
    /// <c>bool Value(const math_Vector&amp; theX, math_Vector&amp; theF)</c>". theX has one value.
    /// </summary>
    public interface ISetFunction : ISystemFunction
    {
        /// <summary>Number of output components.</summary>
        int NbEquations();
    }

    /// <summary>Result for vector function integration.</summary>
    public struct SetResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Integral of each component.</summary>
        public math_Vector? Values;
        public int NbEquations = 0;

        public SetResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(SetResult result) => result.IsDone;

        public static bool operator true(SetResult result) => result.IsDone;

        public static bool operator false(SetResult result) => !result.IsDone;
    }

    /// <summary>
    /// Gauss-Legendre integration of a vector-valued function over [theLower, theUpper]: Result[i] is the integral of
    /// F_i. theOrder is clamped to [1, 61]. Only 1D input is supported, as in the legacy API.
    /// </summary>
    public static SetResult GaussSet<Func>(Func func, double lower, double upper, int order)
        where Func : ISetFunction
    {
        SetResult result = new();

        // Get function dimensions
        int nbEqua = func.NbEquations();
        if (nbEqua <= 0)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        result.NbEquations = nbEqua;

        // Clamp order
        int aOrder = Min(order, 61);
        aOrder = Max(aOrder, 1);

        // Get Gauss points and weights
        math_Vector gp = new(1, aOrder);
        math_Vector gw = new(1, aOrder);
        if (!GetOrderedGaussPointsAndWeights(aOrder, gp, gw))
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        math_Vector points = new(0, aOrder - 1);
        math_Vector weights = new(0, aOrder - 1);
        for (int i = 0; i < aOrder; ++i)
        {
            points[i] = gp[i + 1];
            weights[i] = gw[i + 1];
        }

        // Coordinate transformation
        double xm = 0.5 * (lower + upper);
        double xr = 0.5 * (upper - lower);

        // Initialize result vector
        math_Vector val = new(1, nbEqua, 0.0);
        math_Vector tval = new(1, 1); // Input vector (1D)
        math_Vector fVal1 = new(1, nbEqua);
        math_Vector fVal2 = new(1, nbEqua);

        int ind = aOrder / 2;
        int ind1 = (aOrder + 1) / 2;

        // Handle odd order case (middle point)
        if (ind1 > ind)
        {
            tval[1] = xm;
            if (!func.Value(tval, val))
            {
                result.Status = Status.NotConverged;
                return result;
            }
            for (int j = 1; j <= nbEqua; ++j)
            {
                val[j] *= weights[ind1 - 1];
            }
        }

        // Symmetric Gauss quadrature
        for (int i = 0; i < ind; ++i)
        {
            tval[1] = xm + xr * points[i];
            if (!func.Value(tval, fVal1))
            {
                result.Status = Status.NotConverged;
                return result;
            }

            tval[1] = xm - xr * points[i];
            if (!func.Value(tval, fVal2))
            {
                result.Status = Status.NotConverged;
                return result;
            }

            for (int j = 1; j <= nbEqua; ++j)
            {
                val[j] += (fVal1[j] + fVal2[j]) * weights[i];
            }
        }

        // Scale by half-width
        for (int j = 1; j <= nbEqua; ++j)
        {
            val[j] *= xr;
        }

        result.Values = val;
        result.Status = Status.OK;
        return result;
    }

    /// <summary>GaussSet with bounds taken from the first values of theLower and theUpper.</summary>
    public static SetResult GaussSet<Func>(Func func, math_Vector lower, math_Vector upper, int order)
        where Func : ISetFunction
    {
        return GaussSet(func, lower[lower.Lower()], upper[upper.Lower()], order);
    }
}
