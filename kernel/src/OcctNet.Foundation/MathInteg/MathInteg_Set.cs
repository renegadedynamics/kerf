// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_Set.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Integration of vector-valued functions.</summary>
public static partial class MathInteg
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
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Integral of each component.</summary>
        public math_Vector? Values;
        public int NbEquations = 0;

        public SetResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(SetResult theResult) => theResult.IsDone();

        public static bool operator true(SetResult theResult) => theResult.IsDone();

        public static bool operator false(SetResult theResult) => !theResult.IsDone();
    }

    /// <summary>
    /// Gauss-Legendre integration of a vector-valued function over [theLower, theUpper]: Result[i] is the integral of
    /// F_i. theOrder is clamped to [1, 61]. Only 1D input is supported, as in the legacy API.
    /// </summary>
    public static SetResult GaussSet<Func>(Func theFunc, double theLower, double theUpper, int theOrder)
        where Func : ISetFunction
    {
        SetResult aResult = new();

        // Get function dimensions
        int aNbEqua = theFunc.NbEquations();
        if (aNbEqua <= 0)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        aResult.NbEquations = aNbEqua;

        // Clamp order
        int aOrder = Min(theOrder, 61);
        aOrder = Max(aOrder, 1);

        // Get Gauss points and weights
        math_Vector aGP = new(1, aOrder);
        math_Vector aGW = new(1, aOrder);
        if (!GetOrderedGaussPointsAndWeights(aOrder, aGP, aGW))
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        math_Vector aPoints = new(0, aOrder - 1);
        math_Vector aWeights = new(0, aOrder - 1);
        for (int i = 0; i < aOrder; ++i)
        {
            aPoints[i] = aGP[i + 1];
            aWeights[i] = aGW[i + 1];
        }

        // Coordinate transformation
        double aXm = 0.5 * (theLower + theUpper);
        double aXr = 0.5 * (theUpper - theLower);

        // Initialize result vector
        math_Vector aVal = new(1, aNbEqua, 0.0);
        math_Vector aTval = new(1, 1); // Input vector (1D)
        math_Vector aFVal1 = new(1, aNbEqua);
        math_Vector aFVal2 = new(1, aNbEqua);

        int aInd = aOrder / 2;
        int aInd1 = (aOrder + 1) / 2;

        // Handle odd order case (middle point)
        if (aInd1 > aInd)
        {
            aTval[1] = aXm;
            if (!theFunc.Value(aTval, aVal))
            {
                aResult.Status = Status.NotConverged;
                return aResult;
            }
            for (int j = 1; j <= aNbEqua; ++j)
            {
                aVal[j] *= aWeights[aInd1 - 1];
            }
        }

        // Symmetric Gauss quadrature
        for (int i = 0; i < aInd; ++i)
        {
            aTval[1] = aXm + aXr * aPoints[i];
            if (!theFunc.Value(aTval, aFVal1))
            {
                aResult.Status = Status.NotConverged;
                return aResult;
            }

            aTval[1] = aXm - aXr * aPoints[i];
            if (!theFunc.Value(aTval, aFVal2))
            {
                aResult.Status = Status.NotConverged;
                return aResult;
            }

            for (int j = 1; j <= aNbEqua; ++j)
            {
                aVal[j] += (aFVal1[j] + aFVal2[j]) * aWeights[i];
            }
        }

        // Scale by half-width
        for (int j = 1; j <= aNbEqua; ++j)
        {
            aVal[j] *= aXr;
        }

        aResult.Values = aVal;
        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>GaussSet with bounds taken from the first values of theLower and theUpper.</summary>
    public static SetResult GaussSet<Func>(Func theFunc, math_Vector theLower, math_Vector theUpper, int theOrder)
        where Func : ISetFunction
    {
        return GaussSet(theFunc, theLower[theLower.Lower()], theUpper[theUpper.Lower()], theOrder);
    }
}
