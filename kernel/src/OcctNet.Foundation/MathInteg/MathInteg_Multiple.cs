// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_Multiple.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

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

        public MultipleConfig(MultipleConfig theOther)
        {
            MaxOrder = theOther.MaxOrder;
        }
    }

    /// <summary>
    /// Gauss-Legendre integration of a function of theNVars variables over the box [theLower, theUpper], with
    /// theOrder points per variable (clamped to [1, MaxOrder]): tensor product of 1D rules.
    /// </summary>
    public static IntegResult GaussMultiple<Func>(Func theFunc,
                                                  int theNVars,
                                                  math_Vector theLower,
                                                  math_Vector theUpper,
                                                  math_IntegerVector theOrder,
                                                  MultipleConfig? theConfig = null)
        where Func : IVectorFunction
    {
        theConfig ??= new MultipleConfig();
        IntegResult aResult = new();

        // Validate inputs
        if (theNVars <= 0 || theLower.Length() != theNVars || theUpper.Length() != theNVars
            || theOrder.Length() != theNVars)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aLowerL = theLower.Lower();
        int aLowerU = theUpper.Lower();
        int aLowerOr = theOrder.Lower();

        // Find maximum order and clamp orders
        math_IntegerVector aOrd = new(0, theNVars - 1);
        int aMaxOrder = 0;
        for (int i = 0; i < theNVars; ++i)
        {
            aOrd[i] = Min(theOrder[i + aLowerOr], theConfig.MaxOrder);
            aOrd[i] = Max(aOrd[i], 1);
            if (aOrd[i] > aMaxOrder)
            {
                aMaxOrder = aOrd[i];
            }
        }

        // Compute midpoints and half-widths for coordinate transformation
        math_Vector aXm = new(0, theNVars - 1);
        math_Vector aXr = new(0, theNVars - 1);
        for (int i = 0; i < theNVars; ++i)
        {
            aXm[i] = 0.5 * (theLower[i + aLowerL] + theUpper[i + aLowerU]);
            aXr[i] = 0.5 * (theUpper[i + aLowerU] - theLower[i + aLowerL]);
        }

        // Get Gauss points and weights for each variable
        NCollection_DynamicArray<math_Vector> aGaussPoints = new();
        NCollection_DynamicArray<math_Vector> aGaussWeights = new();

        for (int i = 0; i < theNVars; ++i)
        {
            aGaussPoints.Append(new math_Vector(0, aOrd[i] - 1));
            aGaussWeights.Append(new math_Vector(0, aOrd[i] - 1));

            math_Vector aGP = new(1, aOrd[i]);
            math_Vector aGW = new(1, aOrd[i]);
            if (!GetOrderedGaussPointsAndWeights(aOrd[i], aGP, aGW))
            {
                aResult.Status = Status.InvalidInput;
                return aResult;
            }

            for (int k = 0; k < aOrd[i]; ++k)
            {
                aGaussPoints.ChangeValue(i)[k] = aGP[k + 1];
                aGaussWeights.ChangeValue(i)[k] = aGW[k + 1];
            }
        }

        // Recursive integration
        double aVal = 0.0;
        math_Vector aX = new(1, theNVars);
        math_Vector aDx = new(1, theNVars);

        // Index array for iteration
        math_IntegerVector aInc = new(0, theNVars - 1, 0);

        // C++'s recursive std::function lambda is a recursive local function
        bool aRecurse(int theN)
        {
            if (theN == theNVars)
            {
                // Compute function value at current Gauss point
                for (int j = 0; j < theNVars; ++j)
                {
                    aDx[j + 1] = aXr[j] * aGaussPoints.Value(j)[aInc[j]];
                    aX[j + 1] = aXm[j] + aDx[j + 1];
                }

                if (!theFunc.Value(aX, out double aF1))
                {
                    return false;
                }

                // Compute product of weights
                double aWeight = 1.0;
                for (int j = 0; j < theNVars; ++j)
                {
                    aWeight *= aGaussWeights.Value(j)[aInc[j]];
                }

                aVal += aWeight * aF1;
                return true;
            }

            // Iterate over Gauss points for variable theN
            for (aInc[theN] = 0; aInc[theN] < aOrd[theN]; ++aInc[theN])
            {
                if (!aRecurse(theN + 1))
                {
                    return false;
                }
            }
            return true;
        }

        if (!aRecurse(0))
        {
            aResult.Status = Status.NotConverged;
            return aResult;
        }

        // Scale by half-widths
        for (int i = 0; i < theNVars; ++i)
        {
            aVal *= aXr[i];
        }

        aResult.Value = aVal;
        aResult.Status = Status.OK;
        return aResult;
    }

    /// <summary>Gauss-Legendre multiple integration with the same order for all variables.</summary>
    public static IntegResult GaussMultipleUniform<Func>(Func theFunc, int theNVars, math_Vector theLower, math_Vector theUpper, int theOrder)
        where Func : IVectorFunction
    {
        math_IntegerVector aOrders = new(0, theNVars - 1, theOrder);
        return GaussMultiple(theFunc, theNVars, theLower, theUpper, aOrders);
    }
}
