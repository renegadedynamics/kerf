// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Bisection.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public static partial class MathRoot
{
    /// <summary>Bisection method for root finding; f(theLower) and f(theUpper) must have opposite signs.</summary>
    public static MathUtils.ScalarResult Bisection<Function>(Function theFunc, double theLower, double theUpper, MathUtils.Config? theConfig = null)
        where Function : MathUtils.IScalarFunction
    {
        theConfig ??= new MathUtils.Config();
        MathUtils.ScalarResult aResult = new();

        double aA = theLower;
        double aB = theUpper;

        // Evaluate at endpoints
        if (!theFunc.Value(aA, out double aFa))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }
        if (!theFunc.Value(aB, out double aFb))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }

        // Check that bracket is valid (sign change)
        if (aFa * aFb > 0.0)
        {
            aResult.Status = MathUtils.Status.InvalidInput;
            return aResult;
        }

        // Ensure a < b
        if (aA > aB)
        {
            (aA, aB) = (aB, aA);
            (aFa, aFb) = (aFb, aFa);
        }

        // Track which endpoint has the negative value
        // We use separate tracking instead of swapping to maintain aA < aB
        bool aIsNegAtA = (aFa < 0.0);

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            // Compute midpoint
            double aM = 0.5 * (aA + aB);

            if (!theFunc.Value(aM, out double aFm))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                aResult.Root = aM;
                aResult.NbIterations = anIter + 1;
                return aResult;
            }

            aResult.NbIterations = anIter + 1;

            // Check convergence on function value
            if (Math.Abs(aFm) < theConfig.FTolerance)
            {
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aM;
                aResult.Value = aFm;
                return aResult;
            }

            // Check convergence on interval size
            if ((aB - aA) < theConfig.XTolerance * Max(1.0, Math.Abs(aM)))
            {
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aM;
                aResult.Value = aFm;
                return aResult;
            }

            // Update bracket based on sign tracking
            bool aIsNegAtM = (aFm < 0.0);
            if (aIsNegAtM == aIsNegAtA)
            {
                // Same sign as endpoint A, replace A
                aA = aM;
                aFa = aFm;
            }
            else
            {
                // Same sign as endpoint B, replace B
                aB = aM;
                aFb = aFm;
            }
        }

        // Maximum iterations reached
        aResult.Status = MathUtils.Status.MaxIterations;
        aResult.Root = 0.5 * (aA + aB);
        theFunc.Value(aResult.Root.Value, out double aValue);
        aResult.Value = aValue;
        return aResult;
    }

    /// <summary>Bisection safeguarded Newton (Newton steps that stay inside the bracket, bisection otherwise).</summary>
    public static MathUtils.ScalarResult BisectionNewton<Function>(Function theFunc, double theLower, double theUpper, MathUtils.Config? theConfig = null)
        where Function : MathUtils.IScalarFunctionWithDerivative
    {
        theConfig ??= new MathUtils.Config();
        MathUtils.ScalarResult aResult = new();

        double aA = theLower;
        double aB = theUpper;

        // Evaluate at endpoints
        if (!theFunc.Values(aA, out double aFa, out _))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }
        if (!theFunc.Values(aB, out double aFb, out _))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }

        // Check that bracket is valid
        if (aFa * aFb > 0.0)
        {
            aResult.Status = MathUtils.Status.InvalidInput;
            return aResult;
        }

        // Ensure a < b
        if (aA > aB)
        {
            (aA, aB) = (aB, aA);
            (aFa, aFb) = (aFb, aFa);
        }

        // Track which endpoint has the negative value
        bool aIsNegAtA = (aFa < 0.0);

        // Start from midpoint
        double aX = 0.5 * (aA + aB);

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            if (!theFunc.Values(aX, out double aFx, out double aDfx))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                aResult.Root = aX;
                aResult.NbIterations = anIter + 1;
                return aResult;
            }

            aResult.NbIterations = anIter + 1;

            // Check convergence
            if (Math.Abs(aFx) < theConfig.FTolerance)
            {
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aX;
                aResult.Value = aFx;
                aResult.Derivative = aDfx;
                return aResult;
            }

            if ((aB - aA) < theConfig.XTolerance * Max(1.0, Math.Abs(aX)))
            {
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aX;
                aResult.Value = aFx;
                aResult.Derivative = aDfx;
                return aResult;
            }

            // Try Newton step
            double aXNew = aX;
            if (!MathUtils.IsZero(aDfx))
            {
                aXNew = aX - aFx / aDfx;
            }

            // Check if Newton step stays within bracket
            if (aXNew <= aA || aXNew >= aB)
            {
                // Fall back to bisection
                aXNew = 0.5 * (aA + aB);
            }

            // Update bracket based on current point using sign tracking
            bool aIsNegAtX = (aFx < 0.0);
            if (aIsNegAtX == aIsNegAtA)
            {
                // Same sign as endpoint A, replace A
                aA = aX;
            }
            else
            {
                // Same sign as endpoint B, replace B
                aB = aX;
            }

            aX = aXNew;
        }

        // Maximum iterations reached
        aResult.Status = MathUtils.Status.MaxIterations;
        aResult.Root = aX;
        theFunc.Values(aX, out double aValue, out double aDerivative);
        aResult.Value = aValue;
        aResult.Derivative = aDerivative;
        return aResult;
    }
}
