// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_Gauss.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

// C++ code names MathUtils' types through MathInteg (MathInteg::IntegResult, MathInteg::IntegConfig,
// MathInteg::Status) by `using namespace MathUtils`; C# can't re-export them, so callers write MathUtils.IntegResult.

/// <summary>Numerical integration (quadrature): Gauss-Legendre, Gauss-Kronrod and double exponential rules.</summary>
public static partial class MathInteg
{
    /// <summary>Fixed-order Gauss-Legendre integration of theFunc over [theLower, theUpper] (theNbPoints &gt;= 1).</summary>
    public static IntegResult Gauss<Function>(Function theFunc, double theLower, double theUpper, int theNbPoints = 15)
        where Function : IScalarFunction
    {
        IntegResult aResult = new();
        if (theNbPoints < 1)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        math_Vector aPoints = new(1, theNbPoints);
        math_Vector aWeights = new(1, theNbPoints);

        if (!MathUtils.GetGaussPointsAndWeights(theNbPoints, aPoints, aWeights))
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        double aHalfLen = 0.5 * (theUpper - theLower);
        double aMid = 0.5 * (theUpper + theLower);

        double aSum = 0.0;
        for (int i = 1; i <= theNbPoints; ++i)
        {
            double aX = aMid + aHalfLen * aPoints[i];
            if (!theFunc.Value(aX, out double aF))
            {
                aResult.Status = Status.NumericalError;
                return aResult;
            }
            aSum += aWeights[i] * aF;
        }

        aResult.Status = Status.OK;
        aResult.Value = aHalfLen * aSum;
        aResult.NbPoints = theNbPoints;
        aResult.NbIterations = 1;
        return aResult;
    }

    /// <summary>
    /// Adaptive Gauss-Legendre integration: compares two orders (InitialOrder and min(MaxOrder, 2*InitialOrder))
    /// and bisects recursively, up to MaxIterations levels, until they agree within Tolerance.
    /// </summary>
    public static IntegResult GaussAdaptive<Function>(Function theFunc, double theLower, double theUpper, IntegConfig? theConfig = null)
        where Function : IScalarFunction
    {
        theConfig ??= new IntegConfig();
        IntegResult aResult = new();

        if (theConfig.InitialOrder < 1 || theConfig.MaxOrder < theConfig.InitialOrder
            || theConfig.MaxOrder > 61 || theConfig.MaxIterations < 1)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aCoarseOrder = theConfig.InitialOrder;
        int aFineOrder = Min(theConfig.MaxOrder, Min(61, 2 * aCoarseOrder));
        if (aFineOrder == aCoarseOrder)
        {
            if (aCoarseOrder > 1)
            {
                aCoarseOrder -= 1;
            }
            else if (theConfig.MaxOrder > 1)
            {
                aFineOrder = 2;
            }
        }

        IntegResult aCoarse = Gauss(theFunc, theLower, theUpper, aCoarseOrder);
        if (!aCoarse.IsDone())
        {
            return aCoarse;
        }

        IntegResult aFine = Gauss(theFunc, theLower, theUpper, aFineOrder);
        if (!aFine.IsDone())
        {
            return aFine;
        }

        double aError = Math.Abs(aFine.Value!.Value - aCoarse.Value!.Value);
        double aScale = Max(Math.Abs(aFine.Value.Value), 1.0e-15);

        if (aError < theConfig.Tolerance * aScale)
        {
            aResult.Status = Status.OK;
            aResult.Value = aFine.Value.Value;
            aResult.AbsoluteError = aError;
            aResult.RelativeError = aError / aScale;
            aResult.NbPoints = aFineOrder;
            aResult.NbIterations = 1;
            return aResult;
        }

        if (theConfig.MaxIterations <= 1)
        {
            aResult.Status = Status.MaxIterations;
            aResult.Value = aFine.Value.Value;
            aResult.AbsoluteError = aError;
            aResult.RelativeError = aError / aScale;
            aResult.NbPoints = aFineOrder;
            aResult.NbIterations = 1;
            return aResult;
        }

        double aMid = 0.5 * (theLower + theUpper);

        IntegConfig aSubConfig = new(theConfig);
        aSubConfig.MaxIterations = theConfig.MaxIterations - 1;

        IntegResult aLeft = GaussAdaptive(theFunc, theLower, aMid, aSubConfig);
        if (!aLeft.IsDone())
        {
            aResult.Status = aLeft.Status;
            aResult.Value = aLeft.Value;
            return aResult;
        }

        IntegResult aRight = GaussAdaptive(theFunc, aMid, theUpper, aSubConfig);
        if (!aRight.IsDone())
        {
            aResult.Status = aRight.Status;
            aResult.Value = aLeft.Value!.Value + (aRight.Value.HasValue ? aRight.Value.Value : 0.0);
            return aResult;
        }

        aResult.Status = Status.OK;
        aResult.Value = aLeft.Value!.Value + aRight.Value!.Value;
        aResult.AbsoluteError = aLeft.AbsoluteError!.Value + aRight.AbsoluteError!.Value;
        aResult.RelativeError = aResult.AbsoluteError.Value / Max(Math.Abs(aResult.Value.Value), 1.0e-15);
        aResult.NbPoints = aLeft.NbPoints + aRight.NbPoints;
        aResult.NbIterations = Max(aLeft.NbIterations, aRight.NbIterations) + 1;
        return aResult;
    }

    /// <summary>Composite Gauss-Legendre rule: theNbIntervals equal subintervals, theNbPoints points each.</summary>
    public static IntegResult GaussComposite<Function>(Function theFunc, double theLower, double theUpper, int theNbIntervals, int theNbPoints = 7)
        where Function : IScalarFunction
    {
        IntegResult aResult = new();

        if (theNbIntervals < 1)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        double aH = (theUpper - theLower) / theNbIntervals;
        double aSum = 0.0;
        int aTotalPoints = 0;

        for (int i = 0; i < theNbIntervals; ++i)
        {
            double aA = theLower + i * aH;
            double aB = aA + aH;

            IntegResult aSubResult = Gauss(theFunc, aA, aB, theNbPoints);
            if (!aSubResult.IsDone())
            {
                aResult.Status = aSubResult.Status;
                aResult.Value = aSum;
                aResult.NbPoints = aTotalPoints;
                return aResult;
            }

            aSum += aSubResult.Value!.Value;
            aTotalPoints += aSubResult.NbPoints;
        }

        aResult.Status = Status.OK;
        aResult.Value = aSum;
        aResult.NbPoints = aTotalPoints;
        aResult.NbIterations = 1;
        return aResult;
    }
}
