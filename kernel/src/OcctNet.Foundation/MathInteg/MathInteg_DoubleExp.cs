// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_DoubleExp.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

/// <summary>Double exponential (tanh-sinh, exp-sinh, sinh-sinh) quadrature.</summary>
public static partial class MathInteg
{
    /// <summary>Configuration for double exponential integration.</summary>
    public class DoubleExpConfig : IntegConfig
    {
        /// <summary>Number of refinement levels (each halves the step).</summary>
        public int NbLevels = 6;
        /// <summary>Initial step size factor (unused by the algorithms, as in OCCT).</summary>
        public double StepFactor = 0.5;

        public DoubleExpConfig()
        {
        }

        public DoubleExpConfig(double theTolerance, int theMaxIter = 100)
            : base(theTolerance, theMaxIter)
        {
        }

        public DoubleExpConfig(DoubleExpConfig theOther)
            : base(theOther)
        {
            NbLevels = theOther.NbLevels;
            StepFactor = theOther.StepFactor;
        }
    }

    /// <summary>
    /// Tanh-sinh quadrature over the finite interval [theLower, theUpper]; handles endpoint singularities.
    /// Points where the function fails or is not finite are skipped.
    /// </summary>
    public static IntegResult TanhSinh<Function>(Function theFunc, double theLower, double theUpper, DoubleExpConfig? theConfig = null)
        where Function : IScalarFunction
    {
        theConfig ??= new DoubleExpConfig();
        IntegResult aResult = new();

        if (theLower >= theUpper)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        double aHalfPi = M_PI / 2.0;
        double aMid = 0.5 * (theUpper + theLower);
        double aHalf = 0.5 * (theUpper - theLower);

        double aH = 1.0;

        double aPrevSum = 0.0;
        int aTotalPoints = 0;

        for (int aLevel = 0; aLevel < theConfig.NbLevels; ++aLevel)
        {
            double aSum = 0.0;
            int aNbPoints = 0;

            int aStart = (aLevel == 0) ? 0 : 1;
            int aStep = (aLevel == 0) ? 1 : 2;

            for (int k = aStart; ; k += aStep)
            {
                double aT = k * aH;

                double aSinhT = Math.Sinh(aT);
                double aCoshT = Math.Cosh(aT);

                double aU = aHalfPi * aSinhT;

                if (Math.Abs(aU) > 700.0)
                {
                    break; // tanh(u) ~= +/-1, contribution is negligible
                }

                double aTanhU = Math.Tanh(aU);
                double aCoshU = Math.Cosh(aU);
                double aSech2U = 1.0 / (aCoshU * aCoshU);

                double aX = aMid + aHalf * aTanhU;

                if (aX <= theLower + MathUtils.THE_ZERO_TOL || aX >= theUpper - MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                double aWeight = aHalf * aHalfPi * aCoshT * aSech2U;

                if (aWeight < MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                if (!theFunc.Value(aX, out double aF))
                {
                    continue;
                }

                if (!double.IsFinite(aF))
                {
                    continue;
                }

                aSum += aWeight * aF;
                ++aNbPoints;

                if (aNbPoints > 10000)
                {
                    break;
                }
            }

            int aNegStart = (aLevel == 0) ? 1 : aStart; // Start at 1 for level 0, aStart otherwise
            for (int k = aNegStart; ; k += aStep)
            {
                double aT = -k * aH;

                double aSinhT = Math.Sinh(aT);
                double aCoshT = Math.Cosh(aT);
                double aU = aHalfPi * aSinhT;

                if (Math.Abs(aU) > 700.0)
                {
                    break;
                }

                double aTanhU = Math.Tanh(aU);
                double aCoshU = Math.Cosh(aU);
                double aSech2U = 1.0 / (aCoshU * aCoshU);

                double aX = aMid + aHalf * aTanhU;

                if (aX <= theLower + MathUtils.THE_ZERO_TOL || aX >= theUpper - MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                double aWeight = aHalf * aHalfPi * aCoshT * aSech2U;

                if (aWeight < MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                if (!theFunc.Value(aX, out double aF))
                {
                    continue;
                }

                if (!double.IsFinite(aF))
                {
                    continue;
                }

                aSum += aWeight * aF;
                ++aNbPoints;

                if (aNbPoints > 10000)
                {
                    break;
                }
            }

            double aLevelSum = aH * aSum;
            aTotalPoints += aNbPoints;

            double aNewSum = (aLevel == 0) ? aLevelSum : 0.5 * aPrevSum + aLevelSum;

            if (aLevel > 0)
            {
                double aAbsError = Math.Abs(aNewSum - aPrevSum);
                double aRelError = aAbsError / Max(Math.Abs(aNewSum), 1.0e-15);

                if (aRelError < theConfig.Tolerance)
                {
                    aResult.Status = Status.OK;
                    aResult.Value = aNewSum;
                    aResult.AbsoluteError = aAbsError;
                    aResult.RelativeError = aRelError;
                    aResult.NbPoints = aTotalPoints;
                    aResult.NbIterations = aLevel + 1;
                    return aResult;
                }
            }

            aPrevSum = aNewSum;
            aH *= 0.5; // Halve step size for next level
        }

        aResult.Status = Status.OK; // Still return a result
        aResult.Value = aPrevSum;
        aResult.NbPoints = aTotalPoints;
        aResult.NbIterations = theConfig.NbLevels;
        return aResult;
    }

    /// <summary>Exp-sinh quadrature over the semi-infinite interval [theLower, +infinity).</summary>
    public static IntegResult ExpSinh<Function>(Function theFunc, double theLower, DoubleExpConfig? theConfig = null)
        where Function : IScalarFunction
    {
        theConfig ??= new DoubleExpConfig();
        IntegResult aResult = new();

        double aHalfPi = M_PI / 2.0;
        double aH = 1.0;
        double aPrevSum = 0.0;
        int aTotalPoints = 0;

        for (int aLevel = 0; aLevel < theConfig.NbLevels; ++aLevel)
        {
            double aSum = 0.0;
            int aNbPoints = 0;

            int aStart = (aLevel == 0) ? 0 : 1;
            int aStep = (aLevel == 0) ? 1 : 2;

            for (int aSign = -1; aSign <= 1; aSign += 2)
            {
                for (int k = (aSign < 0 && aLevel == 0) ? 1 : aStart; ; k += aStep)
                {
                    if (aSign < 0 && k == 0)
                    {
                        continue;
                    }

                    double aT = aSign * k * aH;

                    double aSinhT = Math.Sinh(aT);
                    double aCoshT = Math.Cosh(aT);
                    double aU = aHalfPi * aSinhT;

                    if (aU > 700.0)
                    {
                        break; // Overflow
                    }

                    double aExpU = Math.Exp(aU);

                    double aX = theLower + aExpU;

                    double aWeight = aHalfPi * aCoshT * aExpU;

                    if (aWeight < MathUtils.THE_ZERO_TOL || !double.IsFinite(aWeight))
                    {
                        break;
                    }

                    if (aU < -30.0)
                    {
                        break;
                    }

                    if (!theFunc.Value(aX, out double aF))
                    {
                        continue;
                    }

                    if (!double.IsFinite(aF))
                    {
                        continue;
                    }

                    aSum += aWeight * aF;
                    ++aNbPoints;

                    if (aNbPoints > 10000)
                    {
                        break;
                    }
                }
            }

            double aLevelSum = aH * aSum;
            aTotalPoints += aNbPoints;

            double aNewSum = (aLevel == 0) ? aLevelSum : 0.5 * aPrevSum + aLevelSum;

            if (aLevel > 0)
            {
                double aAbsError = Math.Abs(aNewSum - aPrevSum);
                double aRelError = aAbsError / Max(Math.Abs(aNewSum), 1.0e-15);

                if (aRelError < theConfig.Tolerance)
                {
                    aResult.Status = Status.OK;
                    aResult.Value = aNewSum;
                    aResult.AbsoluteError = aAbsError;
                    aResult.RelativeError = aRelError;
                    aResult.NbPoints = aTotalPoints;
                    aResult.NbIterations = aLevel + 1;
                    return aResult;
                }
            }

            aPrevSum = aNewSum;
            aH *= 0.5;
        }

        aResult.Status = Status.OK;
        aResult.Value = aPrevSum;
        aResult.NbPoints = aTotalPoints;
        aResult.NbIterations = theConfig.NbLevels;
        return aResult;
    }

    /// <summary>Sinh-sinh quadrature over the infinite interval (-infinity, +infinity).</summary>
    public static IntegResult SinhSinh<Function>(Function theFunc, DoubleExpConfig? theConfig = null)
        where Function : IScalarFunction
    {
        theConfig ??= new DoubleExpConfig();
        IntegResult aResult = new();

        double aHalfPi = M_PI / 2.0;
        double aH = 1.0;
        double aPrevSum = 0.0;
        int aTotalPoints = 0;

        for (int aLevel = 0; aLevel < theConfig.NbLevels; ++aLevel)
        {
            double aSum = 0.0;
            int aNbPoints = 0;

            int aStart = (aLevel == 0) ? 0 : 1;
            int aStep = (aLevel == 0) ? 1 : 2;

            for (int aSign = -1; aSign <= 1; aSign += 2)
            {
                for (int k = (aSign < 0 && aLevel == 0) ? 1 : aStart; ; k += aStep)
                {
                    if (aSign < 0 && k == 0)
                    {
                        continue;
                    }

                    double aT = aSign * k * aH;

                    double aSinhT = Math.Sinh(aT);
                    double aCoshT = Math.Cosh(aT);
                    double aU = aHalfPi * aSinhT;

                    if (Math.Abs(aU) > 700.0)
                    {
                        break;
                    }

                    double aSinhU = Math.Sinh(aU);
                    double aCoshU = Math.Cosh(aU);

                    double aX = aSinhU;

                    double aWeight = aHalfPi * aCoshT * aCoshU;

                    if (aWeight < MathUtils.THE_ZERO_TOL || !double.IsFinite(aWeight))
                    {
                        break;
                    }

                    if (!theFunc.Value(aX, out double aF))
                    {
                        continue;
                    }

                    if (!double.IsFinite(aF))
                    {
                        continue;
                    }

                    aSum += aWeight * aF;
                    ++aNbPoints;

                    if (aNbPoints > 10000)
                    {
                        break;
                    }
                }
            }

            double aLevelSum = aH * aSum;
            aTotalPoints += aNbPoints;

            double aNewSum = (aLevel == 0) ? aLevelSum : 0.5 * aPrevSum + aLevelSum;

            if (aLevel > 0)
            {
                double aAbsError = Math.Abs(aNewSum - aPrevSum);
                double aRelError = aAbsError / Max(Math.Abs(aNewSum), 1.0e-15);

                if (aRelError < theConfig.Tolerance)
                {
                    aResult.Status = Status.OK;
                    aResult.Value = aNewSum;
                    aResult.AbsoluteError = aAbsError;
                    aResult.RelativeError = aRelError;
                    aResult.NbPoints = aTotalPoints;
                    aResult.NbIterations = aLevel + 1;
                    return aResult;
                }
            }

            aPrevSum = aNewSum;
            aH *= 0.5;
        }

        aResult.Status = Status.OK;
        aResult.Value = aPrevSum;
        aResult.NbPoints = aTotalPoints;
        aResult.NbIterations = theConfig.NbLevels;
        return aResult;
    }

    /// <summary>f(-x): the integrand DoubleExponential uses for (-infinity, b] (a local class in C++).</summary>
    private struct DoubleExpNegatedFunc<Function> : IScalarFunction
        where Function : IScalarFunction
    {
        private Function myFunc;

        public DoubleExpNegatedFunc(Function theF)
        {
            myFunc = theF;
        }

        public bool Value(double theX, out double theF) => myFunc.Value(-theX, out theF);
    }

    /// <summary>
    /// Double exponential integration choosing the rule from the bounds: a bound beyond 1e300 in magnitude is
    /// infinite (SinhSinh, ExpSinh, ExpSinh of f(-x)); otherwise TanhSinh.
    /// </summary>
    public static IntegResult DoubleExponential<Function>(Function theFunc, double theLower, double theUpper, DoubleExpConfig? theConfig = null)
        where Function : IScalarFunction
    {
        double aHuge = 1.0e300;

        bool aIsLowerInf = (theLower < -aHuge);
        bool aIsUpperInf = (theUpper > aHuge);

        if (aIsLowerInf && aIsUpperInf)
        {
            return SinhSinh(theFunc, theConfig);
        }
        else if (aIsUpperInf)
        {
            return ExpSinh(theFunc, theLower, theConfig);
        }
        else if (aIsLowerInf)
        {
            DoubleExpNegatedFunc<Function> aNegated = new(theFunc);
            return ExpSinh(aNegated, -theUpper, theConfig);
        }
        else
        {
            return TanhSinh(theFunc, theLower, theUpper, theConfig);
        }
    }

    /// <summary>Tanh-sinh with 8 levels, for integrands singular at the endpoints.</summary>
    public static IntegResult TanhSinhSingular<Function>(Function theFunc, double theLower, double theUpper, double theTolerance = 1.0e-10)
        where Function : IScalarFunction
    {
        DoubleExpConfig aConfig = new();
        aConfig.Tolerance = theTolerance;
        aConfig.NbLevels = 8; // More levels for singular functions

        return TanhSinh(theFunc, theLower, theUpper, aConfig);
    }

    /// <summary>Tanh-sinh split at a known interior singularity theSingularity.</summary>
    public static IntegResult TanhSinhWithSingularity<Function>(Function theFunc, double theLower, double theUpper,
                                                                double theSingularity, DoubleExpConfig? theConfig = null)
        where Function : IScalarFunction
    {
        IntegResult aResult = new();

        if (theSingularity <= theLower || theSingularity >= theUpper)
        {
            return TanhSinh(theFunc, theLower, theUpper, theConfig);
        }

        IntegResult aLeft = TanhSinh(theFunc, theLower, theSingularity, theConfig);
        if (!aLeft.IsDone())
        {
            return aLeft;
        }

        IntegResult aRight = TanhSinh(theFunc, theSingularity, theUpper, theConfig);
        if (!aRight.IsDone())
        {
            return aRight;
        }

        aResult.Status = Status.OK;
        aResult.Value = aLeft.Value!.Value + aRight.Value!.Value;
        aResult.NbPoints = aLeft.NbPoints + aRight.NbPoints;
        aResult.NbIterations = Max(aLeft.NbIterations, aRight.NbIterations);

        if (aLeft.AbsoluteError.HasValue && aRight.AbsoluteError.HasValue)
        {
            aResult.AbsoluteError = aLeft.AbsoluteError.Value + aRight.AbsoluteError.Value;
            aResult.RelativeError = aResult.AbsoluteError.Value / Max(Math.Abs(aResult.Value.Value), 1.0e-15);
        }

        return aResult;
    }
}
