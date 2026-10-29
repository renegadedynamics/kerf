// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_MultipleUtils.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

// The C++ functor adapters (structs holding references, called through operator()) are structs here implementing
// the small interfaces below, whose single method `Invoke` is the operator(). FindAllRootsImpl is generic over them,
// so the JIT specialises it for each combination as the C++ template is instantiated. The adapters hold the
// function by value: a class functor is shared, a struct functor is copied (see MathUtils_FunctorScalar.cs).

/// <summary>Internal utilities for FindAllRoots / FindAllRootsWithDerivative.</summary>
public static partial class MathRoot
{
    // ============================================================================
    //  Result and configuration types
    // ============================================================================

    /// <summary>Result for multiple root finding: all found roots sorted in ascending order.</summary>
    /// <remarks>
    /// A C++ value holding two arrays; here a struct holding two array references, so a copy shares them (C++ copies
    /// the arrays). Ported code writes `new MultipleResult()`; `default` has no arrays.
    /// </remarks>
    public struct MultipleResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Total iterations across all roots.</summary>
        public int NbIterations = 0;
        /// <summary>Found roots (sorted).</summary>
        public NCollection_DynamicArray<double> Roots = new();
        /// <summary>Function values at roots.</summary>
        public NCollection_DynamicArray<double> Values = new();
        /// <summary>True if function is essentially zero in range.</summary>
        public bool IsAllNull = false;

        public MultipleResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(MultipleResult theResult) => theResult.IsDone();

        public static bool operator true(MultipleResult theResult) => theResult.IsDone();

        public static bool operator false(MultipleResult theResult) => !theResult.IsDone();

        /// <summary>Returns the number of roots found.</summary>
        public readonly int NbRoots() => Roots.Length();

        /// <summary>Access root by index (0-based).</summary>
        public readonly double this[int theIndex] => Roots.Value(theIndex);
    }

    /// <summary>Configuration for multiple root finding.</summary>
    /// <remarks>A class, as the other solver configurations (see MathUtils_Config.cs).</remarks>
    public class MultipleConfig
    {
        /// <summary>Number of sample points for initial search.</summary>
        public int NbSamples = 100;
        /// <summary>Tolerance on X for convergence.</summary>
        public double XTolerance = 1e-10;
        /// <summary>Tolerance on F(X) for convergence.</summary>
        public double FTolerance = 1e-10;
        /// <summary>Tolerance to consider function as null.</summary>
        public double NullTolerance = 1e-12;
        /// <summary>Max iterations per root refinement.</summary>
        public int MaxIterations = 100;
        /// <summary>Find roots of f(x) - Offset = 0.</summary>
        public double Offset = 0.0;

        public MultipleConfig()
        {
        }

        public MultipleConfig(MultipleConfig theOther)
        {
            NbSamples = theOther.NbSamples;
            XTolerance = theOther.XTolerance;
            FTolerance = theOther.FTolerance;
            NullTolerance = theOther.NullTolerance;
            MaxIterations = theOther.MaxIterations;
            Offset = theOther.Offset;
        }
    }

    // ============================================================================
    //  Functor interfaces (C++'s callable template parameters of FindAllRootsImpl)
    // ============================================================================

    /// <summary>SampleFn: callable (int theIndex, double theX) -> bool.</summary>
    public interface IMultipleSampleFn
    {
        bool Invoke(int theIndex, double theX);
    }

    /// <summary>GetValueFn: callable (int theIndex) -> double, returns f(x)-offset at sample.</summary>
    public interface IMultipleGetValueFn
    {
        double Invoke(int theIndex);
    }

    /// <summary>GetRootValueFn: callable (double theX) -> double, returns original f(x).</summary>
    public interface IMultipleGetRootValueFn
    {
        double Invoke(double theX);
    }

    /// <summary>IntervalExtraFn: callable (int, x0, x1, f0, f1, result, epsX) -> void.</summary>
    public interface IMultipleIntervalExtraFn
    {
        void Invoke(int theIndex, double theX0, double theX1, double theF0, double theF1, ref MultipleResult theResult, double theEpsX);
    }

    // ============================================================================
    //  Helper functions
    // ============================================================================

    /// <summary>In-place insertion sort of roots and corresponding values by ascending root value.</summary>
    public static void SortRoots(ref MultipleResult theResult)
    {
        for (int i = 1; i < theResult.Roots.Length(); ++i)
        {
            double aKeyRoot = theResult.Roots[i];
            double aKeyVal = theResult.Values[i];
            int j = i - 1;
            while (j >= 0 && theResult.Roots[j] > aKeyRoot)
            {
                theResult.Roots[j + 1] = theResult.Roots[j];
                theResult.Values[j + 1] = theResult.Values[j];
                --j;
            }
            theResult.Roots[j + 1] = aKeyRoot;
            theResult.Values[j + 1] = aKeyVal;
        }
    }

    /// <summary>Helper to add a root if it is not a duplicate of an already found root.</summary>
    public static void AddRoot(ref MultipleResult theResult, double theEpsX, double theRoot, double theValue)
    {
        for (int k = 0; k < theResult.Roots.Length(); ++k)
        {
            if (Math.Abs(theRoot - theResult.Roots.Value(k)) < theEpsX)
            {
                return;
            }
        }
        theResult.Roots.Append(theRoot);
        theResult.Values.Append(theValue);
    }

    /// <summary>Compute the minimal X tolerance compatible with the established multi-root behavior.</summary>
    public static double EffectiveXTolerance(double theLower, double theUpper, double theXTolerance)
    {
        double aMinEpsX = 1.0e-10 * (Math.Abs(theLower) + Math.Abs(theUpper));
        return Max(theXTolerance, aMinEpsX);
    }

    // ============================================================================
    //  Functor adapters for Value-only interface
    // ============================================================================

    /// <summary>Samples a Value-only function and stores f(x)-offset into a math_Vector.</summary>
    public readonly struct MultipleSampleValueFn<Function> : IMultipleSampleFn
        where Function : MathUtils.IScalarFunction
    {
        private readonly Function myFunc;
        private readonly math_Vector mySamples;
        private readonly double myOffset;

        public MultipleSampleValueFn(Function theFunc, math_Vector theSamples, double theOffset)
        {
            myFunc = theFunc;
            mySamples = theSamples;
            myOffset = theOffset;
        }

        public bool Invoke(int theIndex, double theX)
        {
            if (!myFunc.Value(theX, out double aF))
                return false;
            mySamples[theIndex] = aF - myOffset;
            return true;
        }
    }

    /// <summary>Returns the sampled value at a given index from a math_Vector.</summary>
    public readonly struct MultipleGetValueFn : IMultipleGetValueFn
    {
        private readonly math_Vector mySamples;

        public MultipleGetValueFn(math_Vector theSamples)
        {
            mySamples = theSamples;
        }

        public double Invoke(int theIndex) => mySamples[theIndex];
    }

    /// <summary>Brent wrapper that adapts a Value-only function for offset root finding.</summary>
    public readonly struct MultipleBrentValueWrapper<Function> : MathUtils.IScalarFunction
        where Function : MathUtils.IScalarFunction
    {
        private readonly Function myFunc;
        private readonly double myOffset;

        public MultipleBrentValueWrapper(Function theFunc, double theOffset)
        {
            myFunc = theFunc;
            myOffset = theOffset;
        }

        public bool Value(double theX, out double theY)
        {
            if (!myFunc.Value(theX, out theY))
                return false;
            theY -= myOffset;
            return true;
        }
    }

    /// <summary>Evaluates original (non-offset) function value at a root point via Value interface.</summary>
    public readonly struct MultipleGetRootValueFn<Function> : IMultipleGetRootValueFn
        where Function : MathUtils.IScalarFunction
    {
        private readonly Function myFunc;

        public MultipleGetRootValueFn(Function theFunc)
        {
            myFunc = theFunc;
        }

        public double Invoke(double theX)
        {
            myFunc.Value(theX, out double aF);
            return aF;
        }
    }

    // ============================================================================
    //  Functor adapters for Values (with derivative) interface
    // ============================================================================

    /// <summary>Wrapper exposing a function derivative through the Value() contract required by Brent.</summary>
    public readonly struct MultipleDerivativeValueWrapper<Function> : MathUtils.IScalarFunction
        where Function : MathUtils.IScalarFunctionWithDerivative
    {
        private readonly Function myFunc;

        public MultipleDerivativeValueWrapper(Function theFunc)
        {
            myFunc = theFunc;
        }

        public bool Value(double theX, out double theY)
        {
            return myFunc.Values(theX, out _, out theY);
        }
    }

    /// <summary>Evaluate the original function value.</summary>
    public static bool EvaluateValue<Function>(Function theFunc, double theX, out double theValue)
        where Function : MathUtils.IScalarFunction
    {
        return theFunc.Value(theX, out theValue);
    }

    /// <summary>Evaluate the function value shifted by the requested offset.</summary>
    public static bool EvaluateShiftedValue<Function>(Function theFunc, double theX, double theOffset, out double theValue)
        where Function : MathUtils.IScalarFunction
    {
        if (!theFunc.Value(theX, out theValue))
        {
            return false;
        }

        theValue -= theOffset;
        return true;
    }

    /// <summary>Evaluate the function value and derivative, then shift the value by the requested offset.</summary>
    public static bool EvaluateShiftedValues<Function>(Function theFunc,
                                                       double theX,
                                                       double theOffset,
                                                       out double theValue,
                                                       out double theDerivative)
        where Function : MathUtils.IScalarFunctionWithDerivative
    {
        if (!theFunc.Values(theX, out theValue, out theDerivative))
        {
            return false;
        }

        theValue -= theOffset;
        return true;
    }

    /// <summary>Refine a bracketed sign change using the same Brent/Newton sequence as math_FunctionRoots.</summary>
    public static bool RefineBracketedRoot<Function>(Function theFunc,
                                                     double theOffset,
                                                     double theX1,
                                                     double theY1,
                                                     double theX2,
                                                     double theY2,
                                                     double theTolerance,
                                                     double theEpsX,
                                                     ref MultipleResult theResult)
        where Function : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        const int THE_MAX_ITERATIONS = 100;
        const double THE_EPS2 = 2.0e-14;
        const double THE_DERIV_EPS = 1.0e-10;

        int anIter = 0;
        double aTol2 = 0.5 * theTolerance;
        double aA = theX1;
        double aB = theX2;
        double aC = theX2;
        double aD = 0.0;
        double anE = 0.0;
        double aFa = theY1;
        double aFb = theY2;
        double aFc = theY2;

        for (anIter = 1; anIter <= THE_MAX_ITERATIONS; ++anIter)
        {
            if ((aFb > 0.0 && aFc > 0.0) || (aFb < 0.0 && aFc < 0.0))
            {
                aC = aA;
                aFc = aFa;
                anE = aD = aB - aA;
            }

            if (Math.Abs(aFc) < Math.Abs(aFb))
            {
                double aPrevA = aA;
                double aPrevFa = aFa;
                aA = aB;
                aB = aC;
                aC = aPrevA;
                aFa = aFb;
                aFb = aFc;
                aFc = aPrevFa;
            }

            double aTol1 = THE_EPS2 * Math.Abs(aB) + aTol2;
            double aXm = 0.5 * (aC - aB);
            if (Math.Abs(aXm) < aTol1 || aFb == 0.0)
            {
                double aNewtonX = aB;
                for (int aNewtonIter = 0; aNewtonIter < 5; ++aNewtonIter)
                {
                    if (!EvaluateShiftedValues(theFunc, aNewtonX, theOffset, out double aY, out double aDfdx))
                    {
                        return false;
                    }

                    if (Math.Abs(aDfdx) <= THE_DERIV_EPS)
                    {
                        break;
                    }

                    aNewtonX -= aY / aDfdx;
                    if (aNewtonX < theX1 || aNewtonX > theX2)
                    {
                        break;
                    }

                    if (!EvaluateShiftedValue(theFunc, aNewtonX, theOffset, out aY))
                    {
                        return false;
                    }

                    if (Math.Abs(aY) < Math.Abs(aFb))
                    {
                        aB = aNewtonX;
                        aFb = aY;
                    }
                }

                if (!EvaluateValue(theFunc, aB, out double aRootValue))
                {
                    return false;
                }

                theResult.NbIterations += anIter;
                AddRoot(ref theResult, theEpsX, aB, aRootValue);
                return true;
            }

            if (Math.Abs(anE) >= aTol1 && Math.Abs(aFa) > Math.Abs(aFb))
            {
                double aP = 0.0;
                double aQ = 0.0;
                double aS = aFb / aFa;
                if (aA == aC)
                {
                    aP = 2.0 * aXm * aS;
                    aQ = 1.0 - aS;
                }
                else
                {
                    aQ = aFa / aFc;
                    double aR = aFb / aFc;
                    aP = aS * (2.0 * aXm * aQ * (aQ - aR) - (aB - aA) * (aR - 1.0));
                    aQ = (aQ - 1.0) * (aR - 1.0) * (aS - 1.0);
                }

                if (aP > 0.0)
                {
                    aQ = -aQ;
                }

                aP = Math.Abs(aP);
                double aMin1 = 3.0 * aXm * aQ - Math.Abs(aTol1 * aQ);
                double aMin2 = Math.Abs(anE * aQ);
                if (2.0 * aP < Min(aMin1, aMin2))
                {
                    anE = aD;
                    aD = aP / aQ;
                }
                else
                {
                    aD = aXm;
                    anE = aD;
                }
            }
            else
            {
                aD = aXm;
                anE = aD;
            }

            aA = aB;
            aFa = aFb;
            if (Math.Abs(aD) > aTol1)
            {
                aB += aD;
            }
            else
            {
                aB += (aXm >= 0.0) ? Math.Abs(aTol1) : -Math.Abs(aTol1);
            }

            if (!EvaluateShiftedValue(theFunc, aB, theOffset, out aFb))
            {
                return false;
            }
        }

        theResult.NbIterations += THE_MAX_ITERATIONS;
        return true;
    }

    /// <summary>
    /// Derivative-aware implementation mirroring the math_FunctionRoots heuristics while operating on the modern
    /// callable-based MathRoot API.
    /// </summary>
    public static MultipleResult FindAllRootsWithDerivativeImpl<Function>(Function theFunc,
                                                                          double theLower,
                                                                          double theUpper,
                                                                          MultipleConfig theConfig)
        where Function : MathUtils.IScalarFunction, MathUtils.IScalarFunctionWithDerivative
    {
        MultipleResult aResult = new();
        aResult.Status = MathUtils.Status.OK;

        double aLower = Min(theLower, theUpper);
        double aUpper = Max(theLower, theUpper);
        int aNbSamples = Max(2 * theConfig.NbSamples, 20);
        double aDx = (aUpper - aLower) / aNbSamples;
        double aEpsX = EffectiveXTolerance(aLower, aUpper, theConfig.XTolerance);
        double aRawXTolerance = theConfig.XTolerance;
        double aMajorDx = 5.0 * aDx;

        math_Vector aValues = new(0, aNbSamples);
        double aX = aLower;
        for (int i = 0; i <= aNbSamples; ++i, aX += aDx)
        {
            if (aX > aUpper)
            {
                aX = aUpper;
            }

            if (!EvaluateShiftedValue(theFunc, aX, theConfig.Offset, out aValues[i]))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                return aResult;
            }
        }

        aResult.IsAllNull = true;
        for (int i = 0; i <= aNbSamples; ++i)
        {
            if (aValues[i] > theConfig.NullTolerance || aValues[i] < -theConfig.NullTolerance)
            {
                aResult.IsAllNull = false;
                break;
            }
        }
        if (aResult.IsAllNull)
        {
            return aResult;
        }

        double aTolerance = aEpsX;
        double aX1 = aLower;
        for (int i = 0, anIp1 = 1; i < aNbSamples; ++i, ++anIp1, aX1 += aDx)
        {
            double aX2 = aX1 + aDx;
            if (aX2 > aUpper)
            {
                aX2 = aUpper;
            }

            if ((aValues[i] < 0.0 && aValues[anIp1] > 0.0) || (aValues[i] > 0.0 && aValues[anIp1] < 0.0))
            {
                if (!RefineBracketedRoot(theFunc,
                                         theConfig.Offset,
                                         aX1,
                                         aValues[i],
                                         aX2,
                                         aValues[anIp1],
                                         aTolerance,
                                         aEpsX,
                                         ref aResult))
                {
                    aResult.Status = MathUtils.Status.NumericalError;
                    return aResult;
                }
            }
        }

        for (int i = 0; i <= aNbSamples; ++i)
        {
            if (aValues[i] != 0.0)
            {
                continue;
            }

            double aZeroX = Min(aLower + i * aDx, aUpper);
            double aLeftX = aZeroX - 0.5 * aDx;
            double aRightX = aZeroX + 0.5 * aDx;
            if (aLeftX < aLower)
            {
                aLeftX = aLower;
            }
            if (aLeftX > aUpper)
            {
                aLeftX = aUpper;
            }
            if (aRightX < aLower)
            {
                aRightX = aLower;
            }
            if (aRightX > aUpper)
            {
                aRightX = aUpper;
            }

            double aLeftY = 0.0;
            double aRightY = 0.0;
            if (!EvaluateShiftedValue(theFunc, aLeftX, theConfig.Offset, out aLeftY)
                || !EvaluateShiftedValue(theFunc, aRightX, theConfig.Offset, out aRightY))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                return aResult;
            }

            if (aLeftY * aRightY < 0.0)
            {
                if (!RefineBracketedRoot(theFunc,
                                         theConfig.Offset,
                                         aLeftX,
                                         aLeftY,
                                         aRightX,
                                         aRightY,
                                         aTolerance,
                                         aEpsX,
                                         ref aResult))
                {
                    aResult.Status = MathUtils.Status.NumericalError;
                    return aResult;
                }
            }
            else if (aLeftY != 0.0 || aRightY != 0.0)
            {
                if (!EvaluateValue(theFunc, aZeroX, out double aRootValue))
                {
                    aResult.Status = MathUtils.Status.NumericalError;
                    return aResult;
                }
                AddRoot(ref aResult, aEpsX, aZeroX, aRootValue);
            }
        }

        if (aValues[0] <= theConfig.FTolerance && aValues[0] >= -theConfig.FTolerance)
        {
            if (!EvaluateValue(theFunc, aLower, out double aRootValue))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                return aResult;
            }
            AddRoot(ref aResult, aEpsX, aLower, aRootValue);
        }

        if (aValues[aNbSamples] <= theConfig.FTolerance && aValues[aNbSamples] >= -theConfig.FTolerance)
        {
            if (!EvaluateValue(theFunc, aUpper, out double aRootValue))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                return aResult;
            }
            AddRoot(ref aResult, aEpsX, aUpper, aRootValue);
        }

        int anIm1 = 0;
        int anIp1b = 2;
        double aMidX = aLower + aDx;
        for (int i = 1; i < aNbSamples; ++i, ++anIm1, ++anIp1b, aMidX += aDx)
        {
            int anIp1 = anIp1b; // C++'s anIp1 (the name is taken by the loop above in C#)
            if (aMidX > aUpper)
            {
                aMidX = aUpper;
            }

            bool isRediscretize = false;
            if (aValues[i] > 0.0)
            {
                if (aValues[anIm1] > aValues[i] && aValues[anIp1] > aValues[i])
                {
                    double aProbeX = Max(aLower, aMidX - aDx);
                    if (!EvaluateShiftedValues(theFunc, aProbeX, theConfig.Offset, out double aProbeY, out double aProbeDy))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }

                    if (Math.Abs(aProbeDy) > 1.0e-10)
                    {
                        double aStep = aProbeY / aProbeDy;
                        if (aStep < aMajorDx && aStep > -aMajorDx)
                        {
                            isRediscretize = true;
                        }
                    }

                    if (!isRediscretize)
                    {
                        aProbeX = Min(aUpper, aMidX + aDx);
                        if (!EvaluateShiftedValues(theFunc, aProbeX, theConfig.Offset, out aProbeY, out aProbeDy))
                        {
                            aResult.Status = MathUtils.Status.NumericalError;
                            return aResult;
                        }

                        if (Math.Abs(aProbeDy) > 1.0e-10)
                        {
                            double aStep = aProbeY / aProbeDy;
                            if (aStep < aMajorDx && aStep > -aMajorDx)
                            {
                                isRediscretize = true;
                            }
                        }
                    }
                }
            }
            else if (aValues[i] < 0.0)
            {
                if (aValues[anIm1] < aValues[i] && aValues[anIp1] < aValues[i])
                {
                    double aProbeX = Max(aLower, aMidX - aDx);
                    if (!EvaluateShiftedValues(theFunc, aProbeX, theConfig.Offset, out double aProbeY, out double aProbeDy))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }

                    if (Math.Abs(aProbeDy) > 1.0e-10)
                    {
                        double aStep = aProbeY / aProbeDy;
                        if (aStep < aMajorDx && aStep > -aMajorDx)
                        {
                            isRediscretize = true;
                        }
                    }

                    if (!isRediscretize)
                    {
                        aProbeX = Min(aUpper, aMidX + aDx);
                        if (!EvaluateShiftedValues(theFunc, aProbeX, theConfig.Offset, out aProbeY, out aProbeDy))
                        {
                            aResult.Status = MathUtils.Status.NumericalError;
                            return aResult;
                        }

                        if (Math.Abs(aProbeDy) > 1.0e-10)
                        {
                            double aStep = aProbeY / aProbeDy;
                            if (aStep < aMajorDx && aStep > -aMajorDx)
                            {
                                isRediscretize = true;
                            }
                        }
                    }
                }
            }

            if (!isRediscretize)
            {
                continue;
            }

            double aX0 = Max(aLower, aMidX - aDx);
            double aX3 = Min(aUpper, aMidX + aDx);
            double aRoot1 = 0.0;
            double aRoot2 = 0.0;
            double aVal1 = 0.0;
            double aVal2 = 0.0;
            double aDer1 = 0.0;
            double aDer2 = 0.0;
            bool hasRoot1 = false;
            bool hasRoot2 = false;

            MultipleDerivativeValueWrapper<Function> aDerivativeWrapper = new(theFunc);
            MathUtils.Config aDerivativeConfig = new();
            aDerivativeConfig.XTolerance = aRawXTolerance;
            aDerivativeConfig.FTolerance = 0.0;
            aDerivativeConfig.MaxIterations = theConfig.MaxIterations;

            MathUtils.ScalarResult aDerivativeRoot =
              Brent(aDerivativeWrapper, aX0, aX3, aDerivativeConfig);
            aResult.NbIterations += aDerivativeRoot.NbIterations;
            if (aDerivativeRoot.IsDone() && aDerivativeRoot.Root.HasValue)
            {
                aRoot1 = aDerivativeRoot.Root.Value;
                if (!EvaluateValue(theFunc, aRoot1, out double aOriginalValue))
                {
                    aResult.Status = MathUtils.Status.NumericalError;
                    return aResult;
                }
                aVal1 = Math.Abs(aOriginalValue - theConfig.Offset);
                if (aVal1 < theConfig.FTolerance)
                {
                    hasRoot1 = true;
                    if (!EvaluateShiftedValues(theFunc, aRoot1, theConfig.Offset, out aOriginalValue, out aDer1))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }
                }
            }

            double aXProbe1 = 0.0;
            double aXProbe2 = 0.0;
            const double THE_GOLDEN_INV_RATIO = 1.0 / MathUtils.THE_GOLDEN_RATIO;
            const double THE_GOLDEN_INV_COMP = 1.0 - THE_GOLDEN_INV_RATIO;
            double aTolCR = aEpsX * 10.0;
            double aLocalTolX = 0.001 * aEpsX;
            double aF0 = aValues[anIm1];
            double aF3 = aValues[anIp1];
            bool isSearchMinimum = (aF0 > 0.0);

            if (Math.Abs(aX3 - aMidX) > Math.Abs(aX0 - aMidX))
            {
                aXProbe1 = aMidX;
                aXProbe2 = aMidX + THE_GOLDEN_INV_COMP * (aX3 - aMidX);
            }
            else
            {
                aXProbe2 = aMidX;
                aXProbe1 = aMidX - THE_GOLDEN_INV_COMP * (aMidX - aX0);
            }

            double aF1 = 0.0;
            double aF2 = 0.0;
            if (!EvaluateShiftedValue(theFunc, aXProbe1, theConfig.Offset, out aF1)
                || !EvaluateShiftedValue(theFunc, aXProbe2, theConfig.Offset, out aF2))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                return aResult;
            }

            while (Math.Abs(aX3 - aX0) > aTolCR * (Math.Abs(aXProbe1) + Math.Abs(aXProbe2))
                   && Math.Abs(aXProbe1 - aXProbe2) > aLocalTolX)
            {
                if (isSearchMinimum)
                {
                    if (aF2 < aF1)
                    {
                        aX0 = aXProbe1;
                        aXProbe1 = aXProbe2;
                        aXProbe2 = THE_GOLDEN_INV_RATIO * aXProbe1 + THE_GOLDEN_INV_COMP * aX3;
                        aF0 = aF1;
                        aF1 = aF2;
                        if (!EvaluateShiftedValue(theFunc, aXProbe2, theConfig.Offset, out aF2))
                        {
                            aResult.Status = MathUtils.Status.NumericalError;
                            return aResult;
                        }
                    }
                    else
                    {
                        aX3 = aXProbe2;
                        aXProbe2 = aXProbe1;
                        aXProbe1 = THE_GOLDEN_INV_RATIO * aXProbe2 + THE_GOLDEN_INV_COMP * aX0;
                        aF3 = aF2;
                        aF2 = aF1;
                        if (!EvaluateShiftedValue(theFunc, aXProbe1, theConfig.Offset, out aF1))
                        {
                            aResult.Status = MathUtils.Status.NumericalError;
                            return aResult;
                        }
                    }
                }
                else
                {
                    if (aF2 > aF1)
                    {
                        aX0 = aXProbe1;
                        aXProbe1 = aXProbe2;
                        aXProbe2 = THE_GOLDEN_INV_RATIO * aXProbe1 + THE_GOLDEN_INV_COMP * aX3;
                        aF0 = aF1;
                        aF1 = aF2;
                        if (!EvaluateShiftedValue(theFunc, aXProbe2, theConfig.Offset, out aF2))
                        {
                            aResult.Status = MathUtils.Status.NumericalError;
                            return aResult;
                        }
                    }
                    else
                    {
                        aX3 = aXProbe2;
                        aXProbe2 = aXProbe1;
                        aXProbe1 = THE_GOLDEN_INV_RATIO * aXProbe2 + THE_GOLDEN_INV_COMP * aX0;
                        aF3 = aF2;
                        aF2 = aF1;
                        if (!EvaluateShiftedValue(theFunc, aXProbe1, theConfig.Offset, out aF1))
                        {
                            aResult.Status = MathUtils.Status.NumericalError;
                            return aResult;
                        }
                    }
                }

                if (aF1 * aF0 < 0.0)
                {
                    if (!RefineBracketedRoot(theFunc,
                                             theConfig.Offset,
                                             aX0,
                                             aF0,
                                             aXProbe1,
                                             aF1,
                                             aTolerance,
                                             aEpsX,
                                             ref aResult))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }
                }

                if (aF2 * aF3 < 0.0)
                {
                    if (!RefineBracketedRoot(theFunc,
                                             theConfig.Offset,
                                             aXProbe2,
                                             aF2,
                                             aX3,
                                             aF3,
                                             aTolerance,
                                             aEpsX,
                                             ref aResult))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }
                }
            }

            if ((isSearchMinimum && aF1 < aF2) || (!isSearchMinimum && aF1 > aF2))
            {
                if (Math.Abs(aF1) < theConfig.FTolerance)
                {
                    hasRoot2 = true;
                    aRoot2 = aXProbe1;
                    aVal2 = Math.Abs(aF1);
                }
            }
            else if (Math.Abs(aF2) < theConfig.FTolerance)
            {
                hasRoot2 = true;
                aRoot2 = aXProbe2;
                aVal2 = Math.Abs(aF2);
            }

            if (hasRoot1 && hasRoot2)
            {
                if (aVal2 - aVal1 > theConfig.FTolerance)
                {
                    if (!EvaluateValue(theFunc, aRoot1, out double aRootValue))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }
                    AddRoot(ref aResult, aEpsX, aRoot1, aRootValue);
                }
                else if (aVal1 - aVal2 > theConfig.FTolerance)
                {
                    if (!EvaluateValue(theFunc, aRoot2, out double aRootValue))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }
                    AddRoot(ref aResult, aEpsX, aRoot2, aRootValue);
                }
                else
                {
                    if (!EvaluateShiftedValues(theFunc, aRoot2, theConfig.Offset, out double aShiftedValue, out aDer2))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }

                    double aChosenRoot = (Math.Abs(aDer1) < Math.Abs(aDer2)) ? aRoot1 : aRoot2;
                    if (!EvaluateValue(theFunc, aChosenRoot, out double aRootValue))
                    {
                        aResult.Status = MathUtils.Status.NumericalError;
                        return aResult;
                    }
                    AddRoot(ref aResult, aEpsX, aChosenRoot, aRootValue);
                }
            }
            else if (hasRoot1)
            {
                if (!EvaluateValue(theFunc, aRoot1, out double aRootValue))
                {
                    aResult.Status = MathUtils.Status.NumericalError;
                    return aResult;
                }
                AddRoot(ref aResult, aEpsX, aRoot1, aRootValue);
            }
            else if (hasRoot2)
            {
                if (!EvaluateValue(theFunc, aRoot2, out double aRootValue))
                {
                    aResult.Status = MathUtils.Status.NumericalError;
                    return aResult;
                }
                AddRoot(ref aResult, aEpsX, aRoot2, aRootValue);
            }
        }

        SortRoots(ref aResult);
        return aResult;
    }

    // ============================================================================
    //  Interval handlers
    // ============================================================================

    /// <summary>No-op interval handler for functions without derivative.</summary>
    public readonly struct MultipleNoExtraHandler : IMultipleIntervalExtraFn
    {
        public void Invoke(int theIndex, double theX0, double theX1, double theF0, double theF1, ref MultipleResult theResult, double theEpsX)
        {
        }
    }

    // ============================================================================
    //  Core implementation
    // ============================================================================

    /// <summary>
    /// Core implementation for finding all roots in an interval, shared by the Value-only and Values (with
    /// derivative) interfaces.
    /// </summary>
    public static MultipleResult FindAllRootsImpl<SampleFn, GetValueFn, BrentWrapperT, GetRootValueFn, IntervalExtraFn>(
        double theLower,
        double theUpper,
        MultipleConfig theConfig,
        SampleFn theSampleFn,
        GetValueFn theGetValue,
        BrentWrapperT theBrentWrapper,
        GetRootValueFn theGetRootValue,
        IntervalExtraFn theIntervalExtra)
        where SampleFn : IMultipleSampleFn
        where GetValueFn : IMultipleGetValueFn
        where BrentWrapperT : MathUtils.IScalarFunction
        where GetRootValueFn : IMultipleGetRootValueFn
        where IntervalExtraFn : IMultipleIntervalExtraFn
    {
        MultipleResult aResult = new();
        aResult.Status = MathUtils.Status.OK;

        // Ensure proper ordering
        double aLower = Min(theLower, theUpper);
        double aUpper = Max(theLower, theUpper);

        // Minimum samples
        int aNbSamples = Max(2 * theConfig.NbSamples, 20);
        double aDx = (aUpper - aLower) / aNbSamples;

        // Ensure EpsX is not too small relative to interval
        double aEpsX = EffectiveXTolerance(aLower, aUpper, theConfig.XTolerance);

        // Sample function values
        math_Vector aXValues = new(0, aNbSamples);
        for (int i = 0; i <= aNbSamples; ++i)
        {
            double aX = aLower + i * aDx;
            if (aX > aUpper)
                aX = aUpper;
            aXValues[i] = aX;

            if (!theSampleFn.Invoke(i, aX))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                return aResult;
            }
        }

        // Check if function is essentially null everywhere
        aResult.IsAllNull = true;
        for (int i = 0; i <= aNbSamples; ++i)
        {
            if (Math.Abs(theGetValue.Invoke(i)) > theConfig.NullTolerance)
            {
                aResult.IsAllNull = false;
                break;
            }
        }

        if (aResult.IsAllNull)
        {
            return aResult;
        }

        // Find sign changes
        for (int i = 0; i < aNbSamples; ++i)
        {
            double aF0 = theGetValue.Invoke(i);
            double aF1 = theGetValue.Invoke(i + 1);
            double aX0 = aXValues[i];
            double aX1 = aXValues[i + 1];

            // Exact zero at sample point
            if (Math.Abs(aF0) < theConfig.FTolerance)
            {
                AddRoot(ref aResult, aEpsX, aX0, aF0 + theConfig.Offset);
                continue;
            }

            // Sign change detected
            if (aF0 * aF1 < 0.0)
            {
                MathUtils.Config aBrentConfig = new();
                aBrentConfig.XTolerance = aEpsX;
                aBrentConfig.FTolerance = theConfig.FTolerance;
                aBrentConfig.MaxIterations = theConfig.MaxIterations;

                MathUtils.ScalarResult aBrentResult = Brent(theBrentWrapper, aX0, aX1, aBrentConfig);
                aResult.NbIterations += aBrentResult.NbIterations;

                if (aBrentResult.IsDone() && aBrentResult.Root.HasValue)
                {
                    AddRoot(ref aResult, aEpsX, aBrentResult.Root.Value, theGetRootValue.Invoke(aBrentResult.Root.Value));
                }
            }
            else
            {
                // Additional per-interval processing (e.g., tangential root detection)
                theIntervalExtra.Invoke(i, aX0, aX1, aF0, aF1, ref aResult, aEpsX);
            }
        }

        // Check last sample point
        if (Math.Abs(theGetValue.Invoke(aNbSamples)) < theConfig.FTolerance)
        {
            AddRoot(ref aResult, aEpsX, aXValues[aNbSamples], theGetValue.Invoke(aNbSamples) + theConfig.Offset);
        }

        SortRoots(ref aResult);
        return aResult;
    }
}
