// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_MultipleUtils.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Roots;

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
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Total iterations across all roots.</summary>
        public int NbIterations = 0;
        /// <summary>Found roots (sorted).</summary>
        public List<double> Roots = new();
        /// <summary>Function values at roots.</summary>
        public List<double> Values = new();
        /// <summary>True if function is essentially zero in range.</summary>
        public bool IsAllNull = false;

        public MultipleResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(MultipleResult result) => result.IsDone;

        public static bool operator true(MultipleResult result) => result.IsDone;

        public static bool operator false(MultipleResult result) => !result.IsDone;

        /// <summary>Returns the number of roots found.</summary>
        public readonly int RootCount => Roots.Count;

        /// <summary>Access root by index (0-based).</summary>
        public readonly double this[int index] => Roots[index];
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

        public MultipleConfig(MultipleConfig other)
        {
            NbSamples = other.NbSamples;
            XTolerance = other.XTolerance;
            FTolerance = other.FTolerance;
            NullTolerance = other.NullTolerance;
            MaxIterations = other.MaxIterations;
            Offset = other.Offset;
        }
    }

    // ============================================================================
    //  Functor interfaces (C++'s callable template parameters of FindAllRootsImpl)
    // ============================================================================

    /// <summary>TSampleFn: callable (int theIndex, double theX) -> bool.</summary>
    public interface IMultipleSampleFn
    {
        bool Invoke(int index, double x);
    }

    /// <summary>TGetValueFn: callable (int theIndex) -> double, returns f(x)-offset at sample.</summary>
    public interface IMultipleGetValueFn
    {
        double Invoke(int index);
    }

    /// <summary>TGetRootValueFn: callable (double theX) -> double, returns original f(x).</summary>
    public interface IMultipleGetRootValueFn
    {
        double Invoke(double x);
    }

    /// <summary>TIntervalExtraFn: callable (int, x0, x1, f0, f1, result, epsX) -> void.</summary>
    public interface IMultipleIntervalExtraFn
    {
        void Invoke(int index, double x0, double x1, double f0, double f1, ref MultipleResult result, double epsX);
    }

    // ============================================================================
    //  Helper functions
    // ============================================================================

    /// <summary>In-place insertion sort of roots and corresponding values by ascending root value.</summary>
    public static void SortRoots(ref MultipleResult result)
    {
        for (int i = 1; i < result.Roots.Count; ++i)
        {
            double keyRoot = result.Roots[i];
            double keyVal = result.Values[i];
            int j = i - 1;
            while (j >= 0 && result.Roots[j] > keyRoot)
            {
                result.Roots[j + 1] = result.Roots[j];
                result.Values[j + 1] = result.Values[j];
                --j;
            }
            result.Roots[j + 1] = keyRoot;
            result.Values[j + 1] = keyVal;
        }
    }

    /// <summary>Helper to add a root if it is not a duplicate of an already found root.</summary>
    public static void AddRoot(ref MultipleResult result, double epsX, double root, double value)
    {
        for (int k = 0; k < result.Roots.Count; ++k)
        {
            if (Math.Abs(root - result.Roots[k]) < epsX)
            {
                return;
            }
        }
        result.Roots.Add(root);
        result.Values.Add(value);
    }

    /// <summary>Compute the minimal X tolerance compatible with the established multi-root behavior.</summary>
    public static double EffectiveXTolerance(double lower, double upper, double xTolerance)
    {
        double minEpsX = 1.0e-10 * (Math.Abs(lower) + Math.Abs(upper));
        return Max(xTolerance, minEpsX);
    }

    // ============================================================================
    //  Functor adapters for Value-only interface
    // ============================================================================

    /// <summary>Samples a Value-only function and stores f(x)-offset into a math_Vector.</summary>
    public readonly struct MultipleSampleValueFn<Function> : IMultipleSampleFn
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        private readonly Function _func;
        private readonly math_Vector _samples;
        private readonly double _offset;

        public MultipleSampleValueFn(Function func, math_Vector samples, double offset)
        {
            _func = func;
            _samples = samples;
            _offset = offset;
        }

        public bool Invoke(int index, double x)
        {
            if (!_func.Value(x, out double f))
                return false;
            _samples[index] = f - _offset;
            return true;
        }
    }

    /// <summary>Returns the sampled value at a given index from a math_Vector.</summary>
    public readonly struct MultipleGetValueFn : IMultipleGetValueFn
    {
        private readonly math_Vector _samples;

        public MultipleGetValueFn(math_Vector samples)
        {
            _samples = samples;
        }

        public double Invoke(int index) => _samples[index];
    }

    /// <summary>Brent wrapper that adapts a Value-only function for offset root finding.</summary>
    public readonly struct MultipleBrentValueWrapper<Function> : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        private readonly Function _func;
        private readonly double _offset;

        public MultipleBrentValueWrapper(Function func, double offset)
        {
            _func = func;
            _offset = offset;
        }

        public bool Value(double x, out double y)
        {
            if (!_func.Value(x, out y))
                return false;
            y -= _offset;
            return true;
        }
    }

    /// <summary>Evaluates original (non-offset) function value at a root point via Value interface.</summary>
    public readonly struct MultipleGetRootValueFn<Function> : IMultipleGetRootValueFn
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        private readonly Function _func;

        public MultipleGetRootValueFn(Function func)
        {
            _func = func;
        }

        public double Invoke(double x)
        {
            _func.Value(x, out double f);
            return f;
        }
    }

    // ============================================================================
    //  Functor adapters for Values (with derivative) interface
    // ============================================================================

    /// <summary>Wrapper exposing a function derivative through the Value() contract required by Brent.</summary>
    public readonly struct MultipleDerivativeValueWrapper<Function> : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        private readonly Function _func;

        public MultipleDerivativeValueWrapper(Function func)
        {
            _func = func;
        }

        public bool Value(double x, out double y)
        {
            return _func.Values(x, out _, out y);
        }
    }

    /// <summary>Evaluate the original function value.</summary>
    public static bool EvaluateValue<Function>(Function func, double x, out double value)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        return func.Value(x, out value);
    }

    /// <summary>Evaluate the function value shifted by the requested offset.</summary>
    public static bool EvaluateShiftedValue<Function>(Function func, double x, double offset, out double value)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
    {
        if (!func.Value(x, out value))
        {
            return false;
        }

        value -= offset;
        return true;
    }

    /// <summary>Evaluate the function value and derivative, then shift the value by the requested offset.</summary>
    public static bool EvaluateShiftedValues<Function>(Function func,
                                                       double x,
                                                       double offset,
                                                       out double value,
                                                       out double derivative)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        if (!func.Values(x, out value, out derivative))
        {
            return false;
        }

        value -= offset;
        return true;
    }

    /// <summary>Refine a bracketed sign change using the same Brent/Newton sequence as math_FunctionRoots.</summary>
    public static bool RefineBracketedRoot<Function>(Function func,
                                                     double offset,
                                                     double x1,
                                                     double y1,
                                                     double x2,
                                                     double y2,
                                                     double tolerance,
                                                     double epsX,
                                                     ref MultipleResult result)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        const int THE_MAX_ITERATIONS = 100;
        const double THE_EPS2 = 2.0e-14;
        const double THE_DERIV_EPS = 1.0e-10;

        int iter = 0;
        double tol2 = 0.5 * tolerance;
        double a = x1;
        double b = x2;
        double c = x2;
        double d = 0.0;
        double e = 0.0;
        double fa = y1;
        double fb = y2;
        double fc = y2;

        for (iter = 1; iter <= THE_MAX_ITERATIONS; ++iter)
        {
            if ((fb > 0.0 && fc > 0.0) || (fb < 0.0 && fc < 0.0))
            {
                c = a;
                fc = fa;
                e = d = b - a;
            }

            if (Math.Abs(fc) < Math.Abs(fb))
            {
                double prevA = a;
                double prevFa = fa;
                a = b;
                b = c;
                c = prevA;
                fa = fb;
                fb = fc;
                fc = prevFa;
            }

            double tol1 = THE_EPS2 * Math.Abs(b) + tol2;
            double xm = 0.5 * (c - b);
            if (Math.Abs(xm) < tol1 || fb == 0.0)
            {
                double newtonX = b;
                for (int newtonIter = 0; newtonIter < 5; ++newtonIter)
                {
                    if (!EvaluateShiftedValues(func, newtonX, offset, out double y, out double dfdx))
                    {
                        return false;
                    }

                    if (Math.Abs(dfdx) <= THE_DERIV_EPS)
                    {
                        break;
                    }

                    newtonX -= y / dfdx;
                    if (newtonX < x1 || newtonX > x2)
                    {
                        break;
                    }

                    if (!EvaluateShiftedValue(func, newtonX, offset, out y))
                    {
                        return false;
                    }

                    if (Math.Abs(y) < Math.Abs(fb))
                    {
                        b = newtonX;
                        fb = y;
                    }
                }

                if (!EvaluateValue(func, b, out double rootValue))
                {
                    return false;
                }

                result.NbIterations += iter;
                AddRoot(ref result, epsX, b, rootValue);
                return true;
            }

            if (Math.Abs(e) >= tol1 && Math.Abs(fa) > Math.Abs(fb))
            {
                double p = 0.0;
                double q = 0.0;
                double s = fb / fa;
                if (a == c)
                {
                    p = 2.0 * xm * s;
                    q = 1.0 - s;
                }
                else
                {
                    q = fa / fc;
                    double r = fb / fc;
                    p = s * (2.0 * xm * q * (q - r) - (b - a) * (r - 1.0));
                    q = (q - 1.0) * (r - 1.0) * (s - 1.0);
                }

                if (p > 0.0)
                {
                    q = -q;
                }

                p = Math.Abs(p);
                double min1 = 3.0 * xm * q - Math.Abs(tol1 * q);
                double min2 = Math.Abs(e * q);
                if (2.0 * p < Min(min1, min2))
                {
                    e = d;
                    d = p / q;
                }
                else
                {
                    d = xm;
                    e = d;
                }
            }
            else
            {
                d = xm;
                e = d;
            }

            a = b;
            fa = fb;
            if (Math.Abs(d) > tol1)
            {
                b += d;
            }
            else
            {
                b += (xm >= 0.0) ? Math.Abs(tol1) : -Math.Abs(tol1);
            }

            if (!EvaluateShiftedValue(func, b, offset, out fb))
            {
                return false;
            }
        }

        result.NbIterations += THE_MAX_ITERATIONS;
        return true;
    }

    /// <summary>
    /// Derivative-aware implementation mirroring the math_FunctionRoots heuristics while operating on the modern
    /// callable-based MathRoot API.
    /// </summary>
    public static MultipleResult FindAllRootsWithDerivativeImpl<Function>(Function func,
                                                                          double lower,
                                                                          double upper,
                                                                          MultipleConfig config)
        where Function : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction, global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunctionWithDerivative
    {
        MultipleResult result = new();
        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        double aLower = Min(lower, upper);
        double aUpper = Max(lower, upper);
        int nbSamples = Max(2 * config.NbSamples, 20);
        double dx = (aUpper - aLower) / nbSamples;
        double epsX = EffectiveXTolerance(aLower, aUpper, config.XTolerance);
        double rawXTolerance = config.XTolerance;
        double majorDx = 5.0 * dx;

        math_Vector values = new(0, nbSamples);
        double x = aLower;
        for (int i = 0; i <= nbSamples; ++i, x += dx)
        {
            if (x > aUpper)
            {
                x = aUpper;
            }

            if (!EvaluateShiftedValue(func, x, config.Offset, out values[i]))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return result;
            }
        }

        result.IsAllNull = true;
        for (int i = 0; i <= nbSamples; ++i)
        {
            if (values[i] > config.NullTolerance || values[i] < -config.NullTolerance)
            {
                result.IsAllNull = false;
                break;
            }
        }
        if (result.IsAllNull)
        {
            return result;
        }

        double tolerance = epsX;
        double x1 = aLower;
        for (int i = 0, ip1 = 1; i < nbSamples; ++i, ++ip1, x1 += dx)
        {
            double x2 = x1 + dx;
            if (x2 > aUpper)
            {
                x2 = aUpper;
            }

            if ((values[i] < 0.0 && values[ip1] > 0.0) || (values[i] > 0.0 && values[ip1] < 0.0))
            {
                if (!RefineBracketedRoot(func,
                                         config.Offset,
                                         x1,
                                         values[i],
                                         x2,
                                         values[ip1],
                                         tolerance,
                                         epsX,
                                         ref result))
                {
                    result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return result;
                }
            }
        }

        for (int i = 0; i <= nbSamples; ++i)
        {
            if (values[i] != 0.0)
            {
                continue;
            }

            double zeroX = Min(aLower + i * dx, aUpper);
            double leftX = zeroX - 0.5 * dx;
            double rightX = zeroX + 0.5 * dx;
            if (leftX < aLower)
            {
                leftX = aLower;
            }
            if (leftX > aUpper)
            {
                leftX = aUpper;
            }
            if (rightX < aLower)
            {
                rightX = aLower;
            }
            if (rightX > aUpper)
            {
                rightX = aUpper;
            }

            double leftY = 0.0;
            double rightY = 0.0;
            if (!EvaluateShiftedValue(func, leftX, config.Offset, out leftY)
                || !EvaluateShiftedValue(func, rightX, config.Offset, out rightY))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return result;
            }

            if (leftY * rightY < 0.0)
            {
                if (!RefineBracketedRoot(func,
                                         config.Offset,
                                         leftX,
                                         leftY,
                                         rightX,
                                         rightY,
                                         tolerance,
                                         epsX,
                                         ref result))
                {
                    result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return result;
                }
            }
            else if (leftY != 0.0 || rightY != 0.0)
            {
                if (!EvaluateValue(func, zeroX, out double rootValue))
                {
                    result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return result;
                }
                AddRoot(ref result, epsX, zeroX, rootValue);
            }
        }

        if (values[0] <= config.FTolerance && values[0] >= -config.FTolerance)
        {
            if (!EvaluateValue(func, aLower, out double rootValue))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return result;
            }
            AddRoot(ref result, epsX, aLower, rootValue);
        }

        if (values[nbSamples] <= config.FTolerance && values[nbSamples] >= -config.FTolerance)
        {
            if (!EvaluateValue(func, aUpper, out double rootValue))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return result;
            }
            AddRoot(ref result, epsX, aUpper, rootValue);
        }

        int im1 = 0;
        int ip1b = 2;
        double midX = aLower + dx;
        for (int i = 1; i < nbSamples; ++i, ++im1, ++ip1b, midX += dx)
        {
            int ip1 = ip1b; // C++'s anIp1 (the name is taken by the loop above in C#)
            if (midX > aUpper)
            {
                midX = aUpper;
            }

            bool isRediscretize = false;
            if (values[i] > 0.0)
            {
                if (values[im1] > values[i] && values[ip1] > values[i])
                {
                    double probeX = Max(aLower, midX - dx);
                    if (!EvaluateShiftedValues(func, probeX, config.Offset, out double probeY, out double probeDy))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }

                    if (Math.Abs(probeDy) > 1.0e-10)
                    {
                        double step = probeY / probeDy;
                        if (step < majorDx && step > -majorDx)
                        {
                            isRediscretize = true;
                        }
                    }

                    if (!isRediscretize)
                    {
                        probeX = Min(aUpper, midX + dx);
                        if (!EvaluateShiftedValues(func, probeX, config.Offset, out probeY, out probeDy))
                        {
                            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                            return result;
                        }

                        if (Math.Abs(probeDy) > 1.0e-10)
                        {
                            double step = probeY / probeDy;
                            if (step < majorDx && step > -majorDx)
                            {
                                isRediscretize = true;
                            }
                        }
                    }
                }
            }
            else if (values[i] < 0.0)
            {
                if (values[im1] < values[i] && values[ip1] < values[i])
                {
                    double probeX = Max(aLower, midX - dx);
                    if (!EvaluateShiftedValues(func, probeX, config.Offset, out double probeY, out double probeDy))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }

                    if (Math.Abs(probeDy) > 1.0e-10)
                    {
                        double step = probeY / probeDy;
                        if (step < majorDx && step > -majorDx)
                        {
                            isRediscretize = true;
                        }
                    }

                    if (!isRediscretize)
                    {
                        probeX = Min(aUpper, midX + dx);
                        if (!EvaluateShiftedValues(func, probeX, config.Offset, out probeY, out probeDy))
                        {
                            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                            return result;
                        }

                        if (Math.Abs(probeDy) > 1.0e-10)
                        {
                            double step = probeY / probeDy;
                            if (step < majorDx && step > -majorDx)
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

            double x0 = Max(aLower, midX - dx);
            double x3 = Min(aUpper, midX + dx);
            double root1 = 0.0;
            double root2 = 0.0;
            double val1 = 0.0;
            double val2 = 0.0;
            double der1 = 0.0;
            double der2 = 0.0;
            bool hasRoot1 = false;
            bool hasRoot2 = false;

            MultipleDerivativeValueWrapper<Function> derivativeWrapper = new(func);
            global::Kerf.Engine.Numerics.Utilities.MathUtils.Config derivativeConfig = new();
            derivativeConfig.XTolerance = rawXTolerance;
            derivativeConfig.FTolerance = 0.0;
            derivativeConfig.MaxIterations = config.MaxIterations;

            global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult derivativeRoot =
              Brent(derivativeWrapper, x0, x3, derivativeConfig);
            result.NbIterations += derivativeRoot.NbIterations;
            if (derivativeRoot.IsDone && derivativeRoot.Root.HasValue)
            {
                root1 = derivativeRoot.Root.Value;
                if (!EvaluateValue(func, root1, out double originalValue))
                {
                    result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return result;
                }
                val1 = Math.Abs(originalValue - config.Offset);
                if (val1 < config.FTolerance)
                {
                    hasRoot1 = true;
                    if (!EvaluateShiftedValues(func, root1, config.Offset, out originalValue, out der1))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }
                }
            }

            double xProbe1 = 0.0;
            double xProbe2 = 0.0;
            const double THE_GOLDEN_INV_RATIO = 1.0 / global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_GOLDEN_RATIO;
            const double THE_GOLDEN_INV_COMP = 1.0 - THE_GOLDEN_INV_RATIO;
            double tolCR = epsX * 10.0;
            double localTolX = 0.001 * epsX;
            double f0 = values[im1];
            double f3 = values[ip1];
            bool isSearchMinimum = (f0 > 0.0);

            if (Math.Abs(x3 - midX) > Math.Abs(x0 - midX))
            {
                xProbe1 = midX;
                xProbe2 = midX + THE_GOLDEN_INV_COMP * (x3 - midX);
            }
            else
            {
                xProbe2 = midX;
                xProbe1 = midX - THE_GOLDEN_INV_COMP * (midX - x0);
            }

            double f1 = 0.0;
            double f2 = 0.0;
            if (!EvaluateShiftedValue(func, xProbe1, config.Offset, out f1)
                || !EvaluateShiftedValue(func, xProbe2, config.Offset, out f2))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return result;
            }

            while (Math.Abs(x3 - x0) > tolCR * (Math.Abs(xProbe1) + Math.Abs(xProbe2))
                   && Math.Abs(xProbe1 - xProbe2) > localTolX)
            {
                if (isSearchMinimum)
                {
                    if (f2 < f1)
                    {
                        x0 = xProbe1;
                        xProbe1 = xProbe2;
                        xProbe2 = THE_GOLDEN_INV_RATIO * xProbe1 + THE_GOLDEN_INV_COMP * x3;
                        f0 = f1;
                        f1 = f2;
                        if (!EvaluateShiftedValue(func, xProbe2, config.Offset, out f2))
                        {
                            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                            return result;
                        }
                    }
                    else
                    {
                        x3 = xProbe2;
                        xProbe2 = xProbe1;
                        xProbe1 = THE_GOLDEN_INV_RATIO * xProbe2 + THE_GOLDEN_INV_COMP * x0;
                        f3 = f2;
                        f2 = f1;
                        if (!EvaluateShiftedValue(func, xProbe1, config.Offset, out f1))
                        {
                            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                            return result;
                        }
                    }
                }
                else
                {
                    if (f2 > f1)
                    {
                        x0 = xProbe1;
                        xProbe1 = xProbe2;
                        xProbe2 = THE_GOLDEN_INV_RATIO * xProbe1 + THE_GOLDEN_INV_COMP * x3;
                        f0 = f1;
                        f1 = f2;
                        if (!EvaluateShiftedValue(func, xProbe2, config.Offset, out f2))
                        {
                            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                            return result;
                        }
                    }
                    else
                    {
                        x3 = xProbe2;
                        xProbe2 = xProbe1;
                        xProbe1 = THE_GOLDEN_INV_RATIO * xProbe2 + THE_GOLDEN_INV_COMP * x0;
                        f3 = f2;
                        f2 = f1;
                        if (!EvaluateShiftedValue(func, xProbe1, config.Offset, out f1))
                        {
                            result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                            return result;
                        }
                    }
                }

                if (f1 * f0 < 0.0)
                {
                    if (!RefineBracketedRoot(func,
                                             config.Offset,
                                             x0,
                                             f0,
                                             xProbe1,
                                             f1,
                                             tolerance,
                                             epsX,
                                             ref result))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }
                }

                if (f2 * f3 < 0.0)
                {
                    if (!RefineBracketedRoot(func,
                                             config.Offset,
                                             xProbe2,
                                             f2,
                                             x3,
                                             f3,
                                             tolerance,
                                             epsX,
                                             ref result))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }
                }
            }

            if ((isSearchMinimum && f1 < f2) || (!isSearchMinimum && f1 > f2))
            {
                if (Math.Abs(f1) < config.FTolerance)
                {
                    hasRoot2 = true;
                    root2 = xProbe1;
                    val2 = Math.Abs(f1);
                }
            }
            else if (Math.Abs(f2) < config.FTolerance)
            {
                hasRoot2 = true;
                root2 = xProbe2;
                val2 = Math.Abs(f2);
            }

            if (hasRoot1 && hasRoot2)
            {
                if (val2 - val1 > config.FTolerance)
                {
                    if (!EvaluateValue(func, root1, out double rootValue))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }
                    AddRoot(ref result, epsX, root1, rootValue);
                }
                else if (val1 - val2 > config.FTolerance)
                {
                    if (!EvaluateValue(func, root2, out double rootValue))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }
                    AddRoot(ref result, epsX, root2, rootValue);
                }
                else
                {
                    if (!EvaluateShiftedValues(func, root2, config.Offset, out double shiftedValue, out der2))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }

                    double chosenRoot = (Math.Abs(der1) < Math.Abs(der2)) ? root1 : root2;
                    if (!EvaluateValue(func, chosenRoot, out double rootValue))
                    {
                        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                        return result;
                    }
                    AddRoot(ref result, epsX, chosenRoot, rootValue);
                }
            }
            else if (hasRoot1)
            {
                if (!EvaluateValue(func, root1, out double rootValue))
                {
                    result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return result;
                }
                AddRoot(ref result, epsX, root1, rootValue);
            }
            else if (hasRoot2)
            {
                if (!EvaluateValue(func, root2, out double rootValue))
                {
                    result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                    return result;
                }
                AddRoot(ref result, epsX, root2, rootValue);
            }
        }

        SortRoots(ref result);
        return result;
    }

    // ============================================================================
    //  Interval handlers
    // ============================================================================

    /// <summary>No-op interval handler for functions without derivative.</summary>
    public readonly struct MultipleNoExtraHandler : IMultipleIntervalExtraFn
    {
        public void Invoke(int index, double x0, double x1, double f0, double f1, ref MultipleResult result, double epsX)
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
    public static MultipleResult FindAllRootsImpl<TSampleFn, TGetValueFn, TBrentWrapperT, TGetRootValueFn, TIntervalExtraFn>(
        double lower,
        double upper,
        MultipleConfig config,
        TSampleFn sampleFn,
        TGetValueFn getValue,
        TBrentWrapperT brentWrapper,
        TGetRootValueFn getRootValue,
        TIntervalExtraFn intervalExtra)
        where TSampleFn : IMultipleSampleFn
        where TGetValueFn : IMultipleGetValueFn
        where TBrentWrapperT : global::Kerf.Engine.Numerics.Utilities.MathUtils.IScalarFunction
        where TGetRootValueFn : IMultipleGetRootValueFn
        where TIntervalExtraFn : IMultipleIntervalExtraFn
    {
        MultipleResult result = new();
        result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        // Ensure proper ordering
        double aLower = Min(lower, upper);
        double aUpper = Max(lower, upper);

        // Minimum samples
        int nbSamples = Max(2 * config.NbSamples, 20);
        double dx = (aUpper - aLower) / nbSamples;

        // Ensure EpsX is not too small relative to interval
        double epsX = EffectiveXTolerance(aLower, aUpper, config.XTolerance);

        // Sample function values
        math_Vector xValues = new(0, nbSamples);
        for (int i = 0; i <= nbSamples; ++i)
        {
            double x = aLower + i * dx;
            if (x > aUpper)
                x = aUpper;
            xValues[i] = x;

            if (!sampleFn.Invoke(i, x))
            {
                result.Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NumericalError;
                return result;
            }
        }

        // Check if function is essentially null everywhere
        result.IsAllNull = true;
        for (int i = 0; i <= nbSamples; ++i)
        {
            if (Math.Abs(getValue.Invoke(i)) > config.NullTolerance)
            {
                result.IsAllNull = false;
                break;
            }
        }

        if (result.IsAllNull)
        {
            return result;
        }

        // Find sign changes
        for (int i = 0; i < nbSamples; ++i)
        {
            double f0 = getValue.Invoke(i);
            double f1 = getValue.Invoke(i + 1);
            double x0 = xValues[i];
            double x1 = xValues[i + 1];

            // Exact zero at sample point
            if (Math.Abs(f0) < config.FTolerance)
            {
                AddRoot(ref result, epsX, x0, f0 + config.Offset);
                continue;
            }

            // Sign change detected
            if (f0 * f1 < 0.0)
            {
                global::Kerf.Engine.Numerics.Utilities.MathUtils.Config brentConfig = new();
                brentConfig.XTolerance = epsX;
                brentConfig.FTolerance = config.FTolerance;
                brentConfig.MaxIterations = config.MaxIterations;

                global::Kerf.Engine.Numerics.Utilities.MathUtils.ScalarResult brentResult = Brent(brentWrapper, x0, x1, brentConfig);
                result.NbIterations += brentResult.NbIterations;

                if (brentResult.IsDone && brentResult.Root.HasValue)
                {
                    AddRoot(ref result, epsX, brentResult.Root.Value, getRootValue.Invoke(brentResult.Root.Value));
                }
            }
            else
            {
                // Additional per-interval processing (e.g., tangential root detection)
                intervalExtra.Invoke(i, x0, x1, f0, f1, ref result, epsX);
            }
        }

        // Check last sample point
        if (Math.Abs(getValue.Invoke(nbSamples)) < config.FTolerance)
        {
            AddRoot(ref result, epsX, xValues[nbSamples], getValue.Invoke(nbSamples) + config.Offset);
        }

        SortRoots(ref result);
        return result;
    }
}
