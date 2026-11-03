// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_Kronrod.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Integration;

/// <summary>Gauss-Kronrod quadrature with embedded error estimation.</summary>
public static partial class MathInteg
{
    /// <summary>Configuration for Gauss-Kronrod integration.</summary>
    public class KronrodConfig : IntegConfig
    {
        /// <summary>Number of Gauss points (n); Kronrod uses 2n+1 points.</summary>
        public int NbGaussPoints = 7;
        /// <summary>Whether to use adaptive subdivision.</summary>
        public bool Adaptive = true;

        public KronrodConfig()
        {
        }

        public KronrodConfig(double tolerance, int maxIter = 100)
            : base(tolerance, maxIter)
        {
        }

        public KronrodConfig(KronrodConfig other)
            : base(other)
        {
            NbGaussPoints = other.NbGaussPoints;
            Adaptive = other.Adaptive;
        }
    }

    /// <summary>
    /// One application of the (2n+1)-point Gauss-Kronrod rule (n = theNbGauss), with the QUADPACK error estimate.
    /// </summary>
    public static IntegResult KronrodRule<Function>(Function func, double lower, double upper, int nbGauss = 7)
        where Function : IScalarFunction
    {
        IntegResult result = new();

        int nbKronrod = 2 * nbGauss + 1;

        math_Vector gaussP = new(1, nbGauss);
        math_Vector gaussW = new(1, nbGauss);
        math_Vector kronrodP = new(1, nbKronrod);
        math_Vector kronrodW = new(1, nbKronrod);

        if (!GetKronrodPointsAndWeights(nbKronrod, kronrodP, kronrodW)
            || !GetOrderedGaussPointsAndWeights(nbGauss, gaussP, gaussW))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        double halfLen = 0.5 * (upper - lower);
        double mid = 0.5 * (upper + lower);

        int nPnt2 = (nbKronrod + 1) / 2;

        double gaussVal = 0.0;
        double kronrodVal = 0.0;

        math_Vector f1 = new(0, nPnt2 - 1);
        math_Vector f2 = new(0, nPnt2 - 1);

        for (int i = 2; i < nPnt2; i += 2)
        {
            double dx = halfLen * kronrodP[i];
            double val1 = 0.0, val2 = 0.0;

            if (!func.Value(mid + dx, out val1) || !func.Value(mid - dx, out val2))
            {
                result.Status = Status.NumericalError;
                return result;
            }

            f1[i] = val1;
            f2[i] = val2;
            gaussVal += (val1 + val2) * gaussW[i / 2];
            kronrodVal += (val1 + val2) * kronrodW[i];
        }

        if (!func.Value(mid, out double fc))
        {
            result.Status = Status.NumericalError;
            return result;
        }

        kronrodVal += fc * kronrodW[nPnt2];

        if (nPnt2 % 2 == 0)
        {
            gaussVal += fc * gaussW[nPnt2 / 2];
        }

        for (int i = 1; i < nPnt2; i += 2)
        {
            double dx = halfLen * kronrodP[i];
            double val1 = 0.0, val2 = 0.0;

            if (!func.Value(mid + dx, out val1) || !func.Value(mid - dx, out val2))
            {
                result.Status = Status.NumericalError;
                return result;
            }

            f1[i] = val1;
            f2[i] = val2;
            kronrodVal += (val1 + val2) * kronrodW[i];
        }

        double mean = 0.5 * kronrodVal;
        double asc = Math.Abs(fc - mean) * kronrodW[nPnt2];
        for (int i = 1; i < nPnt2; ++i)
        {
            asc += kronrodW[i] * (Math.Abs(f1[i] - mean) + Math.Abs(f2[i] - mean));
        }

        asc *= halfLen;
        kronrodVal *= halfLen;
        gaussVal *= halfLen;

        double absError = Math.Abs(kronrodVal - gaussVal);

        if (asc != 0.0 && absError != 0.0)
        {
            double scale = Math.Pow(200.0 * absError / asc, 1.5);
            if (scale < 1.0)
            {
                absError = Min(absError, asc * scale);
            }
        }

        result.Status = Status.OK;
        result.Value = kronrodVal;
        result.AbsoluteError = absError;
        result.RelativeError = absError / Max(Math.Abs(kronrodVal), 1.0e-15);
        result.NbPoints = nbKronrod;
        result.NbIterations = 1;
        return result;
    }

    /// <summary>An interval of the adaptive Kronrod subdivision (a local struct in C++).</summary>
    private struct KronrodInterval
    {
        public double Lower;
        public double Upper;
        public double Value;
        public double Error;

        public KronrodInterval(double lower, double upper, double value, double error)
        {
            Lower = lower;
            Upper = upper;
            Value = value;
            Error = error;
        }
    }

    /// <summary>
    /// Adaptive Gauss-Kronrod integration: bisects the interval with the largest error estimate until the total
    /// error is within Tolerance or MaxIterations is reached. Without Adaptive, one KronrodRule.
    /// </summary>
    public static IntegResult Kronrod<Function>(Function func, double lower, double upper, KronrodConfig? config = null)
        where Function : IScalarFunction
    {
        config ??= new KronrodConfig();
        IntegResult result = new();

        if (!config.Adaptive)
        {
            return KronrodRule(func, lower, upper, config.NbGaussPoints);
        }

        IntegResult initResult = KronrodRule(func, lower, upper, config.NbGaussPoints);
        if (!initResult.IsDone)
        {
            return initResult;
        }

        List<KronrodInterval> heap = new();
        heap.Add(new KronrodInterval(lower, upper, initResult.Value!.Value, initResult.AbsoluteError!.Value));

        double totalValue = initResult.Value.Value;
        double totalError = initResult.AbsoluteError.Value;
        int totalPoints = initResult.NbPoints;
        int iterations = 1;

        while (iterations < config.MaxIterations)
        {
            if (totalError < config.Tolerance * Max(Math.Abs(totalValue), 1.0e-15))
            {
                break;
            }

            int maxIdx = 0;
            double maxError = 0.0;
            for (int i = 0; i < heap.Count; ++i)
            {
                if (heap[i].Error > maxError)
                {
                    maxError = heap[i].Error;
                    maxIdx = i;
                }
            }

            if (maxError < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
            {
                break; // No more refinement needed
            }

            KronrodInterval worst = heap[maxIdx];
            double bisMid = 0.5 * (worst.Lower + worst.Upper);

            IntegResult leftResult = KronrodRule(func, worst.Lower, bisMid, config.NbGaussPoints);
            IntegResult rightResult = KronrodRule(func, bisMid, worst.Upper, config.NbGaussPoints);

            if (!leftResult.IsDone || !rightResult.IsDone)
            {
                result.Status = Status.NumericalError;
                result.Value = totalValue;
                result.AbsoluteError = totalError;
                result.NbPoints = totalPoints;
                result.NbIterations = iterations;
                return result;
            }

            totalValue -= worst.Value;
            totalError -= worst.Error;
            totalValue += leftResult.Value!.Value + rightResult.Value!.Value;
            totalError += leftResult.AbsoluteError!.Value + rightResult.AbsoluteError!.Value;
            totalPoints += leftResult.NbPoints + rightResult.NbPoints;
            ++iterations;

            heap[maxIdx] = new KronrodInterval(worst.Lower, bisMid, leftResult.Value.Value, leftResult.AbsoluteError.Value);
            heap.Add(new KronrodInterval(bisMid, worst.Upper, rightResult.Value.Value, rightResult.AbsoluteError.Value));
        }

        result.Status = Status.OK;
        result.Value = totalValue;
        result.AbsoluteError = totalError;
        result.RelativeError = totalError / Max(Math.Abs(totalValue), 1.0e-15);
        result.NbPoints = totalPoints;
        result.NbIterations = iterations;
        return result;
    }

    /// <summary>
    /// Kronrod with automatic order selection: tries single rules of 7, 11, ..., theMaxOrder Gauss points and
    /// returns the first within theTolerance, else falls back to adaptive Kronrod.
    /// </summary>
    public static IntegResult KronrodAuto<Function>(Function func, double lower, double upper,
                                                    double tolerance = 1.0e-10, int maxOrder = 30)
        where Function : IScalarFunction
    {
        // aBestResult is kept as in C++, though it is never returned
        IntegResult bestResult = new();
        bestResult.Status = Status.NotConverged;

        for (int order = 7; order <= maxOrder; order += 4)
        {
            IntegResult result = KronrodRule(func, lower, upper, order);
            if (!result.IsDone)
            {
                continue;
            }

            bestResult = result;

            if (result.RelativeError.HasValue && result.RelativeError.Value < tolerance)
            {
                return result;
            }
        }

        KronrodConfig config = new();
        config.Tolerance = tolerance;
        config.NbGaussPoints = 7;
        config.Adaptive = true;
        config.MaxIterations = 50;

        return Kronrod(func, lower, upper, config);
    }

    /// <summary>f(a + t/(1-t)) / (1-t)^2 on [0, 1): the integrand of KronrodSemiInfinite (a local class in C++).</summary>
    private struct KronrodSemiInfiniteFunc<Function> : IScalarFunction
        where Function : IScalarFunction
    {
        private Function _func;
        private readonly double _a;

        public KronrodSemiInfiniteFunc(Function f, double a)
        {
            _func = f;
            _a = a;
        }

        public bool Value(double t, out double f)
        {
            if (t >= 1.0)
            {
                f = 0.0;
                return true;
            }

            double x = _a + t / (1.0 - t);
            double jacob = 1.0 / global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(1.0 - t);

            f = 0.0;
            if (!_func.Value(x, out double fx))
            {
                return false;
            }

            f = fx * jacob;
            return true;
        }
    }

    /// <summary>Kronrod integration over [theLower, +infinity) by the substitution x = a + t/(1-t).</summary>
    public static IntegResult KronrodSemiInfinite<Function>(Function func, double lower, KronrodConfig? config = null)
        where Function : IScalarFunction
    {
        KronrodSemiInfiniteFunc<Function> transformed = new(func, lower);
        return Kronrod(transformed, 0.0, 1.0, config);
    }

    /// <summary>f(t/(1-t^2)) (1+t^2)/(1-t^2)^2 on (-1, 1): the integrand of KronrodInfinite (a local class in C++).</summary>
    private struct KronrodInfiniteFunc<Function> : IScalarFunction
        where Function : IScalarFunction
    {
        private Function _func;

        public KronrodInfiniteFunc(Function f)
        {
            _func = f;
        }

        public bool Value(double t, out double f)
        {
            if (Math.Abs(t) >= 1.0)
            {
                f = 0.0;
                return true;
            }

            double t2 = t * t;
            double x = t / (1.0 - t2);
            double jacob = (1.0 + t2) / global::Kerf.Engine.Numerics.Utilities.MathUtils.Sqr(1.0 - t2);

            f = 0.0;
            if (!_func.Value(x, out double fx))
            {
                return false;
            }

            f = fx * jacob;
            return true;
        }
    }

    /// <summary>Kronrod integration over (-infinity, +infinity) by the substitution x = t/(1-t^2).</summary>
    public static IntegResult KronrodInfinite<Function>(Function func, KronrodConfig? config = null)
        where Function : IScalarFunction
    {
        KronrodInfiniteFunc<Function> transformed = new(func);
        return Kronrod(transformed, -1.0, 1.0, config);
    }
}
