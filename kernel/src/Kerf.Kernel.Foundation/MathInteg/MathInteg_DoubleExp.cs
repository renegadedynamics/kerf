// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_DoubleExp.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Integration;

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

        public DoubleExpConfig(double tolerance, int maxIter = 100)
            : base(tolerance, maxIter)
        {
        }

        public DoubleExpConfig(DoubleExpConfig other)
            : base(other)
        {
            NbLevels = other.NbLevels;
            StepFactor = other.StepFactor;
        }
    }

    /// <summary>
    /// Tanh-sinh quadrature over the finite interval [theLower, theUpper]; handles endpoint singularities.
    /// Points where the function fails or is not finite are skipped.
    /// </summary>
    public static IntegResult TanhSinh<Function>(Function func, double lower, double upper, DoubleExpConfig? config = null)
        where Function : IScalarFunction
    {
        config ??= new DoubleExpConfig();
        IntegResult result = new();

        if (lower >= upper)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        double halfPi = M_PI / 2.0;
        double mid = 0.5 * (upper + lower);
        double half = 0.5 * (upper - lower);

        double h = 1.0;

        double prevSum = 0.0;
        int totalPoints = 0;

        for (int level = 0; level < config.NbLevels; ++level)
        {
            double sum = 0.0;
            int nbPoints = 0;

            int start = (level == 0) ? 0 : 1;
            int step = (level == 0) ? 1 : 2;

            for (int k = start; ; k += step)
            {
                double t = k * h;

                double sinhT = Math.Sinh(t);
                double coshT = Math.Cosh(t);

                double u = halfPi * sinhT;

                if (Math.Abs(u) > 700.0)
                {
                    break; // tanh(u) ~= +/-1, contribution is negligible
                }

                double tanhU = Math.Tanh(u);
                double coshU = Math.Cosh(u);
                double sech2U = 1.0 / (coshU * coshU);

                double x = mid + half * tanhU;

                if (x <= lower + global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL || x >= upper - global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                double weight = half * halfPi * coshT * sech2U;

                if (weight < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                if (!func.Value(x, out double f))
                {
                    continue;
                }

                if (!double.IsFinite(f))
                {
                    continue;
                }

                sum += weight * f;
                ++nbPoints;

                if (nbPoints > 10000)
                {
                    break;
                }
            }

            int negStart = (level == 0) ? 1 : start; // Start at 1 for level 0, aStart otherwise
            for (int k = negStart; ; k += step)
            {
                double t = -k * h;

                double sinhT = Math.Sinh(t);
                double coshT = Math.Cosh(t);
                double u = halfPi * sinhT;

                if (Math.Abs(u) > 700.0)
                {
                    break;
                }

                double tanhU = Math.Tanh(u);
                double coshU = Math.Cosh(u);
                double sech2U = 1.0 / (coshU * coshU);

                double x = mid + half * tanhU;

                if (x <= lower + global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL || x >= upper - global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                double weight = half * halfPi * coshT * sech2U;

                if (weight < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL)
                {
                    break;
                }

                if (!func.Value(x, out double f))
                {
                    continue;
                }

                if (!double.IsFinite(f))
                {
                    continue;
                }

                sum += weight * f;
                ++nbPoints;

                if (nbPoints > 10000)
                {
                    break;
                }
            }

            double levelSum = h * sum;
            totalPoints += nbPoints;

            double newSum = (level == 0) ? levelSum : 0.5 * prevSum + levelSum;

            if (level > 0)
            {
                double absError = Math.Abs(newSum - prevSum);
                double relError = absError / Max(Math.Abs(newSum), 1.0e-15);

                if (relError < config.Tolerance)
                {
                    result.Status = Status.OK;
                    result.Value = newSum;
                    result.AbsoluteError = absError;
                    result.RelativeError = relError;
                    result.NbPoints = totalPoints;
                    result.NbIterations = level + 1;
                    return result;
                }
            }

            prevSum = newSum;
            h *= 0.5; // Halve step size for next level
        }

        result.Status = Status.OK; // Still return a result
        result.Value = prevSum;
        result.NbPoints = totalPoints;
        result.NbIterations = config.NbLevels;
        return result;
    }

    /// <summary>Exp-sinh quadrature over the semi-infinite interval [theLower, +infinity).</summary>
    public static IntegResult ExpSinh<Function>(Function func, double lower, DoubleExpConfig? config = null)
        where Function : IScalarFunction
    {
        config ??= new DoubleExpConfig();
        IntegResult result = new();

        double halfPi = M_PI / 2.0;
        double h = 1.0;
        double prevSum = 0.0;
        int totalPoints = 0;

        for (int level = 0; level < config.NbLevels; ++level)
        {
            double sum = 0.0;
            int nbPoints = 0;

            int start = (level == 0) ? 0 : 1;
            int step = (level == 0) ? 1 : 2;

            for (int sign = -1; sign <= 1; sign += 2)
            {
                for (int k = (sign < 0 && level == 0) ? 1 : start; ; k += step)
                {
                    if (sign < 0 && k == 0)
                    {
                        continue;
                    }

                    double t = sign * k * h;

                    double sinhT = Math.Sinh(t);
                    double coshT = Math.Cosh(t);
                    double u = halfPi * sinhT;

                    if (u > 700.0)
                    {
                        break; // Overflow
                    }

                    double expU = Math.Exp(u);

                    double x = lower + expU;

                    double weight = halfPi * coshT * expU;

                    if (weight < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL || !double.IsFinite(weight))
                    {
                        break;
                    }

                    if (u < -30.0)
                    {
                        break;
                    }

                    if (!func.Value(x, out double f))
                    {
                        continue;
                    }

                    if (!double.IsFinite(f))
                    {
                        continue;
                    }

                    sum += weight * f;
                    ++nbPoints;

                    if (nbPoints > 10000)
                    {
                        break;
                    }
                }
            }

            double levelSum = h * sum;
            totalPoints += nbPoints;

            double newSum = (level == 0) ? levelSum : 0.5 * prevSum + levelSum;

            if (level > 0)
            {
                double absError = Math.Abs(newSum - prevSum);
                double relError = absError / Max(Math.Abs(newSum), 1.0e-15);

                if (relError < config.Tolerance)
                {
                    result.Status = Status.OK;
                    result.Value = newSum;
                    result.AbsoluteError = absError;
                    result.RelativeError = relError;
                    result.NbPoints = totalPoints;
                    result.NbIterations = level + 1;
                    return result;
                }
            }

            prevSum = newSum;
            h *= 0.5;
        }

        result.Status = Status.OK;
        result.Value = prevSum;
        result.NbPoints = totalPoints;
        result.NbIterations = config.NbLevels;
        return result;
    }

    /// <summary>Sinh-sinh quadrature over the infinite interval (-infinity, +infinity).</summary>
    public static IntegResult SinhSinh<Function>(Function func, DoubleExpConfig? config = null)
        where Function : IScalarFunction
    {
        config ??= new DoubleExpConfig();
        IntegResult result = new();

        double halfPi = M_PI / 2.0;
        double h = 1.0;
        double prevSum = 0.0;
        int totalPoints = 0;

        for (int level = 0; level < config.NbLevels; ++level)
        {
            double sum = 0.0;
            int nbPoints = 0;

            int start = (level == 0) ? 0 : 1;
            int step = (level == 0) ? 1 : 2;

            for (int sign = -1; sign <= 1; sign += 2)
            {
                for (int k = (sign < 0 && level == 0) ? 1 : start; ; k += step)
                {
                    if (sign < 0 && k == 0)
                    {
                        continue;
                    }

                    double t = sign * k * h;

                    double sinhT = Math.Sinh(t);
                    double coshT = Math.Cosh(t);
                    double u = halfPi * sinhT;

                    if (Math.Abs(u) > 700.0)
                    {
                        break;
                    }

                    double sinhU = Math.Sinh(u);
                    double coshU = Math.Cosh(u);

                    double x = sinhU;

                    double weight = halfPi * coshT * coshU;

                    if (weight < global::Kerf.Engine.Numerics.Utilities.MathUtils.THE_ZERO_TOL || !double.IsFinite(weight))
                    {
                        break;
                    }

                    if (!func.Value(x, out double f))
                    {
                        continue;
                    }

                    if (!double.IsFinite(f))
                    {
                        continue;
                    }

                    sum += weight * f;
                    ++nbPoints;

                    if (nbPoints > 10000)
                    {
                        break;
                    }
                }
            }

            double levelSum = h * sum;
            totalPoints += nbPoints;

            double newSum = (level == 0) ? levelSum : 0.5 * prevSum + levelSum;

            if (level > 0)
            {
                double absError = Math.Abs(newSum - prevSum);
                double relError = absError / Max(Math.Abs(newSum), 1.0e-15);

                if (relError < config.Tolerance)
                {
                    result.Status = Status.OK;
                    result.Value = newSum;
                    result.AbsoluteError = absError;
                    result.RelativeError = relError;
                    result.NbPoints = totalPoints;
                    result.NbIterations = level + 1;
                    return result;
                }
            }

            prevSum = newSum;
            h *= 0.5;
        }

        result.Status = Status.OK;
        result.Value = prevSum;
        result.NbPoints = totalPoints;
        result.NbIterations = config.NbLevels;
        return result;
    }

    /// <summary>f(-x): the integrand DoubleExponential uses for (-infinity, b] (a local class in C++).</summary>
    private struct DoubleExpNegatedFunc<Function> : IScalarFunction
        where Function : IScalarFunction
    {
        private Function _func;

        public DoubleExpNegatedFunc(Function f)
        {
            _func = f;
        }

        public bool Value(double x, out double f) => _func.Value(-x, out f);
    }

    /// <summary>
    /// Double exponential integration choosing the rule from the bounds: a bound beyond 1e300 in magnitude is
    /// infinite (SinhSinh, ExpSinh, ExpSinh of f(-x)); otherwise TanhSinh.
    /// </summary>
    public static IntegResult DoubleExponential<Function>(Function func, double lower, double upper, DoubleExpConfig? config = null)
        where Function : IScalarFunction
    {
        double huge = 1.0e300;

        bool isLowerInf = (lower < -huge);
        bool isUpperInf = (upper > huge);

        if (isLowerInf && isUpperInf)
        {
            return SinhSinh(func, config);
        }
        else if (isUpperInf)
        {
            return ExpSinh(func, lower, config);
        }
        else if (isLowerInf)
        {
            DoubleExpNegatedFunc<Function> negated = new(func);
            return ExpSinh(negated, -upper, config);
        }
        else
        {
            return TanhSinh(func, lower, upper, config);
        }
    }

    /// <summary>Tanh-sinh with 8 levels, for integrands singular at the endpoints.</summary>
    public static IntegResult TanhSinhSingular<Function>(Function func, double lower, double upper, double tolerance = 1.0e-10)
        where Function : IScalarFunction
    {
        DoubleExpConfig config = new();
        config.Tolerance = tolerance;
        config.NbLevels = 8; // More levels for singular functions

        return TanhSinh(func, lower, upper, config);
    }

    /// <summary>Tanh-sinh split at a known interior singularity theSingularity.</summary>
    public static IntegResult TanhSinhWithSingularity<Function>(Function func, double lower, double upper,
                                                                double singularity, DoubleExpConfig? config = null)
        where Function : IScalarFunction
    {
        IntegResult result = new();

        if (singularity <= lower || singularity >= upper)
        {
            return TanhSinh(func, lower, upper, config);
        }

        IntegResult left = TanhSinh(func, lower, singularity, config);
        if (!left.IsDone)
        {
            return left;
        }

        IntegResult right = TanhSinh(func, singularity, upper, config);
        if (!right.IsDone)
        {
            return right;
        }

        result.Status = Status.OK;
        result.Value = left.Value!.Value + right.Value!.Value;
        result.NbPoints = left.NbPoints + right.NbPoints;
        result.NbIterations = Max(left.NbIterations, right.NbIterations);

        if (left.AbsoluteError.HasValue && right.AbsoluteError.HasValue)
        {
            result.AbsoluteError = left.AbsoluteError.Value + right.AbsoluteError.Value;
            result.RelativeError = result.AbsoluteError.Value / Max(Math.Abs(result.Value.Value), 1.0e-15);
        }

        return result;
    }
}
