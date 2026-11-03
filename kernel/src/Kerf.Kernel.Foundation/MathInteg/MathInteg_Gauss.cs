// Port of: src/FoundationClasses/TKMath/MathInteg/MathInteg_Gauss.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Integration;

// C++ code names MathUtils' types through MathInteg (MathInteg::IntegResult, MathInteg::IntegConfig,
// MathInteg::Status) by `using namespace MathUtils`; C# can't re-export them, so callers write MathUtils.IntegResult.

/// <summary>Numerical integration (quadrature): Gauss-Legendre, Gauss-Kronrod and double exponential rules.</summary>
public static partial class MathInteg
{
    /// <summary>Fixed-order Gauss-Legendre integration of theFunc over [theLower, theUpper] (theNbPoints &gt;= 1).</summary>
    public static IntegResult Gauss<Function>(Function func, double lower, double upper, int nbPoints = 15)
        where Function : IScalarFunction
    {
        IntegResult result = new();
        if (nbPoints < 1)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        math_Vector points = new(1, nbPoints);
        math_Vector weights = new(1, nbPoints);

        if (!global::Kerf.Engine.Numerics.Utilities.MathUtils.GetGaussPointsAndWeights(nbPoints, points, weights))
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        double halfLen = 0.5 * (upper - lower);
        double mid = 0.5 * (upper + lower);

        double sum = 0.0;
        for (int i = 1; i <= nbPoints; ++i)
        {
            double x = mid + halfLen * points[i];
            if (!func.Value(x, out double f))
            {
                result.Status = Status.NumericalError;
                return result;
            }
            sum += weights[i] * f;
        }

        result.Status = Status.OK;
        result.Value = halfLen * sum;
        result.NbPoints = nbPoints;
        result.NbIterations = 1;
        return result;
    }

    /// <summary>
    /// Adaptive Gauss-Legendre integration: compares two orders (InitialOrder and min(MaxOrder, 2*InitialOrder))
    /// and bisects recursively, up to MaxIterations levels, until they agree within Tolerance.
    /// </summary>
    public static IntegResult GaussAdaptive<Function>(Function func, double lower, double upper, IntegConfig? config = null)
        where Function : IScalarFunction
    {
        config ??= new IntegConfig();
        IntegResult result = new();

        if (config.InitialOrder < 1 || config.MaxOrder < config.InitialOrder
            || config.MaxOrder > 61 || config.MaxIterations < 1)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int coarseOrder = config.InitialOrder;
        int fineOrder = Min(config.MaxOrder, Min(61, 2 * coarseOrder));
        if (fineOrder == coarseOrder)
        {
            if (coarseOrder > 1)
            {
                coarseOrder -= 1;
            }
            else if (config.MaxOrder > 1)
            {
                fineOrder = 2;
            }
        }

        IntegResult coarse = Gauss(func, lower, upper, coarseOrder);
        if (!coarse.IsDone)
        {
            return coarse;
        }

        IntegResult fine = Gauss(func, lower, upper, fineOrder);
        if (!fine.IsDone)
        {
            return fine;
        }

        double error = Math.Abs(fine.Value!.Value - coarse.Value!.Value);
        double scale = Max(Math.Abs(fine.Value.Value), 1.0e-15);

        if (error < config.Tolerance * scale)
        {
            result.Status = Status.OK;
            result.Value = fine.Value.Value;
            result.AbsoluteError = error;
            result.RelativeError = error / scale;
            result.NbPoints = fineOrder;
            result.NbIterations = 1;
            return result;
        }

        if (config.MaxIterations <= 1)
        {
            result.Status = Status.MaxIterations;
            result.Value = fine.Value.Value;
            result.AbsoluteError = error;
            result.RelativeError = error / scale;
            result.NbPoints = fineOrder;
            result.NbIterations = 1;
            return result;
        }

        double mid = 0.5 * (lower + upper);

        IntegConfig subConfig = new(config);
        subConfig.MaxIterations = config.MaxIterations - 1;

        IntegResult left = GaussAdaptive(func, lower, mid, subConfig);
        if (!left.IsDone)
        {
            result.Status = left.Status;
            result.Value = left.Value;
            return result;
        }

        IntegResult right = GaussAdaptive(func, mid, upper, subConfig);
        if (!right.IsDone)
        {
            result.Status = right.Status;
            result.Value = left.Value!.Value + (right.Value.HasValue ? right.Value.Value : 0.0);
            return result;
        }

        result.Status = Status.OK;
        result.Value = left.Value!.Value + right.Value!.Value;
        result.AbsoluteError = left.AbsoluteError!.Value + right.AbsoluteError!.Value;
        result.RelativeError = result.AbsoluteError.Value / Max(Math.Abs(result.Value.Value), 1.0e-15);
        result.NbPoints = left.NbPoints + right.NbPoints;
        result.NbIterations = Max(left.NbIterations, right.NbIterations) + 1;
        return result;
    }

    /// <summary>Composite Gauss-Legendre rule: theNbIntervals equal subintervals, theNbPoints points each.</summary>
    public static IntegResult GaussComposite<Function>(Function func, double lower, double upper, int nbIntervals, int nbPoints = 7)
        where Function : IScalarFunction
    {
        IntegResult result = new();

        if (nbIntervals < 1)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        double h = (upper - lower) / nbIntervals;
        double sum = 0.0;
        int totalPoints = 0;

        for (int i = 0; i < nbIntervals; ++i)
        {
            double a = lower + i * h;
            double b = a + h;

            IntegResult subResult = Gauss(func, a, b, nbPoints);
            if (!subResult.IsDone)
            {
                result.Status = subResult.Status;
                result.Value = sum;
                result.NbPoints = totalPoints;
                return result;
            }

            sum += subResult.Value!.Value;
            totalPoints += subResult.NbPoints;
        }

        result.Status = Status.OK;
        result.Value = sum;
        result.NbPoints = totalPoints;
        result.NbIterations = 1;
        return result;
    }
}
