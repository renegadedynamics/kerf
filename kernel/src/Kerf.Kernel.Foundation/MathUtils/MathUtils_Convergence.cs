// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Convergence.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Numerics.Utilities;

/// <summary>Convergence testing utilities.</summary>
internal static partial class MathUtils
{
    /// <summary>Check convergence based on relative change in X.</summary>
    public static bool IsXConverged(double xOld, double xNew, double tolerance)
    {
        double diff = Math.Abs(xNew - xOld);
        double scale = Max(1.0, Math.Abs(xNew));
        return diff < tolerance * scale;
    }

    /// <summary>Check convergence based on absolute function value: |f(x)| &lt; tolerance.</summary>
    public static bool IsFConverged(double fValue, double tolerance)
    {
        return Math.Abs(fValue) < tolerance;
    }

    /// <summary>Combined convergence test for scalar root finders: X or function value.</summary>
    public static bool IsConverged(double xOld, double xNew, double fValue, Config config)
    {
        return IsXConverged(xOld, xNew, config.XTolerance)
               || IsFConverged(fValue, config.FTolerance);
    }

    /// <summary>Convergence test for minimization (checks both X and F change).</summary>
    public static bool IsMinConverged(double xOld, double xNew, double fOld, double fNew, Config config)
    {
        // X convergence
        if (IsXConverged(xOld, xNew, config.XTolerance))
        {
            return true;
        }

        // Relative function change convergence
        double fDiff = Math.Abs(fNew - fOld);
        double fScale = Max(1.0, Math.Abs(fNew));
        return fDiff < config.FTolerance * fScale;
    }

    /// <summary>Convergence test for vector solvers: max|new_i - old_i| / max(1, |new_i|) &lt; tolerance.</summary>
    public static bool IsVectorConverged(math_Vector old, math_Vector theNew, double tolerance)
    {
        double maxDiff = 0.0;
        double maxScale = 1.0;
        for (int i = old.Lower(); i <= old.Upper(); ++i)
        {
            maxDiff = Max(maxDiff, Math.Abs(theNew[i] - old[i]));
            maxScale = Max(maxScale, Math.Abs(theNew[i]));
        }
        return maxDiff < tolerance * maxScale;
    }

    /// <summary>Convergence test using gradient norm for minimization.</summary>
    public static bool IsGradientConverged(math_Vector gradient, double tolerance)
    {
        double normSq = 0.0;
        for (int i = gradient.Lower(); i <= gradient.Upper(); ++i)
        {
            normSq += Sqr(gradient[i]);
        }
        return Math.Sqrt(normSq) < tolerance;
    }

    /// <summary>Compute infinity norm of a vector: max|v_i|.</summary>
    public static double InfinityNorm(math_Vector vector)
    {
        double max = 0.0;
        for (int i = vector.Lower(); i <= vector.Upper(); ++i)
        {
            max = Max(max, Math.Abs(vector[i]));
        }
        return max;
    }

    /// <summary>Compute Euclidean (L2) norm of a vector.</summary>
    public static double EuclideanNorm(math_Vector vector)
    {
        double sumSq = 0.0;
        for (int i = vector.Lower(); i <= vector.Upper(); ++i)
        {
            sumSq += Sqr(vector[i]);
        }
        return Math.Sqrt(sumSq);
    }
}
