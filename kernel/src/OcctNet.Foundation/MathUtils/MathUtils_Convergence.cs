// Port of: src/FoundationClasses/TKMath/MathUtils/MathUtils_Convergence.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Convergence testing utilities.</summary>
public static partial class MathUtils
{
    /// <summary>Check convergence based on relative change in X.</summary>
    public static bool IsXConverged(double theXOld, double theXNew, double theTolerance)
    {
        double aDiff = Math.Abs(theXNew - theXOld);
        double aScale = Max(1.0, Math.Abs(theXNew));
        return aDiff < theTolerance * aScale;
    }

    /// <summary>Check convergence based on absolute function value: |f(x)| &lt; tolerance.</summary>
    public static bool IsFConverged(double theFValue, double theTolerance)
    {
        return Math.Abs(theFValue) < theTolerance;
    }

    /// <summary>Combined convergence test for scalar root finders: X or function value.</summary>
    public static bool IsConverged(double theXOld, double theXNew, double theFValue, Config theConfig)
    {
        return IsXConverged(theXOld, theXNew, theConfig.XTolerance)
               || IsFConverged(theFValue, theConfig.FTolerance);
    }

    /// <summary>Convergence test for minimization (checks both X and F change).</summary>
    public static bool IsMinConverged(double theXOld, double theXNew, double theFOld, double theFNew, Config theConfig)
    {
        // X convergence
        if (IsXConverged(theXOld, theXNew, theConfig.XTolerance))
        {
            return true;
        }

        // Relative function change convergence
        double aFDiff = Math.Abs(theFNew - theFOld);
        double aFScale = Max(1.0, Math.Abs(theFNew));
        return aFDiff < theConfig.FTolerance * aFScale;
    }

    /// <summary>Convergence test for vector solvers: max|new_i - old_i| / max(1, |new_i|) &lt; tolerance.</summary>
    public static bool IsVectorConverged(math_Vector theOld, math_Vector theNew, double theTolerance)
    {
        double aMaxDiff = 0.0;
        double aMaxScale = 1.0;
        for (int i = theOld.Lower(); i <= theOld.Upper(); ++i)
        {
            aMaxDiff = Max(aMaxDiff, Math.Abs(theNew[i] - theOld[i]));
            aMaxScale = Max(aMaxScale, Math.Abs(theNew[i]));
        }
        return aMaxDiff < theTolerance * aMaxScale;
    }

    /// <summary>Convergence test using gradient norm for minimization.</summary>
    public static bool IsGradientConverged(math_Vector theGradient, double theTolerance)
    {
        double aNormSq = 0.0;
        for (int i = theGradient.Lower(); i <= theGradient.Upper(); ++i)
        {
            aNormSq += Sqr(theGradient[i]);
        }
        return Math.Sqrt(aNormSq) < theTolerance;
    }

    /// <summary>Compute infinity norm of a vector: max|v_i|.</summary>
    public static double InfinityNorm(math_Vector theVector)
    {
        double aMax = 0.0;
        for (int i = theVector.Lower(); i <= theVector.Upper(); ++i)
        {
            aMax = Max(aMax, Math.Abs(theVector[i]));
        }
        return aMax;
    }

    /// <summary>Compute Euclidean (L2) norm of a vector.</summary>
    public static double EuclideanNorm(math_Vector theVector)
    {
        double aSumSq = 0.0;
        for (int i = theVector.Lower(); i <= theVector.Upper(); ++i)
        {
            aSumSq += Sqr(theVector[i]);
        }
        return Math.Sqrt(aSumSq);
    }
}
