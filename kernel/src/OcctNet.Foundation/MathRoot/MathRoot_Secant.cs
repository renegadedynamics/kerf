// Port of: src/FoundationClasses/TKMath/MathRoot/MathRoot_Secant.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

public static partial class MathRoot
{
    /// <summary>Secant method from two initial points (no derivative needed).</summary>
    public static MathUtils.ScalarResult Secant<Function>(Function theFunc, double theX0, double theX1, MathUtils.Config? theConfig = null)
        where Function : MathUtils.IScalarFunction
    {
        theConfig ??= new MathUtils.Config();
        MathUtils.ScalarResult aResult = new();

        double aX0 = theX0;
        double aX1 = theX1;

        // Evaluate at initial points
        if (!theFunc.Value(aX0, out double aF0))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }
        if (!theFunc.Value(aX1, out double aF1))
        {
            aResult.Status = MathUtils.Status.NumericalError;
            return aResult;
        }

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Check convergence
            if (MathUtils.IsFConverged(aF1, theConfig.FTolerance))
            {
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aX1;
                aResult.Value = aF1;
                return aResult;
            }

            // Secant step
            double aDenom = aF1 - aF0;
            if (MathUtils.IsZero(aDenom))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                aResult.Root = aX1;
                aResult.Value = aF1;
                return aResult;
            }

            double aXNew = aX1 - aF1 * (aX1 - aX0) / aDenom;

            // Check X convergence
            if (MathUtils.IsXConverged(aX1, aXNew, theConfig.XTolerance))
            {
                theFunc.Value(aXNew, out double aFNew);
                aResult.Status = MathUtils.Status.OK;
                aResult.Root = aXNew;
                aResult.Value = aFNew;
                return aResult;
            }

            // Update for next iteration
            aX0 = aX1;
            aF0 = aF1;
            aX1 = aXNew;

            if (!theFunc.Value(aX1, out aF1))
            {
                aResult.Status = MathUtils.Status.NumericalError;
                aResult.Root = aX1;
                return aResult;
            }
        }

        aResult.Status = MathUtils.Status.MaxIterations;
        aResult.Root = aX1;
        aResult.Value = aF1;
        return aResult;
    }

    /// <summary>Secant method with the second point a small perturbation of theX0.</summary>
    public static MathUtils.ScalarResult SecantAuto<Function>(Function theFunc, double theX0, MathUtils.Config? theConfig = null)
        where Function : MathUtils.IScalarFunction
    {
        // Create second point by small perturbation
        double aDelta = (Math.Abs(theX0) > 1.0) ? 0.01 * theX0 : 0.01;
        return Secant(theFunc, theX0, theX0 + aDelta, theConfig);
    }
}
