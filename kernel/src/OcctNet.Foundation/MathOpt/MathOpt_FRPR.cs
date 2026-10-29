// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_FRPR.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

public static partial class MathOpt
{
    /// <summary>Conjugate gradient formula selection.</summary>
    public enum ConjugateGradientFormula
    {
        /// <summary>beta = g_new^T g_new / g^T g (original, guaranteed descent).</summary>
        FletcherReeves,
        /// <summary>beta = g_new^T (g_new - g) / g^T g (often faster, may need restarts).</summary>
        PolakRibiere,
        /// <summary>beta = g_new^T (g_new - g) / d^T (g_new - g).</summary>
        HestenesStiefel,
        /// <summary>beta = g_new^T g_new / d^T (g_new - g).</summary>
        DaiYuan,
    }

    /// <summary>Configuration for the FRPR conjugate gradient method.</summary>
    public class FRPRConfig : Config
    {
        /// <summary>Beta formula.</summary>
        public ConjugateGradientFormula Formula = ConjugateGradientFormula.PolakRibiere;
        /// <summary>Restart every N iterations (0 = n, where n is dimension).</summary>
        public int RestartInterval = 0;

        public FRPRConfig()
        {
        }

        public FRPRConfig(double theTolerance, int theMaxIter = 100)
            : base(theTolerance, theMaxIter)
        {
        }

        public FRPRConfig(FRPRConfig theOther)
            : base(theOther)
        {
            Formula = theOther.Formula;
            RestartInterval = theOther.RestartInterval;
        }
    }

    /// <summary>Fletcher-Reeves-Polak-Ribiere nonlinear conjugate gradient minimization.</summary>
    public static VectorResult FRPR<Function>(Function theFunc, math_Vector theStartingPoint, FRPRConfig? theConfig = null)
        where Function : IVectorFunctionWithGradient
    {
        theConfig ??= new FRPRConfig();
        VectorResult aResult = new();

        int aLower = theStartingPoint.Lower();
        int aUpper = theStartingPoint.Upper();
        int aN = aUpper - aLower + 1;

        // Restart interval
        int aRestartInterval = (theConfig.RestartInterval > 0) ? theConfig.RestartInterval : aN;

        // Current point
        math_Vector aX = new(aLower, aUpper);
        aX.Initialized(theStartingPoint);

        if (!theFunc.Value(aX, out double aFx))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        // Gradient at current point
        math_Vector aGrad = new(aLower, aUpper);
        if (!theFunc.Gradient(aX, aGrad))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        // Check if already at minimum
        double aGradNormSq = 0.0;
        for (int i = aLower; i <= aUpper; ++i)
        {
            aGradNormSq += MathUtils.Sqr(aGrad[i]);
        }

        if (Math.Sqrt(aGradNormSq) < theConfig.FTolerance)
        {
            aResult.Status = Status.OK;
            aResult.Solution = aX;
            aResult.Value = aFx;
            aResult.Gradient = aGrad;
            return aResult;
        }

        // Search direction (initially steepest descent)
        math_Vector aDir = new(aLower, aUpper);
        for (int i = aLower; i <= aUpper; ++i)
        {
            aDir[i] = -aGrad[i];
        }

        // Working vectors
        math_Vector aXNew = new(aLower, aUpper);
        math_Vector aGradNew = new(aLower, aUpper);
        math_Vector aGradDiff = new(aLower, aUpper);

        int aRestartCount = 0;

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Line search
            LineSearchResult aLineResult =
              MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, 1.0, 1.0e-4, 0.5, 50);

            if (!aLineResult.IsValid || aLineResult.Alpha < MathUtils.THE_EPSILON)
            {
                // Line search failed, try steepest descent
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aDir[i] = -aGrad[i];
                }
                aLineResult = MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, 1.0, 1.0e-4, 0.5, 50);

                if (!aLineResult.IsValid)
                {
                    aResult.Status = Status.NotConverged;
                    aResult.Solution = aX;
                    aResult.Value = aFx;
                    aResult.Gradient = aGrad;
                    return aResult;
                }
                aRestartCount = 0; // Reset restart counter after steepest descent
            }

            // Compute new point
            for (int i = aLower; i <= aUpper; ++i)
            {
                aXNew[i] = aX[i] + aLineResult.Alpha * aDir[i];
            }

            // Check X convergence
            double aMaxDiff = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aMaxDiff = Max(aMaxDiff, Math.Abs(aXNew[i] - aX[i]));
            }

            // Evaluate gradient at new point
            if (!theFunc.Gradient(aXNew, aGradNew))
            {
                aResult.Status = Status.NumericalError;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }

            // Check gradient convergence
            double aGradNewNormSq = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aGradNewNormSq += MathUtils.Sqr(aGradNew[i]);
            }

            if (Math.Sqrt(aGradNewNormSq) < theConfig.FTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aLineResult.FNew;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            if (aMaxDiff < theConfig.XTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aLineResult.FNew;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            // Compute gradient difference for some formulas
            for (int i = aLower; i <= aUpper; ++i)
            {
                aGradDiff[i] = aGradNew[i] - aGrad[i];
            }

            // Compute beta based on selected formula
            double aBeta = 0.0;
            ++aRestartCount;

            if (aRestartCount >= aRestartInterval)
            {
                // Periodic restart with steepest descent
                aBeta = 0.0;
                aRestartCount = 0;
            }
            else
            {
                switch (theConfig.Formula)
                {
                    case ConjugateGradientFormula.FletcherReeves:
                        // beta = g_new^T g_new / g^T g
                        if (aGradNormSq > MathUtils.THE_ZERO_TOL)
                        {
                            aBeta = aGradNewNormSq / aGradNormSq;
                        }
                        break;

                    case ConjugateGradientFormula.PolakRibiere:
                    {
                        // beta = g_new^T (g_new - g) / g^T g
                        double aDot = 0.0;
                        for (int i = aLower; i <= aUpper; ++i)
                        {
                            aDot += aGradNew[i] * aGradDiff[i];
                        }
                        if (aGradNormSq > MathUtils.THE_ZERO_TOL)
                        {
                            aBeta = aDot / aGradNormSq;
                        }
                        // Restart if beta < 0 (PR+ variant)
                        if (aBeta < 0.0)
                        {
                            aBeta = 0.0;
                            aRestartCount = 0;
                        }
                    }
                    break;

                    case ConjugateGradientFormula.HestenesStiefel:
                    {
                        // beta = g_new^T (g_new - g) / d^T (g_new - g)
                        double aNum = 0.0;
                        double aDen = 0.0;
                        for (int i = aLower; i <= aUpper; ++i)
                        {
                            aNum += aGradNew[i] * aGradDiff[i];
                            aDen += aDir[i] * aGradDiff[i];
                        }
                        if (Math.Abs(aDen) > MathUtils.THE_ZERO_TOL)
                        {
                            aBeta = aNum / aDen;
                        }
                        if (aBeta < 0.0)
                        {
                            aBeta = 0.0;
                            aRestartCount = 0;
                        }
                    }
                    break;

                    case ConjugateGradientFormula.DaiYuan:
                    {
                        // beta = g_new^T g_new / d^T (g_new - g)
                        double aDen = 0.0;
                        for (int i = aLower; i <= aUpper; ++i)
                        {
                            aDen += aDir[i] * aGradDiff[i];
                        }
                        if (Math.Abs(aDen) > MathUtils.THE_ZERO_TOL)
                        {
                            aBeta = aGradNewNormSq / aDen;
                        }
                    }
                    break;
                }
            }

            // Update search direction: p = -g_new + beta * p
            for (int i = aLower; i <= aUpper; ++i)
            {
                aDir[i] = -aGradNew[i] + aBeta * aDir[i];
            }

            // Check if direction is still a descent direction
            double aDirDeriv = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aDirDeriv += aGradNew[i] * aDir[i];
            }

            if (aDirDeriv >= 0.0)
            {
                // Not a descent direction, restart with steepest descent
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aDir[i] = -aGradNew[i];
                }
                aRestartCount = 0;
            }

            // Update for next iteration
            aX.Initialized(aXNew);
            aGrad.Initialized(aGradNew);
            aGradNormSq = aGradNewNormSq;
            aFx = aLineResult.FNew;
        }

        // Maximum iterations reached
        aResult.Status = Status.MaxIterations;
        aResult.Solution = aX;
        aResult.Value = aFx;
        aResult.Gradient = aGrad;
        return aResult;
    }

    /// <summary>The wrapper class local to FRPRNumerical: adds a central-difference gradient to a value-only function.</summary>
    private sealed class FRPRNumerical_FuncWithGradient<Function> : IVectorFunctionWithGradient
        where Function : IVectorFunction
    {
        private Function myFunc;
        private readonly double myStep;

        public FRPRNumerical_FuncWithGradient(Function theF, double theStep)
        {
            myFunc = theF;
            myStep = theStep;
        }

        public bool Value(math_Vector theX, out double theF) => myFunc.Value(theX, out theF);

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            math_Vector aXMod = new(theX);
            return MathUtils.NumericalGradientAdaptive(myFunc, aXMod, theGrad, myStep);
        }
    }

    /// <summary>FRPR with a numerical (central-difference) gradient for value-only functions.</summary>
    public static VectorResult FRPRNumerical<Function>(Function theFunc,
                                                       math_Vector theStartingPoint,
                                                       double theGradStep = 1.0e-8,
                                                       FRPRConfig? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new FRPRConfig();
        FRPRNumerical_FuncWithGradient<Function> aWrapper = new(theFunc, theGradStep);
        return FRPR(aWrapper, theStartingPoint, theConfig);
    }
}
