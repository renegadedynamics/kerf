// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_Newton.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

public static partial class MathOpt
{
    /// <summary>Configuration for Newton minimization with Hessian.</summary>
    public class NewtonConfig : Config
    {
        /// <summary>Diagonal regularization for non-positive definite Hessian.</summary>
        public double Regularization = 1.0e-8;
        /// <summary>Whether to use line search (recommended).</summary>
        public bool UseLineSearch = true;

        public NewtonConfig()
        {
        }

        public NewtonConfig(double theTolerance, int theMaxIter = 100)
            : base(theTolerance, theMaxIter)
        {
        }

        public NewtonConfig(NewtonConfig theOther)
            : base(theOther)
        {
            Regularization = theOther.Regularization;
            UseLineSearch = theOther.UseLineSearch;
        }
    }

    /// <summary>
    /// Newton's method for N-dimensional minimization using the Hessian, with line search and diagonal
    /// regularization when the Hessian system can't be solved.
    /// </summary>
    public static VectorResult Newton<Function>(Function theFunc, math_Vector theStartingPoint, NewtonConfig? theConfig = null)
        where Function : IVectorFunctionWithHessian
    {
        theConfig ??= new NewtonConfig();
        VectorResult aResult = new();

        int aLower = theStartingPoint.Lower();
        int aUpper = theStartingPoint.Upper();

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
        double aGradNorm = 0.0;
        for (int i = aLower; i <= aUpper; ++i)
        {
            aGradNorm += MathUtils.Sqr(aGrad[i]);
        }
        aGradNorm = Math.Sqrt(aGradNorm);

        if (aGradNorm < theConfig.FTolerance)
        {
            aResult.Status = Status.OK;
            aResult.Solution = aX;
            aResult.Value = aFx;
            aResult.Gradient = aGrad;
            return aResult;
        }

        // Working vectors and matrices
        math_Vector aDir = new(aLower, aUpper);
        math_Vector aXNew = new(aLower, aUpper);
        math_Vector aGradNew = new(aLower, aUpper);
        math_Matrix aHessian = new(aLower, aUpper, aLower, aUpper);
        math_Vector aNegGrad = new(aLower, aUpper);

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Compute Hessian
            if (!theFunc.Hessian(aX, aHessian))
            {
                aResult.Status = Status.NumericalError;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }

            // Prepare negative gradient
            for (int i = aLower; i <= aUpper; ++i)
            {
                aNegGrad[i] = -aGrad[i];
            }

            // Try to solve H * p = -g
            LinearResult aLinResult = MathLin.Solve(aHessian, aNegGrad);

            if (!aLinResult.IsDone())
            {
                // Hessian is singular or not positive definite, add regularization
                double aLambda = theConfig.Regularization;
                bool aSolved = false;

                for (int k = 0; k < 10 && !aSolved; ++k)
                {
                    math_Matrix aRegHessian = new(aHessian);
                    for (int i = aLower; i <= aUpper; ++i)
                    {
                        aRegHessian[i, i] += aLambda;
                    }

                    aLinResult = MathLin.Solve(aRegHessian, aNegGrad);
                    if (aLinResult.IsDone())
                    {
                        aSolved = true;
                    }
                    else
                    {
                        aLambda *= 10.0;
                    }
                }

                if (!aSolved)
                {
                    // Fall back to steepest descent
                    for (int i = aLower; i <= aUpper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                    goto perform_line_search;
                }
            }

            aDir.Initialized(aLinResult.Solution!);

            // Check if direction is descent
            {
                double aDirDeriv = 0.0;
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aDirDeriv += aGrad[i] * aDir[i];
                }

                if (aDirDeriv >= 0.0)
                {
                    // Not a descent direction, use steepest descent
                    for (int i = aLower; i <= aUpper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                }
            }

        perform_line_search:
            if (theConfig.UseLineSearch)
            {
                // Line search
                LineSearchResult aLineResult =
                  MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, 1.0, 1.0e-4, 0.5, 50);

                if (!aLineResult.IsValid)
                {
                    // Line search failed, try steepest descent
                    for (int i = aLower; i <= aUpper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                    aLineResult =
                      MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, 1.0, 1.0e-4, 0.5, 50);

                    if (!aLineResult.IsValid)
                    {
                        aResult.Status = Status.NotConverged;
                        aResult.Solution = aX;
                        aResult.Value = aFx;
                        aResult.Gradient = aGrad;
                        return aResult;
                    }
                }

                // Compute new point
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aXNew[i] = aX[i] + aLineResult.Alpha * aDir[i];
                }
                aFx = aLineResult.FNew;
            }
            else
            {
                // Full Newton step (no line search)
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aXNew[i] = aX[i] + aDir[i];
                }

                if (!theFunc.Value(aXNew, out aFx))
                {
                    aResult.Status = Status.NumericalError;
                    aResult.Solution = aX;
                    return aResult;
                }
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
                aResult.Solution = aXNew;
                aResult.Value = aFx;
                return aResult;
            }

            // Check gradient convergence
            aGradNorm = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aGradNorm += MathUtils.Sqr(aGradNew[i]);
            }
            aGradNorm = Math.Sqrt(aGradNorm);

            if (aGradNorm < theConfig.FTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aFx;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            if (aMaxDiff < theConfig.XTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aFx;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            // Update for next iteration
            aX.Initialized(aXNew);
            aGrad.Initialized(aGradNew);
        }

        // Maximum iterations reached
        aResult.Status = Status.MaxIterations;
        aResult.Solution = aX;
        aResult.Value = aFx;
        aResult.Gradient = aGrad;
        return aResult;
    }

    /// <summary>Modified Newton's method with automatic Hessian regularization: the same as Newton.</summary>
    public static VectorResult NewtonModified<Function>(Function theFunc, math_Vector theStartingPoint, NewtonConfig? theConfig = null)
        where Function : IVectorFunctionWithHessian
    {
        theConfig ??= new NewtonConfig();
        return Newton(theFunc, theStartingPoint, theConfig);
    }

    /// <summary>The wrapper class local to NewtonNumericalHessian: adds a finite-difference Hessian.</summary>
    private sealed class NewtonNumericalHessian_FuncWithHessian<Function> : IVectorFunctionWithHessian
        where Function : IVectorFunctionWithGradient
    {
        private Function myFunc;
        private readonly double myStep;

        public NewtonNumericalHessian_FuncWithHessian(Function theF, double theStep)
        {
            myFunc = theF;
            myStep = theStep;
        }

        public bool Value(math_Vector theX, out double theF) => myFunc.Value(theX, out theF);

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            return myFunc.Gradient(theX, theGrad);
        }

        public bool Hessian(math_Vector theX, math_Matrix theHess)
        {
            math_Vector aXMod = new(theX);
            // OCCT: NumericalHessian fills the matrix from index 1, but Newton's Hessian has the starting point's
            // bounds: with a lower bound other than 1 the Hessian is shifted or out of range.
            return MathUtils.NumericalHessian(myFunc, aXMod, theHess, myStep);
        }
    }

    /// <summary>Newton's method with a numerical (finite-difference) Hessian.</summary>
    public static VectorResult NewtonNumericalHessian<Function>(Function theFunc,
                                                                math_Vector theStartingPoint,
                                                                double theHessStep = 1.0e-6,
                                                                NewtonConfig? theConfig = null)
        where Function : IVectorFunctionWithGradient
    {
        theConfig ??= new NewtonConfig();
        NewtonNumericalHessian_FuncWithHessian<Function> aWrapper = new(theFunc, theHessStep);
        return Newton(aWrapper, theStartingPoint, theConfig);
    }

    /// <summary>The wrapper class local to NewtonNumerical: adds a numerical gradient and Hessian.</summary>
    private sealed class NewtonNumerical_FuncWithDerivatives<Function> : IVectorFunctionWithHessian
        where Function : IVectorFunction
    {
        private Function myFunc;
        private readonly double myGradStep;
        private readonly double myHessStep;

        public NewtonNumerical_FuncWithDerivatives(Function theF, double theGStep, double theHStep)
        {
            myFunc = theF;
            myGradStep = theGStep;
            myHessStep = theHStep;
        }

        public bool Value(math_Vector theX, out double theF) => myFunc.Value(theX, out theF);

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            math_Vector aXMod = new(theX);
            return MathUtils.NumericalGradientAdaptive(myFunc, aXMod, theGrad, myGradStep);
        }

        public bool Hessian(math_Vector theX, math_Matrix theHess)
        {
            // Compute Hessian from finite differences of gradient
            int aLower = theX.Lower();
            int aUpper = theX.Upper();

            math_Vector aXMod = new(theX);
            math_Vector aGradPlus = new(aLower, aUpper);
            math_Vector aGradMinus = new(aLower, aUpper);

            for (int j = aLower; j <= aUpper; ++j)
            {
                double aXj = aXMod[j];

                aXMod[j] = aXj + myHessStep;
                if (!MathUtils.NumericalGradientAdaptive(myFunc, aXMod, aGradPlus, myGradStep))
                {
                    aXMod[j] = aXj;
                    return false;
                }

                aXMod[j] = aXj - myHessStep;
                if (!MathUtils.NumericalGradientAdaptive(myFunc, aXMod, aGradMinus, myGradStep))
                {
                    aXMod[j] = aXj;
                    return false;
                }

                aXMod[j] = aXj;

                for (int i = aLower; i <= aUpper; ++i)
                {
                    theHess[i, j] = (aGradPlus[i] - aGradMinus[i]) / (2.0 * myHessStep);
                }
            }

            // Symmetrize
            for (int i = aLower; i <= aUpper; ++i)
            {
                for (int j = i + 1; j <= aUpper; ++j)
                {
                    double aAvg = 0.5 * (theHess[i, j] + theHess[j, i]);
                    theHess[i, j] = aAvg;
                    theHess[j, i] = aAvg;
                }
            }

            return true;
        }
    }

    /// <summary>Newton's method with numerical gradient and Hessian for value-only functions.</summary>
    public static VectorResult NewtonNumerical<Function>(Function theFunc,
                                                         math_Vector theStartingPoint,
                                                         double theGradStep = 1.0e-8,
                                                         double theHessStep = 1.0e-6,
                                                         NewtonConfig? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new NewtonConfig();
        NewtonNumerical_FuncWithDerivatives<Function> aWrapper = new(theFunc, theGradStep, theHessStep);
        return Newton(aWrapper, theStartingPoint, theConfig);
    }

    /// <summary>Newton's method with box constraints, by projection (as BFGSBounded).</summary>
    public static VectorResult NewtonBounded<Function>(Function theFunc,
                                                       math_Vector theStartingPoint,
                                                       math_Vector theLowerBounds,
                                                       math_Vector theUpperBounds,
                                                       NewtonConfig? theConfig = null)
        where Function : IVectorFunctionWithHessian
    {
        theConfig ??= new NewtonConfig();
        VectorResult aResult = new();

        int aLower = theStartingPoint.Lower();
        int aUpper = theStartingPoint.Upper();
        int aN = aUpper - aLower + 1;

        // Check dimensions
        if (theLowerBounds.Length() != aN || theUpperBounds.Length() != aN)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        // Lambda to clamp a point to bounds
        void ClampToBounds(math_Vector theX)
        {
            for (int i = aLower; i <= aUpper; ++i)
            {
                int aBndIdx = theLowerBounds.Lower() + (i - aLower);
                if (theX[i] < theLowerBounds[aBndIdx])
                {
                    theX[i] = theLowerBounds[aBndIdx];
                }
                if (theX[i] > theUpperBounds[aBndIdx])
                {
                    theX[i] = theUpperBounds[aBndIdx];
                }
            }
        }

        // Lambda to project gradient (zero components at active bounds)
        void ProjectGradient(math_Vector theX, math_Vector theGrad)
        {
            for (int i = aLower; i <= aUpper; ++i)
            {
                int aBndIdx = theLowerBounds.Lower() + (i - aLower);
                double aTol = MathUtils.THE_EPSILON * Max(1.0, Math.Abs(theX[i]));

                if (theX[i] - theLowerBounds[aBndIdx] < aTol && theGrad[i] > 0.0)
                {
                    theGrad[i] = 0.0;
                }
                if (theUpperBounds[aBndIdx] - theX[i] < aTol && theGrad[i] < 0.0)
                {
                    theGrad[i] = 0.0;
                }
            }
        }

        // Lambda to compute max step to boundary
        double ComputeAlphaMax(math_Vector theX, math_Vector theDir)
        {
            double aAlphaMax = 1.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                int aBndIdx = theLowerBounds.Lower() + (i - aLower);
                if (theDir[i] < -MathUtils.THE_EPSILON)
                {
                    double aMaxStep = (theLowerBounds[aBndIdx] - theX[i]) / theDir[i];
                    aAlphaMax = Min(aAlphaMax, aMaxStep);
                }
                else if (theDir[i] > MathUtils.THE_EPSILON)
                {
                    double aMaxStep = (theUpperBounds[aBndIdx] - theX[i]) / theDir[i];
                    aAlphaMax = Min(aAlphaMax, aMaxStep);
                }
            }
            return Max(aAlphaMax, MathUtils.THE_EPSILON);
        }

        // Current point
        math_Vector aX = new(aLower, aUpper);
        aX.Initialized(theStartingPoint);
        ClampToBounds(aX);

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
        ProjectGradient(aX, aGrad);

        // Check if already at minimum
        double aGradNorm = 0.0;
        for (int i = aLower; i <= aUpper; ++i)
        {
            aGradNorm += MathUtils.Sqr(aGrad[i]);
        }
        aGradNorm = Math.Sqrt(aGradNorm);

        if (aGradNorm < theConfig.FTolerance)
        {
            aResult.Status = Status.OK;
            aResult.Solution = aX;
            aResult.Value = aFx;
            aResult.Gradient = aGrad;
            return aResult;
        }

        // Working vectors and matrices
        math_Vector aDir = new(aLower, aUpper);
        math_Vector aXNew = new(aLower, aUpper);
        math_Vector aGradNew = new(aLower, aUpper);
        math_Matrix aHessian = new(aLower, aUpper, aLower, aUpper);
        math_Vector aNegGrad = new(aLower, aUpper);

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Compute Hessian
            if (!theFunc.Hessian(aX, aHessian))
            {
                aResult.Status = Status.NumericalError;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }

            // Prepare negative gradient
            for (int i = aLower; i <= aUpper; ++i)
            {
                aNegGrad[i] = -aGrad[i];
            }

            // Try to solve H * p = -g
            LinearResult aLinResult = MathLin.Solve(aHessian, aNegGrad);

            if (!aLinResult.IsDone())
            {
                // Add regularization
                double aLambda = theConfig.Regularization;
                bool aSolved = false;

                for (int k = 0; k < 10 && !aSolved; ++k)
                {
                    math_Matrix aRegHessian = new(aHessian);
                    for (int i = aLower; i <= aUpper; ++i)
                    {
                        aRegHessian[i, i] += aLambda;
                    }

                    aLinResult = MathLin.Solve(aRegHessian, aNegGrad);
                    if (aLinResult.IsDone())
                    {
                        aSolved = true;
                    }
                    else
                    {
                        aLambda *= 10.0;
                    }
                }

                if (!aSolved)
                {
                    // Fall back to steepest descent
                    for (int i = aLower; i <= aUpper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                    goto perform_bounded_line_search;
                }
            }

            aDir.Initialized(aLinResult.Solution!);

            // Check if direction is descent
            {
                double aDirDeriv = 0.0;
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aDirDeriv += aGrad[i] * aDir[i];
                }

                if (aDirDeriv >= 0.0)
                {
                    for (int i = aLower; i <= aUpper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                }
            }

        perform_bounded_line_search:
            if (theConfig.UseLineSearch)
            {
                double aAlphaMax = ComputeAlphaMax(aX, aDir);

                LineSearchResult aLineResult =
                  MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, aAlphaMax, 1.0e-4, 0.5, 50);

                if (!aLineResult.IsValid)
                {
                    // Try steepest descent
                    for (int i = aLower; i <= aUpper; ++i)
                    {
                        aDir[i] = -aGrad[i];
                    }
                    aAlphaMax = ComputeAlphaMax(aX, aDir);
                    aLineResult =
                      MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, aAlphaMax, 1.0e-4, 0.5, 50);

                    if (!aLineResult.IsValid)
                    {
                        aResult.Status = Status.NotConverged;
                        aResult.Solution = aX;
                        aResult.Value = aFx;
                        aResult.Gradient = aGrad;
                        return aResult;
                    }
                }

                for (int i = aLower; i <= aUpper; ++i)
                {
                    aXNew[i] = aX[i] + aLineResult.Alpha * aDir[i];
                }
                ClampToBounds(aXNew);

                if (!theFunc.Value(aXNew, out aFx))
                {
                    aResult.Status = Status.NumericalError;
                    aResult.Solution = aX;
                    return aResult;
                }
            }
            else
            {
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aXNew[i] = aX[i] + aDir[i];
                }
                ClampToBounds(aXNew);

                if (!theFunc.Value(aXNew, out aFx))
                {
                    aResult.Status = Status.NumericalError;
                    aResult.Solution = aX;
                    return aResult;
                }
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
                aResult.Solution = aXNew;
                aResult.Value = aFx;
                return aResult;
            }
            ProjectGradient(aXNew, aGradNew);

            // Check gradient convergence
            aGradNorm = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aGradNorm += MathUtils.Sqr(aGradNew[i]);
            }
            aGradNorm = Math.Sqrt(aGradNorm);

            if (aGradNorm < theConfig.FTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aFx;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            if (aMaxDiff < theConfig.XTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aFx;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            aX.Initialized(aXNew);
            aGrad.Initialized(aGradNew);
        }

        aResult.Status = Status.MaxIterations;
        aResult.Solution = aX;
        aResult.Value = aFx;
        aResult.Gradient = aGrad;
        return aResult;
    }
}
