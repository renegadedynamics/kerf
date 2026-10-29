// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_BFGS.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

public static partial class MathOpt
{
    /// <summary>
    /// BFGS quasi-Newton minimization with Armijo backtracking (falling back to steepest descent when the line
    /// search fails).
    /// </summary>
    public static VectorResult BFGS<Function>(Function theFunc, math_Vector theStartingPoint, Config? theConfig = null)
        where Function : IVectorFunctionWithGradient
    {
        theConfig ??= new Config();
        VectorResult aResult = new();

        int aLower = theStartingPoint.Lower();
        int aUpper = theStartingPoint.Upper();
        int aN = aUpper - aLower + 1;

        // Current point and function value
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

        // Check if already at minimum (gradient near zero)
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

        // Initialize inverse Hessian approximation to identity
        math_Matrix aH = new(1, aN, 1, aN, 0.0);
        for (int i = 1; i <= aN; ++i)
        {
            aH[i, i] = 1.0;
        }

        // Working vectors
        math_Vector aDir = new(aLower, aUpper);     // Search direction
        math_Vector aXNew = new(aLower, aUpper);    // New point
        math_Vector aGradNew = new(aLower, aUpper); // New gradient
        math_Vector aS = new(1, aN);                // Step: x_new - x
        math_Vector aY = new(1, aN);                // Gradient difference: grad_new - grad

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Compute search direction: p = -H * grad
            for (int i = 1; i <= aN; ++i)
            {
                double aSum = 0.0;
                for (int j = 1; j <= aN; ++j)
                {
                    aSum += aH[i, j] * aGrad[aLower + j - 1];
                }
                aDir[aLower + i - 1] = -aSum;
            }

            // Line search with Wolfe conditions
            LineSearchResult aLineResult =
              MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, 1.0, 1.0e-4, 0.5, 50);

            if (!aLineResult.IsValid || aLineResult.Alpha < MathUtils.THE_EPSILON)
            {
                // Line search failed, try steepest descent direction
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aDir[i] = -aGrad[i];
                }
                aLineResult = MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, 1.0, 1.0e-4, 0.5, 50);

                if (!aLineResult.IsValid)
                {
                    // Both BFGS and steepest descent failed
                    aResult.Status = Status.NotConverged;
                    aResult.Solution = aX;
                    aResult.Value = aFx;
                    aResult.Gradient = aGrad;
                    return aResult;
                }

                // Reset Hessian to identity after steepest descent step
                for (int i = 1; i <= aN; ++i)
                {
                    for (int j = 1; j <= aN; ++j)
                    {
                        aH[i, j] = (i == j) ? 1.0 : 0.0;
                    }
                }
            }

            // Compute new point
            for (int i = aLower; i <= aUpper; ++i)
            {
                aXNew[i] = aX[i] + aLineResult.Alpha * aDir[i];
            }

            // Compute s = x_new - x
            for (int i = 1; i <= aN; ++i)
            {
                aS[i] = aXNew[aLower + i - 1] - aX[aLower + i - 1];
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
                aResult.Value = aLineResult.FNew;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            // Check X convergence
            double aMaxDiff = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aMaxDiff = Max(aMaxDiff, Math.Abs(aXNew[i] - aX[i]));
            }
            if (aMaxDiff < theConfig.XTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aLineResult.FNew;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            // Compute y = grad_new - grad
            for (int i = 1; i <= aN; ++i)
            {
                aY[i] = aGradNew[aLower + i - 1] - aGrad[aLower + i - 1];
            }

            // Compute s^T * y (curvature condition)
            double aSY = 0.0;
            for (int i = 1; i <= aN; ++i)
            {
                aSY += aS[i] * aY[i];
            }

            // Skip update if curvature condition is not satisfied
            if (aSY > MathUtils.THE_ZERO_TOL)
            {
                // BFGS update: H_new = (I - rho*sy^T) H (I - rho*ys^T) + rho*ss^T
                // where rho = 1 / (s^T y)
                double aRho = 1.0 / aSY;

                // Compute H * y
                math_Vector aHy = new(1, aN, 0.0);
                for (int i = 1; i <= aN; ++i)
                {
                    for (int j = 1; j <= aN; ++j)
                    {
                        aHy[i] += aH[i, j] * aY[j];
                    }
                }

                // Compute y^T * H * y
                double aYHy = 0.0;
                for (int i = 1; i <= aN; ++i)
                {
                    aYHy += aY[i] * aHy[i];
                }

                // Update H using the formula:
                // H_new = H - (Hy*s^T + s*y^T*H)/(s^T*y) + (1 + y^T*H*y/(s^T*y)) * s*s^T/(s^T*y)
                double aFactor = 1.0 + aRho * aYHy;

                for (int i = 1; i <= aN; ++i)
                {
                    for (int j = 1; j <= aN; ++j)
                    {
                        aH[i, j] =
                          aH[i, j] - aRho * (aHy[i] * aS[j] + aS[i] * aHy[j]) + aFactor * aRho * aS[i] * aS[j];
                    }
                }
            }

            // Update for next iteration
            aX.Initialized(aXNew);
            aGrad.Initialized(aGradNew);
            aFx = aLineResult.FNew;
        }

        // Maximum iterations reached
        aResult.Status = Status.MaxIterations;
        aResult.Solution = aX;
        aResult.Value = aFx;
        aResult.Gradient = aGrad;
        return aResult;
    }

    /// <summary>The wrapper class local to BFGSNumerical: adds a central-difference gradient to a value-only function.</summary>
    private sealed class BFGSNumerical_FuncWithGradient<Function> : IVectorFunctionWithGradient
        where Function : IVectorFunction
    {
        private Function myFunc;
        private readonly double myStep;

        public BFGSNumerical_FuncWithGradient(Function theF, double theStep)
        {
            myFunc = theF;
            myStep = theStep;
        }

        public bool Value(math_Vector theX, out double theF) => myFunc.Value(theX, out theF);

        public bool Gradient(math_Vector theX, math_Vector theGrad)
        {
            math_Vector aXMod = new(theX); // Make mutable copy
            return MathUtils.NumericalGradientAdaptive(myFunc, aXMod, theGrad, myStep);
        }
    }

    /// <summary>BFGS with a numerical (central-difference, adaptive step) gradient for value-only functions.</summary>
    public static VectorResult BFGSNumerical<Function>(Function theFunc,
                                                       math_Vector theStartingPoint,
                                                       double theGradStep = 1.0e-8,
                                                       Config? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new Config();
        BFGSNumerical_FuncWithGradient<Function> aWrapper = new(theFunc, theGradStep);
        return BFGS(aWrapper, theStartingPoint, theConfig);
    }

    /// <summary>L-BFGS (limited-memory BFGS) keeping the theMemorySize most recent {s, y} pairs.</summary>
    public static VectorResult LBFGS<Function>(Function theFunc,
                                               math_Vector theStartingPoint,
                                               int theMemorySize = 10,
                                               Config? theConfig = null)
        where Function : IVectorFunctionWithGradient
    {
        theConfig ??= new Config();
        VectorResult aResult = new();

        int aLower = theStartingPoint.Lower();
        int aUpper = theStartingPoint.Upper();
        int aN = aUpper - aLower + 1;
        int aM = theMemorySize;

        math_Vector aX = new(aLower, aUpper);
        aX.Initialized(theStartingPoint);

        if (!theFunc.Value(aX, out double aFx))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        math_Vector aGrad = new(aLower, aUpper);
        if (!theFunc.Gradient(aX, aGrad))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        // Storage for {s, y} pairs (circular buffer)
        NCollection_DynamicArray<math_Vector> aSVec = new();
        NCollection_DynamicArray<math_Vector> aYVec = new();
        math_Vector aRhoVec = new(0, aM - 1, 0.0);
        for (int i = 0; i < aM; ++i)
        {
            aSVec.Append(new math_Vector(1, aN));
            aYVec.Append(new math_Vector(1, aN));
        }
        int aHead = 0;  // Index of oldest entry
        int aCount = 0; // Number of stored pairs

        math_Vector aDir = new(aLower, aUpper);
        math_Vector aXNew = new(aLower, aUpper);
        math_Vector aGradNew = new(aLower, aUpper);
        math_Vector aQ = new(1, aN);
        math_Vector aAlphaVec = new(0, aM - 1);

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Check gradient convergence
            double aGradNorm = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aGradNorm += MathUtils.Sqr(aGrad[i]);
            }
            if (Math.Sqrt(aGradNorm) < theConfig.FTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aX;
                aResult.Value = aFx;
                aResult.Gradient = aGrad;
                return aResult;
            }

            // L-BFGS two-loop recursion to compute search direction
            // q = gradient
            for (int i = 1; i <= aN; ++i)
            {
                aQ[i] = aGrad[aLower + i - 1];
            }

            // First loop (backward)
            for (int k = aCount - 1; k >= 0; --k)
            {
                int aIdx = (aHead + k) % aM;
                aAlphaVec[aIdx] = aRhoVec[aIdx] * MathUtils.DotProduct(aSVec.Value(aIdx), aQ);
                for (int i = 1; i <= aN; ++i)
                {
                    aQ[i] -= aAlphaVec[aIdx] * aYVec.Value(aIdx)[i];
                }
            }

            // Initial Hessian: H0 = gamma*I where gamma = s^T y / y^T y
            double aGamma = 1.0;
            if (aCount > 0)
            {
                int aLastIdx = (aHead + aCount - 1) % aM;
                double aYY = MathUtils.DotProduct(aYVec.Value(aLastIdx), aYVec.Value(aLastIdx));
                if (aYY > MathUtils.THE_ZERO_TOL)
                {
                    aGamma = 1.0 / (aRhoVec[aLastIdx] * aYY);
                }
            }

            // r = H0 * q = gamma * q
            math_Vector aR = new(1, aN);
            for (int i = 1; i <= aN; ++i)
            {
                aR[i] = aGamma * aQ[i];
            }

            // Second loop (forward)
            for (int k = 0; k < aCount; ++k)
            {
                int aIdx = (aHead + k) % aM;
                double aBeta = aRhoVec[aIdx] * MathUtils.DotProduct(aYVec.Value(aIdx), aR);
                for (int i = 1; i <= aN; ++i)
                {
                    aR[i] += (aAlphaVec[aIdx] - aBeta) * aSVec.Value(aIdx)[i];
                }
            }

            // Search direction: p = -r
            for (int i = aLower; i <= aUpper; ++i)
            {
                aDir[i] = -aR[i - aLower + 1];
            }

            // Line search
            LineSearchResult aLineResult =
              MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, 1.0, 1.0e-4, 0.5, 50);

            if (!aLineResult.IsValid)
            {
                // Fall back to steepest descent
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
                    return aResult;
                }
                // Reset history after steepest descent
                aCount = 0;
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
            if (aMaxDiff < theConfig.XTolerance)
            {
                if (!theFunc.Gradient(aXNew, aGradNew))
                {
                    aGradNew.Initialized(aGrad);
                }
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aLineResult.FNew;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            // Evaluate gradient at new point
            if (!theFunc.Gradient(aXNew, aGradNew))
            {
                aResult.Status = Status.NumericalError;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }

            // Store new {s, y} pair
            int aNewIdx = (aHead + aCount) % aM;
            for (int i = 1; i <= aN; ++i)
            {
                aSVec.ChangeValue(aNewIdx)[i] = aXNew[aLower + i - 1] - aX[aLower + i - 1];
                aYVec.ChangeValue(aNewIdx)[i] = aGradNew[aLower + i - 1] - aGrad[aLower + i - 1];
            }

            double aSY = MathUtils.DotProduct(aSVec.Value(aNewIdx), aYVec.Value(aNewIdx));
            if (aSY > MathUtils.THE_ZERO_TOL)
            {
                aRhoVec[aNewIdx] = 1.0 / aSY;
                if (aCount < aM)
                {
                    ++aCount;
                }
                else
                {
                    aHead = (aHead + 1) % aM;
                }
            }

            // Update for next iteration
            aX.Initialized(aXNew);
            aGrad.Initialized(aGradNew);
            aFx = aLineResult.FNew;
        }

        aResult.Status = Status.MaxIterations;
        aResult.Solution = aX;
        aResult.Value = aFx;
        aResult.Gradient = aGrad;
        return aResult;
    }

    /// <summary>
    /// BFGS with box constraints theLowerBounds &lt;= x &lt;= theUpperBounds, by projection: points are clamped and
    /// gradient components pointing out of an active bound are zeroed.
    /// </summary>
    public static VectorResult BFGSBounded<Function>(Function theFunc,
                                                     math_Vector theStartingPoint,
                                                     math_Vector theLowerBounds,
                                                     math_Vector theUpperBounds,
                                                     Config? theConfig = null)
        where Function : IVectorFunctionWithGradient
    {
        theConfig ??= new Config();
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

                // At lower bound and gradient points downward -> zero gradient
                if (theX[i] - theLowerBounds[aBndIdx] < aTol && theGrad[i] > 0.0)
                {
                    theGrad[i] = 0.0;
                }
                // At upper bound and gradient points upward -> zero gradient
                if (theUpperBounds[aBndIdx] - theX[i] < aTol && theGrad[i] < 0.0)
                {
                    theGrad[i] = 0.0;
                }
            }
        }

        // Current point and function value
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

        // Initialize inverse Hessian approximation to identity
        math_Matrix aH = new(1, aN, 1, aN, 0.0);
        for (int i = 1; i <= aN; ++i)
        {
            aH[i, i] = 1.0;
        }

        // Working vectors
        math_Vector aDir = new(aLower, aUpper);
        math_Vector aXNew = new(aLower, aUpper);
        math_Vector aGradNew = new(aLower, aUpper);
        math_Vector aS = new(1, aN);
        math_Vector aY = new(1, aN);

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            // Compute search direction: p = -H * grad
            for (int i = 1; i <= aN; ++i)
            {
                double aSum = 0.0;
                for (int j = 1; j <= aN; ++j)
                {
                    aSum += aH[i, j] * aGrad[aLower + j - 1];
                }
                aDir[aLower + i - 1] = -aSum;
            }

            // Line search with bounds-aware step
            double aAlphaMax = 1.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                int aBndIdx = theLowerBounds.Lower() + (i - aLower);
                if (aDir[i] < -MathUtils.THE_EPSILON)
                {
                    // Moving toward lower bound
                    double aMaxStep = (theLowerBounds[aBndIdx] - aX[i]) / aDir[i];
                    aAlphaMax = Min(aAlphaMax, aMaxStep);
                }
                else if (aDir[i] > MathUtils.THE_EPSILON)
                {
                    // Moving toward upper bound
                    double aMaxStep = (theUpperBounds[aBndIdx] - aX[i]) / aDir[i];
                    aAlphaMax = Min(aAlphaMax, aMaxStep);
                }
            }
            aAlphaMax = Max(aAlphaMax, MathUtils.THE_EPSILON);

            LineSearchResult aLineResult =
              MathUtils.ArmijoBacktrack(theFunc, aX, aDir, aGrad, aFx, aAlphaMax, 1.0e-4, 0.5, 50);

            if (!aLineResult.IsValid || aLineResult.Alpha < MathUtils.THE_EPSILON)
            {
                // Fall back to projected steepest descent
                for (int i = aLower; i <= aUpper; ++i)
                {
                    aDir[i] = -aGrad[i];
                }
                // Recompute alpha max for steepest descent
                aAlphaMax = 1.0;
                for (int i = aLower; i <= aUpper; ++i)
                {
                    int aBndIdx = theLowerBounds.Lower() + (i - aLower);
                    if (aDir[i] < -MathUtils.THE_EPSILON)
                    {
                        aAlphaMax = Min(aAlphaMax, (theLowerBounds[aBndIdx] - aX[i]) / aDir[i]);
                    }
                    else if (aDir[i] > MathUtils.THE_EPSILON)
                    {
                        aAlphaMax = Min(aAlphaMax, (theUpperBounds[aBndIdx] - aX[i]) / aDir[i]);
                    }
                }
                aAlphaMax = Max(aAlphaMax, MathUtils.THE_EPSILON);

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

                // Reset Hessian
                for (int i = 1; i <= aN; ++i)
                {
                    for (int j = 1; j <= aN; ++j)
                    {
                        aH[i, j] = (i == j) ? 1.0 : 0.0;
                    }
                }
            }

            // Compute and clamp new point
            for (int i = aLower; i <= aUpper; ++i)
            {
                aXNew[i] = aX[i] + aLineResult.Alpha * aDir[i];
            }
            ClampToBounds(aXNew);

            // Compute s = x_new - x
            for (int i = 1; i <= aN; ++i)
            {
                aS[i] = aXNew[aLower + i - 1] - aX[aLower + i - 1];
            }

            // Evaluate at new point
            if (!theFunc.Value(aXNew, out double aFxNew))
            {
                aResult.Status = Status.NumericalError;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }

            if (!theFunc.Gradient(aXNew, aGradNew))
            {
                aResult.Status = Status.NumericalError;
                aResult.Solution = aX;
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
                aResult.Value = aFxNew;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            // Check X convergence
            double aMaxDiff = 0.0;
            for (int i = aLower; i <= aUpper; ++i)
            {
                aMaxDiff = Max(aMaxDiff, Math.Abs(aXNew[i] - aX[i]));
            }
            if (aMaxDiff < theConfig.XTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aXNew;
                aResult.Value = aFxNew;
                aResult.Gradient = aGradNew;
                return aResult;
            }

            // Compute y = grad_new - grad
            for (int i = 1; i <= aN; ++i)
            {
                aY[i] = aGradNew[aLower + i - 1] - aGrad[aLower + i - 1];
            }

            // Curvature condition
            double aSY = 0.0;
            for (int i = 1; i <= aN; ++i)
            {
                aSY += aS[i] * aY[i];
            }

            if (aSY > MathUtils.THE_ZERO_TOL)
            {
                double aRho = 1.0 / aSY;

                math_Vector aHy = new(1, aN, 0.0);
                for (int i = 1; i <= aN; ++i)
                {
                    for (int j = 1; j <= aN; ++j)
                    {
                        aHy[i] += aH[i, j] * aY[j];
                    }
                }

                double aYHy = 0.0;
                for (int i = 1; i <= aN; ++i)
                {
                    aYHy += aY[i] * aHy[i];
                }

                double aFactor = 1.0 + aRho * aYHy;
                for (int i = 1; i <= aN; ++i)
                {
                    for (int j = 1; j <= aN; ++j)
                    {
                        aH[i, j] =
                          aH[i, j] - aRho * (aHy[i] * aS[j] + aS[i] * aHy[j]) + aFactor * aRho * aS[i] * aS[j];
                    }
                }
            }

            aX.Initialized(aXNew);
            aGrad.Initialized(aGradNew);
            aFx = aFxNew;
        }

        aResult.Status = Status.MaxIterations;
        aResult.Solution = aX;
        aResult.Value = aFx;
        aResult.Gradient = aGrad;
        return aResult;
    }
}
