// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_Powell.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

public static partial class MathOpt
{
    /// <summary>
    /// Powell's conjugate direction method for N-dimensional minimization (gradient-free), starting from the
    /// coordinate axes.
    /// </summary>
    public static VectorResult Powell<Function>(Function theFunc, math_Vector theStartingPoint, Config? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new Config();
        VectorResult aResult = new();

        int aLower = theStartingPoint.Lower();
        int aUpper = theStartingPoint.Upper();
        int aN = aUpper - aLower + 1;

        // Current point
        math_Vector aX = new(aLower, aUpper);
        aX.Initialized(theStartingPoint);

        // Function value at current point
        if (!theFunc.Value(aX, out double aFx))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        // Initialize direction set to coordinate axes
        // Directions stored as rows of matrix
        math_Matrix aDirections = new(1, aN, 1, aN, 0.0);
        for (int i = 1; i <= aN; ++i)
        {
            aDirections[i, i] = 1.0;
        }

        // Working vectors
        math_Vector aDir = new(aLower, aUpper);
        math_Vector aXOld = new(aLower, aUpper);
        math_Vector aPt = new(aLower, aUpper);
        math_Vector aPtt = new(aLower, aUpper);
        math_Vector aXit = new(aLower, aUpper);
        _ = aPt; // unused, as in C++

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            double aFp = aFx;
            aXOld.Initialized(aX);

            // Track largest decrease and its direction index
            double aDel = 0.0;
            int aIBig = 0;

            // Minimize along each direction
            for (int i = 1; i <= aN; ++i)
            {
                // Extract direction i
                for (int j = aLower; j <= aUpper; ++j)
                {
                    aDir[j] = aDirections[i, j - aLower + 1];
                }

                double aFpPrev = aFx;

                // Line minimization along direction
                LineSearchResult aLineResult =
                  MathUtils.ExactLineSearch(theFunc, aX, aDir, 10.0, theConfig.XTolerance);

                if (aLineResult.IsValid)
                {
                    // Update position
                    for (int j = aLower; j <= aUpper; ++j)
                    {
                        aX[j] += aLineResult.Alpha * aDir[j];
                    }
                    aFx = aLineResult.FNew;

                    // Track direction with largest decrease
                    double aDecrease = aFpPrev - aFx;
                    if (aDecrease > aDel)
                    {
                        aDel = aDecrease;
                        aIBig = i;
                    }
                }
            }

            // Check convergence
            if (2.0 * Math.Abs(aFp - aFx)
                <= theConfig.FTolerance * (Math.Abs(aFp) + Math.Abs(aFx) + MathUtils.THE_ZERO_TOL))
            {
                aResult.Status = Status.OK;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }

            // Construct extrapolated point and new direction
            for (int j = aLower; j <= aUpper; ++j)
            {
                aPtt[j] = 2.0 * aX[j] - aXOld[j];
                aXit[j] = aX[j] - aXOld[j];
            }

            // Evaluate at extrapolated point
            if (!theFunc.Value(aPtt, out double aFptt))
            {
                // If evaluation fails, continue with current directions
                continue;
            }

            // Check if new direction should be added
            if (aFptt < aFp)
            {
                double aT = 2.0 * (aFp - 2.0 * aFx + aFptt) * MathUtils.Sqr(aFp - aFx - aDel)
                            - aDel * MathUtils.Sqr(aFp - aFptt);

                if (aT < 0.0)
                {
                    // Minimize along new direction
                    LineSearchResult aLineResult =
                      MathUtils.ExactLineSearch(theFunc, aX, aXit, 10.0, theConfig.XTolerance);

                    if (aLineResult.IsValid)
                    {
                        // Update position
                        for (int j = aLower; j <= aUpper; ++j)
                        {
                            aX[j] += aLineResult.Alpha * aXit[j];
                        }
                        aFx = aLineResult.FNew;

                        // Replace direction with largest decrease
                        if (aIBig > 0)
                        {
                            for (int j = 1; j <= aN; ++j)
                            {
                                aDirections[aIBig, j] = aDirections[aN, j];
                                aDirections[aN, j] = aXit[aLower + j - 1];
                            }
                        }
                    }
                }
            }

            // Check X convergence
            double aMaxDiff = 0.0;
            for (int j = aLower; j <= aUpper; ++j)
            {
                aMaxDiff = Max(aMaxDiff, Math.Abs(aX[j] - aXOld[j]));
            }
            if (aMaxDiff < theConfig.XTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }
        }

        // Maximum iterations reached
        aResult.Status = Status.MaxIterations;
        aResult.Solution = aX;
        aResult.Value = aFx;
        return aResult;
    }

    /// <summary>Powell's method with a custom initial direction set (N x N, directions as rows).</summary>
    public static VectorResult PowellWithDirections<Function>(Function theFunc,
                                                              math_Vector theStartingPoint,
                                                              math_Matrix theInitialDirections,
                                                              Config? theConfig = null)
        where Function : IVectorFunction
    {
        theConfig ??= new Config();
        VectorResult aResult = new();

        int aLower = theStartingPoint.Lower();
        int aUpper = theStartingPoint.Upper();
        int aN = aUpper - aLower + 1;

        // Validate dimensions
        if (theInitialDirections.RowNumber() != aN || theInitialDirections.ColNumber() != aN)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        math_Vector aX = new(aLower, aUpper);
        aX.Initialized(theStartingPoint);

        if (!theFunc.Value(aX, out double aFx))
        {
            aResult.Status = Status.NumericalError;
            return aResult;
        }

        // Copy initial directions
        math_Matrix aDirections = new(1, aN, 1, aN);
        aDirections.Initialized(theInitialDirections);

        math_Vector aDir = new(aLower, aUpper);
        math_Vector aXOld = new(aLower, aUpper);
        math_Vector aPtt = new(aLower, aUpper);
        math_Vector aXit = new(aLower, aUpper);

        for (int anIter = 0; anIter < theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter + 1;

            double aFp = aFx;
            aXOld.Initialized(aX);

            double aDel = 0.0;
            int aIBig = 0;

            for (int i = 1; i <= aN; ++i)
            {
                for (int j = aLower; j <= aUpper; ++j)
                {
                    aDir[j] = aDirections[i, j - aLower + 1];
                }

                double aFpPrev = aFx;

                LineSearchResult aLineResult =
                  MathUtils.ExactLineSearch(theFunc, aX, aDir, 10.0, theConfig.XTolerance);

                if (aLineResult.IsValid)
                {
                    for (int j = aLower; j <= aUpper; ++j)
                    {
                        aX[j] += aLineResult.Alpha * aDir[j];
                    }
                    aFx = aLineResult.FNew;

                    double aDecrease = aFpPrev - aFx;
                    if (aDecrease > aDel)
                    {
                        aDel = aDecrease;
                        aIBig = i;
                    }
                }
            }

            // Check convergence
            if (2.0 * Math.Abs(aFp - aFx)
                <= theConfig.FTolerance * (Math.Abs(aFp) + Math.Abs(aFx) + MathUtils.THE_ZERO_TOL))
            {
                aResult.Status = Status.OK;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }

            // Construct extrapolated point
            for (int j = aLower; j <= aUpper; ++j)
            {
                aPtt[j] = 2.0 * aX[j] - aXOld[j];
                aXit[j] = aX[j] - aXOld[j];
            }

            if (!theFunc.Value(aPtt, out double aFptt))
            {
                continue;
            }

            if (aFptt < aFp)
            {
                double aT = 2.0 * (aFp - 2.0 * aFx + aFptt) * MathUtils.Sqr(aFp - aFx - aDel)
                            - aDel * MathUtils.Sqr(aFp - aFptt);

                if (aT < 0.0)
                {
                    LineSearchResult aLineResult =
                      MathUtils.ExactLineSearch(theFunc, aX, aXit, 10.0, theConfig.XTolerance);

                    if (aLineResult.IsValid)
                    {
                        for (int j = aLower; j <= aUpper; ++j)
                        {
                            aX[j] += aLineResult.Alpha * aXit[j];
                        }
                        aFx = aLineResult.FNew;

                        if (aIBig > 0)
                        {
                            for (int j = 1; j <= aN; ++j)
                            {
                                aDirections[aIBig, j] = aDirections[aN, j];
                                aDirections[aN, j] = aXit[aLower + j - 1];
                            }
                        }
                    }
                }
            }

            double aMaxDiff = 0.0;
            for (int j = aLower; j <= aUpper; ++j)
            {
                aMaxDiff = Max(aMaxDiff, Math.Abs(aX[j] - aXOld[j]));
            }
            if (aMaxDiff < theConfig.XTolerance)
            {
                aResult.Status = Status.OK;
                aResult.Solution = aX;
                aResult.Value = aFx;
                return aResult;
            }
        }

        aResult.Status = Status.MaxIterations;
        aResult.Solution = aX;
        aResult.Value = aFx;
        return aResult;
    }
}
