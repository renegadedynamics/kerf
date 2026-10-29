// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_Uzawa.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.MathUtils;

namespace Occt;

public static partial class MathOpt
{
    /// <summary>Result for Uzawa constrained optimization.</summary>
    public struct UzawaResult
    {
        public MathUtils.Status Status = MathUtils.Status.NotConverged;
        /// <summary>Solution vector X.</summary>
        public math_Vector? Solution;
        /// <summary>Dual (Lagrange) variables.</summary>
        public math_Vector? Dual;
        /// <summary>X - X0 (difference from starting point).</summary>
        public math_Vector? Error;
        /// <summary>C*X0 - S (initial constraint violation).</summary>
        public math_Vector? InitialError;
        /// <summary>(C * C^T)^-1 for gradient computation.</summary>
        public math_Matrix? InverseCTC;
        public int NbIterations = 0;

        public UzawaResult()
        {
        }

        public readonly bool IsDone() => Status == MathUtils.Status.OK;

        public static explicit operator bool(UzawaResult theResult) => theResult.IsDone();

        public static bool operator true(UzawaResult theResult) => theResult.IsDone();

        public static bool operator false(UzawaResult theResult) => !theResult.IsDone();
    }

    /// <summary>Configuration for Uzawa algorithm.</summary>
    public class UzawaConfig
    {
        /// <summary>Tolerance for X convergence.</summary>
        public double EpsLix = 1.0e-6;
        /// <summary>Tolerance for dual variable convergence.</summary>
        public double EpsLic = 1.0e-6;
        /// <summary>Maximum iterations.</summary>
        public int MaxIterations = 500;

        public UzawaConfig()
        {
        }

        public UzawaConfig(UzawaConfig theOther)
        {
            EpsLix = theOther.EpsLix;
            EpsLic = theOther.EpsLic;
            MaxIterations = theOther.MaxIterations;
        }
    }

    /// <summary>
    /// Constrained least squares min ||X - X0||^2 subject to C*X = S (the first theNce rows) and C*X &lt;= S (the
    /// last theNci rows): direct Crout solution for equalities only, the iterative Uzawa method otherwise. Result
    /// vectors are indexed from 1.
    /// </summary>
    public static UzawaResult Uzawa(math_Matrix theCont,
                                    math_Vector theSecont,
                                    math_Vector theStartingPoint,
                                    int theNce,
                                    int theNci,
                                    UzawaConfig? theConfig = null)
    {
        theConfig ??= new UzawaConfig();
        UzawaResult aResult = new();

        int aNlig = theCont.RowNumber();
        int aNcol = theCont.ColNumber();

        // Validate dimensions
        if (theSecont.Length() != aNlig || (theNce + theNci) != aNlig
            || theStartingPoint.Length() != aNcol)
        {
            aResult.Status = Status.InvalidInput;
            return aResult;
        }

        int aRowLower = theCont.LowerRow();
        int aColLower = theCont.LowerCol();
        int aSecLower = theSecont.Lower();
        int aXLower = theStartingPoint.Lower();

        // Compute initial error: C*X0 - S
        math_Vector aErrinit = new(1, aNlig);
        for (int i = 1; i <= aNlig; ++i)
        {
            double aSum = 0.0;
            for (int j = 1; j <= aNcol; ++j)
            {
                aSum += theCont[i + aRowLower - 1, j + aColLower - 1] * theStartingPoint[j + aXLower - 1];
            }
            aErrinit[i] = aSum - theSecont[i + aSecLower - 1];
        }

        aResult.InitialError = aErrinit;

        // Initialize dual variables and error
        math_Vector aVardua = new(1, aNlig);
        math_Vector aErruza = new(1, aNcol, 0.0);
        math_Matrix aCTCinv = new(1, aNlig, 1, aNlig, 0.0);

        if (theNci == 0)
        {
            // Direct solution for equality constraints only
            aResult.NbIterations = 1;

            // Compute C * C^T (symmetric)
            for (int i = 1; i <= aNlig; ++i)
            {
                for (int j = 1; j <= i; ++j)
                {
                    double aSum = 0.0;
                    for (int k = 1; k <= aNcol; ++k)
                    {
                        aSum += theCont[i + aRowLower - 1, k + aColLower - 1]
                                * theCont[j + aRowLower - 1, k + aColLower - 1];
                    }
                    aCTCinv[i, j] = aSum;
                    if (i != j)
                    {
                        aCTCinv[j, i] = aSum;
                    }
                }
            }

            // Invert using Crout
            InverseResult aInvResult = MathLin.InvertCrout(aCTCinv);
            if (!aInvResult.IsDone())
            {
                aResult.Status = Status.Singular;
                return aResult;
            }
            aCTCinv.Initialized(aInvResult.Inverse!);

            // Compute dual variables: (C*C^T)^-1 * (C*X0 - S)
            for (int i = 1; i <= aNlig; ++i)
            {
                double aSum = 0.0;
                for (int j = 1; j <= aNlig; ++j)
                {
                    aSum += aCTCinv[i, j] * aErrinit[j];
                }
                aVardua[i] = aSum;
            }

            // Compute error: -C^T * dual
            for (int i = 1; i <= aNcol; ++i)
            {
                double aSum = 0.0;
                for (int j = 1; j <= aNlig; ++j)
                {
                    aSum -= theCont[j + aRowLower - 1, i + aColLower - 1] * aVardua[j];
                }
                aErruza[i] = aSum;
            }

            // Compute solution
            math_Vector aResul = new(1, aNcol);
            for (int i = 1; i <= aNcol; ++i)
            {
                aResul[i] = theStartingPoint[i + aXLower - 1] + aErruza[i];
            }

            aResult.Solution = aResul;
            aResult.Dual = aVardua;
            aResult.Error = aErruza;
            aResult.InverseCTC = aCTCinv;
            aResult.Status = Status.OK;
            return aResult;
        }

        // Iterative Uzawa for mixed equality/inequality constraints
        // Initialize dual variables
        for (int i = 1; i <= aNlig; ++i)
        {
            aVardua[i] = (i <= theNce) ? 0.0 : 1.0;
        }

        // Compute step size rho
        double aNormat = 0.0;
        for (int i = 1; i <= aNlig; ++i)
        {
            double aNormli = 0.0;
            for (int j = 1; j <= aNcol; ++j)
            {
                double aVal = theCont[i + aRowLower - 1, j + aColLower - 1];
                aNormli += aVal * aVal;
            }
            aNormat += aNormli;
        }
        double aRho = 1.0 / (Math.Sqrt(2.0) * aNormat);

        // Uzawa iterations
        for (int anIter = 1; anIter <= theConfig.MaxIterations; ++anIter)
        {
            aResult.NbIterations = anIter;

            double aXmax = 0.0;

            // Update primal: X = X0 - C^T * dual
            for (int i = 1; i <= aNcol; ++i)
            {
                double aXprev = aErruza[i];
                double aSum = 0.0;
                for (int j = 1; j <= aNlig; ++j)
                {
                    aSum -= theCont[j + aRowLower - 1, i + aColLower - 1] * aVardua[j];
                }
                aErruza[i] = aSum;

                if (anIter > 1)
                {
                    aXmax = Max(aXmax, Math.Abs(aErruza[i] - aXprev));
                }
            }

            // Update dual variables and compute constraint error
            double aErrMax = 0.0;
            for (int i = 1; i <= aNlig; ++i)
            {
                // Constraint violation: C*X - S
                double aErr = aErrinit[i];
                for (int j = 1; j <= aNcol; ++j)
                {
                    aErr += theCont[i + aRowLower - 1, j + aColLower - 1] * aErruza[j];
                }

                double aErr1;
                if (i <= theNce)
                {
                    // Equality constraint: update freely
                    aVardua[i] += aRho * aErr;
                    aErr1 = Math.Abs(aRho * aErr);
                }
                else
                {
                    // Inequality constraint: project to non-negative
                    double aXmuPrev = aVardua[i];
                    aVardua[i] = Max(0.0, aVardua[i] + aRho * aErr);
                    aErr1 = Math.Abs(aVardua[i] - aXmuPrev);
                }
                aErrMax = Max(aErrMax, aErr1);
            }

            // Check convergence
            if (anIter > 1)
            {
                if (aXmax <= theConfig.EpsLix)
                {
                    if (aErrMax <= theConfig.EpsLic)
                    {
                        // Converged
                        math_Vector aResul = new(1, aNcol);
                        for (int i = 1; i <= aNcol; ++i)
                        {
                            aResul[i] = theStartingPoint[i + aXLower - 1] + aErruza[i];
                        }

                        aResult.Solution = aResul;
                        aResult.Dual = aVardua;
                        aResult.Error = aErruza;
                        aResult.Status = Status.OK;
                        return aResult;
                    }
                    else
                    {
                        // Dual did not converge
                        aResult.Status = Status.NotConverged;
                        return aResult;
                    }
                }
            }
        }

        aResult.Status = Status.MaxIterations;
        return aResult;
    }

    /// <summary>Constrained least squares with equality constraints only: C*X = S with min ||X - X0||.</summary>
    public static UzawaResult UzawaEquality(math_Matrix theCont,
                                            math_Vector theSecont,
                                            math_Vector theStartingPoint,
                                            UzawaConfig? theConfig = null)
    {
        theConfig ??= new UzawaConfig();
        return Uzawa(theCont, theSecont, theStartingPoint, theCont.RowNumber(), 0, theConfig);
    }
}
