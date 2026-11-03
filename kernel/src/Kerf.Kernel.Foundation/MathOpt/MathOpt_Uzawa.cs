// Port of: src/FoundationClasses/TKMath/MathOpt/MathOpt_Uzawa.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Numerics.Utilities.MathUtils;

namespace Kerf.Engine.Numerics.Optimisation;

public static partial class MathOpt
{
    /// <summary>Result for Uzawa constrained optimization.</summary>
    public struct UzawaResult
    {
        public global::Kerf.Engine.Numerics.Utilities.MathUtils.Status Status = global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.NotConverged;
        /// <summary>Solution vector X.</summary>
        public math_Vector? Solution;
        /// <summary>Dual (Lagrange) variables.</summary>
        public math_Vector? Dual;
        /// <summary>X - X0 (difference from starting point).</summary>
        public math_Vector? Error;
        /// <summary>C*X0 - S (initial constraint violation).</summary>
        public math_Vector? InitialError;
        /// <summary>(C * C^T)^-1 for gradient computation.</summary>
        public Matrix? InverseCTC;
        public int NbIterations = 0;

        public UzawaResult()
        {
        }

        public readonly bool IsDone => Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.OK;

        public static explicit operator bool(UzawaResult result) => result.IsDone;

        public static bool operator true(UzawaResult result) => result.IsDone;

        public static bool operator false(UzawaResult result) => !result.IsDone;
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

        public UzawaConfig(UzawaConfig other)
        {
            EpsLix = other.EpsLix;
            EpsLic = other.EpsLic;
            MaxIterations = other.MaxIterations;
        }
    }

    /// <summary>
    /// Constrained least squares min ||X - X0||^2 subject to C*X = S (the first theNce rows) and C*X &lt;= S (the
    /// last theNci rows): direct Crout solution for equalities only, the iterative Uzawa method otherwise. Result
    /// vectors are indexed from 1.
    /// </summary>
    public static UzawaResult Uzawa(Matrix cont,
                                    math_Vector secont,
                                    math_Vector startingPoint,
                                    int nce,
                                    int nci,
                                    UzawaConfig? config = null)
    {
        config ??= new UzawaConfig();
        UzawaResult result = new();

        int nlig = cont.RowNumber();
        int ncol = cont.ColNumber();

        // Validate dimensions
        if (secont.Length() != nlig || (nce + nci) != nlig
            || startingPoint.Length() != ncol)
        {
            result.Status = Status.InvalidInput;
            return result;
        }

        int rowLower = cont.LowerRow();
        int colLower = cont.LowerCol();
        int secLower = secont.Lower();
        int xLower = startingPoint.Lower();

        // Compute initial error: C*X0 - S
        math_Vector errinit = new(1, nlig);
        for (int i = 1; i <= nlig; ++i)
        {
            double sum = 0.0;
            for (int j = 1; j <= ncol; ++j)
            {
                sum += cont[i + rowLower - 1, j + colLower - 1] * startingPoint[j + xLower - 1];
            }
            errinit[i] = sum - secont[i + secLower - 1];
        }

        result.InitialError = errinit;

        // Initialize dual variables and error
        math_Vector vardua = new(1, nlig);
        math_Vector erruza = new(1, ncol, 0.0);
        Matrix ctCinv = new(1, nlig, 1, nlig, 0.0);

        if (nci == 0)
        {
            // Direct solution for equality constraints only
            result.NbIterations = 1;

            // Compute C * C^T (symmetric)
            for (int i = 1; i <= nlig; ++i)
            {
                for (int j = 1; j <= i; ++j)
                {
                    double sum = 0.0;
                    for (int k = 1; k <= ncol; ++k)
                    {
                        sum += cont[i + rowLower - 1, k + colLower - 1]
                                * cont[j + rowLower - 1, k + colLower - 1];
                    }
                    ctCinv[i, j] = sum;
                    if (i != j)
                    {
                        ctCinv[j, i] = sum;
                    }
                }
            }

            // Invert using Crout
            InverseResult invResult = global::Kerf.Engine.Numerics.Linear.MathLin.InvertCrout(ctCinv);
            if (!invResult.IsDone)
            {
                result.Status = Status.Singular;
                return result;
            }
            ctCinv.Initialized(invResult.Inverse!);

            // Compute dual variables: (C*C^T)^-1 * (C*X0 - S)
            for (int i = 1; i <= nlig; ++i)
            {
                double sum = 0.0;
                for (int j = 1; j <= nlig; ++j)
                {
                    sum += ctCinv[i, j] * errinit[j];
                }
                vardua[i] = sum;
            }

            // Compute error: -C^T * dual
            for (int i = 1; i <= ncol; ++i)
            {
                double sum = 0.0;
                for (int j = 1; j <= nlig; ++j)
                {
                    sum -= cont[j + rowLower - 1, i + colLower - 1] * vardua[j];
                }
                erruza[i] = sum;
            }

            // Compute solution
            math_Vector resul = new(1, ncol);
            for (int i = 1; i <= ncol; ++i)
            {
                resul[i] = startingPoint[i + xLower - 1] + erruza[i];
            }

            result.Solution = resul;
            result.Dual = vardua;
            result.Error = erruza;
            result.InverseCTC = ctCinv;
            result.Status = Status.OK;
            return result;
        }

        // Iterative Uzawa for mixed equality/inequality constraints
        // Initialize dual variables
        for (int i = 1; i <= nlig; ++i)
        {
            vardua[i] = (i <= nce) ? 0.0 : 1.0;
        }

        // Compute step size rho
        double normat = 0.0;
        for (int i = 1; i <= nlig; ++i)
        {
            double normli = 0.0;
            for (int j = 1; j <= ncol; ++j)
            {
                double val = cont[i + rowLower - 1, j + colLower - 1];
                normli += val * val;
            }
            normat += normli;
        }
        double rho = 1.0 / (Math.Sqrt(2.0) * normat);

        // Uzawa iterations
        for (int iter = 1; iter <= config.MaxIterations; ++iter)
        {
            result.NbIterations = iter;

            double xmax = 0.0;

            // Update primal: X = X0 - C^T * dual
            for (int i = 1; i <= ncol; ++i)
            {
                double xprev = erruza[i];
                double sum = 0.0;
                for (int j = 1; j <= nlig; ++j)
                {
                    sum -= cont[j + rowLower - 1, i + colLower - 1] * vardua[j];
                }
                erruza[i] = sum;

                if (iter > 1)
                {
                    xmax = Max(xmax, Math.Abs(erruza[i] - xprev));
                }
            }

            // Update dual variables and compute constraint error
            double errMax = 0.0;
            for (int i = 1; i <= nlig; ++i)
            {
                // Constraint violation: C*X - S
                double err = errinit[i];
                for (int j = 1; j <= ncol; ++j)
                {
                    err += cont[i + rowLower - 1, j + colLower - 1] * erruza[j];
                }

                double err1;
                if (i <= nce)
                {
                    // Equality constraint: update freely
                    vardua[i] += rho * err;
                    err1 = Math.Abs(rho * err);
                }
                else
                {
                    // Inequality constraint: project to non-negative
                    double xmuPrev = vardua[i];
                    vardua[i] = Max(0.0, vardua[i] + rho * err);
                    err1 = Math.Abs(vardua[i] - xmuPrev);
                }
                errMax = Max(errMax, err1);
            }

            // Check convergence
            if (iter > 1)
            {
                if (xmax <= config.EpsLix)
                {
                    if (errMax <= config.EpsLic)
                    {
                        // Converged
                        math_Vector resul = new(1, ncol);
                        for (int i = 1; i <= ncol; ++i)
                        {
                            resul[i] = startingPoint[i + xLower - 1] + erruza[i];
                        }

                        result.Solution = resul;
                        result.Dual = vardua;
                        result.Error = erruza;
                        result.Status = Status.OK;
                        return result;
                    }
                    else
                    {
                        // Dual did not converge
                        result.Status = Status.NotConverged;
                        return result;
                    }
                }
            }
        }

        result.Status = Status.MaxIterations;
        return result;
    }

    /// <summary>Constrained least squares with equality constraints only: C*X = S with min ||X - X0||.</summary>
    public static UzawaResult UzawaEquality(Matrix cont,
                                            math_Vector secont,
                                            math_Vector startingPoint,
                                            UzawaConfig? config = null)
    {
        config ??= new UzawaConfig();
        return Uzawa(cont, secont, startingPoint, cont.RowNumber(), 0, config);
    }
}
