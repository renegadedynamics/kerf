// Port of: src/FoundationClasses/TKMath/math/math_Crout.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Crout.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Crout.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// The Crout algorithm, to solve A*X = B where A is a symmetric matrix, or to invert A. Faster than Gauss. Only the
/// lower triangle and the diagonal of A are used.
/// </summary>
public class math_Crout
{
    private math_Matrix InvA;
    private bool Done;
    private double Det;

    /// <summary>
    /// Inverts <paramref name="A"/> = L * D * T(L) by the Crout algorithm, reading only its lower triangle. If a pivot
    /// is less than <paramref name="MinPivot"/>, A is considered singular. Throws <see cref="math_NotSquare"/> if A
    /// is not square.
    /// </summary>
    public math_Crout(math_Matrix A, double MinPivot = 1.0e-20)
    {
        InvA = new math_Matrix(1, A.RowNumber(), 1, A.ColNumber());

        int i, j, k;
        int Nctl = A.RowNumber();
        int lowr = A.LowerRow(), lowc = A.LowerCol();
        double scale;

        math_Matrix L = new(1, Nctl, 1, Nctl);
        math_Vector Diag = new(1, Nctl);

        math_NotSquare_Raise_if(Nctl != A.ColNumber(), " ");

        Det = 1;
        for (i = 1; i <= Nctl; i++)
        {
            for (j = 1; j <= i - 1; j++)
            {
                scale = 0.0;
                for (k = 1; k <= j - 1; k++)
                {
                    scale += L[i, k] * L[j, k] * Diag[k];
                }

                L[i, j] = (A[i + lowr - 1, j + lowc - 1] - scale) / Diag[j];
            }

            scale = 0.0;
            for (k = 1; k <= i - 1; k++)
            {
                scale += L[i, k] * L[i, k] * Diag[k];
            }

            Diag[i] = A[i + lowr - 1, i + lowc - 1] - scale;
            Det *= Diag[i];
            if (Math.Abs(Diag[i]) <= MinPivot)
            {
                Done = false;
                return;
            }

            L[i, i] = 1.0;
        }

        // Calcul de l inverse de L:
        //==========================
        L[1, 1] = 1.0 / L[1, 1];
        for (i = 2; i <= Nctl; i++)
        {
            for (k = 1; k <= i - 1; k++)
            {
                scale = 0.0;
                for (j = k; j <= i - 1; j++)
                {
                    scale += L[i, j] * L[j, k];
                }

                L[i, k] = -scale / L[i, i];
            }

            L[i, i] = 1.0 / L[i, i];
        }

        // Calcul de l inverse de Mat:
        //============================
        for (j = 1; j <= Nctl; j++)
        {
            scale = L[j, j] * L[j, j] / Diag[j];
            for (k = j + 1; k <= Nctl; k++)
            {
                scale += L[k, j] * L[k, j] / Diag[k];
            }

            InvA[j, j] = scale;
            for (i = j + 1; i <= Nctl; i++)
            {
                scale = L[i, j] * L[i, i] / Diag[i];
                for (k = i + 1; k <= Nctl; k++)
                {
                    scale += L[k, j] * L[k, i] / Diag[k];
                }

                InvA[i, j] = scale;
            }
        }

        Done = true;
    }

    public bool IsDone() => Done;

    /// <summary>
    /// Solves A . X = B. Throws <see cref="StdFail_NotDone"/> if the decomposition failed and
    /// <see cref="Standard_DimensionError"/> if the lengths of B or X differ from the rows of A.
    /// </summary>
    public void Solve(math_Vector B, math_Vector X)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_DimensionError_Raise_if((B.Length() != InvA.RowNumber()) || (X.Length() != B.Length()), " ");

        int n = InvA.RowNumber();
        int lowb = B.Lower(), lowx = X.Lower();
        int i, j;

        for (i = 1; i <= n; i++)
        {
            X[i + lowx - 1] = InvA[i, 1] * B[1 + lowb - 1];
            for (j = 2; j <= i; j++)
            {
                X[i + lowx - 1] += InvA[i, j] * B[j + lowb - 1];
            }

            for (j = i + 1; j <= n; j++)
            {
                X[i + lowx - 1] += InvA[j, i] * B[j + lowb - 1];
            }
        }
    }

    /// <summary>The inverse of A, lower triangle only (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public math_Matrix Inverse()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return InvA;
    }

    /// <summary>Sets <paramref name="Inv"/> to the inverse of A, lower triangle only. Throws <see cref="StdFail_NotDone"/> if not done.</summary>
    public void Invert(math_Matrix Inv)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Inv.Initialized(InvA);
    }

    /// <summary>The determinant of A. Throws <see cref="StdFail_NotDone"/> if the decomposition failed.</summary>
    public double Determinant()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return Det;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Crout ");
        if (Done)
        {
            o.Write(" Status = Done \n");
        }
        else
        {
            o.Write(" Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
