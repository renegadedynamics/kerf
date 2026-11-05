// Port of: src/FoundationClasses/TKMath/math/math_Crout.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Crout.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Crout.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The Crout algorithm, to solve A*X = B where A is a symmetric matrix, or to invert A. Faster than Gauss. Only the
/// lower triangle and the diagonal of A are used.
/// </summary>
internal class Crout
{
    private Matrix _invA;
    private bool _done;
    private double _det;

    /// <summary>
    /// Inverts <paramref name="a"/> = L * D * T(L) by the Crout algorithm, reading only its lower triangle. If a pivot
    /// is less than <paramref name="minPivot"/>, A is considered singular. Throws <see cref="NotSquareException"/> if A
    /// is not square.
    /// </summary>
    public Crout(Matrix a, double minPivot = 1.0e-20)
    {
        _invA = new Matrix(1, a.RowNumber(), 1, a.ColNumber());

        int i, j, k;
        int nctl = a.RowNumber();
        int lowr = a.LowerRow(), lowc = a.LowerCol();
        double scale;

        Matrix l = new(1, nctl, 1, nctl);
        math_Vector diag = new(1, nctl);

        math_NotSquare_Raise_if(nctl != a.ColNumber(), " ");

        _det = 1;
        for (i = 1; i <= nctl; i++)
        {
            for (j = 1; j <= i - 1; j++)
            {
                scale = 0.0;
                for (k = 1; k <= j - 1; k++)
                {
                    scale += l[i, k] * l[j, k] * diag[k];
                }

                l[i, j] = (a[i + lowr - 1, j + lowc - 1] - scale) / diag[j];
            }

            scale = 0.0;
            for (k = 1; k <= i - 1; k++)
            {
                scale += l[i, k] * l[i, k] * diag[k];
            }

            diag[i] = a[i + lowr - 1, i + lowc - 1] - scale;
            _det *= diag[i];
            if (Math.Abs(diag[i]) <= minPivot)
            {
                _done = false;
                return;
            }

            l[i, i] = 1.0;
        }

        // Calcul de l inverse de L:
        //==========================
        l[1, 1] = 1.0 / l[1, 1];
        for (i = 2; i <= nctl; i++)
        {
            for (k = 1; k <= i - 1; k++)
            {
                scale = 0.0;
                for (j = k; j <= i - 1; j++)
                {
                    scale += l[i, j] * l[j, k];
                }

                l[i, k] = -scale / l[i, i];
            }

            l[i, i] = 1.0 / l[i, i];
        }

        // Calcul de l inverse de Mat:
        //============================
        for (j = 1; j <= nctl; j++)
        {
            scale = l[j, j] * l[j, j] / diag[j];
            for (k = j + 1; k <= nctl; k++)
            {
                scale += l[k, j] * l[k, j] / diag[k];
            }

            _invA[j, j] = scale;
            for (i = j + 1; i <= nctl; i++)
            {
                scale = l[i, j] * l[i, i] / diag[i];
                for (k = i + 1; k <= nctl; k++)
                {
                    scale += l[k, j] * l[k, i] / diag[k];
                }

                _invA[i, j] = scale;
            }
        }

        _done = true;
    }

    public bool IsDone => _done;

    /// <summary>
    /// Solves A . X = B. Throws <see cref="NotDoneException"/> if the decomposition failed and
    /// <see cref="DimensionErrorException"/> if the lengths of B or X differ from the rows of A.
    /// </summary>
    public void Solve(math_Vector b, math_Vector x)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_DimensionError_Raise_if((b.Length() != _invA.RowNumber()) || (x.Length() != b.Length()), " ");

        int n = _invA.RowNumber();
        int lowb = b.Lower(), lowx = x.Lower();
        int i, j;

        for (i = 1; i <= n; i++)
        {
            x[i + lowx - 1] = _invA[i, 1] * b[1 + lowb - 1];
            for (j = 2; j <= i; j++)
            {
                x[i + lowx - 1] += _invA[i, j] * b[j + lowb - 1];
            }

            for (j = i + 1; j <= n; j++)
            {
                x[i + lowx - 1] += _invA[j, i] * b[j + lowb - 1];
            }
        }
    }

    /// <summary>The inverse of A, lower triangle only (shared: don't modify). Throws <see cref="NotDoneException"/> if not done.</summary>
    public Matrix Inverse()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _invA;
    }

    /// <summary>Sets <paramref name="inv"/> to the inverse of A, lower triangle only. Throws <see cref="NotDoneException"/> if not done.</summary>
    public void Invert(Matrix inv)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        inv.Initialized(_invA);
    }

    /// <summary>The determinant of A. Throws <see cref="NotDoneException"/> if the decomposition failed.</summary>
    public double Determinant()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _det;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Crout ");
        if (_done)
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
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
