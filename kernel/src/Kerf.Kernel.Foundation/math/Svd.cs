// Port of: src/FoundationClasses/TKMath/math/math_SVD.hxx
// Port of: src/FoundationClasses/TKMath/math/math_SVD.lxx
// Port of: src/FoundationClasses/TKMath/math/math_SVD.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;
using static Kerf.Engine.Numerics.Recipes;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The solution of N linear equations in M unknowns, without condition on N or M, by Singular Value Decomposition.
/// Better than Gauss or GaussLeastSquare for singular or nearly singular matrices.
/// </summary>
internal class Svd
{
    private bool _done;
    private Matrix _u;
    private Matrix _v;
    private math_Vector _diag;
    private int _rowA;

    /// <summary>Performs the Singular Value Decomposition of the n x m matrix <paramref name="a"/>.</summary>
    public Svd(Matrix a)
    {
        _u = new Matrix(1, Max(a.RowNumber(), a.ColNumber()), 1, a.ColNumber());
        _v = new Matrix(1, a.ColNumber(), 1, a.ColNumber());
        _diag = new math_Vector(1, a.ColNumber());

        _u.Init(0.0);
        _rowA = a.RowNumber();
        _u.Set(1, a.RowNumber(), 1, a.ColNumber(), a);
        int error = SVD_Decompose(_u, _diag, _v);
        _done = error == 0;
    }

    public bool IsDone => _done;

    /// <summary>
    /// Solves A . X = B, ignoring singular values below <paramref name="eps"/> times the largest. Throws
    /// <see cref="NotDoneException"/> if the decomposition failed and <see cref="DimensionErrorException"/> if B or X
    /// has the wrong length.
    /// </summary>
    /// <remarks>Zeroes the small singular values for good, as OCCT does.</remarks>
    public void Solve(math_Vector b, math_Vector x, double eps = 1.0e-6)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_DimensionError_Raise_if((_rowA != b.Length()) || (x.Length() != _diag.Length()), " ");

        math_Vector bb = new(1, _u.RowNumber());
        bb.Init(0.0);
        bb.Set(1, b.Length(), b);
        double wmin = eps * _diag[_diag.Max()];
        for (int i = 1; i <= _diag.Upper(); i++)
        {
            if (_diag[i] < wmin)
            {
                _diag[i] = 0.0;
            }
        }

        // Handle custom bounds in X vector - SVD_Solve expects 1-based indexing
        if (x.Lower() != 1)
        {
            math_Vector xTemp = new(x, x.Lower(), 1, x.Length());
            SVD_Solve(_u, _diag, _v, bb, xTemp);
        }
        else
        {
            SVD_Solve(_u, _diag, _v, bb, x);
        }
    }

    /// <summary>
    /// Sets <paramref name="result"/> to the pseudo-inverse of A, ignoring singular values below <paramref name="eps"/>
    /// times the largest. Throws <see cref="NotDoneException"/> if the decomposition failed.
    /// </summary>
    public void PseudoInverse(Matrix result, double eps = 1.0e-6)
    {
        int i, j;

        StdFail_NotDone_Raise_if(!_done, " ");
        double wmin = eps * _diag[_diag.Max()];
        for (i = 1; i <= _diag.Upper(); i++)
        {
            if (_diag[i] < wmin)
            {
                _diag[i] = 0.0;
            }
        }

        int colA = _diag.Length();
        math_Vector vNorme = new(1, _u.RowNumber());
        math_Vector column = new(1, colA);

        for (j = 1; j <= _rowA; j++)
        {
            for (i = 1; i <= vNorme.Upper(); i++)
            {
                vNorme[i] = 0.0;
            }

            vNorme[j] = 1.0;
            SVD_Solve(_u, _diag, _v, vNorme, column);
            for (i = 1; i <= colA; i++)
            {
                // OCCT indexes Result from 1 whatever its bounds
                result[i, j] = column[i];
            }
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_SVD");
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
