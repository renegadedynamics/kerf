// Port of: src/FoundationClasses/TKMath/math/math_SVD.hxx
// Port of: src/FoundationClasses/TKMath/math/math_SVD.lxx
// Port of: src/FoundationClasses/TKMath/math/math_SVD.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
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

    // Working vectors, for Perform and for a solve: both are written before they're read
    private math_Vector? _rv1;
    private math_Vector? _bb;
    private math_Vector? _tmp;

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

    /// <summary>
    /// The storage for n x m matrices, without a decomposition: <see cref="Perform"/> decomposes, as often as needed (unit
    /// 11B: math_FunctionSetRoot repeats it at every iteration).
    /// </summary>
    internal Svd(int n, int m)
    {
        _u = new Matrix(1, Max(n, m), 1, m);
        _v = new Matrix(1, m, 1, m);
        _diag = new math_Vector(1, m);
        _rv1 = new math_Vector(1, m);
    }

    /// <summary>Whether this is the storage for <paramref name="a"/>'s size.</summary>
    internal bool Fits(Matrix a) => _u.RowNumber() == Max(a.RowNumber(), a.ColNumber()) && _u.ColNumber() == a.ColNumber();

    /// <summary>The decomposition of <paramref name="a"/>, of the size given at construction, as the constructor's.</summary>
    internal void Perform(Matrix a)
    {
        _v.Init(0.0);
        _diag.Init(0.0);
        _u.Init(0.0);
        _rowA = a.RowNumber();
        _u.Set(1, a.RowNumber(), 1, a.ColNumber(), a);
        int error = SVD_Decompose(_u, _diag, _v, _rv1!);
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

        math_Vector bb = _bb ??= new(1, _u.RowNumber());
        math_Vector tmp = _tmp ??= new(1, _u.ColNumber());
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
            SVD_Solve(_u, _diag, _v, bb, xTemp, tmp);
        }
        else
        {
            SVD_Solve(_u, _diag, _v, bb, x, tmp);
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
