// Port of: src/FoundationClasses/TKMath/math/math_Householder.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Householder.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Householder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The least square solution of n linear equations in m unknowns (n &gt;= m), A.X = B, by the Householder method.
/// More stable than GaussLeastSquare but slower; use it for singular or nearly singular matrices.
/// </summary>
internal class Householder
{
    private Matrix _sol;
    private Matrix _q;
    private bool _done;
    private int _mylowerArow;
    private int _myupperArow;
    private int _mylowerAcol;
    private int _myupperAcol;

    // Cette classe decrit la methode de Householder qui transforme A en un
    // produit de matrice orthogonale par une triangulaire superieure. Les seconds
    // membres sont modifies dans le meme temps.
    // Les references sur le cote sont celles de l'algorithme explique en page
    // 90 du livre "Introduction a l'analyse numerique matricielle et a
    // l'optimisation." par P.G. CIARLET, edition MASSON. Les secondes
    // references sont celles du sous-programme HOUSEO d'Euclid.
    // A la difference du sous-programme Houseo, la premiere colonne n'est pas
    // traitee separement. Les tests effectues ont montre que le code effectue
    // specialement pour celle-ci etait plus long qu'une simple recopie. C'est
    // donc cette solution de recopie initiale qui a ete retenue.

    /// <summary>
    /// Solves A.X = B in the least square sense. If a column norm is less than <paramref name="eps"/>, it can't be
    /// done. Throws <see cref="DimensionErrorException"/> if the length of B differs from the rows of A.
    /// </summary>
    public Householder(Matrix a, math_Vector b, double eps = 1.0e-20)
    {
        _sol = new Matrix(1, a.ColNumber(), 1, 1);
        _q = new Matrix(1, a.RowNumber(), 1, a.ColNumber());
        _done = false;

        _mylowerArow = a.LowerRow();
        _mylowerAcol = a.LowerCol();
        _myupperArow = a.UpperRow();
        _myupperAcol = a.UpperCol();
        Matrix b1 = new(1, b.Length(), 1, 1);
        b1.SetCol(1, b);
        Perform(a, b1, eps);
    }

    /// <summary>
    /// Solves A.X = B in the least square sense for each column of B. If a column norm is less than
    /// <paramref name="eps"/>, it can't be done. Throws <see cref="DimensionErrorException"/> if the rows of B differ
    /// from the rows of A.
    /// </summary>
    public Householder(Matrix a, Matrix b, double eps = 1.0e-20)
    {
        _sol = new Matrix(1, a.ColNumber(), 1, b.ColNumber());
        // OCCT gives Q A's column range, but Perform indexes its columns from 1: a matrix A whose LowerCol is not 1
        // reads outside Q (out of range here).
        _q = new Matrix(1, a.RowNumber(), a.LowerCol(), a.UpperCol());
        _done = false;

        _mylowerArow = a.LowerRow();
        _mylowerAcol = a.LowerCol();
        _myupperArow = a.UpperRow();
        _myupperAcol = a.UpperCol();
        Perform(a, b, eps);
    }

    /// <summary>
    /// Solves A.X = B in the least square sense for each column of B, using only the given rows and columns of A.
    /// </summary>
    public Householder(Matrix a, Matrix b, int lowerArow, int upperArow, int lowerAcol, int upperAcol,
                            double eps = 1.0e-20)
    {
        _sol = new Matrix(1, upperAcol - lowerAcol + 1, 1, b.ColNumber());
        _q = new Matrix(1, upperArow - lowerArow + 1, 1, upperAcol - lowerAcol + 1);
        _done = false;

        _mylowerArow = lowerArow;
        _myupperArow = upperArow;
        _mylowerAcol = lowerAcol;
        _myupperAcol = upperAcol;
        Perform(a, b, eps);
    }

    /// <summary>Used by the constructors.</summary>
    protected void Perform(Matrix a, Matrix b, double eps)
    {
        int i, j, k, n, l, m;
        double f, g, h = 0.0, alfaii;
        double qki;
        double cj;
        n = _q.ColNumber();
        l = _q.RowNumber();
        m = b.ColNumber();
        Matrix b2 = new(1, l, 1, m);

        int lbrow = b.LowerRow();
        for (i = 1; i <= l; i++)
        {
            for (j = 1; j <= n; j++)
            {
                _q[i, j] = a[i + _mylowerArow - 1, j + _mylowerAcol - 1];
            }

            for (j = 1; j <= m; j++)
            {
                // OCCT indexes B's columns from 1 whatever its LowerCol
                b2[i, j] = b[i + lbrow - 1, j];
            }
        }

        // OCCT checks the dimensions only after copying B, so a B with too few rows reads out of range first
        Standard_DimensionError_Raise_if(l != b.RowNumber() || n > l, " ");

        // Traitement de chaque colonne de A:
        for (i = 1; i <= n; i++)
        {
            h = 0.0;
            for (k = i; k <= l; k++)
            {
                qki = _q[k, i];
                h += qki * qki; // = ||a||*||a||     = EUAI
            }

            f = _q[i, i]; // = a1              = AII
            g = f < 1.0e-15 ? Math.Sqrt(h) : -Math.Sqrt(h);
            if (Math.Abs(g) <= eps)
            {
                _done = false;
                return;
            }

            h -= f * g;     // = (v*v)/2         = C1
            alfaii = g - f; // = v               = ALFAII
            for (j = i + 1; j <= n; j++)
            {
                double scale = 0.0;
                for (k = i; k <= l; k++)
                {
                    scale += _q[k, i] * _q[k, j]; //                   = SCAL
                }

                cj = (g * _q[i, j] - scale) / h;
                _q[i, j] = _q[i, j] - alfaii * cj;
                for (k = i + 1; k <= l; k++)
                {
                    _q[k, j] = _q[k, j] + cj * _q[k, i];
                }
            }

            // Modification de B:
            for (j = 1; j <= m; j++)
            {
                double scale = _q[i, i] * b2[i, j];
                for (k = i + 1; k <= l; k++)
                {
                    scale += _q[k, i] * b2[k, j];
                }

                cj = (g * b2[i, j] - scale) / h;
                b2[i, j] = b2[i, j] - cj * alfaii;
                for (k = i + 1; k <= l; k++)
                {
                    b2[k, j] = b2[k, j] + cj * _q[k, i];
                }
            }

            _q[i, i] = g;
        }

        // Remontee:
        for (j = 1; j <= m; j++)
        {
            _sol[n, j] = b2[n, j] / _q[n, n];
            for (i = n - 1; i >= 1; i--)
            {
                double scale = 0.0;
                for (k = i + 1; k <= n; k++)
                {
                    scale += _q[i, k] * _sol[k, j];
                }

                _sol[i, j] = (b2[i, j] - scale) / _q[i, i];
            }
        }

        _done = true;
    }

    public bool IsDone => _done;

    /// <summary>
    /// Sets <paramref name="sol"/> to the solution for column <paramref name="index"/> of B. Throws
    /// <see cref="NotDoneException"/> if not done and <see cref="OutOfRangeException"/> if Index is out of 1..B's columns.
    /// </summary>
    public void Value(math_Vector sol, int index = 1)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_OutOfRange_Raise_if((index < 1) || (index > _sol.ColNumber()), " ");
        sol.Initialized(_sol.Col(index));
    }

    /// <summary>All the solutions, one per column (shared: don't modify). Throws <see cref="NotDoneException"/> if not done.</summary>
    public Matrix AllValues()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _sol;
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Householder ");
        if (_done)
        {
            o.Write(" Status = Done \n");
        }
        else
        {
            o.Write("Status = not Done \n");
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }
}
