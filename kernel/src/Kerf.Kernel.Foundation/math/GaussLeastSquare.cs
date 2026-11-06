// Port of: src/FoundationClasses/TKMath/math/math_GaussLeastSquare.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussLeastSquare.lxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussLeastSquare.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Globalization;
using static Kerf.Engine.Numerics.Recipes;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The least square solution of n linear equations in m unknowns (n &gt;= m) by Gauss LU decomposition of
/// A.Transposed() * A. More subject to numerical instability than <see cref="Svd"/>.
/// </summary>
internal class GaussLeastSquare
{
    protected bool _singular;
    protected Matrix _lu;
    protected Matrix _a2;
    protected math_IntegerVector _index;
    protected double _d;

    private bool _done;

    // LU_Decompose's work vector, for Perform
    private readonly math_Vector? _work;

    /// <summary>
    /// Performs the LU decomposition with partial pivoting of A.Transposed() * A. If the largest pivot found is less
    /// than <paramref name="minPivot"/>, A is considered singular.
    /// </summary>
    public GaussLeastSquare(Matrix a, double minPivot = 1.0e-20)
    {
        _lu = new Matrix(1, a.ColNumber(), 1, a.ColNumber());
        _a2 = new Matrix(1, a.ColNumber(), 1, a.RowNumber());
        _index = new math_IntegerVector(1, a.ColNumber());

        _a2.Initialized(a.Transposed());
        _lu.Multiply(_a2, a);
        int error = LU_Decompose(_lu, _index, out _d, minPivot);
        _done = error == 0;
    }

    /// <summary>
    /// The storage for decompositions of matrices of <paramref name="rows"/> by <paramref name="columns"/>, without one:
    /// <see cref="Perform"/> decomposes, as often as needed (as <see cref="Gauss"/>'s, unit 11Q).
    /// </summary>
    internal GaussLeastSquare(int rows, int columns)
    {
        _lu = new Matrix(1, columns, 1, columns);
        _a2 = new Matrix(1, columns, 1, rows);
        _index = new math_IntegerVector(1, columns);
        _work = new math_Vector(1, columns);
    }

    /// <summary>The decomposition of <paramref name="a"/>, of the size given at construction, as the constructor's.</summary>
    internal void Perform(Matrix a, double minPivot = 1.0e-20)
    {
        int rowLower = a.LowerRow(), colLower = a.LowerCol();
        for (int i = 1; i <= _a2.RowNumber(); i++)
        {
            for (int j = 1; j <= _a2.ColNumber(); j++)
            {
                _a2[i, j] = a[rowLower + j - 1, colLower + i - 1];
            }
        }
        _lu.Multiply(_a2, a);
        _d = 0.0;
        int error = LU_Decompose(_lu, _index, out _d, _work!, minPivot);
        _done = error == 0;
    }

    /// <summary>Whether this has the storage for <paramref name="a"/>.</summary>
    internal bool Fits(Matrix a) => _a2.ColNumber() == a.RowNumber() && _a2.RowNumber() == a.ColNumber();

    public bool IsDone => _done;

    /// <summary>
    /// Solves A . X = B in the least square sense. Throws <see cref="NotDoneException"/> if the decomposition failed
    /// and <see cref="DimensionErrorException"/> if B's length differs from A's rows or X's from A's columns.
    /// </summary>
    public void Solve(math_Vector b, math_Vector x)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_DimensionError_Raise_if((b.Length() != _a2.ColNumber()) || (x.Length() != _a2.RowNumber()), " ");

        x.Multiply(_a2, b);
        LU_Solve(_lu, _index, x);
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_GaussLeastSquare ");
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
