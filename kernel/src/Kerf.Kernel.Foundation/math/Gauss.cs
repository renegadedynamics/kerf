// Port of: src/FoundationClasses/TKMath/math/math_Gauss.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Gauss.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;
using static Kerf.Engine.Numerics.Recipes;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The Gauss LU decomposition (Crout algorithm) with partial pivoting of a square matrix, and what follows from it:
/// the solution of linear equations, the inverse and the determinant.
/// </summary>
internal class Gauss
{
    protected Matrix _lu;
    protected math_IntegerVector _index;
    protected double _d;
    protected bool _done;

    // LU_Decompose's work vector, for Perform
    private readonly math_Vector? _work;

    /// <summary>
    /// Performs the LU decomposition of the n x n matrix <paramref name="a"/>. If the largest pivot found is less than
    /// <paramref name="minPivot"/> the matrix is considered singular. Throws <see cref="NotSquareException"/> if A is not square.
    /// </summary>
    public Gauss(Matrix a, double minPivot = 1.0e-20, ProgressRange progress = default)
    {
        _lu = new Matrix(1, a.RowNumber(), 1, a.ColNumber());
        _index = new math_IntegerVector(1, a.RowNumber());
        _d = 0.0;
        _done = false;

        math_NotSquare_Raise_if(a.RowNumber() != a.ColNumber(), " ");
        _lu.Initialized(a);
        int error = LU_Decompose(_lu, _index, out _d, minPivot, progress);
        _done = error == 0;
    }

    /// <summary>
    /// The storage for an n x n decomposition, without one: <see cref="Perform"/> decomposes, as often as needed (unit
    /// 10J: math_NewtonMinimum repeats it at every iteration).
    /// </summary>
    internal Gauss(int n)
    {
        _lu = new Matrix(1, n, 1, n);
        _index = new math_IntegerVector(1, n);
        _work = new math_Vector(1, n);
        _d = 0.0;
        _done = false;
    }

    /// <summary>The LU decomposition of <paramref name="a"/>, of the size given at construction, as the constructor's.</summary>
    internal void Perform(Matrix a, double minPivot = 1.0e-20)
    {
        _d = 0.0;
        _done = false;
        math_NotSquare_Raise_if(a.RowNumber() != a.ColNumber(), " ");
        _lu.Initialized(a);
        int error = LU_Decompose(_lu, _index, out _d, _work!, minPivot);
        _done = error == 0;
    }

    public bool IsDone => _done;

    /// <summary>Solves A . X = B. Throws <see cref="NotDoneException"/> if the decomposition failed.</summary>
    public void Solve(math_Vector b, math_Vector x)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        x.Initialized(b);
        LU_Solve(_lu, _index, x);
    }

    /// <summary>Solves A . X = B, replacing B by the solution. Throws <see cref="NotDoneException"/> if the decomposition failed.</summary>
    public void Solve(math_Vector x)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        if (x.Length() != _lu.RowNumber())
        {
            throw new DimensionErrorException();
        }

        LU_Solve(_lu, _index, x);
    }

    /// <summary>The determinant of A.</summary>
    public double Determinant()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        double result = _d;
        for (int j = 1; j <= _lu.UpperRow(); j++)
        {
            result *= _lu[j, j];
        }

        return result;
    }

    /// <summary>Sets <paramref name="inv"/> to the inverse of A.</summary>
    public void Invert(Matrix inv)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        Standard_DimensionError_Raise_if((inv.RowNumber() != _lu.RowNumber()) || (inv.ColNumber() != _lu.ColNumber()), " ");

        int lowerRow = inv.LowerRow();
        int lowerCol = inv.LowerCol();
        math_Vector column = new(1, _lu.UpperRow());

        int i, j;
        for (j = 1; j <= _lu.UpperRow(); j++)
        {
            for (i = 1; i <= _lu.UpperRow(); i++)
            {
                column[i] = 0.0;
            }

            column[j] = 1.0;
            LU_Solve(_lu, _index, column);
            for (i = 1; i <= _lu.RowNumber(); i++)
            {
                inv[i + lowerRow - 1, j + lowerCol - 1] = column[i];
            }
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Gauss ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Determinant of A = " + _d.ToString(CultureInfo.InvariantCulture) + '\n');
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
