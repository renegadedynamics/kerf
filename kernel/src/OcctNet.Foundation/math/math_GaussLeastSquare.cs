// Port of: src/FoundationClasses/TKMath/math/math_GaussLeastSquare.hxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussLeastSquare.lxx
// Port of: src/FoundationClasses/TKMath/math/math_GaussLeastSquare.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Recipes;

namespace Occt;

/// <summary>
/// The least square solution of n linear equations in m unknowns (n &gt;= m) by Gauss LU decomposition of
/// A.Transposed() * A. More subject to numerical instability than <see cref="math_SVD"/>.
/// </summary>
public class math_GaussLeastSquare
{
    protected bool Singular;
    protected math_Matrix LU;
    protected math_Matrix A2;
    protected math_IntegerVector Index;
    protected double D;

    private bool Done;

    /// <summary>
    /// Performs the LU decomposition with partial pivoting of A.Transposed() * A. If the largest pivot found is less
    /// than <paramref name="MinPivot"/>, A is considered singular.
    /// </summary>
    public math_GaussLeastSquare(math_Matrix A, double MinPivot = 1.0e-20)
    {
        LU = new math_Matrix(1, A.ColNumber(), 1, A.ColNumber());
        A2 = new math_Matrix(1, A.ColNumber(), 1, A.RowNumber());
        Index = new math_IntegerVector(1, A.ColNumber());

        A2.Initialized(A.Transposed());
        LU.Multiply(A2, A);
        int Error = LU_Decompose(LU, Index, out D, MinPivot);
        Done = Error == 0;
    }

    public bool IsDone() => Done;

    /// <summary>
    /// Solves A . X = B in the least square sense. Throws <see cref="StdFail_NotDone"/> if the decomposition failed
    /// and <see cref="Standard_DimensionError"/> if B's length differs from A's rows or X's from A's columns.
    /// </summary>
    public void Solve(math_Vector B, math_Vector X)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_DimensionError_Raise_if((B.Length() != A2.ColNumber()) || (X.Length() != A2.RowNumber()), " ");

        X.Multiply(A2, B);
        LU_Solve(LU, Index, X);
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_GaussLeastSquare ");
        if (Done)
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
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }
}
