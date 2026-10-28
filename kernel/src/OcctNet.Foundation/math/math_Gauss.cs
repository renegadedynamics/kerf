// Port of: src/FoundationClasses/TKMath/math/math_Gauss.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Gauss.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Recipes;

namespace Occt;

/// <summary>
/// The Gauss LU decomposition (Crout algorithm) with partial pivoting of a square matrix, and what follows from it:
/// the solution of linear equations, the inverse and the determinant.
/// </summary>
public class math_Gauss
{
    protected math_Matrix LU;
    protected math_IntegerVector Index;
    protected double D;
    protected bool Done;

    /// <summary>
    /// Performs the LU decomposition of the n x n matrix <paramref name="A"/>. If the largest pivot found is less than
    /// <paramref name="MinPivot"/> the matrix is considered singular. Throws <see cref="math_NotSquare"/> if A is not square.
    /// </summary>
    public math_Gauss(math_Matrix A, double MinPivot = 1.0e-20, Message_ProgressRange theProgress = default)
    {
        LU = new math_Matrix(1, A.RowNumber(), 1, A.ColNumber());
        Index = new math_IntegerVector(1, A.RowNumber());
        D = 0.0;
        Done = false;

        math_NotSquare_Raise_if(A.RowNumber() != A.ColNumber(), " ");
        LU.Initialized(A);
        int Error = LU_Decompose(LU, Index, out D, MinPivot, theProgress);
        Done = Error == 0;
    }

    public bool IsDone() => Done;

    /// <summary>Solves A . X = B. Throws <see cref="StdFail_NotDone"/> if the decomposition failed.</summary>
    public void Solve(math_Vector B, math_Vector X)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        X.Initialized(B);
        LU_Solve(LU, Index, X);
    }

    /// <summary>Solves A . X = B, replacing B by the solution. Throws <see cref="StdFail_NotDone"/> if the decomposition failed.</summary>
    public void Solve(math_Vector X)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        if (X.Length() != LU.RowNumber())
        {
            throw new Standard_DimensionError();
        }

        LU_Solve(LU, Index, X);
    }

    /// <summary>The determinant of A.</summary>
    public double Determinant()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        double Result = D;
        for (int J = 1; J <= LU.UpperRow(); J++)
        {
            Result *= LU[J, J];
        }

        return Result;
    }

    /// <summary>Sets <paramref name="Inv"/> to the inverse of A.</summary>
    public void Invert(math_Matrix Inv)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_DimensionError_Raise_if((Inv.RowNumber() != LU.RowNumber()) || (Inv.ColNumber() != LU.ColNumber()), " ");

        int LowerRow = Inv.LowerRow();
        int LowerCol = Inv.LowerCol();
        math_Vector Column = new(1, LU.UpperRow());

        int I, J;
        for (J = 1; J <= LU.UpperRow(); J++)
        {
            for (I = 1; I <= LU.UpperRow(); I++)
            {
                Column[I] = 0.0;
            }

            Column[J] = 1.0;
            LU_Solve(LU, Index, Column);
            for (I = 1; I <= LU.RowNumber(); I++)
            {
                Inv[I + LowerRow - 1, J + LowerCol - 1] = Column[I];
            }
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Gauss ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" Determinant of A = " + D.ToString(CultureInfo.InvariantCulture) + '\n');
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
