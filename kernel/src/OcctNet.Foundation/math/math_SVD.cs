// Port of: src/FoundationClasses/TKMath/math/math_SVD.hxx
// Port of: src/FoundationClasses/TKMath/math/math_SVD.lxx
// Port of: src/FoundationClasses/TKMath/math/math_SVD.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Recipes;

namespace Occt;

/// <summary>
/// The solution of N linear equations in M unknowns, without condition on N or M, by Singular Value Decomposition.
/// Better than Gauss or GaussLeastSquare for singular or nearly singular matrices.
/// </summary>
public class math_SVD
{
    private bool Done;
    private math_Matrix U;
    private math_Matrix V;
    private math_Vector Diag;
    private int RowA;

    /// <summary>Performs the Singular Value Decomposition of the n x m matrix <paramref name="A"/>.</summary>
    public math_SVD(math_Matrix A)
    {
        U = new math_Matrix(1, Max(A.RowNumber(), A.ColNumber()), 1, A.ColNumber());
        V = new math_Matrix(1, A.ColNumber(), 1, A.ColNumber());
        Diag = new math_Vector(1, A.ColNumber());

        U.Init(0.0);
        RowA = A.RowNumber();
        U.Set(1, A.RowNumber(), 1, A.ColNumber(), A);
        int Error = SVD_Decompose(U, Diag, V);
        Done = Error == 0;
    }

    public bool IsDone() => Done;

    /// <summary>
    /// Solves A . X = B, ignoring singular values below <paramref name="Eps"/> times the largest. Throws
    /// <see cref="StdFail_NotDone"/> if the decomposition failed and <see cref="Standard_DimensionError"/> if B or X
    /// has the wrong length.
    /// </summary>
    /// <remarks>Zeroes the small singular values for good, as OCCT does.</remarks>
    public void Solve(math_Vector B, math_Vector X, double Eps = 1.0e-6)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        Standard_DimensionError_Raise_if((RowA != B.Length()) || (X.Length() != Diag.Length()), " ");

        math_Vector BB = new(1, U.RowNumber());
        BB.Init(0.0);
        BB.Set(1, B.Length(), B);
        double wmin = Eps * Diag[Diag.Max()];
        for (int I = 1; I <= Diag.Upper(); I++)
        {
            if (Diag[I] < wmin)
            {
                Diag[I] = 0.0;
            }
        }

        // Handle custom bounds in X vector - SVD_Solve expects 1-based indexing
        if (X.Lower() != 1)
        {
            math_Vector anXTemp = new(X, X.Lower(), 1, X.Length());
            SVD_Solve(U, Diag, V, BB, anXTemp);
        }
        else
        {
            SVD_Solve(U, Diag, V, BB, X);
        }
    }

    /// <summary>
    /// Sets <paramref name="Result"/> to the pseudo-inverse of A, ignoring singular values below <paramref name="Eps"/>
    /// times the largest. Throws <see cref="StdFail_NotDone"/> if the decomposition failed.
    /// </summary>
    public void PseudoInverse(math_Matrix Result, double Eps = 1.0e-6)
    {
        int i, j;

        StdFail_NotDone_Raise_if(!Done, " ");
        double wmin = Eps * Diag[Diag.Max()];
        for (i = 1; i <= Diag.Upper(); i++)
        {
            if (Diag[i] < wmin)
            {
                Diag[i] = 0.0;
            }
        }

        int ColA = Diag.Length();
        math_Vector VNorme = new(1, U.RowNumber());
        math_Vector Column = new(1, ColA);

        for (j = 1; j <= RowA; j++)
        {
            for (i = 1; i <= VNorme.Upper(); i++)
            {
                VNorme[i] = 0.0;
            }

            VNorme[j] = 1.0;
            SVD_Solve(U, Diag, V, VNorme, Column);
            for (i = 1; i <= ColA; i++)
            {
                // OCCT indexes Result from 1 whatever its bounds
                Result[i, j] = Column[i];
            }
        }
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_SVD");
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
