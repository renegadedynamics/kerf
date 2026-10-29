// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using static Occt.math_Recipes;

namespace Occt;

/// <summary>
/// The eigenvalues and eigenvectors of a real symmetric square matrix by the Jacobi method, the eigenvalues sorted.
/// </summary>
public class math_Jacobi
{
    private bool Done;
    private math_Matrix AA;
    private int NbRotations;
    private math_Vector EigenValues;
    private math_Matrix EigenVectors;

    /// <summary>
    /// Computes the eigenvalues and eigenvectors of the n x n matrix <paramref name="A"/>. Throws
    /// <see cref="math_NotSquare"/> if it is not square; that it is symmetric is not checked.
    /// </summary>
    public math_Jacobi(math_Matrix A)
    {
        Done = false;
        AA = new math_Matrix(1, A.RowNumber(), 1, A.RowNumber());
        NbRotations = 0;
        EigenValues = new math_Vector(1, A.RowNumber());
        EigenVectors = new math_Matrix(1, A.RowNumber(), 1, A.RowNumber());

        math_NotSquare_Raise_if(A.RowNumber() != A.ColNumber(), " ");
        AA.Initialized(A);
        int Error = Jacobi(AA, EigenValues, EigenVectors, out NbRotations);
        Done = Error == 0;
    }

    public bool IsDone() => Done;

    /// <summary>The eigenvalues (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if the computation failed.</summary>
    public math_Vector Values()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return EigenValues;
    }

    /// <summary>The eigenvalue number <paramref name="Num"/>, in 1..n.</summary>
    public double Value(int Num)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return EigenValues[Num];
    }

    /// <summary>The eigenvectors, as columns (shared: don't modify). Throws <see cref="StdFail_NotDone"/> if the computation failed.</summary>
    public math_Matrix Vectors()
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        return EigenVectors;
    }

    /// <summary>Sets <paramref name="V"/> to the eigenvector number <paramref name="Num"/>, in 1..n.</summary>
    public void Vector(int Num, math_Vector V)
    {
        StdFail_NotDone_Raise_if(!Done, " ");
        V.Initialized(EigenVectors.Col(Num));
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Jacobi ");
        if (Done)
        {
            o.Write(" Status = Done \n");
            o.Write(" The eigenvalues vector is: " + EigenValues + '\n');
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
