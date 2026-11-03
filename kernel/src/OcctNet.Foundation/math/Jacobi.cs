// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;
using static Kerf.Engine.Numerics.Recipes;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The eigenvalues and eigenvectors of a real symmetric square matrix by the Jacobi method, the eigenvalues sorted.
/// </summary>
public class Jacobi
{
    private bool _done;
    private Matrix _aa;
    private int _nbRotations;
    private math_Vector _eigenValues;
    private Matrix _eigenVectors;

    /// <summary>
    /// Computes the eigenvalues and eigenvectors of the n x n matrix <paramref name="a"/>. Throws
    /// <see cref="NotSquareException"/> if it is not square; that it is symmetric is not checked.
    /// </summary>
    public Jacobi(Matrix a)
    {
        _done = false;
        _aa = new Matrix(1, a.RowNumber(), 1, a.RowNumber());
        _nbRotations = 0;
        _eigenValues = new math_Vector(1, a.RowNumber());
        _eigenVectors = new Matrix(1, a.RowNumber(), 1, a.RowNumber());

        math_NotSquare_Raise_if(a.RowNumber() != a.ColNumber(), " ");
        _aa.Initialized(a);
        int error = Jacobi(_aa, _eigenValues, _eigenVectors, out _nbRotations);
        _done = error == 0;
    }

    public bool IsDone => _done;

    /// <summary>The eigenvalues (shared: don't modify). Throws <see cref="NotDoneException"/> if the computation failed.</summary>
    public math_Vector Values()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _eigenValues;
    }

    /// <summary>The eigenvalue number <paramref name="num"/>, in 1..n.</summary>
    public double Value(int num)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _eigenValues[num];
    }

    /// <summary>The eigenvectors, as columns (shared: don't modify). Throws <see cref="NotDoneException"/> if the computation failed.</summary>
    public Matrix Vectors()
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        return _eigenVectors;
    }

    /// <summary>Sets <paramref name="v"/> to the eigenvector number <paramref name="num"/>, in 1..n.</summary>
    public void Vector(int num, math_Vector v)
    {
        StdFail_NotDone_Raise_if(!_done, " ");
        v.Initialized(_eigenVectors.Col(num));
    }

    public void Dump(TextWriter o)
    {
        o.Write("math_Jacobi ");
        if (_done)
        {
            o.Write(" Status = Done \n");
            o.Write(" The eigenvalues vector is: " + _eigenValues + '\n');
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
