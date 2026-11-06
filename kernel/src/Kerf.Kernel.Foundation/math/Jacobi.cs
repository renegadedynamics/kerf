// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Jacobi.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;
using static Kerf.Engine.Numerics.Recipes;

namespace Kerf.Engine.Numerics;

/// <summary>
/// The eigenvalues and eigenvectors of a real symmetric square matrix by the Jacobi method, the eigenvalues sorted.
/// </summary>
internal class Jacobi
{
    private bool _done;
    private Matrix _aa;
    private math_Vector _eigenValues;
    private Matrix _eigenVectors;
    private readonly math_Vector _work1;
    private readonly math_Vector _work2;

    /// <summary>
    /// Computes the eigenvalues and eigenvectors of the n x n matrix <paramref name="a"/>. Throws
    /// <see cref="NotSquareException"/> if it is not square; that it is symmetric is not checked.
    /// </summary>
    public Jacobi(Matrix a) : this(a.RowNumber())
    {
        Perform(a);
    }

    /// <summary>
    /// The storage for n x n matrices, without a computation: <see cref="Perform"/> computes, as often as needed (unit
    /// 10J: math_NewtonMinimum repeats it at every iteration).
    /// </summary>
    internal Jacobi(int n)
    {
        _done = false;
        _aa = new Matrix(1, n, 1, n);
        _eigenValues = new math_Vector(1, n);
        _eigenVectors = new Matrix(1, n, 1, n);
        _work1 = new math_Vector(1, n);
        _work2 = new math_Vector(1, n);
    }

    /// <summary>Computes the eigenvalues and eigenvectors of <paramref name="a"/>, of the size given at construction.</summary>
    internal void Perform(Matrix a)
    {
        _done = false;
        math_NotSquare_Raise_if(a.RowNumber() != a.ColNumber(), " ");
        _aa.Initialized(a);
        int error = Jacobi(_aa, _eigenValues, _eigenVectors, out _, _work1, _work2);
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
