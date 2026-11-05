// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_SparseMatrix.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_SparseMatrix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Approximation.FiniteElements;

/// <summary>A sparse matrix, with direct and iterative solvers.</summary>
internal abstract class SparseMatrix : Transient
{
    /// <summary>Sets every stored coefficient to <paramref name="value"/>.</summary>
    public abstract void Init(double value);

    /// <summary>The coefficient (<paramref name="i"/>, <paramref name="j"/>), for reading and writing.</summary>
    public abstract ref double ChangeValue(int i, int j);

    /// <summary>Factorises the matrix; false if that fails.</summary>
    public abstract bool Decompose();

    /// <summary>Direct solve of A X = B, after <see cref="Decompose"/>.</summary>
    public abstract void Solve(math_Vector b, math_Vector x);

    /// <summary>Prepares an iterative solve.</summary>
    public abstract bool Prepare();

    /// <summary>Iterative solve of A X = B.</summary>
    public abstract void Solve(math_Vector b, math_Vector init, math_Vector x, math_Vector residual,
                               double tolerance = 1.0e-8, int nbIterations = 50);

    /// <summary>MX = A X. An exception is raised if the dimensions are different.</summary>
    public abstract void Multiplied(math_Vector x, math_Vector mx);

    /// <summary>The number of rows.</summary>
    public abstract int RowNumber();

    /// <summary>The number of columns.</summary>
    public abstract int ColNumber();
}
