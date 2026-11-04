// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Tensor.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Tensor.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Tensor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Fillets.BlendFunctions;

/// <summary>The "gradient of gradient" of a blend function: NbRow × NbCol × NbMat values, indexed from 1.</summary>
/// <remarks>
/// OCCT holds it by value inside the functions and never copies it; here it is a working class (decision 21).
/// </remarks>
internal sealed class Tensor
{
    private readonly NCollection_Array1<double> _tab;
    private readonly int _nbrow;
    private readonly int _nbcol;
    private readonly int _nbmat;
    private readonly int _nbmtcl;

    public Tensor(int nbRow, int nbCol, int nbMat)
    {
        _tab = new NCollection_Array1<double>(1, nbRow * nbMat * nbCol);
        _nbrow = nbRow;
        _nbcol = nbCol;
        _nbmat = nbMat;
        _nbmtcl = nbMat * nbCol;
    }

    /// <summary>Sets every element to <paramref name="initialValue"/>.</summary>
    public void Init(double initialValue)
    {
        _tab.Init(initialValue);
    }

    /// <summary>The value at (<paramref name="row"/>, <paramref name="col"/>, <paramref name="mat"/>).</summary>
    public double Value(int row, int col, int mat)
    {
        return _tab[mat + _nbmat * (col - 1) + _nbmtcl * (row - 1)];
    }

    /// <summary>OCCT's <c>operator()</c> and <c>ChangeValue</c>, for reading and writing.</summary>
    public ref double this[int row, int col, int mat] => ref _tab[mat + _nbmat * (col - 1) + _nbmtcl * (row - 1)];

    /// <summary>The matrix <paramref name="m"/>(i, j) = Σk T(i, j, k) · <paramref name="right"/>(k).</summary>
    public void Multiply(math_Vector right, Matrix m)
    {
        int i, j, k;
        double somme;
        for (i = 1; i <= _nbrow; i++)
        {
            for (j = 1; j <= _nbcol; j++)
            {
                somme = 0;
                for (k = 1; k <= _nbmat; k++)
                {
                    somme += Value(i, j, k) * right[k];
                }

                m[i, j] = somme;
            }
        }
    }
}
