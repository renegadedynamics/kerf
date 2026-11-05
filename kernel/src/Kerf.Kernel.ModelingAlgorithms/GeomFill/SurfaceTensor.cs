// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Tensor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Tensor.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Tensor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Surfaces;

/// <summary>Stores the "gradient of gradient": a three-index array of reals, indices from 1.</summary>
/// <remarks>
/// A working buffer, a class as OCCT's math_Matrix is in the port; OCCT's <c>operator()</c> is the
/// <c>ref</c> indexer (reading and writing), <c>Value</c> and <c>ChangeValue</c> are kept.
/// </remarks>
internal sealed class SurfaceTensor
{
    private readonly NCollection_Array1<double> _tab;
    private readonly int _nbrow;
    private readonly int _nbcol;
    private readonly int _nbmat;
    private readonly int _nbmtcl;

    public SurfaceTensor(int nbRow, int nbCol, int nbMat)
    {
        _tab = new NCollection_Array1<double>(1, nbRow * nbMat * nbCol);
        _nbrow = nbRow;
        _nbcol = nbCol;
        _nbmat = nbMat;
        _nbmtcl = nbMat * nbCol;
    }

    /// <summary>Initializes all the elements of the tensor to InitialValue.</summary>
    public void Init(double initialValue)
    {
        _tab.Init(initialValue);
    }

    /// <summary>The value of index (Row, Col, Mat).</summary>
    public double Value(int row, int col, int mat) => _tab[mat + _nbmat * (col - 1) + _nbmtcl * (row - 1)];

    /// <summary>A reference to the value of index (Row, Col, Mat).</summary>
    public ref double ChangeValue(int row, int col, int mat) => ref _tab[mat + _nbmat * (col - 1) + _nbmtcl * (row - 1)];

    public ref double this[int row, int col, int mat] => ref ChangeValue(row, col, mat);

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
