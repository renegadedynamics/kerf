// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Tensor.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Tensor.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomFill/GeomFill_Tensor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>Stores the "gradient of gradient": a three-index array of reals, indices from 1.</summary>
/// <remarks>
/// A working buffer, a class as OCCT's math_Matrix is in the port; OCCT's <c>operator()</c> is the
/// <c>ref</c> indexer (reading and writing), <c>Value</c> and <c>ChangeValue</c> are kept.
/// </remarks>
public sealed class GeomFill_Tensor
{
    private readonly NCollection_Array1<double> Tab;
    private readonly int nbrow;
    private readonly int nbcol;
    private readonly int nbmat;
    private readonly int nbmtcl;

    public GeomFill_Tensor(int NbRow, int NbCol, int NbMat)
    {
        Tab = new NCollection_Array1<double>(1, NbRow * NbMat * NbCol);
        nbrow = NbRow;
        nbcol = NbCol;
        nbmat = NbMat;
        nbmtcl = NbMat * NbCol;
    }

    /// <summary>Initializes all the elements of the tensor to InitialValue.</summary>
    public void Init(double InitialValue)
    {
        Tab.Init(InitialValue);
    }

    /// <summary>The value of index (Row, Col, Mat).</summary>
    public double Value(int Row, int Col, int Mat) => Tab[Mat + nbmat * (Col - 1) + nbmtcl * (Row - 1)];

    /// <summary>A reference to the value of index (Row, Col, Mat).</summary>
    public ref double ChangeValue(int Row, int Col, int Mat) => ref Tab[Mat + nbmat * (Col - 1) + nbmtcl * (Row - 1)];

    public ref double this[int Row, int Col, int Mat] => ref ChangeValue(Row, Col, Mat);

    public void Multiply(math_Vector Right, math_Matrix M)
    {
        int i, j, k;
        double Somme;
        for (i = 1; i <= nbrow; i++)
        {
            for (j = 1; j <= nbcol; j++)
            {
                Somme = 0;
                for (k = 1; k <= nbmat; k++)
                {
                    Somme += Value(i, j, k) * Right[k];
                }
                M[i, j] = Somme;
            }
        }
    }
}
