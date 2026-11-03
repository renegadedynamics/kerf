// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Tensor.hxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Tensor.lxx
// Port of: src/ModelingAlgorithms/TKFillet/BlendFunc/BlendFunc_Tensor.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>The "gradient of gradient" of a blend function: NbRow × NbCol × NbMat values, indexed from 1.</summary>
/// <remarks>
/// OCCT holds it by value inside the functions and never copies it; here it is a working class (decision 21).
/// </remarks>
public sealed class BlendFunc_Tensor
{
    private readonly NCollection_Array1<double> Tab;
    private readonly int nbrow;
    private readonly int nbcol;
    private readonly int nbmat;
    private readonly int nbmtcl;

    public BlendFunc_Tensor(int NbRow, int NbCol, int NbMat)
    {
        Tab = new NCollection_Array1<double>(1, NbRow * NbMat * NbCol);
        nbrow = NbRow;
        nbcol = NbCol;
        nbmat = NbMat;
        nbmtcl = NbMat * NbCol;
    }

    /// <summary>Sets every element to <paramref name="InitialValue"/>.</summary>
    public void Init(double InitialValue)
    {
        Tab.Init(InitialValue);
    }

    /// <summary>The value at (<paramref name="Row"/>, <paramref name="Col"/>, <paramref name="Mat"/>).</summary>
    public double Value(int Row, int Col, int Mat)
    {
        return Tab[Mat + nbmat * (Col - 1) + nbmtcl * (Row - 1)];
    }

    /// <summary>OCCT's <c>operator()</c> and <c>ChangeValue</c>, for reading and writing.</summary>
    public ref double this[int Row, int Col, int Mat] => ref Tab[Mat + nbmat * (Col - 1) + nbmtcl * (Row - 1)];

    /// <summary>The matrix <paramref name="M"/>(i, j) = Σk T(i, j, k) · <paramref name="Right"/>(k).</summary>
    public void Multiply(math_Vector Right, Matrix M)
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
