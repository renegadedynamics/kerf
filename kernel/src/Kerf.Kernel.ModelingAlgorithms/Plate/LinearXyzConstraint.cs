// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearXYZConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearXYZConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearXYZConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A linear XYZ constraint of a plate: each row i of the coefficient matrix C imposes
/// sum_j C(i, j) * D(PPC(j)) = sum_j C(i, j) * Value(PPC(j)), the same combination on each coordinate.
/// </summary>
/// <remarks>
/// OCCT copies the class by value, but its data are handles, so a copy shares the pinpoint constraints and the
/// coefficients (after a C++ copy, <c>SetPPC</c> on either changes both). A class has the same sharing. It is a
/// working object (decision 21): <c>SetPPC</c> and <c>SetCoeff</c> are public, as in OCCT. The default constructor
/// holds no arrays, as OCCT's null handles.
/// </remarks>
internal class LinearXyzConstraint
{
    private readonly NCollection_HArray1<PinpointConstraint> _ppc = null!;
    private readonly NCollection_HArray2<double> _coef = null!;

    public LinearXyzConstraint()
    {
    }

    public LinearXyzConstraint(NCollection_Array1<PinpointConstraint> ppc, NCollection_Array1<double> coeff)
    {
        if (coeff.Length() != ppc.Length())
        {
            throw new DimensionMismatchException();
        }
        _ppc = new NCollection_HArray1<PinpointConstraint>(1, ppc.Length());
        _coef = new NCollection_HArray2<double>(1, 1, 1, coeff.Length());
        _ppc.ChangeArray1().Assign(ppc);
        for (int i = 1; i <= coeff.Length(); i++)
        {
            _coef.ChangeValue(1, i) = coeff[i + coeff.Lower() - 1];
        }
    }

    public LinearXyzConstraint(NCollection_Array1<PinpointConstraint> ppc, NCollection_Array2<double> coeff)
    {
        if (coeff.RowLength() != ppc.Length())
        {
            throw new DimensionMismatchException();
        }
        _ppc = new NCollection_HArray1<PinpointConstraint>(1, ppc.Length());
        _coef = new NCollection_HArray2<double>(1, coeff.ColLength(), 1, coeff.RowLength());
        _ppc.ChangeArray1().Assign(ppc);
        _coef.ChangeArray2().Assign(coeff);
    }

    public LinearXyzConstraint(int colLen, int rowLen)
    {
        _ppc = new NCollection_HArray1<PinpointConstraint>(1, rowLen);
        _coef = new NCollection_HArray2<double>(1, colLen, 1, rowLen);
        _coef.Init(0.0);
    }

    /// <summary>The pinpoint constraints (the array this constraint holds, shared as OCCT's handle).</summary>
    public NCollection_Array1<PinpointConstraint> GetPPC() => _ppc.Array1();

    /// <summary>The coefficients (the array this constraint holds, shared as OCCT's handle).</summary>
    public NCollection_Array2<double> Coeff() => _coef.Array2();

    /// <summary>Sets the pinpoint constraint of index <paramref name="index"/>.</summary>
    public void SetPPC(int index, in PinpointConstraint value)
    {
        _ppc.ChangeValue(index) = value;
    }

    /// <summary>Sets the coefficient (<paramref name="row"/>, <paramref name="col"/>).</summary>
    public void SetCoeff(int row, int col, double value)
    {
        _coef.ChangeValue(row, col) = value;
    }
}
