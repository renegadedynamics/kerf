// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearScalarConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearScalarConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearScalarConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Plate;

/// <summary>
/// A linear scalar constraint of a plate: each row i of the coefficient matrix C imposes
/// sum_j C(i, j) . D(PPC(j)) = sum_j C(i, j) . Value(PPC(j)), with XYZ coefficients and dot products.
/// </summary>
/// <remarks>
/// OCCT copies the class by value, but its data are handles, so a copy shares the pinpoint constraints and the
/// coefficients. A class has the same sharing. It is a working object (decision 21): <c>SetPPC</c> and
/// <c>SetCoeff</c> are public, as in OCCT. The default constructor holds no arrays, as OCCT's null handles.
/// </remarks>
public class LinearScalarConstraint
{
    private readonly NCollection_HArray1<PinpointConstraint> _ppc = null!;
    private readonly NCollection_HArray2<Xyz> _coef = null!;

    public LinearScalarConstraint()
    {
    }

    public LinearScalarConstraint(in PinpointConstraint ppc1, in Xyz coeff)
    {
        _ppc = new NCollection_HArray1<PinpointConstraint>(1, 1);
        _coef = new NCollection_HArray2<Xyz>(1, 1, 1, 1);
        _ppc.ChangeValue(1) = ppc1;
        _coef.ChangeValue(1, 1) = coeff;
    }

    public LinearScalarConstraint(NCollection_Array1<PinpointConstraint> ppc, NCollection_Array1<Xyz> coeff)
    {
        if (coeff.Length() != ppc.Length())
        {
            throw new DimensionMismatchException();
        }
        _ppc = new NCollection_HArray1<PinpointConstraint>(1, ppc.Length());
        _coef = new NCollection_HArray2<Xyz>(1, 1, 1, coeff.Length());
        _ppc.ChangeArray1().Assign(ppc);
        for (int i = 1; i <= coeff.Length(); i++)
        {
            _coef.ChangeValue(1, i) = coeff[i + coeff.Lower() - 1];
        }
    }

    public LinearScalarConstraint(NCollection_Array1<PinpointConstraint> ppc, NCollection_Array2<Xyz> coeff)
    {
        if (coeff.RowLength() != ppc.Length())
        {
            throw new DimensionMismatchException();
        }
        _ppc = new NCollection_HArray1<PinpointConstraint>(1, ppc.Length());
        _coef = new NCollection_HArray2<Xyz>(1, coeff.ColLength(), 1, coeff.RowLength());
        _ppc.ChangeArray1().Assign(ppc);
        _coef.ChangeArray2().Assign(coeff);
    }

    public LinearScalarConstraint(int colLen, int rowLen)
    {
        _ppc = new NCollection_HArray1<PinpointConstraint>(1, rowLen);
        _coef = new NCollection_HArray2<Xyz>(1, colLen, 1, rowLen);
        _coef.Init(new Xyz(0.0, 0.0, 0.0));
    }

    /// <summary>The pinpoint constraints (the array this constraint holds, shared as OCCT's handle).</summary>
    public NCollection_Array1<PinpointConstraint> GetPPC() => _ppc.Array1();

    /// <summary>The coefficients (the array this constraint holds, shared as OCCT's handle).</summary>
    public NCollection_Array2<Xyz> Coeff() => _coef.Array2();

    /// <summary>Sets the pinpoint constraint of index <paramref name="index"/>.</summary>
    public void SetPPC(int index, in PinpointConstraint value)
    {
        _ppc.ChangeValue(index) = value;
    }

    /// <summary>Sets the coefficient (<paramref name="row"/>, <paramref name="col"/>).</summary>
    public void SetCoeff(int row, int col, in Xyz value)
    {
        _coef.ChangeValue(row, col) = value;
    }
}
