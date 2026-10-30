// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearXYZConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearXYZConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearXYZConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

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
public class Plate_LinearXYZConstraint
{
    private readonly NCollection_HArray1<Plate_PinpointConstraint> myPPC = null!;
    private readonly NCollection_HArray2<double> myCoef = null!;

    public Plate_LinearXYZConstraint()
    {
    }

    public Plate_LinearXYZConstraint(NCollection_Array1<Plate_PinpointConstraint> thePPC, NCollection_Array1<double> theCoeff)
    {
        if (theCoeff.Length() != thePPC.Length())
        {
            throw new Standard_DimensionMismatch();
        }
        myPPC = new NCollection_HArray1<Plate_PinpointConstraint>(1, thePPC.Length());
        myCoef = new NCollection_HArray2<double>(1, 1, 1, theCoeff.Length());
        myPPC.ChangeArray1().Assign(thePPC);
        for (int i = 1; i <= theCoeff.Length(); i++)
        {
            myCoef.ChangeValue(1, i) = theCoeff[i + theCoeff.Lower() - 1];
        }
    }

    public Plate_LinearXYZConstraint(NCollection_Array1<Plate_PinpointConstraint> thePPC, NCollection_Array2<double> theCoeff)
    {
        if (theCoeff.RowLength() != thePPC.Length())
        {
            throw new Standard_DimensionMismatch();
        }
        myPPC = new NCollection_HArray1<Plate_PinpointConstraint>(1, thePPC.Length());
        myCoef = new NCollection_HArray2<double>(1, theCoeff.ColLength(), 1, theCoeff.RowLength());
        myPPC.ChangeArray1().Assign(thePPC);
        myCoef.ChangeArray2().Assign(theCoeff);
    }

    public Plate_LinearXYZConstraint(int ColLen, int RowLen)
    {
        myPPC = new NCollection_HArray1<Plate_PinpointConstraint>(1, RowLen);
        myCoef = new NCollection_HArray2<double>(1, ColLen, 1, RowLen);
        myCoef.Init(0.0);
    }

    /// <summary>The pinpoint constraints (the array this constraint holds, shared as OCCT's handle).</summary>
    public NCollection_Array1<Plate_PinpointConstraint> GetPPC() => myPPC.Array1();

    /// <summary>The coefficients (the array this constraint holds, shared as OCCT's handle).</summary>
    public NCollection_Array2<double> Coeff() => myCoef.Array2();

    /// <summary>Sets the pinpoint constraint of index <paramref name="Index"/>.</summary>
    public void SetPPC(int Index, in Plate_PinpointConstraint Value)
    {
        myPPC.ChangeValue(Index) = Value;
    }

    /// <summary>Sets the coefficient (<paramref name="Row"/>, <paramref name="Col"/>).</summary>
    public void SetCoeff(int Row, int Col, double Value)
    {
        myCoef.ChangeValue(Row, Col) = Value;
    }
}
