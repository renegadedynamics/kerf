// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearScalarConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearScalarConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_LinearScalarConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A linear scalar constraint of a plate: each row i of the coefficient matrix C imposes
/// sum_j C(i, j) . D(PPC(j)) = sum_j C(i, j) . Value(PPC(j)), with XYZ coefficients and dot products.
/// </summary>
/// <remarks>
/// OCCT copies the class by value, but its data are handles, so a copy shares the pinpoint constraints and the
/// coefficients. A class has the same sharing. It is a working object (decision 21): <c>SetPPC</c> and
/// <c>SetCoeff</c> are public, as in OCCT. The default constructor holds no arrays, as OCCT's null handles.
/// </remarks>
public class Plate_LinearScalarConstraint
{
    private readonly NCollection_HArray1<Plate_PinpointConstraint> myPPC = null!;
    private readonly NCollection_HArray2<gp_XYZ> myCoef = null!;

    public Plate_LinearScalarConstraint()
    {
    }

    public Plate_LinearScalarConstraint(in Plate_PinpointConstraint PPC1, in gp_XYZ coeff)
    {
        myPPC = new NCollection_HArray1<Plate_PinpointConstraint>(1, 1);
        myCoef = new NCollection_HArray2<gp_XYZ>(1, 1, 1, 1);
        myPPC.ChangeValue(1) = PPC1;
        myCoef.ChangeValue(1, 1) = coeff;
    }

    public Plate_LinearScalarConstraint(NCollection_Array1<Plate_PinpointConstraint> thePPC, NCollection_Array1<gp_XYZ> theCoeff)
    {
        if (theCoeff.Length() != thePPC.Length())
        {
            throw new Standard_DimensionMismatch();
        }
        myPPC = new NCollection_HArray1<Plate_PinpointConstraint>(1, thePPC.Length());
        myCoef = new NCollection_HArray2<gp_XYZ>(1, 1, 1, theCoeff.Length());
        myPPC.ChangeArray1().Assign(thePPC);
        for (int i = 1; i <= theCoeff.Length(); i++)
        {
            myCoef.ChangeValue(1, i) = theCoeff[i + theCoeff.Lower() - 1];
        }
    }

    public Plate_LinearScalarConstraint(NCollection_Array1<Plate_PinpointConstraint> thePPC, NCollection_Array2<gp_XYZ> theCoeff)
    {
        if (theCoeff.RowLength() != thePPC.Length())
        {
            throw new Standard_DimensionMismatch();
        }
        myPPC = new NCollection_HArray1<Plate_PinpointConstraint>(1, thePPC.Length());
        myCoef = new NCollection_HArray2<gp_XYZ>(1, theCoeff.ColLength(), 1, theCoeff.RowLength());
        myPPC.ChangeArray1().Assign(thePPC);
        myCoef.ChangeArray2().Assign(theCoeff);
    }

    public Plate_LinearScalarConstraint(int ColLen, int RowLen)
    {
        myPPC = new NCollection_HArray1<Plate_PinpointConstraint>(1, RowLen);
        myCoef = new NCollection_HArray2<gp_XYZ>(1, ColLen, 1, RowLen);
        myCoef.Init(new gp_XYZ(0.0, 0.0, 0.0));
    }

    /// <summary>The pinpoint constraints (the array this constraint holds, shared as OCCT's handle).</summary>
    public NCollection_Array1<Plate_PinpointConstraint> GetPPC() => myPPC.Array1();

    /// <summary>The coefficients (the array this constraint holds, shared as OCCT's handle).</summary>
    public NCollection_Array2<gp_XYZ> Coeff() => myCoef.Array2();

    /// <summary>Sets the pinpoint constraint of index <paramref name="Index"/>.</summary>
    public void SetPPC(int Index, in Plate_PinpointConstraint Value)
    {
        myPPC.ChangeValue(Index) = Value;
    }

    /// <summary>Sets the coefficient (<paramref name="Row"/>, <paramref name="Col"/>).</summary>
    public void SetCoeff(int Row, int Col, in gp_XYZ Value)
    {
        myCoef.ChangeValue(Row, Col) = Value;
    }
}
