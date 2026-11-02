// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldListD.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldListD.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>Describes a list of fields, in a general way. This class is for a dynamic size list.</summary>
public class StepData_FieldListD : StepData_FieldList
{
    private NCollection_HArray1<StepData_Field> thefields;

    /// <summary>Creates a FieldListD of nb Fields.</summary>
    public StepData_FieldListD(int nb)
    {
        if (nb > 0)
        {
            thefields = StepData_FieldListN.NewFields(1, nb);
        }
    }

    /// <summary>Sets a new count of Fields. Former contents are lost.</summary>
    public void SetNb(int nb)
    {
        thefields = null;
        if (nb > 0)
        {
            thefields = StepData_FieldListN.NewFields(1, nb);
        }
    }

    /// <summary>Returns the count of fields.</summary>
    public override int NbFields()
    {
        return (thefields is null ? 0 : thefields.Length());
    }

    public override StepData_Field Field(int num)
    {
        if (thefields is null)
        {
            throw new Standard_OutOfRange("StepData_FieldListD::Field");
        }
        return thefields.Value(num);
    }

    public override StepData_Field CField(int num)
    {
        if (thefields is null)
        {
            throw new Standard_OutOfRange("StepData_FieldListD::Field");
        }
        return thefields.ChangeValue(num);
    }
}
