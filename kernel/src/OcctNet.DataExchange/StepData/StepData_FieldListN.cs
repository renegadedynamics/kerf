// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldListN.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldListN.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>Describes a list of fields, in a general way. This class is for a list of definite length.</summary>
public class StepData_FieldListN : StepData_FieldList
{
    private readonly NCollection_Array1<StepData_Field> thefields;

    /// <summary>Creates a FieldListN of nb Fields.</summary>
    public StepData_FieldListN(int nb)
    {
        thefields = NewFields((nb == 0 ? 0 : 1), nb);
    }

    // OCCT's arrays of fields default-construct their elements; the port's hold references, filled here
    internal static NCollection_HArray1<StepData_Field> NewFields(int theLower, int theUpper)
    {
        var aFields = new NCollection_HArray1<StepData_Field>(theLower, theUpper);
        for (int i = theLower; i <= theUpper; i++)
        {
            aFields.SetValue(i, new StepData_Field());
        }
        return aFields;
    }

    /// <summary>Returns the count of fields.</summary>
    public override int NbFields()
    {
        return thefields.Upper();
    }

    public override StepData_Field Field(int num)
    {
        return thefields.Value(num);
    }

    public override StepData_Field CField(int num)
    {
        return thefields.ChangeValue(num);
    }
}
