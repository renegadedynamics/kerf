// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldListN.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldListN.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>Describes a list of fields, in a general way. This class is for a list of definite length.</summary>
internal class FieldListN : FieldList
{
    private readonly NCollection_Array1<Field> _thefields;

    /// <summary>Creates a FieldListN of nb Fields.</summary>
    public FieldListN(int nb)
    {
        _thefields = NewFields((nb == 0 ? 0 : 1), nb);
    }

    // OCCT's arrays of fields default-construct their elements; the port's hold references, filled here
    internal static NCollection_HArray1<Field> NewFields(int lower, int upper)
    {
        var fields = new NCollection_HArray1<Field>(lower, upper);
        for (int i = lower; i <= upper; i++)
        {
            fields.SetValue(i, new Field());
        }
        return fields;
    }

    /// <summary>Returns the count of fields.</summary>
    public override int FieldCount
    {
        get
        {
            return _thefields.Upper();
        }
    }

    public override Field Field(int num)
    {
        return _thefields.Value(num);
    }

    public override Field CField(int num)
    {
        return _thefields.ChangeValue(num);
    }
}
