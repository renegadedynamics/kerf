// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldListD.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldListD.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>Describes a list of fields, in a general way. This class is for a dynamic size list.</summary>
internal class FieldListD : FieldList
{
    private NCollection_HArray1<Field> _thefields;

    /// <summary>Creates a FieldListD of nb Fields.</summary>
    public FieldListD(int nb)
    {
        if (nb > 0)
        {
            _thefields = FieldListN.NewFields(1, nb);
        }
    }

    /// <summary>Sets a new count of Fields. Former contents are lost.</summary>
    public void SetNb(int nb)
    {
        _thefields = null;
        if (nb > 0)
        {
            _thefields = FieldListN.NewFields(1, nb);
        }
    }

    /// <summary>Returns the count of fields.</summary>
    public override int FieldCount
    {
        get
        {
            return (_thefields is null ? 0 : _thefields.Length());
        }
    }

    public override Field Field(int num)
    {
        if (_thefields is null)
        {
            throw new OutOfRangeException("StepData_FieldListD::Field");
        }
        return _thefields.Value(num);
    }

    public override Field CField(int num)
    {
        if (_thefields is null)
        {
            throw new OutOfRangeException("StepData_FieldListD::Field");
        }
        return _thefields.ChangeValue(num);
    }
}
