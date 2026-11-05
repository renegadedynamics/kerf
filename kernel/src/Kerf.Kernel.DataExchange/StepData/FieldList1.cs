// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldList1.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldList1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>Describes a list of ONE field.</summary>
internal class FieldList1 : FieldList
{
    private readonly Field _thefield = new();

    /// <summary>Creates a FieldList of 1 Field.</summary>
    public FieldList1()
    {
    }

    /// <summary>Returns the count of fields. Here, returns 1.</summary>
    public override int FieldCount
    {
        get
        {
            return 1;
        }
    }

    public override Field Field(int num)
    {
        if (num != 1)
        {
            throw new OutOfRangeException("StepData_FieldList1 : Field");
        }
        return _thefield;
    }

    public override Field CField(int num)
    {
        if (num != 1)
        {
            throw new OutOfRangeException("StepData_FieldList1 : CField");
        }
        return _thefield;
    }
}
