// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldList1.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_FieldList1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>Describes a list of ONE field.</summary>
public class StepData_FieldList1 : StepData_FieldList
{
    private readonly StepData_Field thefield = new();

    /// <summary>Creates a FieldList of 1 Field.</summary>
    public StepData_FieldList1()
    {
    }

    /// <summary>Returns the count of fields. Here, returns 1.</summary>
    public override int NbFields()
    {
        return 1;
    }

    public override StepData_Field Field(int num)
    {
        if (num != 1)
        {
            throw new OutOfRangeException("StepData_FieldList1 : Field");
        }
        return thefield;
    }

    public override StepData_Field CField(int num)
    {
        if (num != 1)
        {
            throw new OutOfRangeException("StepData_FieldList1 : CField");
        }
        return thefield;
    }
}
