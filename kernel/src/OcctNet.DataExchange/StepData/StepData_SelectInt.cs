// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectInt.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectInt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A SelectInt is a SelectMember specialised for a basic integer type in a select which also accepts entities: this
/// one has NO NAME. For a named select, see SelectNamed.
/// </summary>
public class StepData_SelectInt : StepData_SelectMember
{
    private int thekind;
    private int theval;

    public StepData_SelectInt()
    {
        thekind = 0;
        theval = 0;
    }

    public override int Kind()
    {
        return thekind;
    }

    public override void SetKind(int kind)
    {
        thekind = kind;
    }

    public override int Int()
    {
        return theval;
    }

    public override void SetInt(int val)
    {
        theval = val;
    }
}
