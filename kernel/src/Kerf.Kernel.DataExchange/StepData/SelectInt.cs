// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectInt.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectInt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// A SelectInt is a SelectMember specialised for a basic integer type in a select which also accepts entities: this
/// one has NO NAME. For a named select, see SelectNamed.
/// </summary>
internal class SelectInt : SelectMember
{
    private int _thekind;
    private int _theval;

    public SelectInt()
    {
        _thekind = 0;
        _theval = 0;
    }

    public override int Kind()
    {
        return _thekind;
    }

    public override void SetKind(int kind)
    {
        _thekind = kind;
    }

    public override int Int()
    {
        return _theval;
    }

    public override void SetInt(int val)
    {
        _theval = val;
    }
}
