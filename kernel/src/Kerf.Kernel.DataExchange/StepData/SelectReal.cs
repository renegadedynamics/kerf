// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectReal.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectReal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>
/// A SelectReal is a SelectMember specialised for a basic real type in a select which also accepts entities: this one
/// has NO NAME. For a named select, see SelectNamed.
/// </summary>
internal class SelectReal : SelectMember
{
    //  Definitions : cf Field
    private const int KindReal = 5;

    private double _theval;

    public SelectReal()
    {
        _theval = 0.0;
    }

    public override int Kind()
    {
        return KindReal;
    }

    public override double Real()
    {
        return _theval;
    }

    public override void SetReal(double val)
    {
        _theval = val;
    }
}
