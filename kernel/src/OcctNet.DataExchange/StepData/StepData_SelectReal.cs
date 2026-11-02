// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectReal.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectReal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A SelectReal is a SelectMember specialised for a basic real type in a select which also accepts entities: this one
/// has NO NAME. For a named select, see SelectNamed.
/// </summary>
public class StepData_SelectReal : StepData_SelectMember
{
    //  Definitions : cf Field
    private const int KindReal = 5;

    private double theval;

    public StepData_SelectReal()
    {
        theval = 0.0;
    }

    public override int Kind()
    {
        return KindReal;
    }

    public override double Real()
    {
        return theval;
    }

    public override void SetReal(double val)
    {
        theval = val;
    }
}
