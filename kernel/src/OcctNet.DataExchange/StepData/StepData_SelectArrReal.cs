// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectArrReal.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectArrReal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>A named select member holding an array of reals.</summary>
public class StepData_SelectArrReal : StepData_SelectNamed
{
    //  Definitions : cf Field
    private const int myKindArrReal = 8;

    private NCollection_HArray1<double>? theArr;

    public StepData_SelectArrReal()
    {
    }

    public override int Kind()
    {
        return myKindArrReal;
    }

    public NCollection_HArray1<double>? ArrReal()
    {
        return theArr;
    }

    public void SetArrReal(NCollection_HArray1<double>? arr)
    {
        theArr = arr;
    }
}
