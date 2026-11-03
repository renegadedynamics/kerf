// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectArrReal.hxx
// Port of: src/DataExchange/TKDESTEP/StepData/StepData_SelectArrReal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2002-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Step.Data;

/// <summary>A named select member holding an array of reals.</summary>
public class SelectArrReal : SelectNamed
{
    //  Definitions : cf Field
    private const int KindArrReal = 8;

    private NCollection_HArray1<double>? _arr;

    public SelectArrReal()
    {
    }

    public override int Kind()
    {
        return KindArrReal;
    }

    public NCollection_HArray1<double>? ArrReal()
    {
        return _arr;
    }

    public void SetArrReal(NCollection_HArray1<double>? arr)
    {
        _arr = arr;
    }
}
