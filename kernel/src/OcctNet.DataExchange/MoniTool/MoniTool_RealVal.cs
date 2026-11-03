// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_RealVal.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_RealVal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A Real through a Handle (i.e. managed as TShared)
/// </summary>
public class MoniTool_RealVal : Transient
{

    public MoniTool_RealVal(double val = 0.0)
    {
        theval = val;
    }

    public double Value()
    {
        return theval;
    }

    public ref double CValue() => ref theval;

    private double theval;
}
