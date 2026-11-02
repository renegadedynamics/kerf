// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_IntVal.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_IntVal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// An Integer through a Handle (i.e. managed as TShared)
/// </summary>
public class MoniTool_IntVal : Standard_Transient
{

    public MoniTool_IntVal(int val = 0)
    {
        theval = val;
    }

    public int Value()
    {
        return theval;
    }

    public ref int CValue() => ref theval;

    private int theval;
}
