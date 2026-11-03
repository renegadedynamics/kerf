// Port of: src/DataExchange/TKXSBase/Interface/Interface_IntVal.hxx
// Port of: src/DataExchange/TKXSBase/Interface/Interface_IntVal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// An Integer through a Handle (i.e. managed as TShared)
/// </summary>
public class IntVal : Transient
{

    public IntVal()
    {
        _theval = 0;
    }

    public int Value()
    {
        return _theval;
    }

    public ref int CValue() => ref _theval;

    private int _theval;
}
