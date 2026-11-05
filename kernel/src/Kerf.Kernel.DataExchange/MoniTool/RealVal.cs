// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_RealVal.hxx
// Port of: src/DataExchange/TKXSBase/MoniTool/MoniTool_RealVal.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Monitoring;

/// <summary>
/// A Real through a Handle (i.e. managed as TShared)
/// </summary>
internal class RealVal : Transient
{

    public RealVal(double val = 0.0)
    {
        _theval = val;
    }

    public double Value()
    {
        return _theval;
    }

    public ref double CValue() => ref _theval;

    private double _theval;
}
