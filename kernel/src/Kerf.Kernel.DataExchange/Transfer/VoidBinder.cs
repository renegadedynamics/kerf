// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_VoidBinder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_VoidBinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// A VoidBinder is used to bind a starting item with a status, error or warning messages, but no result. It is
/// interpreted by TransferProcess, which admits a VoidBinder to be over-written, and copies its check to the new
/// Binder.
/// </summary>
internal class VoidBinder : Binder
{
    /// <summary>A VoidBinder is not Multiple (it is not Simple too), but it can bring next results.</summary>
    public VoidBinder()
    {
    }

    /// <summary>While a VoidBinder admits no Result, its ResultType returns the type of the binder.</summary>
    public override Type? ResultType()
    {
        return DynamicType();
    }

    /// <summary>Returns "(void)".</summary>
    public override string ResultTypeName()
    {
        return "(void)";
    }
}
