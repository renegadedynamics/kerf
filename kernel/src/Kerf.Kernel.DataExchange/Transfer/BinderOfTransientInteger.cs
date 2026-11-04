// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_BinderOfTransientInteger.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_BinderOfTransientInteger.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// This type of Binder allows to attach as result, besides a Transient Object, an Integer Value, which can be an
/// Index in the Object if it defines a List, for instance. Its basic result (for iterators, etc) is the Transient.
/// </summary>
internal class BinderOfTransientInteger : SimpleBinderOfTransient
{
    /// <summary>Creates an empty BinderOfTransientInteger; Default value for the integer part is zero.</summary>
    public BinderOfTransientInteger()
    {
        _theintval = 0;
    }

    /// <summary>Sets a value for the integer part.</summary>
    public void SetInteger(int val)
    {
        _theintval = val;
    }

    /// <summary>Returns the value set for the integer part.</summary>
    public int Integer()
    {
        return _theintval;
    }

    private int _theintval;
}
