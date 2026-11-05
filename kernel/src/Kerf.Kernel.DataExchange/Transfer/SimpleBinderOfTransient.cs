// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_SimpleBinderOfTransient.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_SimpleBinderOfTransient.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// An adapted instantiation of SimpleBinder for Transient Result, i.e. ResultType can be computed from the Result
/// itself, instead of being static.
/// </summary>
internal class SimpleBinderOfTransient : Binder
{
    /// <summary>Creates an empty SimpleBinderOfTransient.</summary>
    public SimpleBinderOfTransient()
    {
    }

    /// <summary>Returns the Effective (Dynamic) Type of the Result (Standard_Transient if no Result is defined).</summary>
    public override Type? ResultType()
    {
        if (!HasResult || _theres is null)
        {
            return typeof(Transient);
        }
        return Result().DynamicType();
    }

    /// <summary>Returns the Effective Name of (Dynamic) Type of the Result, (void) if no result is defined.</summary>
    public override string ResultTypeName()
    {
        if (!HasResult || _theres is null)
        {
            return "(void)";
        }
        return Result().DynamicTypeName();
    }

    /// <summary>Defines the Result.</summary>
    /// <exception cref="TransferFailureException">The result is already set and used.</exception>
    public void SetResult(Transient? res)
    {
        SetResultPresent();
        _theres = res;
    }

    /// <summary>Returns the defined Result, if there is one.</summary>
    public Transient? Result()
    {
        return _theres;
    }

    /// <summary>
    /// Returns a transient result according to its type (IsKind), i.e. the result itself if IsKind(atype), else
    /// searches in NextResult, until first found, then returns True. If not found, returns False (res is NOT touched).
    /// </summary>
    public static bool GetTypedResult(Binder? bnd, Type? atype, ref Transient? res)
    {
        if (atype is null)
        {
            return false;
        }
        Binder? bn = bnd;
        while (bn is not null)
        {
            SimpleBinderOfTransient? trb = bn as SimpleBinderOfTransient;
            bn = bn.NextResult();
            if (trb is null)
            {
                continue;
            }
            Transient? rs = trb.Result();
            if (rs is null)
            {
                continue;
            }
            if (!rs.IsKind(atype))
            {
                continue;
            }
            res = rs;
            return true;
        }
        return false;
    }

    private Transient? _theres;
}
