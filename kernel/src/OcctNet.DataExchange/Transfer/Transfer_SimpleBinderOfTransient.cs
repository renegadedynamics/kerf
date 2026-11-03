// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_SimpleBinderOfTransient.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_SimpleBinderOfTransient.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// An adapted instantiation of SimpleBinder for Transient Result, i.e. ResultType can be computed from the Result
/// itself, instead of being static.
/// </summary>
public class Transfer_SimpleBinderOfTransient : Transfer_Binder
{
    /// <summary>Creates an empty SimpleBinderOfTransient.</summary>
    public Transfer_SimpleBinderOfTransient()
    {
    }

    /// <summary>Returns the Effective (Dynamic) Type of the Result (Standard_Transient if no Result is defined).</summary>
    public override Type? ResultType()
    {
        if (!HasResult() || theres is null)
        {
            return typeof(Transient);
        }
        return Result().DynamicType();
    }

    /// <summary>Returns the Effective Name of (Dynamic) Type of the Result, (void) if no result is defined.</summary>
    public override string ResultTypeName()
    {
        if (!HasResult() || theres is null)
        {
            return "(void)";
        }
        return Result().DynamicTypeName();
    }

    /// <summary>Defines the Result.</summary>
    /// <exception cref="Transfer_TransferFailure">The result is already set and used.</exception>
    public void SetResult(Transient? res)
    {
        SetResultPresent();
        theres = res;
    }

    /// <summary>Returns the defined Result, if there is one.</summary>
    public Transient? Result()
    {
        return theres;
    }

    /// <summary>
    /// Returns a transient result according to its type (IsKind), i.e. the result itself if IsKind(atype), else
    /// searches in NextResult, until first found, then returns True. If not found, returns False (res is NOT touched).
    /// </summary>
    public static bool GetTypedResult(Transfer_Binder? bnd, Type? atype, ref Transient? res)
    {
        if (atype is null)
        {
            return false;
        }
        Transfer_Binder? bn = bnd;
        while (bn is not null)
        {
            Transfer_SimpleBinderOfTransient? trb = bn as Transfer_SimpleBinderOfTransient;
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

    private Transient? theres;
}
