// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorOfProcessForTransient.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorOfProcessForTransient_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// An actor of a <see cref="Transfer_ProcessForTransient"/>: it recognizes starting objects and transfers them. Actors
/// form a list (<see cref="SetNext"/>); the "Last" ones (defaults) stay at its end.
/// </summary>
public class Transfer_ActorOfProcessForTransient : Transient
{
    public Transfer_ActorOfProcessForTransient()
    {
    }

    /// <summary>
    /// Prerequisite for Transfer : the method Transfer is called on a starting object only if Recognize has returned
    /// True on it. This allows to define a list of Actors, each one processing a definite kind of data. TransferProcess
    /// calls Recognize on each one before calling Transfer. But even if Recognize has returned True, Transfer can
    /// reject by returning a Null Binder (afterwards rejection), the next actor is then invoked. The provided default
    /// returns True, can be redefined.
    /// </summary>
    public virtual bool Recognize(Transient? start)
    {
        return true;
    }

    /// <summary>
    /// Specific action of Transfer. The Result is stored in the returned Binder, or a Null Handle for "No result"
    /// (Default defined as doing nothing; should be deferred).
    /// </summary>
    public virtual Transfer_Binder? Transferring(Transient? start, Transfer_ProcessForTransient? TP,
                                                 ProgressRange theProgress = default)
    {
        return NullResult();
    }

    /// <summary>Prepares and Returns a Binder for a Transient Result. Returns a Null Handle if <paramref name="res"/> is itself Null.</summary>
    public Transfer_SimpleBinderOfTransient? TransientResult(Transient? res)
    {
        Transfer_SimpleBinderOfTransient? binder = null;
        if (res is null)
        {
            return binder;
        }
        binder = new Transfer_SimpleBinderOfTransient();
        binder.SetResult(res);
        return binder;
    }

    /// <summary>Returns a Binder for No Result, i.e. a Null Handle.</summary>
    public Transfer_Binder? NullResult()
    {
        return null;
    }

    /// <summary>
    /// If <paramref name="mode"/> is True, commands an Actor to be set at the end of the list of Actors (see SetNext).
    /// If it is False (creation default), each add Actor is set at the beginning of the list. This allows to define
    /// default Actors (which are Last).
    /// </summary>
    public void SetLast(bool mode = true)
    {
        thelast = mode;
    }

    /// <summary>Returns the Last status (see SetLast).</summary>
    public bool IsLast()
    {
        return thelast;
    }

    /// <summary>
    /// Defines a Next Actor : it can then be asked to work if this actor produces no result for a given type of Object.
    /// If Next is already set and is not "Last", calls SetNext on it. If Next defined and "Last", the new actor is added
    /// before it in the list.
    /// </summary>
    public void SetNext(Transfer_ActorOfProcessForTransient? next)
    {
        if (thenext == next)
        {
            return;
        }
        if (thenext is null)
        {
            thenext = next;
        }
        else if (thenext.IsLast())
        {
            next.SetNext(thenext);
            thenext = next;
        }
        else
        {
            thenext.SetNext(next);
        }
    }

    /// <summary>Returns the Actor defined as Next, or a Null Handle.</summary>
    public Transfer_ActorOfProcessForTransient? Next()
    {
        return thenext;
    }

    private Transfer_ActorOfProcessForTransient? thenext;
    private bool thelast;
}
