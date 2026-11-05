// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorOfProcessForTransient.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ActorOfProcessForTransient_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// An actor of a <see cref="ProcessForTransient"/>: it recognizes starting objects and transfers them. Actors
/// form a list (<see cref="SetNext"/>); the "Last" ones (defaults) stay at its end.
/// </summary>
internal class ActorOfProcessForTransient : Transient
{
    public ActorOfProcessForTransient()
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
    public virtual Binder? Transferring(Transient? start, ProcessForTransient? tp,
                                                 ProgressRange progress = default)
    {
        return NullResult();
    }

    /// <summary>Prepares and Returns a Binder for a Transient Result. Returns a Null Handle if <paramref name="res"/> is itself Null.</summary>
    public SimpleBinderOfTransient? TransientResult(Transient? res)
    {
        SimpleBinderOfTransient? binder = null;
        if (res is null)
        {
            return binder;
        }
        binder = new SimpleBinderOfTransient();
        binder.SetResult(res);
        return binder;
    }

    /// <summary>Returns a Binder for No Result, i.e. a Null Handle.</summary>
    public Binder? NullResult()
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
        _thelast = mode;
    }

    /// <summary>Returns the Last status (see SetLast).</summary>
    public bool IsLast
    {
        get
        {
            return _thelast;
        }
    }

    /// <summary>
    /// Defines a Next Actor : it can then be asked to work if this actor produces no result for a given type of Object.
    /// If Next is already set and is not "Last", calls SetNext on it. If Next defined and "Last", the new actor is added
    /// before it in the list.
    /// </summary>
    public void SetNext(ActorOfProcessForTransient? next)
    {
        if (_thenext == next)
        {
            return;
        }
        if (_thenext is null)
        {
            _thenext = next;
        }
        else if (_thenext.IsLast)
        {
            next.SetNext(_thenext);
            _thenext = next;
        }
        else
        {
            _thenext.SetNext(next);
        }
    }

    /// <summary>Returns the Actor defined as Next, or a Null Handle.</summary>
    public ActorOfProcessForTransient? Next()
    {
        return _thenext;
    }

    private ActorOfProcessForTransient? _thenext;
    private bool _thelast;
}
