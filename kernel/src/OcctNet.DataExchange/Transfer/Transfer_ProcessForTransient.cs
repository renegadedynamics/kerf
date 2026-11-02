// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ProcessForTransient.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ProcessForTransient_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.Transfer_StatusExec;
using static Occt.Transfer_StatusResult;

namespace Occt;

/// <summary>
/// Manages Transfer of Transient Objects: a map from starting objects to binders (the results of their transfer, and
/// their checks), the roots, the actors which do the transfer, and the trace. Normally used as TransientProcess,
/// which adds some specifics.
/// </summary>
/// <remarks>
/// <para>
/// A working object, used by one thread at a time. The starting objects are keyed by reference, as OCCT's handles,
/// and numbered in the order they are bound (an indexed map), so the iterations over them don't depend on hashing.
/// </para>
/// <para>
/// Transfer_ProcessForFinder is the same class over <see cref="Transfer_Finder"/> keys, ported from OCCT's own second
/// instantiation (decision 1199).
/// </para>
/// </remarks>
public class Transfer_ProcessForTransient : Standard_Transient
{
    /// <summary>
    /// Sets TransferProcess at initial state. Gives an Initial size (indicative) for the Map when known (default is
    /// 10000). Sets default trace file as a printer and default trace level.
    /// </summary>
    public Transfer_ProcessForTransient(int nb = 10000)
    {
        themap = NewMap(nb);
        theerrh = true;
        therootm = false;
        thelevel = 0;
        therootl = 0;
        themessenger = Message.DefaultMessenger();
        thetrace = 0;
        theindex = 0;
    }

    /// <summary>
    /// Sets TransferProcess at initial state. Gives an Initial size (indicative) for the Map when known (default is
    /// 10000). Sets a specified printer.
    /// </summary>
    public Transfer_ProcessForTransient(Message_Messenger? messenger, int nb = 10000)
    {
        themap = NewMap(nb);
        theerrh = true;
        therootm = false;
        thelevel = 0;
        therootl = 0;
        SetMessenger(messenger);
        thetrace = 1;
        theindex = 0;
    }

    // The initial size is indicative only: the port's map grows as needed
    private static NCollection_IndexedDataMap<Standard_Transient, Transfer_Binder?> NewMap(int nb)
    {
        return new NCollection_IndexedDataMap<Standard_Transient, Transfer_Binder?>(ReferenceEqualityComparer.Instance);
    }

    /// <summary>Resets a TransferProcess as ready for a completely new work. Clears general data (roots) and the Map.</summary>
    public void Clear()
    {
        thelevel = 0;
        therootl = 0;
        theroots.Clear();
        themap.Clear();
        theindex = 0;
        thelastobj = null;
        thelastbnd = null;
    }

    /// <summary>
    /// Rebuilds the Map and the roots to really remove Unbound items. Because Unbind keeps the entity in place, even if
    /// not bound. Hence, working by checking new items is meaningless if a formerly unbound item is rebound.
    /// </summary>
    public void Clean()
    {
        int i, nb = NbMapped();
        int j, unb = 0;
        for (i = 1; i <= nb; i++)
        {
            if (themap[i] is null)
            {
                unb++;
            }
        }
        if (unb == 0)
        {
            return;
        }

        // Redo the map -> offsets
        var unbs = new NCollection_Array1<int>(1, nb);
        unbs.Init(0);
        NCollection_IndexedDataMap<Standard_Transient, Transfer_Binder?> newmap = NewMap(nb * 2);
        for (i = 1; i <= nb; i++)
        {
            Standard_Transient? ent = Mapped(i);
            Transfer_Binder? bnd = MapItem(i);
            if (bnd is null)
            {
                continue;
            }
            j = newmap.Add(ent ?? theNullStart, bnd);
            unbs.SetValue(i, j);
        }
        themap = newmap;

        // Update Root List
        var aNewRoots = new NCollection_IndexedMap<int>();
        for (i = 1; i <= theroots.Extent(); i++)
        {
            j = theroots.FindKey(i);
            int k = unbs.Value(j);
            if (k != 0)
            {
                aNewRoots.Add(k);
            }
        }
        theroots.Clear();
        theroots = aNewRoots;

        // The rest: cleaning
        thelastobj = null;
        thelastbnd = null;
        theindex = 0;
    }

    /// <summary>
    /// Resizes the Map as required (if a new reliable value has been determined). Acts only if <paramref name="nb"/> is
    /// greater than actual NbMapped.
    /// </summary>
    /// <remarks>The port's map grows as needed: nothing to do.</remarks>
    public void Resize(int nb)
    {
    }

    /// <summary>
    /// Defines an Actor, which is used for automatic Transfer. If already defined, the new Actor is cumulated (see
    /// SetNext from Actor).
    /// </summary>
    public void SetActor(Transfer_ActorOfProcessForTransient? actor)
    {
        if (theactor == actor)
        {
            return;
        }
        if (theactor is null)
        {
            theactor = actor;
        }
        else if (actor is null)
        {
            theactor = actor;
        }
        else if (theactor.IsLast())
        {
            actor.SetNext(theactor);
            theactor = actor;
        }
        else
        {
            theactor.SetNext(actor);
        }
    }

    /// <summary>Returns the defined Actor. Returns a Null Handle if not set.</summary>
    public Transfer_ActorOfProcessForTransient? Actor()
    {
        return theactor;
    }

    /// <summary>
    /// Returns the Binder which is linked with a starting Object. It can either bring a Result (Transfer done) or none
    /// (for a pre-binding). If no Binder is linked with <paramref name="start"/>, returns a Null Handle.
    /// </summary>
    public Transfer_Binder? Find(Standard_Transient? start)
    {
        if (thelastobj == start)
        {
            if (theindex > 0)
            {
                return thelastbnd;
            }
        }
        int index = FindIndex(start);
        if (index > 0)
        {
            Transfer_Binder? binder = themap.FindFromIndex(index);
            return binder;
        }
        return null;
    }

    /// <summary>
    /// Returns True if a Result (whatever its form) is Bound with a starting Object. I.e., if a Binder with a Result
    /// set, is linked with it.
    /// </summary>
    public bool IsBound(Standard_Transient? start)
    {
        Transfer_Binder? binder = Find(start);
        if (binder is null)
        {
            return false;
        }
        return binder.HasResult();
    }

    /// <summary>
    /// Returns True if the result of the transfer of an object is already used in other ones. If it is, Rebind cannot
    /// change it.
    /// </summary>
    /// <exception cref="Transfer_TransferFailure">The object is not bound.</exception>
    public bool IsAlreadyUsed(Standard_Transient? start)
    {
        Transfer_Binder? binder = Find(start);
        if (binder is null)
        {
            StartTrace(binder, start, thelevel, 4);
            throw new Transfer_TransferFailure("TransferProcess : IsAlreadyUsed, transfer not done cannot be used...");
        }
        return binder.Status() == Transfer_StatusUsed;
    }

    // Same as Find but stores the last access to the map, for a faster access on next calls (as Bind does too)
    private Transfer_Binder? FindAndMask(Standard_Transient? start)
    {
        if (thelastobj == start)
        {
            if (theindex > 0)
            {
                return thelastbnd;
            }
        }
        thelastobj = start;
        theindex = FindIndex(start);
        if (theindex > 0)
        {
            thelastbnd = themap.FindFromIndex(theindex);
        }
        else
        {
            thelastbnd = null;
        }
        return thelastbnd;
    }

    /// <summary>
    /// Creates a Link a starting Object with a Binder. This Binder can either bring a Result (effective Binding) or none
    /// (it can be set later : pre-binding).
    /// </summary>
    /// <exception cref="Transfer_TransferFailure">The object is bound to a result already used.</exception>
    public void Bind(Standard_Transient? start, Transfer_Binder? binder)
    {
        if (binder is null)
        {
            return;
        }
        Transfer_Binder? former = FindAndMask(start);
        if (former is not null)
        {
            // We admit VoidBinder: then we resume our Check
            if (former.DynamicType() == typeof(Transfer_VoidBinder))
            {
                binder.Merge(former);
                themap[theindex] = binder;
            }
            else if (former.Status() == Transfer_StatusUsed)
            {
                StartTrace(former, start, thelevel, 4);
                throw new Transfer_TransferFailure("TransferProcess : Bind, already Bound");
            }
            else
            {
                if (thetrace > 2)
                {
                    StartTrace(former, start, thelevel, 5);
                }
                binder.CCheck().GetMessages(former.Check());
            }
        }
        if (theindex == 0 || thelastbnd is null)
        {
            if (theindex == 0)
            {
                theindex = themap.Add(start ?? theNullStart, binder);
            }
            else
            {
                themap[theindex] = binder;
            }
            thelastbnd = binder;
        }
        else
        {
            thelastbnd = binder;
            themap[theindex] = binder;
        }
    }

    /// <summary>
    /// Changes the Binder linked with a starting Object for its unitary transfer. This it can be useful when the exact
    /// form of the result is known once the transfer is widely engaged. This can be done only on first transfer.
    /// </summary>
    public void Rebind(Standard_Transient? start, Transfer_Binder? binder)
    {
        Bind(start, binder);
    }

    /// <summary>
    /// Removes the Binder linked with a starting object. If this Binder brings a non-empty Check, it is replaced by a
    /// VoidBinder. Also removes from the list of Roots as required. Returns True if done, False if
    /// <paramref name="start"/> was not bound.
    /// </summary>
    public bool Unbind(Standard_Transient? start)
    {
        Transfer_Binder? former = FindAndMask(start);
        if (theindex == 0)
        {
            return false;
        }
        if (former is null)
        {
            return false;
        }
        if (former.DynamicType() == typeof(Transfer_VoidBinder))
        {
            return true;
        }
        themap[theindex] = thelastbnd; // OCCT: thelastbnd is the former binder here, so nothing is unbound
        if (theroots.Contains(theindex))
        {
            var aNewRoots = new NCollection_IndexedMap<int>();
            for (int i = 1; i <= theroots.Extent(); i++)
            {
                if (theindex != theroots.FindKey(i))
                {
                    aNewRoots.Add(theroots.FindKey(i));
                }
            }
            theroots.Clear();
            theroots = aNewRoots;
        }

        thelastobj = null;
        thelastbnd = null;
        theindex = 0;
        return true;
    }

    /// <summary>
    /// Returns a Binder for a starting entity, as follows : Tries to Find the already bound one. If none found, creates
    /// a VoidBinder and Binds it.
    /// </summary>
    public Transfer_Binder FindElseBind(Standard_Transient? start)
    {
        Transfer_Binder? binder = FindAndMask(start);
        if (binder is not null)
        {
            return binder;
        }
        binder = new Transfer_VoidBinder();
        Bind(start, binder);
        return binder;
    }

    /// <summary>Sets Messenger used for outputting messages (null: the default messenger).</summary>
    public void SetMessenger(Message_Messenger? messenger)
    {
        if (messenger is null)
        {
            themessenger = Message.DefaultMessenger();
        }
        else
        {
            themessenger = messenger;
        }
    }

    /// <summary>Returns Messenger used for outputting messages. The returned object is guaranteed to be non-null.</summary>
    public Message_Messenger Messenger()
    {
        return themessenger;
    }

    /// <summary>
    /// Sets trace level used for outputting messages: 0 : no trace at all; 1 : handled exceptions and calls to
    /// AddError; 2 : also calls to AddWarning; 3 : also traces new Roots. Default is 1 : Errors traced.
    /// </summary>
    public void SetTraceLevel(int tracelev)
    {
        thetrace = tracelev;
    }

    /// <summary>Returns trace level used for outputting messages.</summary>
    public int TraceLevel()
    {
        return thetrace;
    }

    /// <summary>New name for AddFail (Msg).</summary>
    public void SendFail(Standard_Transient? start, Message_Msg amsg)
    {
        AddFail(start, amsg);
    }

    /// <summary>New name for AddWarning (Msg).</summary>
    public void SendWarning(Standard_Transient? start, Message_Msg amsg)
    {
        AddWarning(start, amsg);
    }

    /// <summary>Adds an information message. Trace is filled if trace level is at least 3.</summary>
    public void SendMsg(Standard_Transient? start, Message_Msg amsg)
    {
        Transfer_Binder? binder = FindAndMask(start);
        if (binder is null)
        {
            binder = new Transfer_VoidBinder();
            Bind(start, binder);
        }
        // Feeds the trace: Causing rule (user messages)
        if (thetrace > 0)
        {
            StartTrace(binder, start, thelevel, 6);
            using Message_Messenger.StreamBuffer aSender = themessenger.SendInfo();
            aSender.Append(amsg.Value());
            if (amsg.IsEdited() && thetrace > 2)
            {
                aSender.Append(" [from: ").Append(amsg.Original()).Append("]");
            }
            aSender.Append('\n');
        }
    }

    /// <summary>Adds an Error message to a starting entity (to the check of its Binder of category 0, as a Fail).</summary>
    public void AddFail(Standard_Transient? start, string mess, string orig = "")
    {
        Transfer_Binder? binder = FindAndMask(start);
        if (binder is null)
        {
            binder = new Transfer_VoidBinder();
            Bind(start, binder);
        }
        binder.AddFail(mess, orig);
        if (thetrace > 0)
        {
            StartTrace(binder, start, thelevel, 1);
            using Message_Messenger.StreamBuffer aSender = themessenger.SendFail();
            aSender.Append("    --> Fail : ").Append(mess);
            if (orig.Length != 0 && thetrace > 2)
            {
                aSender.Append(" [from: ").Append(orig).Append("]");
            }
            aSender.Append('\n');
        }
    }

    /// <summary>(other name of AddFail, maintained for compatibility)</summary>
    public void AddError(Standard_Transient? start, string mess, string orig = "")
    {
        AddFail(start, mess, orig);
    }

    /// <summary>Adds an Error Message to a starting entity from the definition of a Msg (Original+Value).</summary>
    public void AddFail(Standard_Transient? start, Message_Msg amsg)
    {
        if (amsg.IsEdited())
        {
            AddFail(start, amsg.Value(), amsg.Original());
        }
        else
        {
            AddFail(start, amsg.Value());
        }
    }

    /// <summary>Adds a Warning message to a starting entity (to the check of its Binder of category 0).</summary>
    public void AddWarning(Standard_Transient? start, string mess, string orig = "")
    {
        Transfer_Binder? binder = FindAndMask(start);
        if (binder is null)
        {
            binder = new Transfer_VoidBinder();
            Bind(start, binder);
        }
        binder.AddWarning(mess, orig);
        if (thetrace > 1)
        {
            StartTrace(binder, start, thelevel, 2);
            using Message_Messenger.StreamBuffer aSender = themessenger.SendWarning();
            aSender.Append("    --> Warning : ").Append(mess);
            if (orig.Length != 0 && thetrace > 2)
            {
                aSender.Append(" [from: ").Append(orig).Append("]");
            }
            aSender.Append('\n');
        }
    }

    /// <summary>Adds a Warning Message to a starting entity from the definition of a Msg (Original+Value).</summary>
    public void AddWarning(Standard_Transient? start, Message_Msg amsg)
    {
        if (amsg.IsEdited())
        {
            AddWarning(start, amsg.Value(), amsg.Original());
        }
        else
        {
            AddWarning(start, amsg.Value());
        }
    }

    public void Mend(Standard_Transient? start, string pref = "")
    {
        Transfer_Binder? binder = FindAndMask(start);
        if (binder is null)
        {
            return;
        }
        Interface_Check ach = binder.CCheck();
        ach.Mend(pref);
    }

    /// <summary>Returns the Check attached to a starting entity. If <paramref name="start"/> is unknown, returns a null Check.</summary>
    public Interface_Check? Check(Standard_Transient? start)
    {
        Transfer_Binder? binder = Find(start);
        if (binder is null)
        {
            return null;
        }
        return binder.Check();
    }

    /// <summary>
    /// Binds a starting object with a Transient Result. Uses a SimpleBinderOfTransient to work. If there is already
    /// one but with no Result set, sets its Result.
    /// </summary>
    public void BindTransient(Standard_Transient? start, Standard_Transient? res)
    {
        if (res is null)
        {
            return;
        }
        Transfer_Binder? former = Find(start);
        Transfer_SimpleBinderOfTransient? binder = former as Transfer_SimpleBinderOfTransient;
        if (binder is not null)
        {
            if (binder.Status() == Transfer_StatusVoid)
            {
                binder.SetResult(res);
                return;
            }
        }
        binder = new Transfer_SimpleBinderOfTransient();
        binder.SetResult(res);
        if (former is null)
        {
            Bind(start, binder);
        }
        else
        {
            Rebind(start, binder);
        }
    }

    /// <summary>
    /// Returns the Result of the Transfer of an object <paramref name="start"/> as a Transient Result. Returns a Null
    /// Handle if there is no Transient Result. Supposes that Binding is done with a SimpleBinderOfTransient.
    /// </summary>
    public Standard_Transient? FindTransient(Standard_Transient? start)
    {
        Transfer_SimpleBinderOfTransient? binder = Find(start) as Transfer_SimpleBinderOfTransient;
        if (binder is null)
        {
            return null;
        }
        if (!binder.HasResult())
        {
            return null;
        }
        return binder.Result();
    }

    /// <summary>
    /// Prepares an object <paramref name="start"/> to be bound with several results. If no Binder is yet attached to
    /// it, a MultipleBinder is created, empty. If a Binder is already set, it must accept Multiple Binding.
    /// </summary>
    /// <exception cref="Transfer_TransferFailure">The binder already set is not a MultipleBinder.</exception>
    public void BindMultiple(Standard_Transient? start)
    {
        Transfer_Binder? binder = FindAndMask(start);
        if (binder is not null)
        {
            if (!binder.IsKind(typeof(Transfer_MultipleBinder)))
            {
                StartTrace(thelastbnd, start, thelevel, 4);
                throw new Transfer_TransferFailure("TransferProcess : BindMultiple");
            }
        }
        else
        {
            Bind(start, new Transfer_MultipleBinder());
        }
    }

    /// <summary>Adds an item to a list of results bound to a starting object.</summary>
    /// <exception cref="Transfer_TransferFailure">Nothing is bound, or the binder is not a MultipleBinder.</exception>
    public void AddMultiple(Standard_Transient? start, Standard_Transient? res)
    {
        Transfer_Binder? binder = FindAndMask(start);
        Transfer_MultipleBinder? multr = binder as Transfer_MultipleBinder;
        if (multr is null)
        {
            StartTrace(binder, start, thelevel, 4);
            if (binder is null)
            {
                throw new Transfer_TransferFailure("TransferProcess : AddMultiple, nothing bound");
            }
            else
            {
                throw new Transfer_TransferFailure("TransferProcess : AddMultiple, Binder not a MultipleBinder");
            }
        }
        multr.AddResult(res);
    }

    /// <summary>
    /// Searches for a transient result attached to a starting object, according to its type, by criterium
    /// IsKind(atype). In case of multiple result, explores the list and gives in <paramref name="val"/> the first
    /// transient result IsKind(atype). Returns True and fills <paramref name="val"/> if found. Else, returns False
    /// (<paramref name="val"/> is not touched, not even nullified).
    /// </summary>
    public bool FindTypedTransient(Standard_Transient? start, Type? atype, ref Standard_Transient? val)
    {
        return GetTypedTransient(Find(start), atype, ref val);
    }

    /// <summary>
    /// Searches for a transient result recorded in a Binder, whatever this Binder is recorded or not in this process.
    /// Strictly equivalent to <see cref="Transfer_SimpleBinderOfTransient.GetTypedResult"/>.
    /// </summary>
    public bool GetTypedTransient(Transfer_Binder? binder, Type? atype, ref Standard_Transient? val)
    {
        return Transfer_SimpleBinderOfTransient.GetTypedResult(binder, atype, ref val);
    }

    /// <summary>Returns the maximum possible value for Map Index (no result can be bound with a value greater than it).</summary>
    public int NbMapped()
    {
        return themap.Extent();
    }

    /// <summary>Returns the Starting Object bound to an Index.</summary>
    public Standard_Transient? Mapped(int num)
    {
        Standard_Transient aKey = themap.FindKey(num);
        return ReferenceEquals(aKey, theNullStart) ? null : aKey;
    }

    /// <summary>Returns the Index value bound to a Starting Object, 0 if none.</summary>
    public int MapIndex(Standard_Transient? start)
    {
        return FindIndex(start);
    }

    /// <summary>Returns the Binder bound to an Index.</summary>
    public Transfer_Binder? MapItem(int num)
    {
        Transfer_Binder? binder = themap.FindFromIndex(num);
        return binder;
    }

    /// <summary>
    /// Declares <paramref name="start"/> (and its Result) as Root. This status will be later exploited by RootResult
    /// (Result can be produced at any time).
    /// </summary>
    public void SetRoot(Standard_Transient? start)
    {
        int index = MapIndex(start);
        if (index == 0)
        {
            return;
        }

        theroots.Add(index);
        if (thetrace > 2)
        {
            StartTrace(MapItem(index), start, thelevel, 3);
        }
    }

    /// <summary>
    /// Enable (if <paramref name="stat"/> True) or Disables (if False) Root Management. If it is set, Transfers are
    /// considered as stacked (a first Transfer commands other Transfers, and so on) and the Transfers commanded by an
    /// external caller are "Root". Remark : SetRoot can be called whatever this status, on every object.
    /// </summary>
    public void SetRootManagement(bool stat)
    {
        therootm = stat;
    }

    /// <summary>Returns the count of recorded Roots.</summary>
    public int NbRoots()
    {
        return theroots.Extent();
    }

    /// <summary>Returns a Root Entity given its number in the list (1-NbRoots).</summary>
    /// <exception cref="Standard_OutOfRange">No root has the number.</exception>
    public Standard_Transient? Root(int num)
    {
        int ind = 0;
        if (num > 0 && num <= theroots.Extent())
        {
            ind = theroots.FindKey(num);
        }
        return Mapped(ind);
    }

    /// <summary>Returns the Binder bound with a Root Entity given its number.</summary>
    /// <exception cref="Standard_OutOfRange">No root has the number.</exception>
    public Transfer_Binder? RootItem(int num)
    {
        int ind = 0;
        if (num > 0 && num <= theroots.Extent())
        {
            ind = theroots.FindKey(num);
        }
        return themap.FindFromIndex(ind);
    }

    /// <summary>Returns the index in the list of roots for a starting item, or 0 if it is not recorded as a root.</summary>
    public int RootIndex(Standard_Transient? start)
    {
        int index = MapIndex(start);
        if (index == 0)
        {
            return 0;
        }
        return theroots.FindIndex(index);
    }

    /// <summary>
    /// Returns Nesting Level of Transfers (managed by methods TranscriptWith &amp; Co). Starts to zero. If no automatic
    /// Transfer is used, it remains to zero. Zero means Root Level.
    /// </summary>
    public int NestingLevel()
    {
        return thelevel;
    }

    /// <summary>Resets Nesting Level of Transfers to Zero (Root Level), whatever its current value.</summary>
    public void ResetNestingLevel()
    {
        thelevel = 0;
    }

    /// <summary>Tells if <paramref name="start"/> has been recognized as good candidate for Transfer, i.e. queries the Actor and its Nexts.</summary>
    public bool Recognize(Standard_Transient? start)
    {
        Transfer_ActorOfProcessForTransient? actor = theactor;
        // We scan the Next until we have a Result
        while (actor is not null)
        {
            if (actor.Recognize(start))
            {
                return true;
            }
            actor = actor.Next();
        }
        return false;
    }

    /// <summary>
    /// Performs the Transfer of a Starting Object, by calling the method TransferProduct (see below). Mapping and
    /// Roots are managed : nothing is done if a Result is already Bound, an exception is raised in case of error.
    /// </summary>
    /// <exception cref="Transfer_TransferFailure">The object's transfer is in error status.</exception>
    public Transfer_Binder? Transferring(Standard_Transient? start, Message_ProgressRange theProgress = default)
    {
        Transfer_Binder? former = FindAndMask(start);
        // We consider that this new Transfer request therefore corresponds to a
        // additional use: note "AlreadyUsed", therefore non-modifiable result
        bool hasDeadLoop = false;
        if (former is not null)
        {
            if (former.HasResult())
            {
                former.SetAlreadyUsed();
                return former;
            }

            using Message_Messenger.StreamBuffer aSender = themessenger.SendInfo();
            Transfer_StatusExec statex = former.StatusExec();
            switch (statex)
            {
                case Transfer_StatusInitial:
                    break;
                case Transfer_StatusDone:
                    aSender.Append(" .. and Transfer done").Append('\n');
                    return former;
                case Transfer_StatusRun:
                    former.SetStatusExec(Transfer_StatusLoop);
                    return former;
                case Transfer_StatusError:
                    if (thetrace != 0)
                    {
                        aSender.Append("                  *** Transfer in Error Status  :").Append('\n');
                        StartTrace(former, start, thelevel, 0);
                    }
                    else
                    {
                        StartTrace(former, start, thelevel, 4);
                    }
                    throw new Transfer_TransferFailure("TransferProcess : Transfer in Error Status");
                case Transfer_StatusLoop:
                    if (thetrace != 0)
                    {
                        aSender.Append("                  *** Transfer  Head of Dead Loop  :").Append('\n');
                        StartTrace(former, start, thelevel, 0);
                    }
                    else
                    {
                        StartTrace(former, start, thelevel, 4);
                    }
                    hasDeadLoop = true;
                    break;
            }
            if (!hasDeadLoop)
            {
                former.SetStatusExec(Transfer_StatusRun);
            }
        }

        Transfer_Binder? binder = null;
        bool newbind = false;

        // Handle dead loop condition detected before calling TransferProduct
        // When hasDeadLoop is true, we know:
        // 1. former is NOT null (hasDeadLoop only set inside !former.IsNull() block)
        // 2. former->StatusExec() == Transfer_StatusLoop (that's what triggered hasDeadLoop)
        if (hasDeadLoop)
        {
            using Message_Messenger.StreamBuffer aSender = themessenger.SendInfo();
            if (thetrace != 0)
            {
                aSender.Append("                  *** Dead Loop : Finding head of Loop :").Append('\n');
                StartTrace(former, start, thelevel - 1, 0);
            }
            else
            {
                StartTrace(former, start, thelevel - 1, 4);
            }
            binder = former;
            binder.AddFail("Transfer: Head of Dead Loop");
            binder.AddFail("Transfer in dead Loop");
        }
        else if (theerrh)
        {
            using Message_Messenger.StreamBuffer aSender = themessenger.SendInfo();
            // Transfer under protection for exceptions (for notification in fact)
            int oldlev = thelevel;
            try
            {
                binder = TransferProduct(start, theProgress);
            }
            catch (Exception anException) when (IMeshTools_ModelAlgo.IsCaughtFailure(anException))
            {
                // OCC_CATCH_SIGNALS: a Standard_Failure, or a system signal (their .NET counterparts, decision 1183)
                if (binder is null)
                {
                    aSender.Append("                  *** Exception Raised with no Result").Append('\n');
                    binder = new Transfer_VoidBinder();
                    Bind(start, binder);
                    newbind = true;
                }
                binder.AddFail("Transfer stopped by exception raising");
                if (thetrace != 0)
                {
                    aSender.Append("    *** Raised : ").Append(What(anException)).Append('\n');
                    StartTrace(binder, start, thelevel - 1, 4);
                }
                thelevel = oldlev;
            }
        }

        // Unprotected transfer (thus, dbx by hand in case of crash by Raise)
        else
        {
            binder = TransferProduct(start, theProgress);
        }

        if (theProgress.UserBreak())
        {
            return null;
        }

        //    Conclusion : Noter dans la Map

        if (!newbind && binder is not null)
        {
            if (former is null)
            {
                if (!IsBound(start))
                {
                    Bind(start, binder);
                }
                else
                {
                    Rebind(start, binder);
                }
            }
            else
            {
                Rebind(start, binder);
            }
        }
        else
        {
            if (former is not null)
            {
                former.SetStatusExec(Transfer_StatusDone); //+
            }
            return null; // Null Binder ... what else to do?
        }

        //  Manage Root
        if (therootl >= thelevel) // OCCT: nested transfers ending at the level of the first one done are roots too
        {
            therootl = 0;
            if (therootm && binder.Status() != Transfer_StatusVoid)
            {
                SetRoot(start);
            }
        }
        return thelastbnd;
    }

    // OCCT's Standard_Failure::what(): the message, empty when there is none; a NullReferenceException is natively an
    // access violation, with Windows' text for a read at address 0 (decision 977)
    internal static string What(Exception theException)
    {
        if (theException is NullReferenceException)
        {
            return "ACCESS VIOLATION at address 0x0000000000000000 during 'READ' operation";
        }
        string aMessage = theException.Message;
        return aMessage == $"Exception of type '{theException.GetType().FullName}' was thrown." ? "" : aMessage;
    }

    // Internal action of Transfer, called by Transferring, with or without ErrorHandle. It invokes the Actor to work
    // (set by SetActor), and tries its Nexts if no result is produced, until a Non Null Binder is produced. But keep in
    // mind that a Null Binder can always be returned if a Starting Entity has not been recognized at all.
    private Transfer_Binder? TransferProduct(Standard_Transient? start, Message_ProgressRange theProgress)
    {
        thelevel++; // decrement and if == 0, root transfer
        Transfer_Binder? binder = null;
        Transfer_ActorOfProcessForTransient? actor = theactor;

        // We scan the Next until we have a Result
        using var aScope = new Message_ProgressScope(theProgress, null, 1, true);
        while (actor is not null)
        {
            if (actor.Recognize(start))
            {
                binder = actor.Transferring(start, this, aScope.Next());
            }
            else
            {
                binder = null;
            }
            if (binder is not null)
            {
                break;
            }
            actor = actor.Next();
        }
        if (aScope.UserBreak())
        {
            return null;
        }

        if (binder is null)
        {
            if (thelevel > 0)
            {
                thelevel--;
            }
            return binder;
        }
        if (therootl == 0 && binder.StatusExec() == Transfer_StatusDone)
        {
            therootl = thelevel - 1;
        }

        if (thelevel > 0)
        {
            thelevel--;
        }
        return binder;
    }

    /// <summary>Same as Transferring but does not return the Binder. Simply returns True in case of success (for user call).</summary>
    public bool Transfer(Standard_Transient? start, Message_ProgressRange theProgress = default)
    {
        Transfer_Binder? binder = Transferring(start, theProgress);
        return binder is not null;
    }

    /// <summary>
    /// Allows controls if exceptions will be handled Transfer Operations. False : they are not handled with
    /// try {} catch {}; True : they are.
    /// </summary>
    public void SetErrorHandle(bool err)
    {
        theerrh = err;
    }

    /// <summary>Returns error handling flag.</summary>
    public bool ErrorHandle()
    {
        return theerrh;
    }

    /// <summary>
    /// Method called when trace is asked. Calls PrintTrace to display information relevant for starting objects
    /// (which can be redefined). <paramref name="level"/> is Nesting Level of Transfer (0 = root).
    /// <paramref name="mode"/> controls the way the trace is done : 0 neutral, 1 for Error, 2 for Warning message, 3
    /// for new Root.
    /// </summary>
    public void StartTrace(Transfer_Binder? binder, Standard_Transient? start, int level, int mode)
    {
        using Message_Messenger.StreamBuffer aSender = themessenger.SendInfo();
        // ###  Fail (Roots:50)  --  Start start->DynamicType()
        // ###  Fail (Roots:50)  --  Start id:#label.. Type:start->DynamicType()
        if (thetrace > 3)
        { // Internal to be switch when searching bug (trace >= 4)
            if (mode == 1)
            {
                aSender.Append("  ###  Fail");
            }
            if (mode == 2)
            {
                aSender.Append("  ###  Warning");
            }
            if (mode == 3)
            {
                aSender.Append("  ###  New Root n0 ").Append(theroots.Extent());
            }
            if (mode == 4)
            {
                aSender.Append("  ###  Exception");
            }
            if (mode == 5)
            {
                aSender.Append("  ###  Substitution");
            }
            if (mode == 6)
            {
                aSender.Append("  ###  Information");
            }
            if (level > 1)
            {
                aSender.Append(" (nested)"); // " at nesting Level:"<<level;
            }
            if (mode >= 0 && mode != 3)
            {
                aSender.Append(" at ").Append(theroots.Extent()).Append(" Roots");
            }
        }
        if (start is not null)
        {
            using var aStream = new StringWriter(aSender.Stream());
            PrintTrace(start, aStream);
        }
        if (binder is not null)
        { // old: if IsNull sout <<endl<< "  ---  Not Bound";
            Transfer_Binder? bnd = binder;
            bool hasres = false;
            while (bnd is not null)
            {
                if (bnd.Status() != Transfer_StatusVoid)
                {
                    // ---  Result Type: binder->ResultType()  ---  Binder : binder->DynamicType();
                    if (!hasres)
                    {
                        aSender.Append("\n  ---  Result Type : ");
                    }
                    else
                    {
                        aSender.Append(" , ");
                    }
                    aSender.Append(bnd.ResultTypeName());
                    // CKY 9-JAN-1999:  waiting for XSTEP Kernel message (not IGES_2075)
                    hasres = true;
                }
                bnd = bnd.NextResult();
            }
            if (!hasres && mode > 2)
            {
                aSender.Append("\n  ---  No Result recorded");
                // CKY 9-JAN-1999 : waiting for XSTEP Kernel message
                //     (not IGES_2075)
            }
            // old: if (isused) sout << "  --    (Already Used in another Transfer)";
        }
        aSender.Append('\n');
    }

    /// <summary>Prints a short information on a starting object. By default prints its Dynamic Type. Can be redefined.</summary>
    public virtual void PrintTrace(Standard_Transient? start, TextWriter S)
    {
        if (start is not null)
        {
            S.Write(" Type:");
            S.Write(start.DynamicType().Name);
        }
    }

    /// <summary>
    /// Returns True if we are surely in a DeadLoop. Evaluation is not exact, it is a "majorant" which must be computed
    /// fast. This "majorant" is : <paramref name="alevel"/> greater than NbMapped.
    /// </summary>
    public bool IsLooping(int alevel)
    {
        return alevel > NbMapped();
    }

    //  ########################################################################
    //  ....                            RESULTS                            ....

    /// <summary>
    /// Returns, as an iterator, the log of root transfer, i.e. the created objects and Binders bound to starting
    /// roots. If withstart is given True, Starting Objects are also returned.
    /// </summary>
    public Transfer_IteratorOfProcessForTransient RootResult(bool withstart = false)
    {
        var iter = new Transfer_IteratorOfProcessForTransient(withstart);
        int max = theroots.Extent();
        for (int j = 1; j <= max; j++)
        {
            int i = theroots.FindKey(j);
            Transfer_Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            if (withstart)
            {
                iter.Add(binder, Mapped(i));
            }
            else
            {
                iter.Add(binder);
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns, as an Iterator, the entire log of transfer (list of created objects and Binders which can bring
    /// errors). If withstart is given True, Starting Objects are also returned.
    /// </summary>
    public Transfer_IteratorOfProcessForTransient CompleteResult(bool withstart = false)
    {
        var iter = new Transfer_IteratorOfProcessForTransient(withstart);
        int max = NbMapped();
        for (int i = 1; i <= max; i++)
        {
            Transfer_Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            if (withstart)
            {
                iter.Add(binder, Mapped(i));
            }
            else
            {
                iter.Add(binder);
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns Binders which are neither "Done" nor "Initial", that is Error,Loop or Run (abnormal states at end of
    /// Transfer). Starting Objects are given in correspondence in the iterator.
    /// </summary>
    public Transfer_IteratorOfProcessForTransient AbnormalResult()
    {
        var iter = new Transfer_IteratorOfProcessForTransient(true);
        int max = NbMapped();
        for (int i = 1; i <= max; i++)
        {
            Transfer_Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            Transfer_StatusExec statex = binder.StatusExec();
            if (statex != Transfer_StatusInitial && statex != Transfer_StatusDone)
            {
                iter.Add(binder, Mapped(i)); // on note les cas "pas normaux"
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns a CheckList as a list of Check : each one is for a starting entity which have either check (warning or
    /// fail) messages are attached, or are in abnormal state : that case gives a specific message. If
    /// <paramref name="erronly"/> is True, checks with Warnings only are ignored.
    /// </summary>
    public Interface_CheckIterator CheckList(bool erronly)
    {
        var list = new Interface_CheckIterator();
        int num, max = NbMapped();
        for (int i = 1; i <= max; i++)
        {
            Transfer_Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            Transfer_StatusExec statex = binder.StatusExec();
            Interface_Check check = binder.Check();
            if (statex != Transfer_StatusInitial && statex != Transfer_StatusDone && !check.HasFailed())
            {
                check.AddFail("Transfer in Abnormal Status (!= Initial or Done)");
            }
            if (!check.HasFailed() && (erronly || check.NbWarnings() == 0))
            {
                continue;
            }
            Standard_Transient? ent = Mapped(i);
            num = CheckNum(ent);
            if (num == 0)
            {
                num = i;
            }
            check.SetEntity(ent);
            list.Add(check, num);
        }
        return list;
    }

    //  #########################################################################
    //  ....                         RESULTS BY ENTITY                         ....

    /// <summary>
    /// Returns, as an Iterator, the log of transfer for one object. <paramref name="level"/> = 0 : this object only;
    /// and if <paramref name="start"/> is a scope owner (else, <paramref name="level"/> is ignored) : 1 : object plus
    /// its immediate scoped ones; 2 : object plus all its scoped ones.
    /// </summary>
    /// <remarks>OCCT's scopes were removed: its list of indices is all zeros, so the iterator is always empty (occt-issues.md).</remarks>
    public Transfer_IteratorOfProcessForTransient ResultOne(Standard_Transient? start, int level, bool withstart = false)
    {
        var iter = new Transfer_IteratorOfProcessForTransient(withstart);
        int max = NbMapped();
        int ind = MapIndex(start);
        if (ind == 0)
        {
            return iter;
        }
        int i1 = level == 0 ? ind : 1;
        int i2 = level == 0 ? ind : max;
        var map = new NCollection_HArray1<int>(i1, i2); // zeros
        // OCCT: map is never filled (the scopes were removed), so nothing is iterated
        for (int i = i1; i <= i2; i++)
        {
            ind = map.Value(i);
            if (ind == 0)
            {
                continue;
            }
            Transfer_Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            if (withstart)
            {
                iter.Add(binder, Mapped(ind));
            }
            else
            {
                iter.Add(binder);
            }
        }
        return iter;
    }

    /// <summary>
    /// Returns a CheckList for one starting object. <paramref name="level"/> interpreted as by ResultOne. If
    /// <paramref name="erronly"/> is True, checks with Warnings only are ignored.
    /// </summary>
    /// <remarks>As <see cref="ResultOne"/>, always empty (occt-issues.md).</remarks>
    public Interface_CheckIterator CheckListOne(Standard_Transient? start, int level, bool erronly)
    {
        var list = new Interface_CheckIterator();
        int max = NbMapped();
        int num, ind = MapIndex(start);
        if (ind == 0)
        {
            return list;
        }

        int i1 = level == 0 ? ind : 1;
        int i2 = level == 0 ? ind : max;
        var map = new NCollection_HArray1<int>(i1, i2); // zeros
        // OCCT: map is never filled (the scopes were removed), so nothing is listed
        for (int i = i1; i <= i2; i++)
        {
            ind = map.Value(i);
            if (ind == 0)
            {
                continue;
            }
            Transfer_Binder? binder = MapItem(ind);
            if (binder is null)
            {
                continue;
            }

            Transfer_StatusExec statex = binder.StatusExec();
            Interface_Check check = binder.Check();
            if (statex != Transfer_StatusInitial && statex != Transfer_StatusDone && !check.HasFailed())
            {
                check.AddFail("Transfer in Abnormal Status (!= Initial or Done)");
            }
            if (!check.HasFailed() && (erronly || check.NbWarnings() == 0))
            {
                continue;
            }
            Standard_Transient? ent = Mapped(ind);
            num = CheckNum(ent);
            if (num == 0)
            {
                num = ind;
            }
            check.SetEntity(ent);
            list.Add(check, num);
        }
        return list;
    }

    /// <summary>
    /// Returns True if no check message is attached to a starting object. <paramref name="level"/> interpreted as by
    /// ResultOne. If <paramref name="erronly"/> is True, checks with Warnings only are ignored.
    /// </summary>
    /// <remarks>As <see cref="ResultOne"/>, nothing is looked at: true for any mapped object (occt-issues.md).</remarks>
    public bool IsCheckListEmpty(Standard_Transient? start, int level, bool erronly)
    {
        int max = NbMapped();
        int ind = MapIndex(start);
        if (ind == 0)
        {
            return false;
        }

        int i1 = level == 0 ? ind : 1;
        int i2 = level == 0 ? ind : max;
        var map = new NCollection_HArray1<int>(i1, i2); // zeros
        // OCCT: map is never filled (the scopes were removed), so nothing is checked
        for (int i = i1; i <= i2; i++)
        {
            ind = map.Value(i);
            if (ind == 0)
            {
                continue;
            }
            Transfer_Binder? binder = MapItem(ind);
            if (binder is null)
            {
                continue;
            }

            Transfer_StatusExec statex = binder.StatusExec();
            Interface_Check check = binder.Check();
            if (statex != Transfer_StatusInitial && statex != Transfer_StatusDone)
            {
                return false;
            }
            if (check.HasFailed() || (!erronly && check.NbWarnings() > 0))
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Removes Results attached to (== Unbinds) a given object and, according <paramref name="level"/> : 0 : only it;
    /// 1 : it plus its immediately owned sub-results(scope); 2 : it plus all its owned sub-results(scope).
    /// </summary>
    /// <remarks>OCCT's body unbinds nothing (the scopes were removed): it does nothing (occt-issues.md).</remarks>
    public void RemoveResult(Standard_Transient? start, int level, bool compute = true)
    {
        // if (compute) ComputeScopes();
        int max = NbMapped();
        int ind = MapIndex(start);
        if (ind == 0)
        {
            return;
        }

        int i1 = level == 0 ? ind : 1;
        int i2 = level == 0 ? ind : max;
        var map = new NCollection_HArray1<int>(i1, i2); // zeros
        int i;
        for (i = i1; i <= i2; i++)
        {
            ind = map.Value(i);
            if (ind == 0)
            {
                continue;
            }
            Transfer_Binder? binder = MapItem(ind);
            if (binder is null)
            {
                continue;
            }
        }
    }

    /// <summary>
    /// Computes a number to be associated to a starting object in a check or a check-list. By default, returns 0; can
    /// be redefined.
    /// </summary>
    public virtual int CheckNum(Standard_Transient? start)
    {
        return 0;
    }

    // The index of a starting object, 0 if none. A null object is mapped as OCCT maps a null handle (a check
    // attached to a missing entity, StepToTopoDS's for instance), under a key of its own (decision 1253)
    private int FindIndex(Standard_Transient? start)
    {
        return themap.FindIndex(start ?? theNullStart);
    }

    // The key standing for a null starting object in the map (.NET's dictionaries refuse null keys)
    private static readonly Standard_Transient theNullStart = new Transfer_VoidBinder();

    private bool theerrh;
    private int thetrace;
    private Message_Messenger themessenger;
    private int thelevel;
    private int therootl;
    private bool therootm;
    private NCollection_IndexedMap<int> theroots = new();
    private Standard_Transient? thelastobj;
    private Transfer_Binder? thelastbnd;
    private int theindex;
    private Transfer_ActorOfProcessForTransient? theactor;
    private NCollection_IndexedDataMap<Standard_Transient, Transfer_Binder?> themap;
}
