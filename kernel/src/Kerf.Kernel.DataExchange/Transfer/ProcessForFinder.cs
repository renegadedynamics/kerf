// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ProcessForFinder.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ProcessForFinder_0.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Occt;
using static Kerf.Engine.Exchange.Framework.Transfer.StatusExec;
using static Kerf.Engine.Exchange.Framework.Transfer.StatusResult;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// Manages Transfer of Finders (objects of an application, such as shapes, through their mappers): a map from starting
/// finders to binders (the results of their transfer, and their checks), the roots, the actors which do the transfer,
/// and the trace. Normally used as FinderProcess, which adds some specifics.
/// </summary>
/// <remarks>
/// <para>
/// A working object, used by one thread at a time. The starting finders are keyed by <see cref="FindHasher"/>
/// (two finders of the same object are the same key) and numbered in the order they are bound (an indexed map), so the
/// iterations over them don't depend on hashing.
/// </para>
/// <para>
/// The same class as <see cref="ProcessForTransient"/> over <see cref="Finder"/> keys: OCCT's second
/// instantiation of its former generic, ported from its own file and kept line for line with the first (decision
/// 12401).
/// </para>
/// </remarks>
internal class ProcessForFinder : Transient
{
    /// <summary>
    /// Sets TransferProcess at initial state. Gives an Initial size (indicative) for the Map when known (default is
    /// 10000). Sets default trace file as a printer and default trace level.
    /// </summary>
    public ProcessForFinder(int nb = 10000)
    {
        _themap = NewMap(nb);
        _theerrh = true;
        _therootm = false;
        _thelevel = 0;
        _therootl = 0;
        _themessenger = global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        _thetrace = 0;
        _theindex = 0;
    }

    /// <summary>
    /// Sets TransferProcess at initial state. Gives an Initial size (indicative) for the Map when known (default is
    /// 10000). Sets a specified printer.
    /// </summary>
    public ProcessForFinder(Messenger? messenger, int nb = 10000)
    {
        _themap = NewMap(nb);
        _theerrh = true;
        _therootm = false;
        _thelevel = 0;
        _therootl = 0;
        SetMessenger(messenger);
        _thetrace = 1;
        _theindex = 0;
    }

    // The initial size is indicative only: the port's map grows as needed
    private static IndexedMap<Finder, Binder?> NewMap(int nb)
    {
        return new IndexedMap<Finder, Binder?>(FindHasher.Instance);
    }

    /// <summary>Resets a TransferProcess as ready for a completely new work. Clears general data (roots) and the Map.</summary>
    public void Clear()
    {
        _thelevel = 0;
        _therootl = 0;
        _theroots.Clear();
        _themap.Clear();
        _theindex = 0;
        _thelastobj = null;
        _thelastbnd = null;
    }

    /// <summary>
    /// Rebuilds the Map and the roots to really remove Unbound items. Because Unbind keeps the entity in place, even if
    /// not bound. Hence, working by checking new items is meaningless if a formerly unbound item is rebound.
    /// </summary>
    public void Clean()
    {
        int i, nb = MappedCount;
        int j, unb = 0;
        for (i = 1; i <= nb; i++)
        {
            if (_themap.ValueAt(i - 1) is null)
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
        IndexedMap<Finder, Binder?> newmap = NewMap(nb * 2);
        for (i = 1; i <= nb; i++)
        {
            Finder? ent = Mapped(i);
            Binder? bnd = MapItem(i);
            if (bnd is null)
            {
                continue;
            }
            j = newmap.Add(ent, bnd) + 1;
            unbs.SetValue(i, j);
        }
        _themap = newmap;

        // Update Root List
        var newRoots = new IndexedSet<int>();
        for (i = 1; i <= _theroots.Count; i++)
        {
            j = _theroots[i - 1];
            int k = unbs.Value(j);
            if (k != 0)
            {
                newRoots.Add(k);
            }
        }
        _theroots.Clear();
        _theroots = newRoots;

        // The rest: cleaning
        _thelastobj = null;
        _thelastbnd = null;
        _theindex = 0;
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
    public void SetActor(ActorOfProcessForFinder? actor)
    {
        if (_theactor == actor)
        {
            return;
        }
        if (_theactor is null)
        {
            _theactor = actor;
        }
        else if (actor is null)
        {
            _theactor = actor;
        }
        else if (_theactor.IsLast)
        {
            actor.SetNext(_theactor);
            _theactor = actor;
        }
        else
        {
            _theactor.SetNext(actor);
        }
    }

    /// <summary>Returns the defined Actor. Returns a Null Handle if not set.</summary>
    public ActorOfProcessForFinder? Actor()
    {
        return _theactor;
    }

    /// <summary>
    /// Returns the Binder which is linked with a starting Object. It can either bring a Result (Transfer done) or none
    /// (for a pre-binding). If no Binder is linked with <paramref name="start"/>, returns a Null Handle.
    /// </summary>
    public Binder? Find(Finder? start)
    {
        if (_thelastobj == start)
        {
            if (_theindex > 0)
            {
                return _thelastbnd;
            }
        }
        int index = FindIndex(start);
        if (index > 0)
        {
            Binder? binder = _themap.ValueAt(index - 1);
            return binder;
        }
        return null;
    }

    /// <summary>
    /// Returns True if a Result (whatever its form) is Bound with a starting Object. I.e., if a Binder with a Result
    /// set, is linked with it.
    /// </summary>
    public bool IsBound(Finder? start)
    {
        Binder? binder = Find(start);
        if (binder is null)
        {
            return false;
        }
        return binder.HasResult;
    }

    /// <summary>
    /// Returns True if the result of the transfer of an object is already used in other ones. If it is, Rebind cannot
    /// change it.
    /// </summary>
    /// <exception cref="TransferFailureException">The object is not bound.</exception>
    public bool IsAlreadyUsed(Finder? start)
    {
        Binder? binder = Find(start);
        if (binder is null)
        {
            StartTrace(binder, start, _thelevel, 4);
            throw new TransferFailureException("TransferProcess : IsAlreadyUsed, transfer not done cannot be used...");
        }
        return binder.Status() == StatusResult.StatusUsed;
    }

    // Same as Find but stores the last access to the map, for a faster access on next calls (as Bind does too)
    private Binder? FindAndMask(Finder? start)
    {
        if (_thelastobj == start)
        {
            if (_theindex > 0)
            {
                return _thelastbnd;
            }
        }
        _thelastobj = start;
        _theindex = FindIndex(start);
        if (_theindex > 0)
        {
            _thelastbnd = _themap.ValueAt(_theindex - 1);
        }
        else
        {
            _thelastbnd = null;
        }
        return _thelastbnd;
    }

    /// <summary>
    /// Creates a Link a starting Object with a Binder. This Binder can either bring a Result (effective Binding) or none
    /// (it can be set later : pre-binding).
    /// </summary>
    /// <exception cref="TransferFailureException">The object is bound to a result already used.</exception>
    public void Bind(Finder? start, Binder? binder)
    {
        if (binder is null)
        {
            return;
        }
        Binder? former = FindAndMask(start);
        if (former is not null)
        {
            // We admit VoidBinder: then we resume our Check
            if (former.DynamicType() == typeof(VoidBinder))
            {
                binder.Merge(former);
                _themap.ValueAt(_theindex - 1) = binder;
            }
            else if (former.Status() == StatusResult.StatusUsed)
            {
                StartTrace(former, start, _thelevel, 4);
                throw new TransferFailureException("TransferProcess : Bind, already Bound");
            }
            else
            {
                if (_thetrace > 2)
                {
                    StartTrace(former, start, _thelevel, 5);
                }
                binder.CCheck().GetMessages(former.Check());
            }
        }
        if (_theindex == 0 || _thelastbnd is null)
        {
            if (_theindex == 0)
            {
                _theindex = _themap.Add(start, binder) + 1;
            }
            else
            {
                _themap.ValueAt(_theindex - 1) = binder;
            }
            _thelastbnd = binder;
        }
        else
        {
            _thelastbnd = binder;
            _themap.ValueAt(_theindex - 1) = binder;
        }
    }

    /// <summary>
    /// Changes the Binder linked with a starting Object for its unitary transfer. This it can be useful when the exact
    /// form of the result is known once the transfer is widely engaged. This can be done only on first transfer.
    /// </summary>
    public void Rebind(Finder? start, Binder? binder)
    {
        Bind(start, binder);
    }

    /// <summary>
    /// Removes the Binder linked with a starting object. If this Binder brings a non-empty Check, it is replaced by a
    /// VoidBinder. Also removes from the list of Roots as required. Returns True if done, False if
    /// <paramref name="start"/> was not bound.
    /// </summary>
    public bool Unbind(Finder? start)
    {
        Binder? former = FindAndMask(start);
        if (_theindex == 0)
        {
            return false;
        }
        if (former is null)
        {
            return false;
        }
        if (former.DynamicType() == typeof(VoidBinder))
        {
            return true;
        }
        _themap.ValueAt(_theindex - 1) = _thelastbnd; // OCCT: thelastbnd is the former binder here, so nothing is unbound
        if (_theroots.Contains(_theindex))
        {
            var newRoots = new IndexedSet<int>();
            for (int i = 0; i < _theroots.Count; i++)
            {
                if (_theindex != _theroots[i])
                {
                    newRoots.Add(_theroots[i]);
                }
            }
            _theroots.Clear();
            _theroots = newRoots;
        }

        _thelastobj = null;
        _thelastbnd = null;
        _theindex = 0;
        return true;
    }

    /// <summary>
    /// Returns a Binder for a starting entity, as follows : Tries to Find the already bound one. If none found, creates
    /// a VoidBinder and Binds it.
    /// </summary>
    public Binder FindElseBind(Finder? start)
    {
        Binder? binder = FindAndMask(start);
        if (binder is not null)
        {
            return binder;
        }
        binder = new VoidBinder();
        Bind(start, binder);
        return binder;
    }

    /// <summary>Sets Messenger used for outputting messages (null: the default messenger).</summary>
    public void SetMessenger(Messenger? messenger)
    {
        if (messenger is null)
        {
            _themessenger = global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        }
        else
        {
            _themessenger = messenger;
        }
    }

    /// <summary>Returns Messenger used for outputting messages. The returned object is guaranteed to be non-null.</summary>
    public Messenger Messenger()
    {
        return _themessenger;
    }

    /// <summary>
    /// Sets trace level used for outputting messages: 0 : no trace at all; 1 : handled exceptions and calls to
    /// AddError; 2 : also calls to AddWarning; 3 : also traces new Roots. Default is 1 : Errors traced.
    /// </summary>
    public void SetTraceLevel(int tracelev)
    {
        _thetrace = tracelev;
    }

    /// <summary>Returns trace level used for outputting messages.</summary>
    public int TraceLevel()
    {
        return _thetrace;
    }

    /// <summary>New name for AddFail (Msg).</summary>
    public void SendFail(Finder? start, Msg amsg)
    {
        AddFail(start, amsg);
    }

    /// <summary>New name for AddWarning (Msg).</summary>
    public void SendWarning(Finder? start, Msg amsg)
    {
        AddWarning(start, amsg);
    }

    /// <summary>Adds an information message. Trace is filled if trace level is at least 3.</summary>
    public void SendMsg(Finder? start, Msg amsg)
    {
        Binder? binder = FindAndMask(start);
        if (binder is null)
        {
            binder = new VoidBinder();
            Bind(start, binder);
        }
        // Feeds the trace: Causing rule (user messages)
        if (_thetrace > 0)
        {
            StartTrace(binder, start, _thelevel, 6);
            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sender = _themessenger.SendInfo();
            sender.Append(amsg.Value());
            if (amsg.IsEdited && _thetrace > 2)
            {
                sender.Append(" [from: ").Append(amsg.Original()).Append("]");
            }
            sender.Append('\n');
        }
    }

    /// <summary>Adds an Error message to a starting entity (to the check of its Binder of category 0, as a Fail).</summary>
    public void AddFail(Finder? start, string mess, string orig = "")
    {
        Binder? binder = FindAndMask(start);
        if (binder is null)
        {
            binder = new VoidBinder();
            Bind(start, binder);
        }
        binder.AddFail(mess, orig);
        if (_thetrace > 0)
        {
            StartTrace(binder, start, _thelevel, 1);
            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sender = _themessenger.SendFail();
            sender.Append("    --> Fail : ").Append(mess);
            if (orig.Length != 0 && _thetrace > 2)
            {
                sender.Append(" [from: ").Append(orig).Append("]");
            }
            sender.Append('\n');
        }
    }

    /// <summary>(other name of AddFail, maintained for compatibility)</summary>
    public void AddError(Finder? start, string mess, string orig = "")
    {
        AddFail(start, mess, orig);
    }

    /// <summary>Adds an Error Message to a starting entity from the definition of a Msg (Original+Value).</summary>
    public void AddFail(Finder? start, Msg amsg)
    {
        if (amsg.IsEdited)
        {
            AddFail(start, amsg.Value(), amsg.Original());
        }
        else
        {
            AddFail(start, amsg.Value());
        }
    }

    /// <summary>Adds a Warning message to a starting entity (to the check of its Binder of category 0).</summary>
    public void AddWarning(Finder? start, string mess, string orig = "")
    {
        Binder? binder = FindAndMask(start);
        if (binder is null)
        {
            binder = new VoidBinder();
            Bind(start, binder);
        }
        binder.AddWarning(mess, orig);
        if (_thetrace > 1)
        {
            StartTrace(binder, start, _thelevel, 2);
            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sender = _themessenger.SendWarning();
            sender.Append("    --> Warning : ").Append(mess);
            if (orig.Length != 0 && _thetrace > 2)
            {
                sender.Append(" [from: ").Append(orig).Append("]");
            }
            sender.Append('\n');
        }
    }

    /// <summary>Adds a Warning Message to a starting entity from the definition of a Msg (Original+Value).</summary>
    public void AddWarning(Finder? start, Msg amsg)
    {
        if (amsg.IsEdited)
        {
            AddWarning(start, amsg.Value(), amsg.Original());
        }
        else
        {
            AddWarning(start, amsg.Value());
        }
    }

    public void Mend(Finder? start, string pref = "")
    {
        Binder? binder = FindAndMask(start);
        if (binder is null)
        {
            return;
        }
        FrameworkCheck ach = binder.CCheck();
        ach.Mend(pref);
    }

    /// <summary>Returns the Check attached to a starting entity. If <paramref name="start"/> is unknown, returns a null Check.</summary>
    public FrameworkCheck? Check(Finder? start)
    {
        Binder? binder = Find(start);
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
    public void BindTransient(Finder? start, Transient? res)
    {
        if (res is null)
        {
            return;
        }
        Binder? former = Find(start);
        SimpleBinderOfTransient? binder = former as SimpleBinderOfTransient;
        if (binder is not null)
        {
            if (binder.Status() == StatusResult.StatusVoid)
            {
                binder.SetResult(res);
                return;
            }
        }
        binder = new SimpleBinderOfTransient();
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
    public Transient? FindTransient(Finder? start)
    {
        SimpleBinderOfTransient? binder = Find(start) as SimpleBinderOfTransient;
        if (binder is null)
        {
            return null;
        }
        if (!binder.HasResult)
        {
            return null;
        }
        return binder.Result();
    }

    /// <summary>
    /// Prepares an object <paramref name="start"/> to be bound with several results. If no Binder is yet attached to
    /// it, a MultipleBinder is created, empty. If a Binder is already set, it must accept Multiple Binding.
    /// </summary>
    /// <exception cref="TransferFailureException">The binder already set is not a MultipleBinder.</exception>
    public void BindMultiple(Finder? start)
    {
        Binder? binder = FindAndMask(start);
        if (binder is not null)
        {
            if (!binder.IsKind(typeof(MultipleBinder)))
            {
                StartTrace(_thelastbnd, start, _thelevel, 4);
                throw new TransferFailureException("TransferProcess : BindMultiple");
            }
        }
        else
        {
            Bind(start, new MultipleBinder());
        }
    }

    /// <summary>Adds an item to a list of results bound to a starting object.</summary>
    /// <exception cref="TransferFailureException">Nothing is bound, or the binder is not a MultipleBinder.</exception>
    public void AddMultiple(Finder? start, Transient? res)
    {
        Binder? binder = FindAndMask(start);
        MultipleBinder? multr = binder as MultipleBinder;
        if (multr is null)
        {
            StartTrace(binder, start, _thelevel, 4);
            if (binder is null)
            {
                throw new TransferFailureException("TransferProcess : AddMultiple, nothing bound");
            }
            else
            {
                throw new TransferFailureException("TransferProcess : AddMultiple, Binder not a MultipleBinder");
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
    public bool FindTypedTransient(Finder? start, Type? atype, ref Transient? val)
    {
        return GetTypedTransient(Find(start), atype, ref val);
    }

    /// <summary>
    /// Searches for a transient result recorded in a Binder, whatever this Binder is recorded or not in this process.
    /// Strictly equivalent to <see cref="SimpleBinderOfTransient.GetTypedResult"/>.
    /// </summary>
    public bool GetTypedTransient(Binder? binder, Type? atype, ref Transient? val)
    {
        return SimpleBinderOfTransient.GetTypedResult(binder, atype, ref val);
    }

    /// <summary>Safe variant for arbitrary type of argument (OCCT's template).</summary>
    public bool FindTypedTransient<T>(Finder? start, Type? atype, ref T? val) where T : Transient
    {
        Transient? aVal = val;
        return FindTypedTransient(start, atype, ref aVal) && (val = aVal as T) is not null;
    }

    /// <summary>Safe variant for arbitrary type of argument (OCCT's template).</summary>
    public bool GetTypedTransient<T>(Binder? start, Type? atype, ref T? val) where T : Transient
    {
        Transient? aVal = val;
        return GetTypedTransient(start, atype, ref aVal) && (val = aVal as T) is not null;
    }

    /// <summary>Returns the maximum possible value for Map Index (no result can be bound with a value greater than it).</summary>
    public int MappedCount
    {
        get
        {
            return _themap.Count;
        }
    }

    /// <summary>Returns the Starting Object bound to an Index.</summary>
    public Finder? Mapped(int num)
    {
        return _themap.KeyAt(num - 1);
    }

    /// <summary>Returns the Index value bound to a Starting Object, 0 if none.</summary>
    public int MapIndex(Finder? start)
    {
        return FindIndex(start);
    }

    /// <summary>Returns the Binder bound to an Index.</summary>
    public Binder? MapItem(int num)
    {
        Binder? binder = _themap.ValueAt(num - 1);
        return binder;
    }

    /// <summary>
    /// Declares <paramref name="start"/> (and its Result) as Root. This status will be later exploited by RootResult
    /// (Result can be produced at any time).
    /// </summary>
    public void SetRoot(Finder? start)
    {
        int index = MapIndex(start);
        if (index == 0)
        {
            return;
        }

        _theroots.Add(index);
        if (_thetrace > 2)
        {
            StartTrace(MapItem(index), start, _thelevel, 3);
        }
    }

    /// <summary>
    /// Enable (if <paramref name="stat"/> True) or Disables (if False) Root Management. If it is set, Transfers are
    /// considered as stacked (a first Transfer commands other Transfers, and so on) and the Transfers commanded by an
    /// external caller are "Root". Remark : SetRoot can be called whatever this status, on every object.
    /// </summary>
    public void SetRootManagement(bool stat)
    {
        _therootm = stat;
    }

    /// <summary>Returns the count of recorded Roots.</summary>
    public int RootCount
    {
        get
        {
            return _theroots.Count;
        }
    }

    /// <summary>Returns a Root Entity given its number in the list (1-NbRoots).</summary>
    /// <exception cref="OutOfRangeException">No root has the number.</exception>
    public Finder? Root(int num)
    {
        int ind = 0;
        if (num > 0 && num <= _theroots.Count)
        {
            ind = _theroots[num - 1];
        }
        return _themap.KeyAt(ind - 1);
    }

    /// <summary>Returns the Binder bound with a Root Entity given its number.</summary>
    /// <exception cref="OutOfRangeException">No root has the number.</exception>
    public Binder? RootItem(int num)
    {
        int ind = 0;
        if (num > 0 && num <= _theroots.Count)
        {
            ind = _theroots[num - 1];
        }
        return _themap.ValueAt(ind - 1);
    }

    /// <summary>Returns the index in the list of roots for a starting item, or 0 if it is not recorded as a root.</summary>
    public int RootIndex(Finder? start)
    {
        int index = MapIndex(start);
        if (index == 0)
        {
            return 0;
        }
        return _theroots.IndexOf(index) + 1;
    }

    /// <summary>
    /// Returns Nesting Level of Transfers (managed by methods TranscriptWith &amp; Co). Starts to zero. If no automatic
    /// Transfer is used, it remains to zero. Zero means Root Level.
    /// </summary>
    public int NestingLevel()
    {
        return _thelevel;
    }

    /// <summary>Resets Nesting Level of Transfers to Zero (Root Level), whatever its current value.</summary>
    public void ResetNestingLevel()
    {
        _thelevel = 0;
    }

    /// <summary>Tells if <paramref name="start"/> has been recognized as good candidate for Transfer, i.e. queries the Actor and its Nexts.</summary>
    public bool Recognize(Finder? start)
    {
        ActorOfProcessForFinder? actor = _theactor;
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
    /// <exception cref="TransferFailureException">The object's transfer is in error status.</exception>
    public Binder? Transferring(Finder? start, ProgressRange progress = default)
    {
        Binder? former = FindAndMask(start);
        // We consider that this new Transfer request therefore corresponds to a
        // additional use: note "AlreadyUsed", therefore non-modifiable result
        bool hasDeadLoop = false;
        if (former is not null)
        {
            if (former.HasResult)
            {
                former.SetAlreadyUsed();
                return former;
            }

            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sender = _themessenger.SendInfo();
            StatusExec statex = former.StatusExec();
            switch (statex)
            {
                case StatusExec.StatusInitial:
                    break;
                case StatusExec.StatusDone:
                    sender.Append(" .. and Transfer done").Append('\n');
                    return former;
                case StatusExec.StatusRun:
                    former.SetStatusExec(StatusExec.StatusLoop);
                    return former;
                case StatusExec.StatusError:
                    if (_thetrace != 0)
                    {
                        sender.Append("                  *** Transfer in Error Status  :").Append('\n');
                        StartTrace(former, start, _thelevel, 0);
                    }
                    else
                    {
                        StartTrace(former, start, _thelevel, 4);
                    }
                    throw new TransferFailureException("TransferProcess : Transfer in Error Status");
                case StatusExec.StatusLoop:
                    if (_thetrace != 0)
                    {
                        sender.Append("                  *** Transfer  Head of Dead Loop  :").Append('\n');
                        StartTrace(former, start, _thelevel, 0);
                    }
                    else
                    {
                        StartTrace(former, start, _thelevel, 4);
                    }
                    hasDeadLoop = true;
                    break;
            }
            if (!hasDeadLoop)
            {
                former.SetStatusExec(StatusExec.StatusRun);
            }
        }

        Binder? binder = null;
        bool newbind = false;

        // Handle dead loop condition detected before calling TransferProduct
        // When hasDeadLoop is true, we know:
        // 1. former is NOT null (hasDeadLoop only set inside !former.IsNull() block)
        // 2. former->StatusExec() == Transfer_StatusLoop (that's what triggered hasDeadLoop)
        if (hasDeadLoop)
        {
            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sender = _themessenger.SendInfo();
            if (_thetrace != 0)
            {
                sender.Append("                  *** Dead Loop : Finding head of Loop :").Append('\n');
                StartTrace(former, start, _thelevel - 1, 0);
            }
            else
            {
                StartTrace(former, start, _thelevel - 1, 4);
            }
            binder = former;
            binder.AddFail("Transfer: Head of Dead Loop");
            binder.AddFail("Transfer in dead Loop");
        }
        else if (_theerrh)
        {
            using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sender = _themessenger.SendInfo();
            // Transfer under protection for exceptions (for notification in fact)
            int oldlev = _thelevel;
            try
            {
                binder = TransferProduct(start, progress);
            }
            catch (Exception exception) when (ModelAlgo.IsCaughtFailure(exception))
            {
                // OCC_CATCH_SIGNALS: a Standard_Failure, or a system signal (their .NET counterparts, decision 1183)
                if (binder is null)
                {
                    sender.Append("                  *** Exception Raised with no Result").Append('\n');
                    binder = new VoidBinder();
                    Bind(start, binder);
                    newbind = true;
                }
                binder.AddFail("Transfer stopped by exception raising");
                if (_thetrace != 0)
                {
                    sender.Append("    *** Raised : ").Append(What(exception)).Append('\n');
                    StartTrace(binder, start, _thelevel - 1, 4);
                }
                _thelevel = oldlev;
            }
        }

        // Unprotected transfer (thus, dbx by hand in case of crash by Raise)
        else
        {
            binder = TransferProduct(start, progress);
        }

        if (progress.UserBreak())
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
                former.SetStatusExec(StatusExec.StatusDone); //+
            }
            return null; // Null Binder ... what else to do?
        }

        //  Manage Root
        if (_therootl >= _thelevel) // OCCT: nested transfers ending at the level of the first one done are roots too
        {
            _therootl = 0;
            if (_therootm && binder.Status() != StatusResult.StatusVoid)
            {
                SetRoot(start);
            }
        }
        return _thelastbnd;
    }

    // OCCT's Standard_Failure::what(): the message, empty when there is none; a NullReferenceException is natively an
    // access violation, with Windows' text for a read at address 0 (decision 977)
    internal static string What(Exception exception)
    {
        exception = Refusal.Unwrapped(exception);
        if (exception is NullReferenceException)
        {
            return "ACCESS VIOLATION at address 0x0000000000000000 during 'READ' operation";
        }
        string message = exception.Message;
        return message == $"Exception of type '{exception.GetType().FullName}' was thrown." ? "" : message;
    }

    // Internal action of Transfer, called by Transferring, with or without ErrorHandle. It invokes the Actor to work
    // (set by SetActor), and tries its Nexts if no result is produced, until a Non Null Binder is produced. But keep in
    // mind that a Null Binder can always be returned if a Starting Entity has not been recognized at all.
    private Binder? TransferProduct(Finder? start, ProgressRange progress)
    {
        _thelevel++; // decrement and if == 0, root transfer
        Binder? binder = null;
        ActorOfProcessForFinder? actor = _theactor;

        // We scan the Next until we have a Result
        using var scope = new ProgressScope(progress, null, 1, true);
        while (actor is not null)
        {
            if (actor.Recognize(start))
            {
                binder = actor.Transferring(start, this, scope.Next());
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
        if (scope.UserBreak())
        {
            return null;
        }

        if (binder is null)
        {
            if (_thelevel > 0)
            {
                _thelevel--;
            }
            return binder;
        }
        if (_therootl == 0 && binder.StatusExec() == StatusExec.StatusDone)
        {
            _therootl = _thelevel - 1;
        }

        if (_thelevel > 0)
        {
            _thelevel--;
        }
        return binder;
    }

    /// <summary>Same as Transferring but does not return the Binder. Simply returns True in case of success (for user call).</summary>
    public bool Transfer(Finder? start, ProgressRange progress = default)
    {
        Binder? binder = Transferring(start, progress);
        return binder is not null;
    }

    /// <summary>
    /// Allows controls if exceptions will be handled Transfer Operations. False : they are not handled with
    /// try {} catch {}; True : they are.
    /// </summary>
    public void SetErrorHandle(bool err)
    {
        _theerrh = err;
    }

    /// <summary>Returns error handling flag.</summary>
    public bool ErrorHandle()
    {
        return _theerrh;
    }

    /// <summary>
    /// Method called when trace is asked. Calls PrintTrace to display information relevant for starting objects
    /// (which can be redefined). <paramref name="level"/> is Nesting Level of Transfer (0 = root).
    /// <paramref name="mode"/> controls the way the trace is done : 0 neutral, 1 for Error, 2 for Warning message, 3
    /// for new Root.
    /// </summary>
    public void StartTrace(Binder? binder, Finder? start, int level, int mode)
    {
        using global::Kerf.Engine.Foundation.Messenger.StreamBuffer sender = _themessenger.SendInfo();
        // ###  Fail (Roots:50)  --  Start start->DynamicType()
        // ###  Fail (Roots:50)  --  Start id:#label.. Type:start->DynamicType()
        if (_thetrace > 3)
        { // Internal to be switch when searching bug (trace >= 4)
            if (mode == 1)
            {
                sender.Append("  ###  Fail");
            }
            if (mode == 2)
            {
                sender.Append("  ###  Warning");
            }
            if (mode == 3)
            {
                sender.Append("  ###  New Root n0 ").Append(_theroots.Count);
            }
            if (mode == 4)
            {
                sender.Append("  ###  Exception");
            }
            if (mode == 5)
            {
                sender.Append("  ###  Substitution");
            }
            if (mode == 6)
            {
                sender.Append("  ###  Information");
            }
            if (level > 1)
            {
                sender.Append(" (nested)"); // " at nesting Level:"<<level;
            }
            if (mode >= 0 && mode != 3)
            {
                sender.Append(" at ").Append(_theroots.Count).Append(" Roots");
            }
        }
        if (start is not null)
        {
            using var stream = new StringWriter(sender.Stream());
            PrintTrace(start, stream);
        }
        if (binder is not null)
        { // old: if IsNull sout <<endl<< "  ---  Not Bound";
            Binder? bnd = binder;
            bool hasres = false;
            while (bnd is not null)
            {
                if (bnd.Status() != StatusResult.StatusVoid)
                {
                    // ---  Result Type: binder->ResultType()  ---  Binder : binder->DynamicType();
                    if (!hasres)
                    {
                        sender.Append("\n  ---  Result Type : ");
                    }
                    else
                    {
                        sender.Append(" , ");
                    }
                    sender.Append(bnd.ResultTypeName());
                    // CKY 9-JAN-1999:  waiting for XSTEP Kernel message (not IGES_2075)
                    hasres = true;
                }
                bnd = bnd.NextResult();
            }
            if (!hasres && mode > 2)
            {
                sender.Append("\n  ---  No Result recorded");
                // CKY 9-JAN-1999 : waiting for XSTEP Kernel message
                //     (not IGES_2075)
            }
            // old: if (isused) sout << "  --    (Already Used in another Transfer)";
        }
        sender.Append('\n');
    }

    /// <summary>Prints a short information on a starting object. By default prints its Dynamic Type. Can be redefined.</summary>
    public virtual void PrintTrace(Finder? start, TextWriter s)
    {
        if (start is not null)
        {
            s.Write(" Type:");
            s.Write(start.DynamicTypeName());
        }
    }

    /// <summary>
    /// Returns True if we are surely in a DeadLoop. Evaluation is not exact, it is a "majorant" which must be computed
    /// fast. This "majorant" is : <paramref name="alevel"/> greater than NbMapped.
    /// </summary>
    public bool IsLooping(int alevel)
    {
        return alevel > MappedCount;
    }

    //  ########################################################################
    //  ....                            RESULTS                            ....

    /// <summary>
    /// Returns, as an iterator, the log of root transfer, i.e. the created objects and Binders bound to starting
    /// roots. If withstart is given True, Starting Objects are also returned.
    /// </summary>
    public IteratorOfProcessForFinder RootResult(bool withstart = false)
    {
        var iter = new IteratorOfProcessForFinder(withstart);
        int max = _theroots.Count;
        for (int j = 0; j < max; j++)
        {
            int i = _theroots[j];
            Binder? binder = MapItem(i);
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
    public IteratorOfProcessForFinder CompleteResult(bool withstart = false)
    {
        var iter = new IteratorOfProcessForFinder(withstart);
        int max = MappedCount;
        for (int i = 1; i <= max; i++)
        {
            Binder? binder = MapItem(i);
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
    public IteratorOfProcessForFinder AbnormalResult()
    {
        var iter = new IteratorOfProcessForFinder(true);
        int max = MappedCount;
        for (int i = 1; i <= max; i++)
        {
            Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            StatusExec statex = binder.StatusExec();
            if (statex != StatusExec.StatusInitial && statex != StatusExec.StatusDone)
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
    public CheckIterator CheckList(bool erronly)
    {
        var list = new CheckIterator();
        int num, max = MappedCount;
        for (int i = 1; i <= max; i++)
        {
            Binder? binder = MapItem(i);
            if (binder is null)
            {
                continue;
            }
            StatusExec statex = binder.StatusExec();
            FrameworkCheck check = binder.Check();
            if (statex != StatusExec.StatusInitial && statex != StatusExec.StatusDone && !check.HasFailed)
            {
                check.AddFail("Transfer in Abnormal Status (!= Initial or Done)");
            }
            if (!check.HasFailed && (erronly || check.WarningCount == 0))
            {
                continue;
            }
            Finder? ent = Mapped(i);
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
    public IteratorOfProcessForFinder ResultOne(Finder? start, int level, bool withstart = false)
    {
        var iter = new IteratorOfProcessForFinder(withstart);
        int max = MappedCount;
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
            Binder? binder = MapItem(i);
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
    public CheckIterator CheckListOne(Finder? start, int level, bool erronly)
    {
        var list = new CheckIterator();
        int max = MappedCount;
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
            Binder? binder = MapItem(ind);
            if (binder is null)
            {
                continue;
            }

            StatusExec statex = binder.StatusExec();
            FrameworkCheck check = binder.Check();
            if (statex != StatusExec.StatusInitial && statex != StatusExec.StatusDone && !check.HasFailed)
            {
                check.AddFail("Transfer in Abnormal Status (!= Initial or Done)");
            }
            if (!check.HasFailed && (erronly || check.WarningCount == 0))
            {
                continue;
            }
            Finder? ent = Mapped(ind);
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
    public bool IsCheckListEmpty(Finder? start, int level, bool erronly)
    {
        int max = MappedCount;
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
            Binder? binder = MapItem(ind);
            if (binder is null)
            {
                continue;
            }

            StatusExec statex = binder.StatusExec();
            FrameworkCheck check = binder.Check();
            if (statex != StatusExec.StatusInitial && statex != StatusExec.StatusDone)
            {
                return false;
            }
            if (check.HasFailed || (!erronly && check.WarningCount > 0))
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
    public void RemoveResult(Finder? start, int level, bool compute = true)
    {
        // if (compute) ComputeScopes();
        int max = MappedCount;
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
            Binder? binder = MapItem(ind);
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
    public virtual int CheckNum(Finder? start)
    {
        return 0;
    }

    // The index of a starting object, 0 if none (a null object is never mapped)
    private int FindIndex(Finder? start)
    {
        return start is null ? 0 : (_themap.IndexOf(start) + 1);
    }

    private bool _theerrh;
    private int _thetrace;
    private Messenger _themessenger;
    private int _thelevel;
    private int _therootl;
    private bool _therootm;
    private IndexedSet<int> _theroots = new();
    private Finder? _thelastobj;
    private Binder? _thelastbnd;
    private int _theindex;
    private ActorOfProcessForFinder? _theactor;
    private IndexedMap<Finder, Binder?> _themap;
}
