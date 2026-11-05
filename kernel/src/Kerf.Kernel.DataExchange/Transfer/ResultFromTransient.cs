// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ResultFromTransient.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ResultFromTransient.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Framework.FrameworkCheckStatus;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// This class, in conjunction with ResultFromModel, allows to record the result of a transfer initially stored in a
/// TransientProcess. A ResultFromTransient records a couple (Transient,Binder for the result and checks) plus a list of
/// "sub-results", which have been recorded in the TransientProcess, under scope attached to the starting transient.
/// </summary>
internal class ResultFromTransient : Transient
{
    /// <summary>Creates a ResultFromTransient, empty.</summary>
    public ResultFromTransient()
    {
    }

    /// <summary>Sets starting entity.</summary>
    public void SetStart(Transient? start)
    {
        _thestart = start;
    }

    /// <summary>Sets Binder (for result plus individual check).</summary>
    public void SetBinder(Binder? binder)
    {
        _thebinder = binder;
    }

    /// <summary>Returns the starting entity.</summary>
    public Transient? Start()
    {
        return _thestart;
    }

    /// <summary>Returns the binder.</summary>
    public Binder? Binder()
    {
        return _thebinder;
    }

    /// <summary>Returns True if a result is recorded.</summary>
    public bool HasResult
    {
        get
        {
            return _thebinder is null ? false : _thebinder.HasResult;
        }
    }

    /// <summary>Returns the check (or an empty one if no binder).</summary>
    /// <remarks>OCCT returns a process-wide empty check, which a caller could fill; the port a new one (decision 1183).</remarks>
    public FrameworkCheck Check()
    {
        if (_thebinder is null)
        {
            return new FrameworkCheck();
        }
        return _thebinder.Check();
    }

    /// <summary>Returns the check status.</summary>
    public FrameworkCheckStatus CheckStatus()
    {
        if (_thebinder is null)
        {
            return FrameworkCheckStatus.CheckOK;
        }
        FrameworkCheck ach = _thebinder.Check();
        return ach.Status();
    }

    /// <summary>Returns the ResultFromTransient attached to a given starting entity (the key). Returns a null handle if not found.</summary>
    public ResultFromTransient? ResultFromKey(Transient? key)
    {
        ResultFromTransient? res = null;
        if (key == _thestart)
        {
            return this;
        }
        int i, nb = SubResultCount;
        for (i = 1; i <= nb; i++)
        {
            res = SubResult(i).ResultFromKey(key);
            if (res is not null)
            {
                return res;
            }
        }
        return res;
    }

    /// <summary>
    /// This method is used by ResultFromModel to collate the list of ResultFromTransient, avoiding duplications with a
    /// map. Remark : this object is already in the map and has not to be bound.
    /// </summary>
    public void FillMap(IndexedSet<Transient> map)
    {
        if (_thesubs is null)
        {
            return;
        }
        int i, nb = _thesubs.Length();
        for (i = 1; i <= nb; i++)
        {
            map.Add(_thesubs.Value(i));
        }
        for (i = 1; i <= nb; i++)
        {
            SubResult(i).FillMap(map);
        }
    }

    //  #####   SUBS   #####

    /// <summary>Clears the list of (immediate) sub-results.</summary>
    public void ClearSubs()
    {
        _thesubs = null;
    }

    /// <summary>Adds a sub-result.</summary>
    public void AddSubResult(ResultFromTransient? sub)
    {
        if (sub is null)
        {
            return;
        }
        if (_thesubs is null)
        {
            _thesubs = new NCollection_HSequence<Transient>();
        }
        _thesubs.Append(sub);
    }

    /// <summary>Returns the count of recorded sub-results.</summary>
    public int SubResultCount
    {
        get
        {
            return _thesubs is null ? 0 : _thesubs.Length();
        }
    }

    /// <summary>Returns a sub-result, given its rank.</summary>
    public ResultFromTransient? SubResult(int num)
    {
        ResultFromTransient? sub = null;
        if (_thesubs is null)
        {
            return sub;
        }
        if (num < 1 || num > _thesubs.Length())
        {
            return sub;
        }
        return _thesubs.Value(num) as ResultFromTransient;
    }

    /// <summary>
    /// Fills from a TransientProcess, with the starting entity which must have been set before. It works with scopes,
    /// calls Fill on each of its sub-results.
    /// </summary>
    public void Fill(TransientProcess? tp)
    {
        // abv: WARNING: to be removed (scopes)
    }

    /// <summary>
    /// Clears some data attached to binders used by TransientProcess, which become useless once the transfer has been
    /// done : the list of sub-scoped binders, which is now recorded as sub-results.
    /// </summary>
    public void Strip()
    {
        // abv: WARNING: to be removed (scopes)
    }

    /// <summary>Fills back a TransientProcess with definition of a ResultFromTransient, respectfully to its structuration in scopes.</summary>
    public void FillBack(TransientProcess? tp)
    {
        // abv: WARNING: to be removed (scopes)
    }

    private Transient? _thestart;
    private Binder? _thebinder;
    private NCollection_HSequence<Transient>? _thesubs;
}
