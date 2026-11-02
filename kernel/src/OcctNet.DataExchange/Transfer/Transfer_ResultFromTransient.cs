// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ResultFromTransient.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ResultFromTransient.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.Interface_CheckStatus;

namespace Occt;

/// <summary>
/// This class, in conjunction with ResultFromModel, allows to record the result of a transfer initially stored in a
/// TransientProcess. A ResultFromTransient records a couple (Transient,Binder for the result and checks) plus a list of
/// "sub-results", which have been recorded in the TransientProcess, under scope attached to the starting transient.
/// </summary>
public class Transfer_ResultFromTransient : Standard_Transient
{
    /// <summary>Creates a ResultFromTransient, empty.</summary>
    public Transfer_ResultFromTransient()
    {
    }

    /// <summary>Sets starting entity.</summary>
    public void SetStart(Standard_Transient? start)
    {
        thestart = start;
    }

    /// <summary>Sets Binder (for result plus individual check).</summary>
    public void SetBinder(Transfer_Binder? binder)
    {
        thebinder = binder;
    }

    /// <summary>Returns the starting entity.</summary>
    public Standard_Transient? Start()
    {
        return thestart;
    }

    /// <summary>Returns the binder.</summary>
    public Transfer_Binder? Binder()
    {
        return thebinder;
    }

    /// <summary>Returns True if a result is recorded.</summary>
    public bool HasResult()
    {
        return thebinder is null ? false : thebinder.HasResult();
    }

    /// <summary>Returns the check (or an empty one if no binder).</summary>
    /// <remarks>OCCT returns a process-wide empty check, which a caller could fill; the port a new one (decision 1183).</remarks>
    public Interface_Check Check()
    {
        if (thebinder is null)
        {
            return new Interface_Check();
        }
        return thebinder.Check();
    }

    /// <summary>Returns the check status.</summary>
    public Interface_CheckStatus CheckStatus()
    {
        if (thebinder is null)
        {
            return Interface_CheckOK;
        }
        Interface_Check ach = thebinder.Check();
        return ach.Status();
    }

    /// <summary>Returns the ResultFromTransient attached to a given starting entity (the key). Returns a null handle if not found.</summary>
    public Transfer_ResultFromTransient? ResultFromKey(Standard_Transient? key)
    {
        Transfer_ResultFromTransient? res = null;
        if (key == thestart)
        {
            return this;
        }
        int i, nb = NbSubResults();
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
    public void FillMap(NCollection_IndexedMap<Standard_Transient> map)
    {
        if (thesubs is null)
        {
            return;
        }
        int i, nb = thesubs.Length();
        for (i = 1; i <= nb; i++)
        {
            map.Add(thesubs.Value(i));
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
        thesubs = null;
    }

    /// <summary>Adds a sub-result.</summary>
    public void AddSubResult(Transfer_ResultFromTransient? sub)
    {
        if (sub is null)
        {
            return;
        }
        if (thesubs is null)
        {
            thesubs = new NCollection_HSequence<Standard_Transient>();
        }
        thesubs.Append(sub);
    }

    /// <summary>Returns the count of recorded sub-results.</summary>
    public int NbSubResults()
    {
        return thesubs is null ? 0 : thesubs.Length();
    }

    /// <summary>Returns a sub-result, given its rank.</summary>
    public Transfer_ResultFromTransient? SubResult(int num)
    {
        Transfer_ResultFromTransient? sub = null;
        if (thesubs is null)
        {
            return sub;
        }
        if (num < 1 || num > thesubs.Length())
        {
            return sub;
        }
        return thesubs.Value(num) as Transfer_ResultFromTransient;
    }

    /// <summary>
    /// Fills from a TransientProcess, with the starting entity which must have been set before. It works with scopes,
    /// calls Fill on each of its sub-results.
    /// </summary>
    public void Fill(Transfer_TransientProcess? TP)
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
    public void FillBack(Transfer_TransientProcess? TP)
    {
        // abv: WARNING: to be removed (scopes)
    }

    private Standard_Transient? thestart;
    private Transfer_Binder? thebinder;
    private NCollection_HSequence<Standard_Transient>? thesubs;
}
