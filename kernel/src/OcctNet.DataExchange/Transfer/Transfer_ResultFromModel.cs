// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ResultFromModel.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ResultFromModel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.Interface_CheckStatus;

namespace Occt;

/// <summary>
/// ResultFromModel is used to store a final result stored in a TransientProcess, respectfully to its structuration in
/// scopes by using a set of ResultFromTransient. Hence, it can be regarded as a passive equivalent of the stored data
/// in the TransientProcess, while an Iterator gives a flat view of it. A ResultFromModel is intended to be attached to
/// the transfer of one entity (typically root entity but it is not mandatory).
/// </summary>
public class Transfer_ResultFromModel : Transient
{
    /// <summary>Creates a ResultFromModel, empty.</summary>
    public Transfer_ResultFromModel()
    {
        themnum = 0;
        themchk = Interface_CheckStatus.Interface_CheckAny;
    }

    /// <summary>Sets starting Model.</summary>
    public void SetModel(Interface_InterfaceModel? model)
    {
        themodel = model;
    }

    /// <summary>Sets starting File Name.</summary>
    public void SetFileName(string filename)
    {
        thename = filename;
    }

    /// <summary>Returns starting Model (null if not set).</summary>
    public Interface_InterfaceModel? Model()
    {
        return themodel;
    }

    /// <summary>Returns starting File Name (empty if not set).</summary>
    public string FileName()
    {
        return thename;
    }

    /// <summary>
    /// Fills from a TransientProcess, with the result attached to a starting entity. Considers its Model if it is set.
    /// This action produces a structured set of ResultFromTransient, considering scopes, starting by that of
    /// <paramref name="ent"/>. If <paramref name="ent"/> has no recorded result, it remains empty. Returns True if a
    /// result is recorded, False else.
    /// </summary>
    public bool Fill(Transfer_TransientProcess? TP, Transient? ent)
    {
        if (TP is null || ent is null)
        {
            return false;
        }
        Transfer_Binder? binder = TP.Find(ent);
        if (binder is null)
        {
            return false;
        }
        themain = new Transfer_ResultFromTransient();
        themain.SetStart(ent);
        themain.SetBinder(binder);
        themain.Fill(TP);
        //   Result substitution for the Shape (-> HShape): not here, we are
        //   in the Transfer package which is general and doesn't know what a Shape is ...
        if (TP.Model() is not null)
        {
            themodel = TP.Model();
        }
        if (themodel is null)
        {
            return true;
        }
        themnum = themodel.Number(ent);
        themlab = "";
        if (themnum > 0)
        {
            themlab += themodel.StringLabel(ent).ToCString();
        }
        return true;
    }

    /// <summary>
    /// Clears some data attached to binders used by TransientProcess, which become useless once the transfer has been
    /// done, by calling Strip on its ResultFromTransient. mode = 0 : minimum, clears data remaining from
    /// TransferProcess; mode = 10 : just keeps file name, label, check status ..., and MainResult but only the result
    /// (Binder); mode = 11 : also clears MainResult (status and names remain).
    /// </summary>
    public void Strip(int mode)
    {
        if (themain is null)
        {
            return;
        }
        themain.Strip();
        if (mode >= 10)
        {
            themchk = ComputeCheckStatus(false);
            themodel = null;
            themain.ClearSubs();
            Transient? nulh = null;
            themain.SetStart(nulh);
            if (mode > 10)
            {
                themain = null;
            }
        }
    }

    /// <summary>Fills back a TransientProcess from the structured set of binders. Also sets the Model.</summary>
    public void FillBack(Transfer_TransientProcess TP)
    {
        if (themodel is not null)
        {
            TP.SetModel(themodel);
        }
        themain.FillBack(TP);
    }

    /// <summary>Returns True if a Result is recorded.</summary>
    public bool HasResult()
    {
        return themain is null ? false : themain.HasResult();
    }

    /// <summary>Returns the main recorded ResultFromTransient, or a null.</summary>
    public Transfer_ResultFromTransient? MainResult()
    {
        return themain;
    }

    /// <summary>Sets a new value for the main recorded ResultFromTransient.</summary>
    public void SetMainResult(Transfer_ResultFromTransient? amain)
    {
        themchk = Interface_CheckStatus.Interface_CheckAny;
        themain = amain;
        if (themodel is null || themain is null)
        {
            return;
        }
        themnum = themodel.Number(themain.Start());
        themlab = "";
        if (themnum > 0)
        {
            themlab += themodel.StringLabel(themain.Start()).ToCString();
        }
    }

    /// <summary>Returns the label in starting model attached to main entity (updated by Fill or SetMainResult, if Model is known).</summary>
    public string MainLabel()
    {
        return themlab;
    }

    /// <summary>Returns the label in starting model attached to main entity.</summary>
    public int MainNumber()
    {
        return themnum;
    }

    //  ############  INFORMATIONS  GLOBALES  ###########

    /// <summary>Searches for a key (starting entity) and returns its result. Returns a null handle if not found.</summary>
    public Transfer_ResultFromTransient? ResultFromKey(Transient? start)
    {
        return themain.ResultFromKey(start);
    }

    /// <summary>
    /// Internal method which returns the list of ResultFromTransient, according level (2:complete; 1:sub-level 1;
    /// 0:main only).
    /// </summary>
    public NCollection_HSequence<Transient> Results(int level)
    {
        int i, nb;
        var list = new NCollection_HSequence<Transient>();
        if (level > 1)
        {
            var map = new NCollection_IndexedMap<Transient>(ReferenceEqualityComparer.Instance);
            map.Add(themain);
            themain.FillMap(map);

            nb = map.Extent();
            for (i = 1; i <= nb; i++)
            {
                list.Append(map.FindKey(i));
            }
        }
        else
        {
            list.Append(themain);
        }
        if (level == 1)
        {
            nb = themain.NbSubResults();
            for (i = 1; i <= nb; i++)
            {
                list.Append(themain.SubResult(i));
            }
            list.Append(themain); // OCCT: the main result twice at level 1 (first and last)
        }
        return list;
    }

    /// <summary>
    /// Returns the list of recorded starting entities, ending by the root. Entities with check but no transfer result
    /// are ignored. <paramref name="level"/> = 2 (D), considers the complete list; 1 considers the main result plus
    /// immediate subs; 0 just the main result.
    /// </summary>
    public NCollection_HSequence<Transient> TransferredList(int level = 2)
    {
        int i, nb;
        var list = new NCollection_HSequence<Transient>();
        NCollection_HSequence<Transient> res = Results(level);
        nb = res.Length();
        for (i = 1; i <= nb; i++)
        {
            Transfer_ResultFromTransient? unres = res.Value(i) as Transfer_ResultFromTransient;
            if (unres is null)
            {
                continue;
            }
            if (unres.HasResult())
            {
                list.Append(unres.Start());
            }
        }
        return list;
    }

    /// <summary>
    /// Returns the list of starting entities to which a check status is attached. <paramref name="check"/> = -2 all
    /// entities whatever the check (see result); -1 entities with no fail (warning allowed); 0 entities with no check
    /// at all; 1 entities with warning but no fail; 2 entities with fail. <paramref name="result"/> : if True, only
    /// entities with an attached result.
    /// </summary>
    public NCollection_HSequence<Transient> CheckedList(Interface_CheckStatus check, bool result)
    {
        int i, nb;
        var list = new NCollection_HSequence<Transient>();
        NCollection_HSequence<Transient> res = Results(2);
        nb = res.Length();
        for (i = 1; i <= nb; i++)
        {
            Transfer_ResultFromTransient? unres = res.Value(i) as Transfer_ResultFromTransient;
            if (unres is null)
            {
                continue;
            }
            if (result && !unres.HasResult())
            {
                continue;
            }
            Interface_Check ach = unres.Check();
            if (ach.Complies(check))
            {
                list.Append(unres.Start());
            }
        }
        return list;
    }

    /// <summary>
    /// Returns the check-list of this set of results. <paramref name="erronly"/> true : only fails are considered.
    /// <paramref name="level"/> = 0 : considers only main binder; 1 : considers main binder plus immediate subs; 2 (D)
    /// : considers all checks.
    /// </summary>
    public Interface_CheckIterator CheckList(bool erronly, int level = 2)
    {
        var chl = new Interface_CheckIterator();
        int i, nb;
        NCollection_HSequence<Transient> res = Results(level);
        nb = res.Length();
        for (i = 1; i <= nb; i++)
        {
            Transfer_ResultFromTransient? unres = res.Value(i) as Transfer_ResultFromTransient;
            if (unres is null)
            {
                continue;
            }
            Interface_CheckStatus stat = unres.CheckStatus();
            if (stat == Interface_CheckStatus.Interface_CheckOK || (stat == Interface_CheckStatus.Interface_CheckWarning && erronly))
            {
                continue;
            }
            Transfer_Binder binder = unres.Binder();
            Interface_Check bch = binder.Check();
            bch.SetEntity(unres.Start());
            chl.Add(bch, themodel is null ? 0 : themodel.Number(unres.Start()));
        }
        return chl;
    }

    /// <summary>
    /// Returns the check status with corresponds to the content of this ResultFromModel; considers all levels of
    /// transfer (worst status). Reads it from recorded status if already computed, else recomputes one.
    /// </summary>
    public Interface_CheckStatus CheckStatus()
    {
        if (themchk != Interface_CheckStatus.Interface_CheckAny)
        {
            return themchk;
        }
        Interface_CheckIterator chl = CheckList(false, 2);
        return chl.Status();
    }

    /// <summary>Computes and records check status (see CheckStatus). Does not computes it if already done and <paramref name="enforce"/> False.</summary>
    public Interface_CheckStatus ComputeCheckStatus(bool enforce)
    {
        if (themchk == Interface_CheckStatus.Interface_CheckAny || enforce)
        {
            themchk = CheckStatus();
        }
        return themchk;
    }

    private Interface_InterfaceModel? themodel;
    private string thename = "";
    private Transfer_ResultFromTransient? themain;
    private string themlab = "";
    private int themnum;
    private Interface_CheckStatus themchk;
}
