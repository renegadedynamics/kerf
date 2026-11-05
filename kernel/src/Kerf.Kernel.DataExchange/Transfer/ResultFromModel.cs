// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ResultFromModel.hxx
// Port of: src/DataExchange/TKXSBase/Transfer/Transfer_ResultFromModel.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Occt;
using static Kerf.Engine.Exchange.Framework.FrameworkCheckStatus;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// ResultFromModel is used to store a final result stored in a TransientProcess, respectfully to its structuration in
/// scopes by using a set of ResultFromTransient. Hence, it can be regarded as a passive equivalent of the stored data
/// in the TransientProcess, while an Iterator gives a flat view of it. A ResultFromModel is intended to be attached to
/// the transfer of one entity (typically root entity but it is not mandatory).
/// </summary>
internal class ResultFromModel : Transient
{
    /// <summary>Creates a ResultFromModel, empty.</summary>
    public ResultFromModel()
    {
        _themnum = 0;
        _themchk = FrameworkCheckStatus.CheckAny;
    }

    /// <summary>Sets starting Model.</summary>
    public void SetModel(InterfaceModel? model)
    {
        _themodel = model;
    }

    /// <summary>Sets starting File Name.</summary>
    public void SetFileName(string filename)
    {
        _thename = filename;
    }

    /// <summary>Returns starting Model (null if not set).</summary>
    public InterfaceModel? Model()
    {
        return _themodel;
    }

    /// <summary>Returns starting File Name (empty if not set).</summary>
    public string FileName()
    {
        return _thename;
    }

    /// <summary>
    /// Fills from a TransientProcess, with the result attached to a starting entity. Considers its Model if it is set.
    /// This action produces a structured set of ResultFromTransient, considering scopes, starting by that of
    /// <paramref name="ent"/>. If <paramref name="ent"/> has no recorded result, it remains empty. Returns True if a
    /// result is recorded, False else.
    /// </summary>
    public bool Fill(TransientProcess? tp, Transient? ent)
    {
        if (tp is null || ent is null)
        {
            return false;
        }
        Binder? binder = tp.Find(ent);
        if (binder is null)
        {
            return false;
        }
        _themain = new ResultFromTransient();
        _themain.SetStart(ent);
        _themain.SetBinder(binder);
        _themain.Fill(tp);
        //   Result substitution for the Shape (-> HShape): not here, we are
        //   in the Transfer package which is general and doesn't know what a Shape is ...
        if (tp.Model() is not null)
        {
            _themodel = tp.Model();
        }
        if (_themodel is null)
        {
            return true;
        }
        _themnum = _themodel.Number(ent);
        _themlab = "";
        if (_themnum > 0)
        {
            _themlab += _themodel.StringLabel(ent).ToCString();
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
        if (_themain is null)
        {
            return;
        }
        _themain.Strip();
        if (mode >= 10)
        {
            _themchk = ComputeCheckStatus(false);
            _themodel = null;
            _themain.ClearSubs();
            Transient? nulh = null;
            _themain.SetStart(nulh);
            if (mode > 10)
            {
                _themain = null;
            }
        }
    }

    /// <summary>Fills back a TransientProcess from the structured set of binders. Also sets the Model.</summary>
    public void FillBack(TransientProcess tp)
    {
        if (_themodel is not null)
        {
            tp.SetModel(_themodel);
        }
        _themain.FillBack(tp);
    }

    /// <summary>Returns True if a Result is recorded.</summary>
    public bool HasResult
    {
        get
        {
            return _themain is null ? false : _themain.HasResult;
        }
    }

    /// <summary>Returns the main recorded ResultFromTransient, or a null.</summary>
    public ResultFromTransient? MainResult()
    {
        return _themain;
    }

    /// <summary>Sets a new value for the main recorded ResultFromTransient.</summary>
    public void SetMainResult(ResultFromTransient? amain)
    {
        _themchk = FrameworkCheckStatus.CheckAny;
        _themain = amain;
        if (_themodel is null || _themain is null)
        {
            return;
        }
        _themnum = _themodel.Number(_themain.Start());
        _themlab = "";
        if (_themnum > 0)
        {
            _themlab += _themodel.StringLabel(_themain.Start()).ToCString();
        }
    }

    /// <summary>Returns the label in starting model attached to main entity (updated by Fill or SetMainResult, if Model is known).</summary>
    public string MainLabel()
    {
        return _themlab;
    }

    /// <summary>Returns the label in starting model attached to main entity.</summary>
    public int MainNumber()
    {
        return _themnum;
    }

    //  ############  INFORMATIONS  GLOBALES  ###########

    /// <summary>Searches for a key (starting entity) and returns its result. Returns a null handle if not found.</summary>
    public ResultFromTransient? ResultFromKey(Transient? start)
    {
        return _themain.ResultFromKey(start);
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
            var map = new IndexedSet<Transient>(ReferenceEqualityComparer.Instance);
            map.Add(_themain);
            _themain.FillMap(map);

            nb = map.Count;
            for (i = 1; i <= nb; i++)
            {
                list.Append(map[i - 1]);
            }
        }
        else
        {
            list.Append(_themain);
        }
        if (level == 1)
        {
            nb = _themain.SubResultCount;
            for (i = 1; i <= nb; i++)
            {
                list.Append(_themain.SubResult(i));
            }
            list.Append(_themain); // OCCT: the main result twice at level 1 (first and last)
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
            ResultFromTransient? unres = res.Value(i) as ResultFromTransient;
            if (unres is null)
            {
                continue;
            }
            if (unres.HasResult)
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
    public NCollection_HSequence<Transient> CheckedList(FrameworkCheckStatus check, bool result)
    {
        int i, nb;
        var list = new NCollection_HSequence<Transient>();
        NCollection_HSequence<Transient> res = Results(2);
        nb = res.Length();
        for (i = 1; i <= nb; i++)
        {
            ResultFromTransient? unres = res.Value(i) as ResultFromTransient;
            if (unres is null)
            {
                continue;
            }
            if (result && !unres.HasResult)
            {
                continue;
            }
            FrameworkCheck ach = unres.Check();
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
    public CheckIterator CheckList(bool erronly, int level = 2)
    {
        var chl = new CheckIterator();
        int i, nb;
        NCollection_HSequence<Transient> res = Results(level);
        nb = res.Length();
        for (i = 1; i <= nb; i++)
        {
            ResultFromTransient? unres = res.Value(i) as ResultFromTransient;
            if (unres is null)
            {
                continue;
            }
            FrameworkCheckStatus stat = unres.CheckStatus();
            if (stat == FrameworkCheckStatus.CheckOK || (stat == FrameworkCheckStatus.CheckWarning && erronly))
            {
                continue;
            }
            Binder binder = unres.Binder();
            FrameworkCheck bch = binder.Check();
            bch.SetEntity(unres.Start());
            chl.Add(bch, _themodel is null ? 0 : _themodel.Number(unres.Start()));
        }
        return chl;
    }

    /// <summary>
    /// Returns the check status with corresponds to the content of this ResultFromModel; considers all levels of
    /// transfer (worst status). Reads it from recorded status if already computed, else recomputes one.
    /// </summary>
    public FrameworkCheckStatus CheckStatus()
    {
        if (_themchk != FrameworkCheckStatus.CheckAny)
        {
            return _themchk;
        }
        CheckIterator chl = CheckList(false, 2);
        return chl.Status();
    }

    /// <summary>Computes and records check status (see CheckStatus). Does not computes it if already done and <paramref name="enforce"/> False.</summary>
    public FrameworkCheckStatus ComputeCheckStatus(bool enforce)
    {
        if (_themchk == FrameworkCheckStatus.CheckAny || enforce)
        {
            _themchk = CheckStatus();
        }
        return _themchk;
    }

    private InterfaceModel? _themodel;
    private string _thename = "";
    private ResultFromTransient? _themain;
    private string _themlab = "";
    private int _themnum;
    private FrameworkCheckStatus _themchk;
}
