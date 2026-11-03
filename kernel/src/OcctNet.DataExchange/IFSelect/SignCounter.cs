// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignCounter.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_SignCounter.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// SignCounter gives the frame to count signatures associated with entities, deducted from them. Ex.: their
/// Dynamic Type. It can sort a set of Entities according a signature, i.e. : - list of different values
/// found for this Signature - for each one, count and list of entities Results are returned as a
/// SignatureList, which can be queried on the count (list of strings, count per signature, or list of
/// entities per signature) A SignCounter can be filled, either directly from lists, or from the result of a
/// Selection : hence, its content can be automatically recomputed as desired SignCounter works by using a
/// Signature in its method AddSign Methods can be redefined to, either - directly compute the value without
/// a Signature - compute the value in the context of a Graph
/// </summary>
public class SignCounter : SignatureList
{

    /// <summary>
    /// Creates a SignCounter, without proper Signature If &lt;withmap&gt; is True (default), added entities are
    /// counted only if they are not yet recorded in the map Map control can be set off if the input guarantees
    /// uniqueness of data &lt;withlist&gt; is transmitted to SignatureList (option to list entities, not only to
    /// count them).
    /// </summary>
    public SignCounter(bool withmap = true, bool withlist = false) : base(withlist)
    {
        _themapstat = withmap;
        _thenbcomp1 = _thenbcomp2 = _theselmode = 0;
    }

    /// <summary>
    /// Creates a SignCounter, with a predefined Signature Other arguments as for Create without Signature.
    /// </summary>
    public SignCounter(Signature? matcher, bool withmap = true, bool withlist = false) : base(withlist)
    {
        _thematcher = matcher;
        _themapstat = withmap;
        _thenbcomp1 = _thenbcomp2 = _theselmode = 0;
        string sign = _thematcher.Name();
        SetName(sign);
    }

    /// <summary>
    /// Returns the Signature used to count entities. It can be null.
    /// </summary>
    public Signature? Signature()
    {
        return _thematcher;
    }

    /// <summary>
    /// Changes the control status. The map is not cleared, simply its use changes
    /// </summary>
    public void SetMap(bool withmap)
    {
        _themapstat = withmap;
    }

    /// <summary>
    /// Adds an entity by considering its signature, which is given by call to method AddSign Returns True if
    /// added, False if already in the map (and map control status set)
    /// </summary>
    public virtual bool AddEntity(Transient? ent, InterfaceModel? model)
    {
        if (_themapstat && ent is not null)
        {
            if (_themap.Contains(ent))
            {
                return false;
            }
            _themap.Add(ent);
        }
        AddSign(ent, model);
        return true;
    }

    /// <summary>
    /// Adds an entity (already filtered by Map) with its signature. This signature can be computed with the
    /// containing model. Its value is provided by the object Signature given at start, if no Signature is
    /// defined, it does nothing. Can be redefined (in this case, see also Sign)
    /// </summary>
    public virtual void AddSign(Transient? ent, InterfaceModel? model)
    {
        const string nulsign = "";
        if (ent is null || _thematcher is null)
        {
            Add(ent, nulsign); // to count the Nulls
        }
        else
        {
            Add(ent, _thematcher.Value(ent, model));
        }
    }

    /// <summary>
    /// Adds a list of entities by adding each of the items
    /// </summary>
    public void AddList(NCollection_HSequence<Transient>? list, InterfaceModel? model)
    {
        if (list is null)
        {
            return;
        }
        int nb = list.Length();
        for (int i = 1; i <= nb; i++)
        {
            AddEntity(list.Value(i), model);
        }
    }

    /// <summary>
    /// Adds a list of entities in the context given by the graph Default just call basic AddList Can be
    /// redefined to get a signature computed with the graph
    /// </summary>
    public virtual void AddWithGraph(NCollection_HSequence<Transient>? list, EntityGraph graph)
    {
        AddList(list, graph.Model());
    }

    /// <summary>
    /// Adds all the entities contained in a Model
    /// </summary>
    public void AddModel(InterfaceModel? model)
    {
        if (model is null)
        {
            return;
        }
        int nb = model.EntityCount;
        //  If we start from empty, we know that each entity is unique in the model
        bool mapstat = _themapstat;
        if (_themap.Extent() == 0)
        {
            _themapstat = false;
        }
        for (int i = 1; i <= nb; i++)
        {
            AddEntity(model.Value(i), model);
        }
        _themapstat = mapstat;
    }

    /// <summary>
    /// Adds the result determined by a Selection from a Graph Remark : does not impact at all data from
    /// SetSelection &amp; Co
    /// </summary>
    public void AddFromSelection(AbstractSelection? sel, EntityGraph g)
    {
        EntityIterator iter = sel.RootResult(g);
        AddWithGraph(iter.Content(), g);
    }

    /// <summary>
    /// Sets a Selection as input : this causes content to be cleared then the Selection to be ready to compute
    /// (but not immediately)
    /// </summary>
    public void SetSelection(AbstractSelection? sel)
    {
        _theselect = sel;
        SetSelMode(-1);
        SetSelMode(sel is null ? 0 : 2);
    }

    /// <summary>
    /// Returns the selection, or a null Handle
    /// </summary>
    public AbstractSelection? Selection()
    {
        return _theselect;
    }

    /// <summary>
    /// Changes the mode of working with the selection : -1 just clears optimisation data and nothing else 0
    /// clears it 1 inhibits it for computing (but no clearing) 2 sets it active for computing Default at
    /// creation is 0, after SetSelection (not null) is 2
    /// </summary>
    public void SetSelMode(int selmode)
    {
        if (selmode < 0)
        {
            _thenbcomp1 = _thenbcomp2 = 0;
        }
        else
        {
            _theselmode = selmode;
        }
        if (selmode == 0)
        {
            _theselect = null;
        }
    }

    /// <summary>
    /// Returns the mode of working with the selection
    /// </summary>
    public int SelMode()
    {
        return _theselmode;
    }

    /// <summary>
    /// Computes from the selection result, if selection is active (mode 2). If selection is not defined (mode 0)
    /// or is inhibited (mode 1) does nothing. Returns True if computation is done (or optimised), False else
    /// This method is called by ComputeCounter from WorkSession If &lt;forced&gt; is True, recomputes
    /// systematically Else (D), if the counter was not cleared and if the former computed result started from
    /// the same total size of Graph and same count of selected entities : computation is not redone unless
    /// &lt;forced&gt; is given as True
    /// </summary>
    public bool ComputeSelected(EntityGraph g, bool forced = false)
    {
        if (_theselmode < 2 || _theselect is null)
        {
            return false;
        }
        bool afaire = forced;
        EntityIterator iter = _theselect.RootResult(g);
        int nb1 = g.Size();
        int nb2 = iter.NbEntities();
        if (!afaire)
        {
            afaire = (nb1 != _thenbcomp1 || nb2 != _thenbcomp2);
        }
        _thenbcomp1 = nb1;
        _thenbcomp2 = nb2;
        if (afaire)
        {
            AddWithGraph(iter.Content(), g);
        }
        return true;
    }

    /// <summary>
    /// Determines and returns the value of the signature for an entity as an HAsciiString. This method works
    /// exactly as AddSign, which is optimized Can be redefined, accorded with AddSign
    /// </summary>
    public virtual TCollection_HAsciiString? Sign(Transient? ent, InterfaceModel? model)
    {
        TCollection_HAsciiString res = null;
        if (ent is null || _thematcher is null)
        {
            return res;
        }
        res = new TCollection_HAsciiString(_thematcher.Value(ent, model));
        return res;
    }

    /// <summary>
    /// Applies AddWithGraph on one entity, and returns the Signature Value which has been recorded To do this,
    /// Add is called with SignOnly Mode True during the call, the returned value is LastValue
    /// </summary>
    public string ComputedSign(Transient? ent, EntityGraph g)
    {
        NCollection_HSequence<Transient> list =
            new NCollection_HSequence<Transient>();
        list.Append(ent);
        ModeSignOnly() = true;
        AddWithGraph(list, g);
        string val = LastValue();
        ModeSignOnly() = false;
        return val;
    }

    private bool _themapstat;
    private NCollection_Map<Transient> _themap = new NCollection_Map<Transient>();
    private Signature? _thematcher;
    private AbstractSelection? _theselect;
    private int _theselmode;
    private int _thenbcomp1;
    private int _thenbcomp2;
}
