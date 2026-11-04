// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ContextModif.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ContextModif.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// This class gathers various information used by Model Modifiers apart from the target model itself, and
/// the CopyTool which must be passed directly. These information report to original data : model, entities,
/// and the selection list if there is one : it allows to query about such or such starting entity, or result
/// entity, or iterate on selection list ... Also data useful for file output are available (because some
/// Modifiers concern models produced for file output). Furthermore, in return, ContextModif can record
/// Checks, either one for all, or one for each Entity. It supports trace too.
/// </summary>
internal class ContextModif
{

    /// <summary>
    /// Prepares a ContextModif with these information : - the graph established from original model (target
    /// passed directly to Modifier) - the CopyTool which detains the CopyControl, which maps starting (in
    /// original) and result (in target) entities - an optional file name (for file output) Such a ContextModif
    /// is considered to be applied on all transferred entities (no filter active)
    /// </summary>
    public ContextModif(EntityGraph graph, CopyTool tc, string filename = "")
    {
        _thegraf = new EntityGraph(graph, false);
        _thefile = filename;
        _thelist = new string(' ', graph.Size()).ToCharArray();
        _themap = tc.Control();
        _thesel = false;
        _thecurr = _thecurt = 0;
        int nb = _thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            if (_themap.Search(graph.Entity(i), out _))
            {
                _thelist[i - 1] = '1';
            }
        }
    }

    /// <summary>
    /// Prepares a ContextModif with these information : - the graph established from original model (target
    /// passed directly to Modifier) - an optional file name (for file output) Here, no CopyControl, hence all
    /// entities are considered equal as starting and result Such a ContextModif is considered to be applied on
    /// all transferred entities (no filter active)
    /// </summary>
    public ContextModif(EntityGraph graph, string filename = "")
    {
        _thegraf = new EntityGraph(graph, false);
        _thefile = filename;
        _thelist = new string(' ', graph.Size()).ToCharArray();
        _thesel = false;
        _thecurr = _thecurt = 0;
        int nb = _thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            _thelist[i - 1] = '1';
        }
    }

    /// <summary>
    /// This method requires ContextModif to be applied with a filter. If a ModelModifier is defined with a
    /// Selection criterium, the result of this Selection is used as a filter : - if none of its items has been
    /// transferred, the modification does not apply at all - else, the Modifier can query for what entities were
    /// selected and what are their results - if this method is not called before working, the Modifier has to
    /// work on the whole Model
    /// </summary>
    public void Select(EntityIterator list)
    {
        _thesel = true;
        int nb = _thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            _thelist[i - 1] = ' ';
        }
        for (list.Start(); list.More(); list.Next())
        {
            Transient start;
            start = list.Value();
            int num = _thegraf.EntityNumber(start);
            if (num > nb || num < 0)
            {
                num = 0;
            }
            if (_themap is null && num > 0)
            {
                _thelist[num - 1] = '1';
            }
            else if (_themap.Search(start, out _))
            {
                if (num > 0)
                {
                    _thelist[num - 1] = '1';
                }
            }
        }
    }

    /// <summary>
    /// Returns the original Graph (compared to OriginalModel, it gives more query capabilitites)
    /// </summary>
    public EntityGraph OriginalGraph()
    {
        return _thegraf;
    }

    /// <summary>
    /// Returns the original model
    /// </summary>
    public InterfaceModel? OriginalModel()
    {
        return _thegraf.Model();
    }

    /// <summary>
    /// Allows to transmit a Protocol as part of a ContextModif
    /// </summary>
    public void SetProtocol(Protocol? prot)
    {
        _theprot = prot;
    }

    /// <summary>
    /// Returns the Protocol (Null if not set)
    /// </summary>
    public Protocol? Protocol()
    {
        return _theprot;
    }

    /// <summary>
    /// Returns True if a non empty file name has been defined
    /// </summary>
    public bool HasFileName
    {
        get
        {
            return (_thefile.Length > 0);
        }
    }

    /// <summary>
    /// Returns File Name (can be empty)
    /// </summary>
    public string FileName()
    {
        return _thefile;
    }

    /// <summary>
    /// Returns the map for a direct use, if required
    /// </summary>
    public CopyControl? Control()
    {
        return _themap;
    }

    /// <summary>
    /// Returns True if Select has determined that a Modifier may not be run (filter defined and empty)
    /// </summary>
    public bool IsForNone()
    {
        if (!_thesel)
        {
            return false;
        }
        int nb = _thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            if (_thelist[i - 1] != ' ')
            {
                return false;
            }
        }
        return true;
    }

    /// <summary>
    /// Returns True if no filter is defined : a Modifier has to work on all entities of the resulting (target)
    /// model
    /// </summary>
    public bool IsForAll
    {
        get
        {
            return (!_thesel);
        }
    }

    /// <summary>
    /// Returns True if a starting item has been transferred
    /// </summary>
    public bool IsTransferred(Transient? ent)
    {
        if (_themap is null)
        {
            return true;
        }
        return _themap.Search(ent, out _);
    }

    /// <summary>
    /// Returns True if a starting item has been transferred and selected
    /// </summary>
    public bool IsSelected(Transient? ent)
    {
        //  Select already verified "IsTransferred"
        int num = _thegraf.EntityNumber(ent);
        if (num == 0)
        {
            return false;
        }
        return (_thelist[num - 1] != ' ');
    }

    // Search(ent, res) is declared by OCCT but never defined: left out.

    /// <summary>
    /// Returns the list of original selected items. See also the iteration
    /// </summary>
    public EntityIterator SelectedOriginal()
    {
        EntityIterator list = new EntityIterator();
        int nb = _thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            if (_thelist[i - 1] != ' ')
            {
                list.GetOneItem(_thegraf.Entity(i));
            }
        }
        return list;
    }

    /// <summary>
    /// Returns the list of resulting counterparts of selected items. See also the iteration
    /// </summary>
    public EntityIterator SelectedResult()
    {
        EntityIterator list = new EntityIterator();
        int nb = _thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            Transient newent = null;
            if (_themap is null)
            {
                newent = _thegraf.Entity(i);
            }
            else if (_thelist[i - 1] != ' ')
            {
                _themap.Search(_thegraf.Entity(i), out newent);
            }
            if (newent is not null)
            {
                list.GetOneItem(newent);
            }
        }
        return list;
    }

    /// <summary>
    /// Returns the count of selected and transferred items
    /// </summary>
    public int SelectedCount()
    {
        int nb = _thelist.Length;
        int ns = 0;
        for (int i = 1; i <= nb; i++)
        {
            if (_thelist[i - 1] != ' ')
            {
                ns++;
            }
        }
        return ns;
    }

    /// <summary>
    /// Starts an iteration on selected items. It takes into account IsForAll/IsForNone, by really iterating on
    /// all selected items.
    /// </summary>
    public void Start()
    {
        _thecurr = _thecurt = 0;
        Next();
    }

    /// <summary>
    /// Returns True until the iteration has finished
    /// </summary>
    public bool More()
    {
        return (_thecurr > 0);
    }

    /// <summary>
    /// Advances the iteration
    /// </summary>
    public void Next()
    {
        int nb = _thelist.Length;
        //  thecurr = thecurt;
        //  if (thecurr <= 0 && thecurt >= 0) return;
        for (int i = _thecurr + 1; i <= nb; i++)
        {
            if (_thelist[i - 1] != ' ')
            {
                _thecurr = i;
                _thecurt++;
                return;
            }
        }
        _thecurr = _thecurt = 0;
    }

    /// <summary>
    /// Returns the current selected item in the original model
    /// </summary>
    public Transient? ValueOriginal()
    {
        if (_thecurr <= 0)
        {
            throw new NoSuchObjectException("IFSelect_ContextModif");
        }
        return _thegraf.Entity(_thecurr);
    }

    /// <summary>
    /// Returns the result counterpart of current selected item (in the target model)
    /// </summary>
    public Transient? ValueResult()
    {
        if (_thecurr <= 0)
        {
            throw new NoSuchObjectException("IFSelect_ContextModif");
        }
        Transient ent, newent;
        ent = _thegraf.Entity(_thecurr);
        if (_themap is null)
        {
            newent = ent;
        }
        else
        {
            _themap.Search(ent, out newent);
        }
        return newent;
    }

    /// <summary>
    /// Traces the application of a Modifier. Works with default trace File and Level. Fills the trace if default
    /// trace level is at least 1. Traces the Modifier (its Label) and its Selection if there is one (its Label).
    /// To be called after Select (because status IsForAll is printed) Worths to trace a global modification. See
    /// also Trace below
    /// </summary>
    public void TraceModifier(GeneralModifier? modif)
    {
        if (modif is null)
        {
            return;
        }

        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
        sout.Append("---   Run Modifier:");
        sout.Append('\n');
        AbstractSelection sel = modif.Selection();
        if (sel is not null)
        {
            sout.Append("      Selection:");
            sout.Append(sel.Label());
        }
        else
        {
            sout.Append("  (no Selection)");
        }

        //  on va simplement compter les entites
        int ne = 0, nb = _thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            if (_thelist[i - 1] != ' ')
            {
                ne++;
            }
        }
        if (nb == ne)
        {
            sout.Append("  All Model (");
            sout.Append(nb);
            sout.Append(" Entities)");
            sout.Append('\n');
        }
        else
        {
            sout.Append("  Entities,Total:");
            sout.Append(nb);
            sout.Append(" Concerned:");
            sout.Append(ne);
            sout.Append('\n');
        }
    }

    /// <summary>
    /// Traces the modification of the current entity (see above, ValueOriginal and ValueResult) for default
    /// trace level &gt;= 2. To be called on each individual entity really modified &lt;mess&gt; is an optional
    /// additional message
    /// </summary>
    public void Trace(string mess = "")
    {
        //  Trace courante
        if (_thecurr <= 0)
        {
            return;
        }
        using Messenger.StreamBuffer sout = global::Kerf.Engine.Foundation.Message.SendInfo();
        if (ValueOriginal() == ValueResult())
        {
            sout.Append("--  ContextModif. Entity  n0 ");
            sout.Append(_thecurr);
            sout.Append('\n');
        }
        else
        {
            sout.Append("--  ContextModif. Entity in Original, n0 ");
            sout.Append(_thecurr);
            sout.Append(" in Result, n0 ");
            sout.Append(_thecurt);
            sout.Append('\n');
        }
        if (mess.Length > 0)
        {
            sout.Append("--  Message:");
            sout.Append(mess);
            sout.Append('\n');
        }
    }

    /// <summary>
    /// Adds a Check to the CheckList. If it is empty, nothing is done If it concerns an Entity from the Original
    /// Model (by SetEntity) to which another Check is attached, it is merged to it. Else, it is added or merged
    /// as to GlobalCheck.
    /// </summary>
    public void AddCheck(FrameworkCheck? check)
    {
        if (check.FailCount + check.WarningCount == 0)
        {
            return;
        }
        Transient ent = check.Entity();
        int num = _thegraf.EntityNumber(ent);
        if (num == 0 && ent is not null)
        {
            num = -1; // force enregistrement
        }
        _thechek.Add(check, num);
    }

    /// <summary>
    /// Adds a Warning Message for an Entity from the original Model If &lt;start&gt; is not an Entity from the
    /// original model (e.g. the model itself) this message is added to Global Check.
    /// </summary>
    public void AddWarning(Transient? start, string mess, string orig = "")
    {
        _thechek.CCheck(_thegraf.EntityNumber(start)).AddWarning(mess, orig);
    }

    /// <summary>
    /// Adds a Fail Message for an Entity from the original Model If &lt;start&gt; is not an Entity from the
    /// original model (e.g. the model itself) this message is added to Global Check.
    /// </summary>
    public void AddFail(Transient? start, string mess, string orig = "")
    {
        _thechek.CCheck(_thegraf.EntityNumber(start)).AddFail(mess, orig);
    }

    /// <summary>
    /// Returns a Check given an Entity number (in the original Model) by default a Global Check. Creates it the
    /// first time. It can then be acknowledged on the spot, in condition that the caller works by reference
    /// ("Interface_Check&amp; check = ...")
    /// </summary>
    public FrameworkCheck? CCheck(int num = 0)
    {
        FrameworkCheck ach = _thechek.CCheck(num);
        if (num > 0 && num <= _thegraf.Size())
        {
            ach.SetEntity(_thegraf.Entity(num));
        }
        return ach;
    }

    /// <summary>
    /// Returns a Check attached to an Entity from the original Model It can then be acknowledged on the spot, in
    /// condition that the caller works by reference ("Interface_Check&amp; check = ...")
    /// </summary>
    public FrameworkCheck? CCheck(Transient? ent)
    {
        int num = _thegraf.EntityNumber(ent);
        if (num == 0)
        {
            num = -1; // force l enregistrement
        }
        FrameworkCheck ach = _thechek.CCheck(num);
        ach.SetEntity(ent);
        return ach;
    }

    /// <summary>
    /// Returns the complete CheckList
    /// </summary>
    public CheckIterator CheckList()
    {
        return _thechek;
    }

    private EntityGraph _thegraf;
    private Protocol? _theprot;
    private CopyControl? _themap;
    private string _thefile = "";
    private char[] _thelist;
    private CheckIterator _thechek = new CheckIterator();
    private bool _thesel;
    private int _thecurr;
    private int _thecurt;
}
