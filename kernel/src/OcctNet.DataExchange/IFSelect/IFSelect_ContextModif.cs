// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ContextModif.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ContextModif.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class gathers various information used by Model Modifiers apart from the target model itself, and
/// the CopyTool which must be passed directly. These information report to original data : model, entities,
/// and the selection list if there is one : it allows to query about such or such starting entity, or result
/// entity, or iterate on selection list ... Also data useful for file output are available (because some
/// Modifiers concern models produced for file output). Furthermore, in return, ContextModif can record
/// Checks, either one for all, or one for each Entity. It supports trace too.
/// </summary>
public class IFSelect_ContextModif
{

    /// <summary>
    /// Prepares a ContextModif with these information : - the graph established from original model (target
    /// passed directly to Modifier) - the CopyTool which detains the CopyControl, which maps starting (in
    /// original) and result (in target) entities - an optional file name (for file output) Such a ContextModif
    /// is considered to be applied on all transferred entities (no filter active)
    /// </summary>
    public IFSelect_ContextModif(Interface_Graph graph, Interface_CopyTool TC, string filename = "")
    {
        thegraf = new Interface_Graph(graph, false);
        thefile = filename;
        thelist = new string(' ', graph.Size()).ToCharArray();
        themap = TC.Control();
        thesel = false;
        thecurr = thecurt = 0;
        int nb = thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            if (themap.Search(graph.Entity(i), out _))
            {
                thelist[i - 1] = '1';
            }
        }
    }

    /// <summary>
    /// Prepares a ContextModif with these information : - the graph established from original model (target
    /// passed directly to Modifier) - an optional file name (for file output) Here, no CopyControl, hence all
    /// entities are considered equal as starting and result Such a ContextModif is considered to be applied on
    /// all transferred entities (no filter active)
    /// </summary>
    public IFSelect_ContextModif(Interface_Graph graph, string filename = "")
    {
        thegraf = new Interface_Graph(graph, false);
        thefile = filename;
        thelist = new string(' ', graph.Size()).ToCharArray();
        thesel = false;
        thecurr = thecurt = 0;
        int nb = thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            thelist[i - 1] = '1';
        }
    }

    /// <summary>
    /// This method requires ContextModif to be applied with a filter. If a ModelModifier is defined with a
    /// Selection criterium, the result of this Selection is used as a filter : - if none of its items has been
    /// transferred, the modification does not apply at all - else, the Modifier can query for what entities were
    /// selected and what are their results - if this method is not called before working, the Modifier has to
    /// work on the whole Model
    /// </summary>
    public void Select(Interface_EntityIterator list)
    {
        thesel = true;
        int nb = thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            thelist[i - 1] = ' ';
        }
        for (list.Start(); list.More(); list.Next())
        {
            Standard_Transient start;
            start = list.Value();
            int num = thegraf.EntityNumber(start);
            if (num > nb || num < 0)
            {
                num = 0;
            }
            if (themap is null && num > 0)
            {
                thelist[num - 1] = '1';
            }
            else if (themap.Search(start, out _))
            {
                if (num > 0)
                {
                    thelist[num - 1] = '1';
                }
            }
        }
    }

    /// <summary>
    /// Returns the original Graph (compared to OriginalModel, it gives more query capabilitites)
    /// </summary>
    public Interface_Graph OriginalGraph()
    {
        return thegraf;
    }

    /// <summary>
    /// Returns the original model
    /// </summary>
    public Interface_InterfaceModel? OriginalModel()
    {
        return thegraf.Model();
    }

    /// <summary>
    /// Allows to transmit a Protocol as part of a ContextModif
    /// </summary>
    public void SetProtocol(Interface_Protocol? prot)
    {
        theprot = prot;
    }

    /// <summary>
    /// Returns the Protocol (Null if not set)
    /// </summary>
    public Interface_Protocol? Protocol()
    {
        return theprot;
    }

    /// <summary>
    /// Returns True if a non empty file name has been defined
    /// </summary>
    public bool HasFileName()
    {
        return (thefile.Length > 0);
    }

    /// <summary>
    /// Returns File Name (can be empty)
    /// </summary>
    public string FileName()
    {
        return thefile;
    }

    /// <summary>
    /// Returns the map for a direct use, if required
    /// </summary>
    public Interface_CopyControl? Control()
    {
        return themap;
    }

    /// <summary>
    /// Returns True if Select has determined that a Modifier may not be run (filter defined and empty)
    /// </summary>
    public bool IsForNone()
    {
        if (!thesel)
        {
            return false;
        }
        int nb = thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            if (thelist[i - 1] != ' ')
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
    public bool IsForAll()
    {
        return (!thesel);
    }

    /// <summary>
    /// Returns True if a starting item has been transferred
    /// </summary>
    public bool IsTransferred(Standard_Transient? ent)
    {
        if (themap is null)
        {
            return true;
        }
        return themap.Search(ent, out _);
    }

    /// <summary>
    /// Returns True if a starting item has been transferred and selected
    /// </summary>
    public bool IsSelected(Standard_Transient? ent)
    {
        //  Select already verified "IsTransferred"
        int num = thegraf.EntityNumber(ent);
        if (num == 0)
        {
            return false;
        }
        return (thelist[num - 1] != ' ');
    }

    // Search(ent, res) is declared by OCCT but never defined: left out.

    /// <summary>
    /// Returns the list of original selected items. See also the iteration
    /// </summary>
    public Interface_EntityIterator SelectedOriginal()
    {
        Interface_EntityIterator list = new Interface_EntityIterator();
        int nb = thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            if (thelist[i - 1] != ' ')
            {
                list.GetOneItem(thegraf.Entity(i));
            }
        }
        return list;
    }

    /// <summary>
    /// Returns the list of resulting counterparts of selected items. See also the iteration
    /// </summary>
    public Interface_EntityIterator SelectedResult()
    {
        Interface_EntityIterator list = new Interface_EntityIterator();
        int nb = thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            Standard_Transient newent = null;
            if (themap is null)
            {
                newent = thegraf.Entity(i);
            }
            else if (thelist[i - 1] != ' ')
            {
                themap.Search(thegraf.Entity(i), out newent);
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
        int nb = thelist.Length;
        int ns = 0;
        for (int i = 1; i <= nb; i++)
        {
            if (thelist[i - 1] != ' ')
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
        thecurr = thecurt = 0;
        Next();
    }

    /// <summary>
    /// Returns True until the iteration has finished
    /// </summary>
    public bool More()
    {
        return (thecurr > 0);
    }

    /// <summary>
    /// Advances the iteration
    /// </summary>
    public void Next()
    {
        int nb = thelist.Length;
        //  thecurr = thecurt;
        //  if (thecurr <= 0 && thecurt >= 0) return;
        for (int i = thecurr + 1; i <= nb; i++)
        {
            if (thelist[i - 1] != ' ')
            {
                thecurr = i;
                thecurt++;
                return;
            }
        }
        thecurr = thecurt = 0;
    }

    /// <summary>
    /// Returns the current selected item in the original model
    /// </summary>
    public Standard_Transient? ValueOriginal()
    {
        if (thecurr <= 0)
        {
            throw new Standard_NoSuchObject("IFSelect_ContextModif");
        }
        return thegraf.Entity(thecurr);
    }

    /// <summary>
    /// Returns the result counterpart of current selected item (in the target model)
    /// </summary>
    public Standard_Transient? ValueResult()
    {
        if (thecurr <= 0)
        {
            throw new Standard_NoSuchObject("IFSelect_ContextModif");
        }
        Standard_Transient ent, newent;
        ent = thegraf.Entity(thecurr);
        if (themap is null)
        {
            newent = ent;
        }
        else
        {
            themap.Search(ent, out newent);
        }
        return newent;
    }

    /// <summary>
    /// Traces the application of a Modifier. Works with default trace File and Level. Fills the trace if default
    /// trace level is at least 1. Traces the Modifier (its Label) and its Selection if there is one (its Label).
    /// To be called after Select (because status IsForAll is printed) Worths to trace a global modification. See
    /// also Trace below
    /// </summary>
    public void TraceModifier(IFSelect_GeneralModifier? modif)
    {
        if (modif is null)
        {
            return;
        }

        using Message_Messenger.StreamBuffer sout = Message.SendInfo();
        sout.Append("---   Run Modifier:");
        sout.Append('\n');
        IFSelect_Selection sel = modif.Selection();
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
        int ne = 0, nb = thelist.Length;
        for (int i = 1; i <= nb; i++)
        {
            if (thelist[i - 1] != ' ')
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
        if (thecurr <= 0)
        {
            return;
        }
        using Message_Messenger.StreamBuffer sout = Message.SendInfo();
        if (ValueOriginal() == ValueResult())
        {
            sout.Append("--  ContextModif. Entity  n0 ");
            sout.Append(thecurr);
            sout.Append('\n');
        }
        else
        {
            sout.Append("--  ContextModif. Entity in Original, n0 ");
            sout.Append(thecurr);
            sout.Append(" in Result, n0 ");
            sout.Append(thecurt);
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
    public void AddCheck(Interface_Check? check)
    {
        if (check.NbFails() + check.NbWarnings() == 0)
        {
            return;
        }
        Standard_Transient ent = check.Entity();
        int num = thegraf.EntityNumber(ent);
        if (num == 0 && ent is not null)
        {
            num = -1; // force enregistrement
        }
        thechek.Add(check, num);
    }

    /// <summary>
    /// Adds a Warning Message for an Entity from the original Model If &lt;start&gt; is not an Entity from the
    /// original model (e.g. the model itself) this message is added to Global Check.
    /// </summary>
    public void AddWarning(Standard_Transient? start, string mess, string orig = "")
    {
        thechek.CCheck(thegraf.EntityNumber(start)).AddWarning(mess, orig);
    }

    /// <summary>
    /// Adds a Fail Message for an Entity from the original Model If &lt;start&gt; is not an Entity from the
    /// original model (e.g. the model itself) this message is added to Global Check.
    /// </summary>
    public void AddFail(Standard_Transient? start, string mess, string orig = "")
    {
        thechek.CCheck(thegraf.EntityNumber(start)).AddFail(mess, orig);
    }

    /// <summary>
    /// Returns a Check given an Entity number (in the original Model) by default a Global Check. Creates it the
    /// first time. It can then be acknowledged on the spot, in condition that the caller works by reference
    /// ("Interface_Check&amp; check = ...")
    /// </summary>
    public Interface_Check? CCheck(int num = 0)
    {
        Interface_Check ach = thechek.CCheck(num);
        if (num > 0 && num <= thegraf.Size())
        {
            ach.SetEntity(thegraf.Entity(num));
        }
        return ach;
    }

    /// <summary>
    /// Returns a Check attached to an Entity from the original Model It can then be acknowledged on the spot, in
    /// condition that the caller works by reference ("Interface_Check&amp; check = ...")
    /// </summary>
    public Interface_Check? CCheck(Standard_Transient? ent)
    {
        int num = thegraf.EntityNumber(ent);
        if (num == 0)
        {
            num = -1; // force l enregistrement
        }
        Interface_Check ach = thechek.CCheck(num);
        ach.SetEntity(ent);
        return ach;
    }

    /// <summary>
    /// Returns the complete CheckList
    /// </summary>
    public Interface_CheckIterator CheckList()
    {
        return thechek;
    }

    private Interface_Graph thegraf;
    private Interface_Protocol? theprot;
    private Interface_CopyControl? themap;
    private string thefile = "";
    private char[] thelist;
    private Interface_CheckIterator thechek = new Interface_CheckIterator();
    private bool thesel;
    private int thecurr;
    private int thecurt;
}
