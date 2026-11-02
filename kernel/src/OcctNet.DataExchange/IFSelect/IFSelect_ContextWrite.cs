// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ContextWrite.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ContextWrite.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

namespace Occt;

/// <summary>
/// This class gathers various information used by File Modifiers apart from the writer object, which is
/// specific of the norm and of the physical format These information are controlled by an object
/// AppliedModifiers (if it is not defined, no modification is allowed on writing) Furthermore, in return,
/// ContextModif can record Checks, either one for all, or one for each Entity. It supports trace too.
/// </summary>
public class IFSelect_ContextWrite
{

    /// <summary>
    /// Prepares a ContextWrite with these information : - the model which is to be written - the protocol to be
    /// used - the filename - an object AppliedModifiers to work. It gives a list of FileModifiers to be ran, and
    /// for each one it can give a restricted list of entities (in the model), else all the model is considered
    /// </summary>
    /// <remarks>
    /// OCCT.NET: <paramref name="theStream"/> (not in OCCT), when given, is where the work library writes, in place
    /// of the file <paramref name="filename"/> (decision 1225).
    /// </remarks>
    public IFSelect_ContextWrite(Interface_InterfaceModel? model, Interface_Protocol? proto, IFSelect_AppliedModifiers? applieds, string filename,
                                 Stream? theStream = null)
    {
        myStream = theStream;
        themodel = model;
        theproto = proto;
        thefile = filename;
        theapply = applieds;
        thenumod = 0;
        thenbent = 0;
        thecurr = 0;
    }

    /// <summary>
    /// Same as above but with an already computed Graph
    /// </summary>
    /// <remarks>
    /// OCCT.NET: <paramref name="theStream"/> (not in OCCT), when given, is where the work library writes, in place
    /// of the file <paramref name="filename"/> (decision 1225).
    /// </remarks>
    public IFSelect_ContextWrite(Interface_HGraph? hgraph, Interface_Protocol? proto, IFSelect_AppliedModifiers? applieds, string filename,
                                 Stream? theStream = null)
    {
        myStream = theStream;
        themodel = hgraph.Graph().Model();
        theproto = proto;
        thefile = filename;
        theapply = applieds;
        thehgraf = hgraph;
        thenumod = 0;
        thenbent = 0;
        thecurr = 0;
    }

    /// <summary>
    /// Returns the Model
    /// </summary>
    public Interface_InterfaceModel? Model()
    {
        return themodel;
    }

    /// <summary>
    /// Returns the Protocol;
    /// </summary>
    public Interface_Protocol? Protocol()
    {
        return theproto;
    }

    /// <summary>
    /// Returns the File Name
    /// </summary>
    public string FileName()
    {
        return thefile;
    }

    /// <summary>
    /// Returns the stream to write to, null if the library writes the file FileName (OCCT.NET, decision 1225)
    /// </summary>
    public Stream? OStream()
    {
        return myStream;
    }

    /// <summary>
    /// Returns the object AppliedModifiers
    /// </summary>
    public IFSelect_AppliedModifiers? AppliedModifiers()
    {
        return theapply;
    }

    /// <summary>
    /// Returns the Graph, either given when created, else created the first time it is queried
    /// </summary>
    public Interface_Graph Graph()
    {
        if (thehgraf is null)
        {
            thehgraf = new Interface_HGraph(themodel, theproto);
        }
        return thehgraf.Graph();
    }

    /// <summary>
    /// Returns the count of recorded File Modifiers
    /// </summary>
    public int NbModifiers()
    {
        return (theapply is null ? 0 : theapply.Count());
    }

    /// <summary>
    /// Sets active the File Modifier n0 &lt;numod&gt; Then, it prepares the list of entities to consider, if any
    /// Returns False if &lt;numod&gt; out of range
    /// </summary>
    public bool SetModifier(int numod)
    {
        themodif = null;
        thenumod = thenbent = thecurr = 0;
        if (theapply is null)
        {
            return false;
        }
        if (numod < 1 || numod > theapply.Count())
        {
            return false;
        }
        theapply.Item(numod, ref themodif, ref thenbent);
        return true;
    }

    /// <summary>
    /// Returns the currently active File Modifier. Cast to be done Null if not properly set : must be test
    /// IsNull after casting
    /// </summary>
    public IFSelect_GeneralModifier? FileModifier()
    {
        return themodif;
    }

    /// <summary>
    /// Returns True if no modifier is currently set
    /// </summary>
    public bool IsForNone()
    {
        return (thenbent == 0);
    }

    /// <summary>
    /// Returns True if the current modifier is to be applied to the whole model. Else, a restricted list of
    /// selected entities is defined, it can be exploited by the File Modifier
    /// </summary>
    public bool IsForAll()
    {
        return theapply.IsForAll();
    }

    /// <summary>
    /// Returns the total count of selected entities
    /// </summary>
    public int NbEntities()
    {
        return thenbent;
    }

    /// <summary>
    /// Starts an iteration on selected items. It takes into account IsForAll/IsForNone, by really iterating on
    /// all selected items.
    /// </summary>
    public void Start()
    {
        thecurr = 1;
    }

    /// <summary>
    /// Returns True until the iteration has finished
    /// </summary>
    public bool More()
    {
        return (thecurr <= thenbent);
    }

    /// <summary>
    /// Advances the iteration
    /// </summary>
    public void Next()
    {
        thecurr++;
    }

    /// <summary>
    /// Returns the current selected entity in the model
    /// </summary>
    public Standard_Transient? Value()
    {
        if (thecurr < 1 || thecurr > thenbent)
        {
            throw new Standard_NoSuchObject("IFSelect_ContextWrite:Value");
        }
        int num = theapply.ItemNum(thecurr);
        return themodel.Value(num);
    }

    /// <summary>
    /// Adds a Check to the CheckList. If it is empty, nothing is done If it concerns an Entity from the Model
    /// (by SetEntity) to which another Check is attached, it is merged to it. Else, it is added or merged as to
    /// GlobalCheck.
    /// </summary>
    public void AddCheck(Interface_Check? check)
    {
        if (check.NbFails() + check.NbWarnings() == 0)
        {
            return;
        }
        Standard_Transient ent = check.Entity();
        int num = themodel.Number(ent);
        if (num == 0 && ent is not null)
        {
            num = -1; // force enregistrement
        }
        thecheck.Add(check, num);
    }

    /// <summary>
    /// Adds a Warning Message for an Entity from the Model If &lt;start&gt; is not an Entity from the model
    /// (e.g. the model itself) this message is added to Global Check.
    /// </summary>
    public void AddWarning(Standard_Transient? start, string mess, string orig = "")
    {
        thecheck.CCheck(themodel.Number(start)).AddWarning(mess, orig);
    }

    /// <summary>
    /// Adds a Fail Message for an Entity from the Model If &lt;start&gt; is not an Entity from the model (e.g.
    /// the model itself) this message is added to Global Check.
    /// </summary>
    public void AddFail(Standard_Transient? start, string mess, string orig = "")
    {
        thecheck.CCheck(themodel.Number(start)).AddFail(mess, orig);
    }

    /// <summary>
    /// Returns a Check given an Entity number (in the Model) by default a Global Check. Creates it the first
    /// time. It can then be acknowledged on the spot, in condition that the caller works by reference
    /// ("Interface_Check&amp; check = ...")
    /// </summary>
    public Interface_Check? CCheck(int num = 0)
    {
        Interface_Check ach = thecheck.CCheck(num);
        if (num > 0 && num <= themodel.NbEntities())
        {
            ach.SetEntity(themodel.Value(num));
        }
        return ach;
    }

    /// <summary>
    /// Returns a Check attached to an Entity from the Model It can then be acknowledged on the spot, in
    /// condition that the caller works by reference ("Interface_Check&amp; check = ...")
    /// </summary>
    public Interface_Check? CCheck(Standard_Transient? ent)
    {
        int num = themodel.Number(ent);
        if (num == 0)
        {
            num = -1; // force l enregistrement
        }
        Interface_Check ach = thecheck.CCheck(num);
        ach.SetEntity(ent);
        return ach;
    }

    /// <summary>
    /// Returns the complete CheckList
    /// </summary>
    public Interface_CheckIterator CheckList()
    {
        return thecheck;
    }

    private Interface_InterfaceModel? themodel;
    private Interface_Protocol? theproto;
    private string thefile = "";
    private IFSelect_AppliedModifiers? theapply;
    private Interface_HGraph? thehgraf;
    private Interface_CheckIterator thecheck = new Interface_CheckIterator();
    private int thenumod;
    private int thenbent;
    private int thecurr;
    private IFSelect_GeneralModifier? themodif;
    private readonly Stream? myStream;
}
