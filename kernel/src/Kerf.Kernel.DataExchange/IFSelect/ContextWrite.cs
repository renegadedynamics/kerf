// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ContextWrite.hxx
// Port of: src/DataExchange/TKXSBase/IFSelect/IFSelect_ContextWrite.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Exchange.Framework.Selection;

/// <summary>
/// This class gathers various information used by File Modifiers apart from the writer object, which is
/// specific of the norm and of the physical format These information are controlled by an object
/// AppliedModifiers (if it is not defined, no modification is allowed on writing) Furthermore, in return,
/// ContextModif can record Checks, either one for all, or one for each Entity. It supports trace too.
/// </summary>
internal class ContextWrite
{

    /// <summary>
    /// Prepares a ContextWrite with these information : - the model which is to be written - the protocol to be
    /// used - the filename - an object AppliedModifiers to work. It gives a list of FileModifiers to be ran, and
    /// for each one it can give a restricted list of entities (in the model), else all the model is considered
    /// </summary>
    /// <remarks>
    /// Kerf: <paramref name="stream"/> (not in OCCT), when given, is where the work library writes, in place
    /// of the file <paramref name="filename"/> (decision 1225).
    /// </remarks>
    public ContextWrite(InterfaceModel? model, Protocol? proto, AppliedModifiers? applieds, string filename,
                                 Stream? stream = null)
    {
        _stream = stream;
        _themodel = model;
        _theproto = proto;
        _thefile = filename;
        _theapply = applieds;
        _thenbent = 0;
        _thecurr = 0;
    }

    /// <summary>
    /// Same as above but with an already computed Graph
    /// </summary>
    /// <remarks>
    /// Kerf: <paramref name="stream"/> (not in OCCT), when given, is where the work library writes, in place
    /// of the file <paramref name="filename"/> (decision 1225).
    /// </remarks>
    public ContextWrite(HGraph? hgraph, Protocol? proto, AppliedModifiers? applieds, string filename,
                                 Stream? stream = null)
    {
        _stream = stream;
        _themodel = hgraph.Graph().Model();
        _theproto = proto;
        _thefile = filename;
        _theapply = applieds;
        _thehgraf = hgraph;
        _thenbent = 0;
        _thecurr = 0;
    }

    /// <summary>
    /// Returns the Model
    /// </summary>
    public InterfaceModel? Model()
    {
        return _themodel;
    }

    /// <summary>
    /// Returns the Protocol;
    /// </summary>
    public Protocol? Protocol()
    {
        return _theproto;
    }

    /// <summary>
    /// Returns the File Name
    /// </summary>
    public string FileName()
    {
        return _thefile;
    }

    /// <summary>
    /// Returns the stream to write to, null if the library writes the file FileName (Kerf, decision 1225)
    /// </summary>
    public Stream? OStream()
    {
        return _stream;
    }

    /// <summary>
    /// Returns the object AppliedModifiers
    /// </summary>
    public AppliedModifiers? AppliedModifiers()
    {
        return _theapply;
    }

    /// <summary>
    /// Returns the Graph, either given when created, else created the first time it is queried
    /// </summary>
    public EntityGraph Graph()
    {
        if (_thehgraf is null)
        {
            _thehgraf = new HGraph(_themodel, _theproto);
        }
        return _thehgraf.Graph();
    }

    /// <summary>
    /// Returns the count of recorded File Modifiers
    /// </summary>
    public int ModifierCount
    {
        get
        {
            return (_theapply is null ? 0 : _theapply.Count());
        }
    }

    /// <summary>
    /// Sets active the File Modifier n0 &lt;numod&gt; Then, it prepares the list of entities to consider, if any
    /// Returns False if &lt;numod&gt; out of range
    /// </summary>
    public bool SetModifier(int numod)
    {
        _themodif = null;
        _thenbent = _thecurr = 0;
        if (_theapply is null)
        {
            return false;
        }
        if (numod < 1 || numod > _theapply.Count())
        {
            return false;
        }
        _theapply.Item(numod, ref _themodif, ref _thenbent);
        return true;
    }

    /// <summary>
    /// Returns the currently active File Modifier. Cast to be done Null if not properly set : must be test
    /// IsNull after casting
    /// </summary>
    public GeneralModifier? FileModifier()
    {
        return _themodif;
    }

    /// <summary>
    /// Returns True if no modifier is currently set
    /// </summary>
    public bool IsForNone
    {
        get
        {
            return (_thenbent == 0);
        }
    }

    /// <summary>
    /// Returns True if the current modifier is to be applied to the whole model. Else, a restricted list of
    /// selected entities is defined, it can be exploited by the File Modifier
    /// </summary>
    public bool IsForAll
    {
        get
        {
            return _theapply.IsForAll;
        }
    }

    /// <summary>
    /// Returns the total count of selected entities
    /// </summary>
    public int EntityCount
    {
        get
        {
            return _thenbent;
        }
    }

    /// <summary>
    /// Starts an iteration on selected items. It takes into account IsForAll/IsForNone, by really iterating on
    /// all selected items.
    /// </summary>
    public void Start()
    {
        _thecurr = 1;
    }

    /// <summary>
    /// Returns True until the iteration has finished
    /// </summary>
    public bool More()
    {
        return (_thecurr <= _thenbent);
    }

    /// <summary>
    /// Advances the iteration
    /// </summary>
    public void Next()
    {
        _thecurr++;
    }

    /// <summary>
    /// Returns the current selected entity in the model
    /// </summary>
    public Transient? Value()
    {
        if (_thecurr < 1 || _thecurr > _thenbent)
        {
            throw new NoSuchObjectException("IFSelect_ContextWrite:Value");
        }
        int num = _theapply.ItemNum(_thecurr);
        return _themodel.Value(num);
    }

    /// <summary>
    /// Adds a Check to the CheckList. If it is empty, nothing is done If it concerns an Entity from the Model
    /// (by SetEntity) to which another Check is attached, it is merged to it. Else, it is added or merged as to
    /// GlobalCheck.
    /// </summary>
    public void AddCheck(FrameworkCheck? check)
    {
        if (check.FailCount + check.WarningCount == 0)
        {
            return;
        }
        Transient ent = check.Entity();
        int num = _themodel.Number(ent);
        if (num == 0 && ent is not null)
        {
            num = -1; // force enregistrement
        }
        _thecheck.Add(check, num);
    }

    /// <summary>
    /// Adds a Warning Message for an Entity from the Model If &lt;start&gt; is not an Entity from the model
    /// (e.g. the model itself) this message is added to Global Check.
    /// </summary>
    public void AddWarning(Transient? start, string mess, string orig = "")
    {
        _thecheck.CCheck(_themodel.Number(start)).AddWarning(mess, orig);
    }

    /// <summary>
    /// Adds a Fail Message for an Entity from the Model If &lt;start&gt; is not an Entity from the model (e.g.
    /// the model itself) this message is added to Global Check.
    /// </summary>
    public void AddFail(Transient? start, string mess, string orig = "")
    {
        _thecheck.CCheck(_themodel.Number(start)).AddFail(mess, orig);
    }

    /// <summary>
    /// Returns a Check given an Entity number (in the Model) by default a Global Check. Creates it the first
    /// time. It can then be acknowledged on the spot, in condition that the caller works by reference
    /// ("Interface_Check&amp; check = ...")
    /// </summary>
    public FrameworkCheck? CCheck(int num = 0)
    {
        FrameworkCheck ach = _thecheck.CCheck(num);
        if (num > 0 && num <= _themodel.EntityCount)
        {
            ach.SetEntity(_themodel.Value(num));
        }
        return ach;
    }

    /// <summary>
    /// Returns a Check attached to an Entity from the Model It can then be acknowledged on the spot, in
    /// condition that the caller works by reference ("Interface_Check&amp; check = ...")
    /// </summary>
    public FrameworkCheck? CCheck(Transient? ent)
    {
        int num = _themodel.Number(ent);
        if (num == 0)
        {
            num = -1; // force l enregistrement
        }
        FrameworkCheck ach = _thecheck.CCheck(num);
        ach.SetEntity(ent);
        return ach;
    }

    /// <summary>
    /// Returns the complete CheckList
    /// </summary>
    public CheckIterator CheckList()
    {
        return _thecheck;
    }

    private InterfaceModel? _themodel;
    private Protocol? _theproto;
    private string _thefile = "";
    private AppliedModifiers? _theapply;
    private HGraph? _thehgraf;
    private CheckIterator _thecheck = new CheckIterator();
    private int _thenbent;
    private int _thecurr;
    private GeneralModifier? _themodif;
    private readonly Stream? _stream;
}
