// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Controller.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Controller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Framework.Selection.ReturnStatus;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This class allows a general X-STEP engine to run generic functions on any interface norm, in the same
/// way. It includes the transfer operations. I.e. it gathers the already available general modules, the
/// engine has just to know it The important point is that a given X-STEP Controller is attached to a given
/// couple made of an Interface Norm (such as IGES-5.1) and an application data model (CasCade Shapes for
/// instance). Finally, Controller can be gathered in a general dictionary then retrieved later by a general
/// call (method Recorded) It does not manage the produced data, but the Actors make the link between the
/// norm and the application
/// </summary>
public abstract class Controller : Transient
{
    // OCCT's dictionary of the recorded controllers (a file static "listad"): process-wide, filled once by the
    // norms' initialisations (Record, AutoRecord) before the sessions select them, under a lock (decision 1222).
    private static readonly NCollection_DataMap<string, Controller> Listad = new NCollection_DataMap<string, Controller>();
    private static readonly object ListadLock = new object();

    private static ReturnStatus TransferFinder(ActorOfFinderProcess? actor, Finder mapper,
                                                        FinderProcess fp, InterfaceModel? model,
                                                        int modeTrans, ProgressRange progress)
    {
        if (actor is null)
        {
            return ReturnStatus.RetError;
        }
        if (model is null)
        {
            return ReturnStatus.RetError;
        }
        actor.ModeTrans() = modeTrans;
        fp.SetModel(model);
        fp.SetActor(actor);
        fp.Transfer(mapper, progress);

        ReturnStatus stat = ReturnStatus.RetFail;
        Binder binder = fp.Find(mapper);
        SimpleBinderOfTransient bindtr = null;
        while (binder is not null)
        {
            bindtr = (binder as SimpleBinderOfTransient);
            if (bindtr is not null)
            {
                Transient ent = bindtr.Result();
                if (ent is not null)
                {
                    stat = ReturnStatus.RetDone;
                    model.AddWithRefs(ent);
                }
            }
            binder = binder.NextResult();
        }
        return stat;
    }

    //=================================================================================================

    /// <summary>
    /// Changes names if a name is empty, the formerly set one remains Remark : Does not call Record or
    /// AutoRecord
    /// </summary>
    public void SetNames(string longName, string shortName)
    {
        if (longName is not null && CStringChar(longName, 0) != '\0')
        {
            _longName = longName;
        }
        if (shortName is not null && CStringChar(shortName, 0) != '\0')
        {
            _shortName = shortName;
        }
    }

    /// <summary>
    /// Records &lt;me&gt; is a general dictionary under Short and Long Names (see method Name)
    /// </summary>
    public void AutoRecord()
    {
        Record(Name(true));
        Record(Name(false));
    }

    /// <summary>
    /// Records &lt;me&gt; in a general dictionary under a name Error if &lt;name&gt; already used for another
    /// one
    /// </summary>
    /// <remarks>
    /// OCCT.NET: the dictionary is process-wide, as OCCT's: a norm records its controller once, in its
    /// initialisation, before sessions select it (decision 1222). Recording is thread-safe; recording again the
    /// same controller, or one of a type it derives from, changes nothing.
    /// </remarks>
    public void Record(string name)
    {
        lock (ListadLock)
        {
            if (Listad.IsBound(name))
            {
                Transient thisadapt = this;
                Transient newadapt = Listad.ChangeFind(name);
                if (newadapt.IsKind(thisadapt.DynamicType()))
                {
                    return;
                }
                if (!(thisadapt.IsKind(newadapt.DynamicType())) && thisadapt != newadapt)
                {
                    throw new DomainErrorException("XSControl_Controller : Record");
                }
            }
            Listad.Bind(name, this);
        }
    }

    /// <summary>
    /// Returns the Controller attached to a given name Returns a Null Handle if &lt;name&gt; is unknown
    /// </summary>
    public static Controller? Recorded(string name)
    {
        lock (ListadLock)
        {
            return Listad.Find(name, out Controller recorded) ? recorded : null;
        }
    }

    /// <summary>
    /// Returns a name, as given when initializing : rsc = False (D) : True Name attached to the Norm (long name)
    /// rsc = True : Name of the resource set (i.e. short name)
    /// </summary>
    public string Name(bool rsc = false)
    {
        return (rsc ? _shortName : _longName);
    }

    /// <summary>
    /// Returns the Protocol attached to the Norm (from field)
    /// </summary>
    public Protocol? Protocol()
    {
        return _adaptorProtocol;
    }

    /// <summary>
    /// Returns the SignType attached to the norm (from field) Returns the WorkLibrary attached to the Norm.
    /// Remark that it has to be in phase with the Protocol (read from field)
    /// </summary>
    public WorkLibrary? WorkLibrary()
    {
        return _adaptorLibrary;
    }

    /// <summary>
    /// Creates a new empty Model ready to receive data of the Norm Used to write data from Imagine to an
    /// interface file
    /// </summary>
    public abstract InterfaceModel? NewModel();

    /// <summary>
    /// Returns the Actor for Read attached to the pair (norm,appli) It can be adapted for data of the input
    /// Model, as required Can be read from field then adapted with Model as required
    /// </summary>
    public virtual ActorOfTransientProcess? ActorRead(InterfaceModel? model)
    {
        return _adaptorRead;
    }

    /// <summary>
    /// Returns the Actor for Write attached to the pair (norm,appli) Read from field. Can be redefined
    /// </summary>
    public virtual ActorOfFinderProcess? ActorWrite()
    {
        return _adaptorWrite;
    }

    /// <summary>
    /// Sets minimum and maximum values for modetrans (write) Erases formerly recorded bounds and values Actually
    /// only for shape Then, for each value a little help can be attached
    /// </summary>
    public void SetModeWrite(int modemin, int modemax, bool shape = true)
    {
        if (modemin > modemax)
        {
            _modeWriteShapeN = null;
            return;
        }
        _modeWriteShapeN =
            new NCollection_HArray1<TCollection_HAsciiString>(modemin, modemax);
    }

    /// <summary>
    /// Attaches a short line of help to a value of modetrans (write)
    /// </summary>
    public void SetModeWriteHelp(int modetrans, string help, bool shape = true)
    {
        if (_modeWriteShapeN is null)
        {
            return;
        }
        if (modetrans < _modeWriteShapeN.Lower() || modetrans > _modeWriteShapeN.Upper())
        {
            return;
        }
        TCollection_HAsciiString hl = new TCollection_HAsciiString(help);
        _modeWriteShapeN.SetValue(modetrans, hl);
    }

    /// <summary>
    /// Returns recorded min and max values for modetrans (write) Actually only for shapes Returns True if bounds
    /// are set, False else (then, free value)
    /// </summary>
    public bool ModeWriteBounds(out int modemin, out int modemax, bool shape = true)
    {
        modemin = modemax = 0;
        if (_modeWriteShapeN is null)
        {
            return false;
        }
        modemin = _modeWriteShapeN.Lower();
        modemax = _modeWriteShapeN.Upper();
        return true;
    }

    /// <summary>
    /// Tells if a value of &lt;modetrans&gt; is a good value(within bounds) Actually only for shapes
    /// </summary>
    public bool IsModeWrite(int modetrans, bool shape = true)
    {
        if (_modeWriteShapeN is null)
        {
            return true;
        }
        if (modetrans < _modeWriteShapeN.Lower())
        {
            return false;
        }
        if (modetrans > _modeWriteShapeN.Upper())
        {
            return false;
        }
        return true;
    }

    /// <summary>
    /// Returns the help line recorded for a value of modetrans empty if help not defined or not within bounds or
    /// if values are free
    /// </summary>
    public string ModeWriteHelp(int modetrans, bool shape = true)
    {
        if (_modeWriteShapeN is null)
        {
            return "";
        }
        if (modetrans < _modeWriteShapeN.Lower())
        {
            return "";
        }
        if (modetrans > _modeWriteShapeN.Upper())
        {
            return "";
        }
        TCollection_HAsciiString str = _modeWriteShapeN.Value(modetrans);
        if (str is null)
        {
            return "";
        }
        return str.ToCString();
    }

    /// <summary>
    /// Tells if &lt;obj&gt; (an application object) is a valid candidate for a transfer to a Model. By default,
    /// asks the ActorWrite if known (through a TransientMapper). Can be redefined
    /// </summary>
    public virtual bool RecognizeWriteTransient(Transient? obj, int modetrans = 0)
    {
        if (_adaptorWrite is null)
        {
            return false;
        }
        _adaptorWrite.ModeTrans() = modetrans;
        return _adaptorWrite.Recognize(new TransientMapper(obj));
    }

    /// <summary>
    /// Takes one Transient Object and transfers it to an InterfaceModel (already created, e.g. by NewModel)
    /// (result is recorded in the model by AddWithRefs) FP records produced results and checks Default uses
    /// ActorWrite; can be redefined as necessary Returned value is a status, as follows : 0 OK , 1 No Result , 2
    /// Fail (e.g. exception raised) -1 bad conditions , -2 bad model or null model For type of object not
    /// recognized : should return 1
    /// </summary>
    public virtual ReturnStatus TransferWriteTransient(Transient? obj, FinderProcess? fp, InterfaceModel? model, int modeTrans = 0, ProgressRange progress = default)
    {
        if (obj is null)
        {
            return ReturnStatus.RetVoid;
        }
        return TransferFinder(_adaptorWrite,
                                                    new TransientMapper(obj),
                                                    fp,
                                                    model,
                                                    modeTrans,
                                                    progress);
    }

    /// <summary>
    /// Tells if a shape is valid for a transfer to a model Asks the ActorWrite (through a ShapeMapper)
    /// </summary>
    public virtual bool RecognizeWriteShape(Shape shape, int modetrans = 0)
    {
        if (_adaptorWrite is null)
        {
            return false;
        }
        _adaptorWrite.ModeTrans() = modetrans;
        return _adaptorWrite.Recognize(new ShapeMapper(shape));
    }

    /// <summary>
    /// Takes one Shape and transfers it to an InterfaceModel (already created, e.g. by NewModel) Default uses
    /// ActorWrite; can be redefined as necessary Returned value is a status, as follows : Done OK , Void : No
    /// Result , Fail : Fail (e.g. exception) Error : bad conditions , bad model or null model
    /// </summary>
    public virtual ReturnStatus TransferWriteShape(Shape shape, FinderProcess? fp, InterfaceModel? model, int modetrans = 0, ProgressRange progress = default)
    {
        if (shape.IsNull)
        {
            return ReturnStatus.RetVoid;
        }

        ReturnStatus theReturnStat = TransferFinder(_adaptorWrite,
                                                                                                                  new ShapeMapper(shape),
                                                                                                                  fp,
                                                                                                                  model,
                                                                                                                  modetrans,
                                                                                                                  progress);
        return theReturnStat;
    }

    /// <summary>
    /// Records a Session Item, to be added for customisation of the Work Session. It must have a specific name.
    /// &lt;setapplied&gt; is used if &lt;item&gt; is a GeneralModifier, to decide If set to true, &lt;item&gt;
    /// will be applied to the hook list "send". Else, it is not applied to any hook list. Remark : this method
    /// is to be called at Create time, the recorded items will be used by Customise Warning : if &lt;name&gt;
    /// conflicts, the last recorded item is kept
    /// </summary>
    public void AddSessionItem(Transient? item, string name, bool toApply = false)
    {
        if (item is null || CStringChar(name, 0) == '\0')
        {
            return;
        }
        _adaptorSession.Bind(name, item);
        if (toApply && item.IsKind(typeof(GeneralModifier)))
        {
            _adaptorApplied.Append(item);
        }
    }

    /// <summary>
    /// Returns an item given its name to record in a Session If &lt;name&gt; is unknown, returns a Null Handle
    /// </summary>
    public Transient? SessionItem(string name)
    {
        Transient item = null;
        if (!_adaptorSession.IsEmpty())
        {
            item = _adaptorSession.Find(name);
        }
        return item;
    }

    /// <summary>
    /// Customises a WorkSession, by adding to it the recorded items (by AddSessionItem)
    /// </summary>
    /// <remarks>
    /// OCCT.NET: OCCT's <c>occ::handle&lt;XSControl_WorkSession&gt;&amp;</c> is never reassigned: the session is
    /// passed by value. The traced statics and the static editors are the session's parameter set's
    /// (<see cref="WorkSession.Statics"/>); the controller's own statics are declared in it before
    /// (<see cref="InitStatics"/>, called by XSControl_WorkSession.SetController). The recorded items are added in
    /// the order they were recorded, where OCCT iterates its map in hash order (decision 1223).
    /// </remarks>
    public virtual void Customise(ExchangeWorkSession ws)
    {
        StaticSet statics = ws.Statics();
        NCollection_DynamicArray<Transient> aParams = new NCollection_DynamicArray<Transient>();
        NCollection_DynamicArray<int> paramUses = new NCollection_DynamicArray<int>();
        for (int ip = 0; ip < _paramNames.Length(); ip++)
        {
            // OCCT's TraceStatic keeps the statics that exist when the controller is built
            Static val = statics.Static(_paramNames.Value(ip));
            if (val is null)
            {
                continue;
            }
            aParams.Append(val);
            paramUses.Append(_paramUses.Value(ip));
        }
        ws.SetParams(aParams, paramUses);

        // General
        if (!_adaptorSession.IsEmpty())
        {
            foreach (KeyValuePair<string, Transient> iter in _adaptorSession)
            {
                ws.AddNamedItem(iter.Key, iter.Value);
            }
        }

        if (ws.NamedItem("xst-model-all") is null)
        {

            SelectModelEntities sle = new SelectModelEntities();
            ws.AddNamedItem("xst-model-all", sle);

            SelectModelRoots slr = new SelectModelRoots();
            ws.AddNamedItem("xst-model-roots", slr);

            if (!string.Equals(ws.SelectedNorm(), "STEP", StringComparison.OrdinalIgnoreCase))
            {
                SelectForTransfer st1 = new SelectForTransfer();
                st1.SetInput(slr);
                st1.SetReader(ws.TransferReader());
                ws.AddNamedItem("xst-transferrable-roots", st1);
            }

            SelectForTransfer st2 = new SelectForTransfer();
            st2.SetInput(sle);
            st2.SetReader(ws.TransferReader());
            ws.AddNamedItem("xst-transferrable-all", st2);

            SignTransferStatus strs = new SignTransferStatus();
            strs.SetReader(ws.TransferReader());
            ws.AddNamedItem("xst-transfer-status", strs);

            ConnectedShapes scs = new ConnectedShapes();
            scs.SetReader(ws.TransferReader());
            ws.AddNamedItem("xst-connected-faces", scs);

            SelectionSignType stp = new SelectionSignType(false);
            ws.AddNamedItem("xst-long-type", stp);

            SelectionSignType stc = new SelectionSignType(true);
            ws.AddNamedItem("xst-type", stc);

            ws.AddNamedItem("xst-ancestor-type", new SignAncestor());
            ws.AddNamedItem("xst-types", new SignCounter(stp, false, true));
            ws.AddNamedItem("xst-category", new SignCategory());
            ws.AddNamedItem("xst-validity", new SignValidity());

            DispPerOne dispone = new DispPerOne();
            dispone.SetFinalSelection(slr);
            ws.AddNamedItem("xst-disp-one", dispone);

            DispPerCount dispcount = new DispPerCount();
            IntParam intcount = new IntParam();
            intcount.SetValue(5);
            dispcount.SetCount(intcount);
            dispcount.SetFinalSelection(slr);
            ws.AddNamedItem("xst-disp-count", dispcount);

            DispPerFiles dispfiles = new DispPerFiles();
            IntParam intfiles = new IntParam();
            intfiles.SetValue(10);
            dispfiles.SetCount(intfiles);
            dispfiles.SetFinalSelection(slr);
            ws.AddNamedItem("xst-disp-files", dispfiles);

            DispPerSignature dispsign = new DispPerSignature();
            dispsign.SetSignCounter(new SignCounter(stc));
            dispsign.SetFinalSelection(slr);
            ws.AddNamedItem("xst-disp-sign", dispsign);

            // Not used directly but useful anyway
            ws.AddNamedItem("xst-pointed", new SelectPointed());
            ws.AddNamedItem("xst-sharing", new SelectSharing());
            ws.AddNamedItem("xst-shared", new SelectShared());
            ws.AddNamedItem("xst-nb-selected", new GraphCounter());

            // szv:mySignType = stp;
            ws.SetSignType(stp);
        }

        // Applied Modifiers
        int i, nb = _adaptorApplied.Length();
        for (i = 1; i <= nb; i++)
        {
            Transient anitem = _adaptorApplied.Value(i);
            TCollection_HAsciiString name = ws.Name(anitem);
            ws.SetAppliedModifier((anitem as GeneralModifier), ws.ShareOut());
        }

        // Editors of Parameters
        // Here for the specific manufacturers of controllers could create the
        // Parameters: So wait here

        NCollection_HSequence<TCollection_HAsciiString> listat = statics.Items();
        ParamEditor paramed = ParamEditor.StaticEditor(statics, listat, "All Static Parameters");
        ws.AddNamedItem("xst-static-params-edit", paramed);
        EditForm paramform = paramed.Form(false);
        ws.AddNamedItem("xst-static-params", paramform);
    }

    public NCollection_DataMap<string, Transient> AdaptorSession()
    {
        return _adaptorSession;
    }

    /// <summary>
    /// Initializing with names &lt;theLongName&gt; is for the complete, official, long name &lt;theShortName&gt;
    /// is for the short name used for resources
    /// </summary>
    protected Controller(string longName, string shortName)
    {
        _shortName = shortName;
        _longName = longName;
        // Standard parameters: OCCT calls Interface_Static::Standards() here; the port declares them in each
        // session's parameter set (InitStatics)
        TraceStatic("read.precision.mode", 5);
        TraceStatic("read.precision.val", 5);
        TraceStatic("write.precision.mode", 6);
        TraceStatic("write.precision.val", 6);
    }

    /// <summary>
    /// Records the name of a Static to be traced for a given use
    /// </summary>
    /// <remarks>
    /// OCCT.NET: the name is kept; the static is looked up in the session's parameter set when the controller
    /// customises it (OCCT keeps the process-wide static, if it exists when the controller is built).
    /// </remarks>
    protected void TraceStatic(string name, int use)
    {
        _paramNames.Append(name);
        _paramUses.Append(use);
    }

    /// <summary>
    /// Declares the statics of the norm in a parameter set (OCCT.NET: OCCT's controllers declare them in the
    /// process-wide statics when they are built). The base declares the standard ones
    /// (<see cref="StaticSet.Standards"/>); a norm's controller adds its own, once per set
    /// (<see cref="StaticSet.StartInit"/>). Called by XSControl_WorkSession.SetController before
    /// <see cref="Customise"/>, and by the readers and writers that make a parameter set for the norm.
    /// </summary>
    public virtual void InitStatics(StaticSet statics)
    {
        statics.Standards();
    }

    protected string _shortName = "";
    protected string _longName = "";
    protected WorkLibrary? _adaptorLibrary;
    protected Protocol? _adaptorProtocol;
    protected ActorOfTransientProcess? _adaptorRead;
    protected ActorOfFinderProcess? _adaptorWrite;
    protected NCollection_DataMap<string, Transient> _adaptorSession = new NCollection_DataMap<string, Transient>();
    private NCollection_Sequence<Transient> _adaptorApplied = new NCollection_Sequence<Transient>();
    private NCollection_DynamicArray<string> _paramNames = new NCollection_DynamicArray<string>(); // OCCT: myParams, the statics
    private NCollection_DynamicArray<int> _paramUses = new NCollection_DynamicArray<int>();
    private NCollection_HArray1<TCollection_HAsciiString>? _modeWriteShapeN;
}
