// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Controller.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_Controller.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using static Occt.IFSelect_ReturnStatus;

namespace Occt;

/// <summary>
/// This class allows a general X-STEP engine to run generic functions on any interface norm, in the same
/// way. It includes the transfer operations. I.e. it gathers the already available general modules, the
/// engine has just to know it The important point is that a given X-STEP Controller is attached to a given
/// couple made of an Interface Norm (such as IGES-5.1) and an application data model (CasCade Shapes for
/// instance). Finally, Controller can be gathered in a general dictionary then retrieved later by a general
/// call (method Recorded) It does not manage the produced data, but the Actors make the link between the
/// norm and the application
/// </summary>
public abstract class XSControl_Controller : Transient
{
    // OCCT's dictionary of the recorded controllers (a file static "listad"): process-wide, filled once by the
    // norms' initialisations (Record, AutoRecord) before the sessions select them, under a lock (decision 1222).
    private static readonly NCollection_DataMap<string, XSControl_Controller> listad = new NCollection_DataMap<string, XSControl_Controller>();
    private static readonly object listadLock = new object();

    private static IFSelect_ReturnStatus TransferFinder(Transfer_ActorOfFinderProcess? theActor, Transfer_Finder theMapper,
                                                        Transfer_FinderProcess theFP, Interface_InterfaceModel? theModel,
                                                        int theModeTrans, ProgressRange theProgress)
    {
        if (theActor is null)
        {
            return IFSelect_RetError;
        }
        if (theModel is null)
        {
            return IFSelect_RetError;
        }
        theActor.ModeTrans() = theModeTrans;
        theFP.SetModel(theModel);
        theFP.SetActor(theActor);
        theFP.Transfer(theMapper, theProgress);

        IFSelect_ReturnStatus stat = IFSelect_RetFail;
        Transfer_Binder binder = theFP.Find(theMapper);
        Transfer_SimpleBinderOfTransient bindtr = null;
        while (binder is not null)
        {
            bindtr = (binder as Transfer_SimpleBinderOfTransient);
            if (bindtr is not null)
            {
                Transient ent = bindtr.Result();
                if (ent is not null)
                {
                    stat = IFSelect_RetDone;
                    theModel.AddWithRefs(ent);
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
    public void SetNames(string theLongName, string theShortName)
    {
        if (theLongName is not null && CStringChar(theLongName, 0) != '\0')
        {
            myLongName = theLongName;
        }
        if (theShortName is not null && CStringChar(theShortName, 0) != '\0')
        {
            myShortName = theShortName;
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
    public void Record(string theName)
    {
        lock (listadLock)
        {
            if (listad.IsBound(theName))
            {
                Transient thisadapt = this;
                Transient newadapt = listad.ChangeFind(theName);
                if (newadapt.IsKind(thisadapt.DynamicType()))
                {
                    return;
                }
                if (!(thisadapt.IsKind(newadapt.DynamicType())) && thisadapt != newadapt)
                {
                    throw new DomainErrorException("XSControl_Controller : Record");
                }
            }
            listad.Bind(theName, this);
        }
    }

    /// <summary>
    /// Returns the Controller attached to a given name Returns a Null Handle if &lt;name&gt; is unknown
    /// </summary>
    public static XSControl_Controller? Recorded(string theName)
    {
        lock (listadLock)
        {
            return listad.Find(theName, out XSControl_Controller recorded) ? recorded : null;
        }
    }

    /// <summary>
    /// Returns a name, as given when initializing : rsc = False (D) : True Name attached to the Norm (long name)
    /// rsc = True : Name of the resource set (i.e. short name)
    /// </summary>
    public string Name(bool rsc = false)
    {
        return (rsc ? myShortName : myLongName);
    }

    /// <summary>
    /// Returns the Protocol attached to the Norm (from field)
    /// </summary>
    public Interface_Protocol? Protocol()
    {
        return myAdaptorProtocol;
    }

    /// <summary>
    /// Returns the SignType attached to the norm (from field) Returns the WorkLibrary attached to the Norm.
    /// Remark that it has to be in phase with the Protocol (read from field)
    /// </summary>
    public IFSelect_WorkLibrary? WorkLibrary()
    {
        return myAdaptorLibrary;
    }

    /// <summary>
    /// Creates a new empty Model ready to receive data of the Norm Used to write data from Imagine to an
    /// interface file
    /// </summary>
    public abstract Interface_InterfaceModel? NewModel();

    /// <summary>
    /// Returns the Actor for Read attached to the pair (norm,appli) It can be adapted for data of the input
    /// Model, as required Can be read from field then adapted with Model as required
    /// </summary>
    public virtual Transfer_ActorOfTransientProcess? ActorRead(Interface_InterfaceModel? model)
    {
        return myAdaptorRead;
    }

    /// <summary>
    /// Returns the Actor for Write attached to the pair (norm,appli) Read from field. Can be redefined
    /// </summary>
    public virtual Transfer_ActorOfFinderProcess? ActorWrite()
    {
        return myAdaptorWrite;
    }

    /// <summary>
    /// Sets minimum and maximum values for modetrans (write) Erases formerly recorded bounds and values Actually
    /// only for shape Then, for each value a little help can be attached
    /// </summary>
    public void SetModeWrite(int modemin, int modemax, bool shape = true)
    {
        if (modemin > modemax)
        {
            myModeWriteShapeN = null;
            return;
        }
        myModeWriteShapeN =
            new NCollection_HArray1<TCollection_HAsciiString>(modemin, modemax);
    }

    /// <summary>
    /// Attaches a short line of help to a value of modetrans (write)
    /// </summary>
    public void SetModeWriteHelp(int modetrans, string help, bool shape = true)
    {
        if (myModeWriteShapeN is null)
        {
            return;
        }
        if (modetrans < myModeWriteShapeN.Lower() || modetrans > myModeWriteShapeN.Upper())
        {
            return;
        }
        TCollection_HAsciiString hl = new TCollection_HAsciiString(help);
        myModeWriteShapeN.SetValue(modetrans, hl);
    }

    /// <summary>
    /// Returns recorded min and max values for modetrans (write) Actually only for shapes Returns True if bounds
    /// are set, False else (then, free value)
    /// </summary>
    public bool ModeWriteBounds(out int modemin, out int modemax, bool shape = true)
    {
        modemin = modemax = 0;
        if (myModeWriteShapeN is null)
        {
            return false;
        }
        modemin = myModeWriteShapeN.Lower();
        modemax = myModeWriteShapeN.Upper();
        return true;
    }

    /// <summary>
    /// Tells if a value of &lt;modetrans&gt; is a good value(within bounds) Actually only for shapes
    /// </summary>
    public bool IsModeWrite(int modetrans, bool shape = true)
    {
        if (myModeWriteShapeN is null)
        {
            return true;
        }
        if (modetrans < myModeWriteShapeN.Lower())
        {
            return false;
        }
        if (modetrans > myModeWriteShapeN.Upper())
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
        if (myModeWriteShapeN is null)
        {
            return "";
        }
        if (modetrans < myModeWriteShapeN.Lower())
        {
            return "";
        }
        if (modetrans > myModeWriteShapeN.Upper())
        {
            return "";
        }
        TCollection_HAsciiString str = myModeWriteShapeN.Value(modetrans);
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
        if (myAdaptorWrite is null)
        {
            return false;
        }
        myAdaptorWrite.ModeTrans() = modetrans;
        return myAdaptorWrite.Recognize(new Transfer_TransientMapper(obj));
    }

    /// <summary>
    /// Takes one Transient Object and transfers it to an InterfaceModel (already created, e.g. by NewModel)
    /// (result is recorded in the model by AddWithRefs) FP records produced results and checks Default uses
    /// ActorWrite; can be redefined as necessary Returned value is a status, as follows : 0 OK , 1 No Result , 2
    /// Fail (e.g. exception raised) -1 bad conditions , -2 bad model or null model For type of object not
    /// recognized : should return 1
    /// </summary>
    public virtual IFSelect_ReturnStatus TransferWriteTransient(Transient? theObj, Transfer_FinderProcess? theFP, Interface_InterfaceModel? theModel, int theModeTrans = 0, ProgressRange theProgress = default)
    {
        if (theObj is null)
        {
            return IFSelect_RetVoid;
        }
        return TransferFinder(myAdaptorWrite,
                                                    new Transfer_TransientMapper(theObj),
                                                    theFP,
                                                    theModel,
                                                    theModeTrans,
                                                    theProgress);
    }

    /// <summary>
    /// Tells if a shape is valid for a transfer to a model Asks the ActorWrite (through a ShapeMapper)
    /// </summary>
    public virtual bool RecognizeWriteShape(TopoDS_Shape shape, int modetrans = 0)
    {
        if (myAdaptorWrite is null)
        {
            return false;
        }
        myAdaptorWrite.ModeTrans() = modetrans;
        return myAdaptorWrite.Recognize(new TransferBRep_ShapeMapper(shape));
    }

    /// <summary>
    /// Takes one Shape and transfers it to an InterfaceModel (already created, e.g. by NewModel) Default uses
    /// ActorWrite; can be redefined as necessary Returned value is a status, as follows : Done OK , Void : No
    /// Result , Fail : Fail (e.g. exception) Error : bad conditions , bad model or null model
    /// </summary>
    public virtual IFSelect_ReturnStatus TransferWriteShape(TopoDS_Shape shape, Transfer_FinderProcess? FP, Interface_InterfaceModel? model, int modetrans = 0, ProgressRange theProgress = default)
    {
        if (shape.IsNull())
        {
            return IFSelect_RetVoid;
        }

        IFSelect_ReturnStatus theReturnStat = TransferFinder(myAdaptorWrite,
                                                                                                                  new TransferBRep_ShapeMapper(shape),
                                                                                                                  FP,
                                                                                                                  model,
                                                                                                                  modetrans,
                                                                                                                  theProgress);
        return theReturnStat;
    }

    /// <summary>
    /// Records a Session Item, to be added for customisation of the Work Session. It must have a specific name.
    /// &lt;setapplied&gt; is used if &lt;item&gt; is a GeneralModifier, to decide If set to true, &lt;item&gt;
    /// will be applied to the hook list "send". Else, it is not applied to any hook list. Remark : this method
    /// is to be called at Create time, the recorded items will be used by Customise Warning : if &lt;name&gt;
    /// conflicts, the last recorded item is kept
    /// </summary>
    public void AddSessionItem(Transient? theItem, string theName, bool toApply = false)
    {
        if (theItem is null || CStringChar(theName, 0) == '\0')
        {
            return;
        }
        myAdaptorSession.Bind(theName, theItem);
        if (toApply && theItem.IsKind(typeof(IFSelect_GeneralModifier)))
        {
            myAdaptorApplied.Append(theItem);
        }
    }

    /// <summary>
    /// Returns an item given its name to record in a Session If &lt;name&gt; is unknown, returns a Null Handle
    /// </summary>
    public Transient? SessionItem(string theName)
    {
        Transient item = null;
        if (!myAdaptorSession.IsEmpty())
        {
            item = myAdaptorSession.Find(theName);
        }
        return item;
    }

    /// <summary>
    /// Customises a WorkSession, by adding to it the recorded items (by AddSessionItem)
    /// </summary>
    /// <remarks>
    /// OCCT.NET: OCCT's <c>occ::handle&lt;XSControl_WorkSession&gt;&amp;</c> is never reassigned: the session is
    /// passed by value. The traced statics and the static editors are the session's parameter set's
    /// (<see cref="IFSelect_WorkSession.Statics"/>); the controller's own statics are declared in it before
    /// (<see cref="InitStatics"/>, called by XSControl_WorkSession.SetController). The recorded items are added in
    /// the order they were recorded, where OCCT iterates its map in hash order (decision 1223).
    /// </remarks>
    public virtual void Customise(XSControl_WorkSession WS)
    {
        Interface_StaticSet aStatics = WS.Statics();
        NCollection_DynamicArray<Transient> aParams = new NCollection_DynamicArray<Transient>();
        NCollection_DynamicArray<int> aParamUses = new NCollection_DynamicArray<int>();
        for (int ip = 0; ip < myParamNames.Length(); ip++)
        {
            // OCCT's TraceStatic keeps the statics that exist when the controller is built
            Interface_Static val = aStatics.Static(myParamNames.Value(ip));
            if (val is null)
            {
                continue;
            }
            aParams.Append(val);
            aParamUses.Append(myParamUses.Value(ip));
        }
        WS.SetParams(aParams, aParamUses);

        // General
        if (!myAdaptorSession.IsEmpty())
        {
            foreach (KeyValuePair<string, Transient> iter in myAdaptorSession)
            {
                WS.AddNamedItem(iter.Key, iter.Value);
            }
        }

        if (WS.NamedItem("xst-model-all") is null)
        {

            IFSelect_SelectModelEntities sle = new IFSelect_SelectModelEntities();
            WS.AddNamedItem("xst-model-all", sle);

            IFSelect_SelectModelRoots slr = new IFSelect_SelectModelRoots();
            WS.AddNamedItem("xst-model-roots", slr);

            if (!string.Equals(WS.SelectedNorm(), "STEP", StringComparison.OrdinalIgnoreCase))
            {
                XSControl_SelectForTransfer st1 = new XSControl_SelectForTransfer();
                st1.SetInput(slr);
                st1.SetReader(WS.TransferReader());
                WS.AddNamedItem("xst-transferrable-roots", st1);
            }

            XSControl_SelectForTransfer st2 = new XSControl_SelectForTransfer();
            st2.SetInput(sle);
            st2.SetReader(WS.TransferReader());
            WS.AddNamedItem("xst-transferrable-all", st2);

            XSControl_SignTransferStatus strs = new XSControl_SignTransferStatus();
            strs.SetReader(WS.TransferReader());
            WS.AddNamedItem("xst-transfer-status", strs);

            XSControl_ConnectedShapes scs = new XSControl_ConnectedShapes();
            scs.SetReader(WS.TransferReader());
            WS.AddNamedItem("xst-connected-faces", scs);

            IFSelect_SignType stp = new IFSelect_SignType(false);
            WS.AddNamedItem("xst-long-type", stp);

            IFSelect_SignType stc = new IFSelect_SignType(true);
            WS.AddNamedItem("xst-type", stc);

            WS.AddNamedItem("xst-ancestor-type", new IFSelect_SignAncestor());
            WS.AddNamedItem("xst-types", new IFSelect_SignCounter(stp, false, true));
            WS.AddNamedItem("xst-category", new IFSelect_SignCategory());
            WS.AddNamedItem("xst-validity", new IFSelect_SignValidity());

            IFSelect_DispPerOne dispone = new IFSelect_DispPerOne();
            dispone.SetFinalSelection(slr);
            WS.AddNamedItem("xst-disp-one", dispone);

            IFSelect_DispPerCount dispcount = new IFSelect_DispPerCount();
            IFSelect_IntParam intcount = new IFSelect_IntParam();
            intcount.SetValue(5);
            dispcount.SetCount(intcount);
            dispcount.SetFinalSelection(slr);
            WS.AddNamedItem("xst-disp-count", dispcount);

            IFSelect_DispPerFiles dispfiles = new IFSelect_DispPerFiles();
            IFSelect_IntParam intfiles = new IFSelect_IntParam();
            intfiles.SetValue(10);
            dispfiles.SetCount(intfiles);
            dispfiles.SetFinalSelection(slr);
            WS.AddNamedItem("xst-disp-files", dispfiles);

            IFSelect_DispPerSignature dispsign = new IFSelect_DispPerSignature();
            dispsign.SetSignCounter(new IFSelect_SignCounter(stc));
            dispsign.SetFinalSelection(slr);
            WS.AddNamedItem("xst-disp-sign", dispsign);

            // Not used directly but useful anyway
            WS.AddNamedItem("xst-pointed", new IFSelect_SelectPointed());
            WS.AddNamedItem("xst-sharing", new IFSelect_SelectSharing());
            WS.AddNamedItem("xst-shared", new IFSelect_SelectShared());
            WS.AddNamedItem("xst-nb-selected", new IFSelect_GraphCounter());

            // szv:mySignType = stp;
            WS.SetSignType(stp);
        }

        // Applied Modifiers
        int i, nb = myAdaptorApplied.Length();
        for (i = 1; i <= nb; i++)
        {
            Transient anitem = myAdaptorApplied.Value(i);
            TCollection_HAsciiString name = WS.Name(anitem);
            WS.SetAppliedModifier((anitem as IFSelect_GeneralModifier), WS.ShareOut());
        }

        // Editors of Parameters
        // Here for the specific manufacturers of controllers could create the
        // Parameters: So wait here

        NCollection_HSequence<TCollection_HAsciiString> listat = aStatics.Items();
        IFSelect_ParamEditor paramed = IFSelect_ParamEditor.StaticEditor(aStatics, listat, "All Static Parameters");
        WS.AddNamedItem("xst-static-params-edit", paramed);
        IFSelect_EditForm paramform = paramed.Form(false);
        WS.AddNamedItem("xst-static-params", paramform);
    }

    public NCollection_DataMap<string, Transient> AdaptorSession()
    {
        return myAdaptorSession;
    }

    /// <summary>
    /// Initializing with names &lt;theLongName&gt; is for the complete, official, long name &lt;theShortName&gt;
    /// is for the short name used for resources
    /// </summary>
    protected XSControl_Controller(string theLongName, string theShortName)
    {
        myShortName = theShortName;
        myLongName = theLongName;
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
    protected void TraceStatic(string theName, int theUse)
    {
        myParamNames.Append(theName);
        myParamUses.Append(theUse);
    }

    /// <summary>
    /// Declares the statics of the norm in a parameter set (OCCT.NET: OCCT's controllers declare them in the
    /// process-wide statics when they are built). The base declares the standard ones
    /// (<see cref="Interface_StaticSet.Standards"/>); a norm's controller adds its own, once per set
    /// (<see cref="Interface_StaticSet.StartInit"/>). Called by XSControl_WorkSession.SetController before
    /// <see cref="Customise"/>, and by the readers and writers that make a parameter set for the norm.
    /// </summary>
    public virtual void InitStatics(Interface_StaticSet theStatics)
    {
        theStatics.Standards();
    }

    protected string myShortName = "";
    protected string myLongName = "";
    protected IFSelect_WorkLibrary? myAdaptorLibrary;
    protected Interface_Protocol? myAdaptorProtocol;
    protected Transfer_ActorOfTransientProcess? myAdaptorRead;
    protected Transfer_ActorOfFinderProcess? myAdaptorWrite;
    protected NCollection_DataMap<string, Transient> myAdaptorSession = new NCollection_DataMap<string, Transient>();
    private NCollection_Sequence<Transient> myAdaptorApplied = new NCollection_Sequence<Transient>();
    private NCollection_DynamicArray<string> myParamNames = new NCollection_DynamicArray<string>(); // OCCT: myParams, the statics
    private NCollection_DynamicArray<int> myParamUses = new NCollection_DynamicArray<int>();
    private NCollection_HArray1<TCollection_HAsciiString>? myModeWriteShapeN;
}
