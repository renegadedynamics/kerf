// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_WorkSession.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_WorkSession.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using static Occt.IFSelect_ReturnStatus;

namespace Occt;

/// <summary>
/// This WorkSession completes the basic one, by adding : - use of Controller, with norm selection... -
/// management of transfers (both ways) with auxiliary classes TransferReader and TransferWriter -&gt; these
/// transfers may work with a Context List : its items are given by the user, according to the transfer to be
/// i.e. it is interpreted by the Actors Each item is accessed by a Name
/// </summary>
public class XSControl_WorkSession : IFSelect_WorkSession
{
    // OCCT's process-wide mutex (GetGlobalMutex) around SelectNorm, NewModel and TransferWriteShape: kept, as the
    // controllers (and their actors) are shared by all sessions (decision 1222)
    private static readonly object WS_GLOBAL_MUTEX = new object();

    public XSControl_WorkSession()
    {
        myTransferReader = new XSControl_TransferReader();
        myTransferWriter = new XSControl_TransferWriter();
        myVars = new XSControl_Vars();
        myTransferReader.SetStatics(Statics());
    }

    /// <summary>
    /// Creates a session working with a given parameter set (OCCT.NET: for instance an interpreter's, or a
    /// reader's copy; decision 1181).
    /// </summary>
    public XSControl_WorkSession(Interface_StaticSet theStatics) : this()
    {
        SetStatics(theStatics);
    }

    /// <summary>
    /// Gives the session a parameter set (OCCT.NET): its transfer reader reads it too, and the controller's
    /// statics are declared in it.
    /// </summary>
    public override void SetStatics(Interface_StaticSet theStatics)
    {
        base.SetStatics(theStatics);
        myController?.InitStatics(Statics());
        myTransferReader?.SetStatics(Statics());
    }

    // OCCT's destructor calls ClearBinders, which empties the finder process and breaks the chains of binders
    // (memory release): left out with ClearBinders, the garbage collector releases them

    /// <summary>
    /// In addition to basic ClearData, clears Transfer and Management for interactive use, for mode = 0,1,2 and
    /// over 4 Plus : mode = 5 to clear Transfers (both ways) only mode = 6 to clear enforced results mode = 7 to
    /// clear transfers, results
    /// </summary>
    public override void ClearData(int mode)
    {
        // 1-2-3-4 : standard IFSelect
        if (mode >= 1 && mode <= 4)
        {
            base.ClearData(mode);
        }

        // 5 : Transfers only
        // 6 : Forced results only
        // 7 : Management, including all transfers (forced/calculated), views

        if (mode == 5 || mode == 7)
        {
            myTransferReader.Clear(-1);
            myTransferWriter.Clear(-1);
        }
        if (mode == 6 && myTransferReader is not null)
        {
            myTransferReader.Clear(1);
        }
        myTransferReader.SetGraph(HGraph());
    }

    /// <summary>
    /// Selects a Norm defined by its name. A Norm is described and handled by a Controller Returns True if done,
    /// False if &lt;normname&gt; is unknown The current Profile for this Norm is taken.
    /// </summary>
    public bool SelectNorm(string normname)
    {
        lock (WS_GLOBAL_MUTEX)
        {
            return SelectNormLocked(normname);
        }
    }

    private bool SelectNormLocked(string normname)
    {
        // Old norm and results
        myTransferReader.Clear(-1);
        //  ????  Strictly speaking, cleanup to do in XWS: remove the items
        //        ( at the limit, why not, redo XWS entirely)

        XSControl_Controller newadapt = XSControl_Controller.Recorded(normname);
        if (newadapt is null)
        {
            return false;
        }
        if (newadapt == myController)
        {
            return true;
        }
        SetController(newadapt);
        return true;
    }

    /// <summary>
    /// Selects a Norm defined by its Controller itself
    /// </summary>
    public void SetController(XSControl_Controller? ctl)
    {
        myController = ctl;

        // OCCT.NET: the controller declares its statics in the session's parameter set (OCCT: in the process-wide
        // statics, when it is built)
        myController.InitStatics(Statics());

        SetLibrary(myController.WorkLibrary());
        SetProtocol(myController.Protocol());

        ClearItems();
        ClearFinalModifiers();
        ClearShareOut(false);
        ClearFile();

        // Set worksession parameters from the controller
        XSControl_WorkSession aWorkSession = this;
        myController.Customise(aWorkSession);

        myTransferReader.SetController(myController);
        myTransferWriter.SetController(myController);
    }

    /// <summary>
    /// Returns the name of the last Selected Norm. If none is defined, returns an empty string By default,
    /// returns the complete name of the norm If &lt;rsc&gt; is True, returns the short name used for resource
    /// </summary>
    public string SelectedNorm(bool rsc = false)
    {
        // JR/Hp :
        string astr = (myController is null ? "" : myController.Name(rsc));
        return astr;
    }

    /// <summary>
    /// Returns the norm controller itself
    /// </summary>
    public XSControl_Controller? NormAdaptor()
    {
        return myController;
    }

    /// <summary>
    /// Returns the current Context List, Null if not defined The Context is given to the TransientProcess for
    /// TransferRead
    /// </summary>
    public NCollection_DataMap<string, Standard_Transient> Context()
    {
        return myContext;
    }

    /// <summary>
    /// Sets the current Context List, as a whole Sets it to the TransferReader
    /// </summary>
    public void SetAllContext(NCollection_DataMap<string, Standard_Transient> context)
    {
        XSControl_TransferReader.AssignContext(myContext, context); // C++'s map assignments: copies
        XSControl_TransferReader.AssignContext(myTransferReader.Context(), context);
    }

    /// <summary>
    /// Clears the whole current Context (nullifies it)
    /// </summary>
    public void ClearContext()
    {
        myContext.Clear();
        myTransferReader.Context().Clear();
    }

    /// <summary>
    /// Prints the transfer status of a transferred item, as being the Mapped n0 &lt;num&gt;, from MapWriter if
    /// &lt;wri&gt; is True, or from MapReader if &lt;wri&gt; is False Returns True when done, False else (i.e.
    /// num out of range)
    /// </summary>
    public bool PrintTransferStatus(int num, bool wri, TextWriter S)
    {
        Transfer_FinderProcess FP = myTransferWriter.FinderProcess();
        Transfer_TransientProcess TP = myTransferReader.TransientProcess();

        Transfer_Binder binder = null;
        Transfer_Finder finder = null;
        Standard_Transient ent = null;

        //   ***   WRITE  ***
        if (wri)
        {
            if (FP is null)
            {
                return false;
            }
            if (num == 0)
            {
                return false;
            }

            int ne = 0, nr = 0, max = FP.NbMapped(), maxr = FP.NbRoots();
            if (num > 0)
            {
                if (num > max)
                {
                    return false;
                }
                ne = num;
                finder = FP.Mapped(ne);
                nr = FP.RootIndex(finder);
            }
            else if (num < 0)
            {
                nr = -num;
                if (nr > maxr)
                {
                    return false;
                }
                finder = FP.Root(nr);
                ne = FP.MapIndex(finder);
            }

            S.Write("Transfer Write item n0.");
            S.Write(ne);
            S.Write(" of ");
            S.Write(max);
            if (nr > 0)
            {
                S.Write("  ** Transfer Root n0.");
                S.Write(ne);
            }
            S.Write('\n');
            ent = FP.FindTransient(finder);
            S.Write(" -> Type ");
            S.Write(finder.DynamicType().Name);
            S.Write('\n');
            FP.StartTrace(binder, finder, 0, 0); // pb sout/S
            if (ent is not null)
            {
                S.Write(" ** Transient Result, type ");
                S.Write(ent.DynamicType().Name);
                Interface_InterfaceModel model = Model();
                if (model is not null)
                {
                    S.Write(" In output Model, Entity ");
                    model.Print(ent, S);
                }
                S.Write('\n');
            }
        }

        //    ***   READ   ***
        else
        {
            if (TP is null)
            {
                return false;
            }
            Interface_InterfaceModel model = TP.Model();
            if (model is null)
            {
                Console.Out.Write("No Model");
                Console.Out.Write('\n');
            }
            else if (model != Model())
            {
                Console.Out.Write("Model different from the session");
                Console.Out.Write('\n');
            }
            if (num == 0)
            {
                return false;
            }

            int ne = 0, nr = 0, max = TP.NbMapped(), maxr = TP.NbRoots();
            if (num > 0)
            {
                if (num > max)
                {
                    return false;
                }
                ne = num;
                ent = TP.Mapped(ne);
                nr = TP.RootIndex(finder);
            }
            else if (num < 0)
            {
                nr = -num;
                if (nr > maxr)
                {
                    return false;
                }
                ent = TP.Root(nr);
                ne = TP.MapIndex(ent);
            }

            S.Write("Transfer Read item n0.");
            S.Write(ne);
            S.Write(" of ");
            S.Write(max);
            if (nr > 0)
            {
                S.Write("  ** Transfer Root n0.");
                S.Write(ne);
            }
            S.Write('\n');
            if (model is not null)
            {
                S.Write(" In Model, Entity ");
                model.Print(ent, S);
            }
            binder = TP.MapItem(ne);
            S.Write('\n');
            TP.StartTrace(binder, ent, 0, 0);
        }

        //   ***   CHECK (common READ+WRITE)   ***
        if (binder is not null)
        {
            Interface_Check ch = binder.Check();
            int i, nbw = ch.NbWarnings(), nbf = ch.NbFails();
            if (nbw > 0)
            {
                S.Write(" - Warnings : ");
                S.Write(nbw);
                S.Write(" :\n");
                for (i = 1; i <= nbw; i++)
                {
                    S.Write(ch.CWarning(i));
                    S.Write('\n');
                }
            }
            if (nbf > 0)
            {
                S.Write(" - Fails : ");
                S.Write(nbf);
                S.Write(" :\n");
                for (i = 1; i <= nbf; i++)
                {
                    S.Write(ch.CFail(i));
                    S.Write('\n');
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Sets a Transfer Reader, by internal ways, according mode : 0 recreates it clear 1 clears it (does not
    /// recreate) 2 aligns Roots of TransientProcess from final Results 3 aligns final Results from Roots of
    /// TransientProcess 4 begins a new transfer (by BeginTransfer) 5 recreates TransferReader then begins a new
    /// transfer
    /// </summary>
    public void InitTransferReader(int mode)
    {
        if (mode == 0 || mode == 5)
        {
            myTransferReader.Clear(-1); // full clear
        }
        if (myTransferReader is null)
        {
            SetTransferReader(new XSControl_TransferReader());
        }
        else
        {
            SetTransferReader(myTransferReader);
        }

        // mode = 0 done by SetTransferReader following Nullify
        if (mode == 1)
        {
            if (myTransferReader is not null)
            {
                myTransferReader.Clear(-1);
            }
            else
            {
                SetTransferReader(new XSControl_TransferReader());
            }
        }
        if (mode == 2)
        {
            Transfer_TransientProcess TP = myTransferReader.TransientProcess();
            if (TP is null)
            {
                TP = new Transfer_TransientProcess();
                myTransferReader.SetTransientProcess(TP);
                TP.SetGraph(HGraph());
            }
            NCollection_HSequence<Standard_Transient> lis =
                myTransferReader.RecordedList();
            int i, nb = lis.Length();
            for (i = 1; i <= nb; i++)
            {
                TP.SetRoot(lis.Value(i));
            }
        }
        if (mode == 3)
        {
            Transfer_TransientProcess TP = myTransferReader.TransientProcess();
            if (TP is null)
            {
                return;
            }
            int i, nb = TP.NbRoots();
            for (i = 1; i <= nb; i++)
            {
                myTransferReader.RecordResult(TP.Root(i));
            }
        }
        if (mode == 4 || mode == 5)
        {
            myTransferReader.BeginTransfer();
        }
    }

    /// <summary>
    /// Sets a Transfer Reader, which manages transfers on reading
    /// </summary>
    public void SetTransferReader(XSControl_TransferReader? TR)
    {
        if (myTransferReader != TR)
        { // i1 pdn 03.04.99 BUC60301
            myTransferReader = TR;
        }
        if (TR is null)
        {
            return;
        }
        TR.SetController(myController);
        TR.SetStatics(Statics()); // OCCT.NET
        TR.SetGraph(HGraph());
        if (TR.TransientProcess() is not null)
        {
            return;
        }
        Transfer_TransientProcess TP =
            new Transfer_TransientProcess(Model() is null ? 100 : Model().NbEntities() + 100);
        TP.SetGraph(HGraph());
        TP.SetErrorHandle(true);
        TR.SetTransientProcess(TP);
    }

    /// <summary>
    /// Returns the Transfer Reader, Null if not set
    /// </summary>
    public XSControl_TransferReader? TransferReader()
    {
        return myTransferReader;
    }

    /// <summary>
    /// Returns the TransientProcess(internal data for TransferReader)
    /// </summary>
    public Transfer_TransientProcess? MapReader()
    {
        return myTransferReader.TransientProcess();
    }

    /// <summary>
    /// Changes the Map Reader, i.e. considers that the new one defines the relevant read results (forgets the
    /// former ones) Returns True when done, False in case of bad definition, i.e. if Model from TP differs from
    /// that of Session
    /// </summary>
    public bool SetMapReader(Transfer_TransientProcess? TP)
    {
        if (TP is null)
        {
            return false;
        }
        if (TP.Model() is null)
        {
            TP.SetModel(Model());
        }
        TP.SetGraph(HGraph());
        if (TP.Model() != Model())
        {
            return false;
        }
        //  TR must not move, it's a "hook" for signatures, selections ...
        //  On the other hand, better to reset it
        //  XSControl_TransferReader TR = new XSControl_TransferReader();
        XSControl_TransferReader TR = myTransferReader;
        TR.Clear(-1);

        SetTransferReader(TR); // with the same but reinitializes it
        TR.SetTransientProcess(TP); // and takes the new TP
        return true;
    }

    /// <summary>
    /// Returns the result attached to a starting entity If &lt;mode&gt; = 0, returns Final Result If
    /// &lt;mode&gt; = 1, considers Last Result If &lt;mode&gt; = 2, considers Final, else if absent, Last
    /// returns it as Transient, if result is not transient returns the Binder &lt;mode&gt; = 10,11,12 idem but
    /// returns the Binder itself (if it is not, e.g. Shape, returns the Binder) &lt;mode&gt; = 20, returns the
    /// ResultFromModel
    /// </summary>
    public Standard_Transient? Result(Standard_Transient? ent, int mode)
    {
        int ouca = (mode % 10);
        int kica = (mode / 10);

        Transfer_Binder binder = null;
        Transfer_ResultFromModel resu = null;

        if (ouca != 1)
        {
            resu = myTransferReader.FinalResult(ent);
        }
        if (mode == 20)
        {
            return resu;
        }

        if (resu is not null)
        {
            binder = resu.MainResult().Binder();
        }
        if (binder is null && ouca > 0)
        {
            binder = myTransferReader.TransientProcess().Find(ent);
        }

        if (kica == 1)
        {
            return binder;
        }
        Transfer_SimpleBinderOfTransient trb = binder as Transfer_SimpleBinderOfTransient;
        if (trb is not null)
        {
            return trb.Result();
        }
        return binder;
    }

    /// <summary>
    /// Commands the transfer of, either one entity, or a list I.E. calls the TransferReader after having
    /// analysed &lt;ents&gt; It is cumulated from the last BeginTransfer &lt;ents&gt; is processed by GiveList,
    /// hence : - &lt;ents&gt; a Selection : its SelectionResult - &lt;ents&gt; a HSequenceOfTransient : this
    /// list - &lt;ents&gt; the Model : in this specific case, all the roots, with no cumulation of former
    /// transfers (TransferReadRoots)
    /// </summary>
    public int TransferReadOne(Standard_Transient? ent, Message_ProgressRange theProgress = default)
    {
        Interface_InterfaceModel model = Model();
        if (ent == model)
        {
            return TransferReadRoots(theProgress);
        }

        NCollection_HSequence<Standard_Transient> list = GiveList(ent);
        if (list.Length() == 1)
        {
            return myTransferReader.TransferOne(list.Value(1), true, theProgress);
        }
        else
        {
            return myTransferReader.TransferList(list, true, theProgress);
        }
    }

    /// <summary>
    /// Commands the transfer of all the root entities of the model i.e. calls TransferRoot from the
    /// TransferReader with the Graph No cumulation with former calls to TransferReadOne
    /// </summary>
    public int TransferReadRoots(Message_ProgressRange theProgress = default)
    {
        return myTransferReader.TransferRoots(Graph(), theProgress);
    }

    /// <summary>
    /// produces and returns a new Model well conditioned It is produced by the Norm Controller It can be Null
    /// (if this function is not implemented)
    /// </summary>
    public Interface_InterfaceModel? NewModel()
    {
        lock (WS_GLOBAL_MUTEX)
        {
            return NewModelLocked();
        }
    }

    private Interface_InterfaceModel? NewModelLocked()
    {
        Interface_InterfaceModel newmod = null;
        if (myController is null)
        {
            return newmod;
        }
        newmod = myController.NewModel();

        SetModel(newmod);
        if (myTransferReader.TransientProcess() is not null)
        {
            myTransferReader.TransientProcess().Clear();
        }
        // clear all contains of WS
        myTransferReader.Clear(3);
        myTransferWriter.Clear(-1);

        return newmod;
    }

    /// <summary>
    /// Returns the Transfer Reader, Null if not set
    /// </summary>
    public XSControl_TransferWriter? TransferWriter()
    {
        return myTransferWriter;
    }

    /// <summary>
    /// Changes the Map Reader, i.e. considers that the new one defines the relevant read results (forgets the
    /// former ones) Returns True when done, False if &lt;FP&gt; is Null
    /// </summary>
    public bool SetMapWriter(Transfer_FinderProcess? theFP)
    {
        if (theFP is null)
        {
            return false;
        }
        myTransferWriter.SetFinderProcess(theFP);
        return true;
    }

    /// <summary>
    /// Transfers a Shape from CasCade to a model of current norm, according to the last call to
    /// SetModeWriteShape Returns status :Done if OK, Fail if error during transfer, Error if transfer badly
    /// initialised
    /// </summary>
    public IFSelect_ReturnStatus TransferWriteShape(TopoDS_Shape shape, bool compgraph = true, Message_ProgressRange theProgress = default)
    {
        lock (WS_GLOBAL_MUTEX)
        {
            return TransferWriteShapeLocked(shape, compgraph, theProgress);
        }
    }

    private IFSelect_ReturnStatus TransferWriteShapeLocked(TopoDS_Shape shape, bool compgraph, Message_ProgressRange theProgress)
    {
        IFSelect_ReturnStatus status;
        if (myController is null)
        {
            return IFSelect_RetError;
        }
        Interface_InterfaceModel model = Model();
        if (model is null || shape.IsNull())
        {
            return IFSelect_RetVoid;
        }

        status = myTransferWriter.TransferWriteShape(model, shape, theProgress);
        if (theProgress.UserBreak())
        {
            return IFSelect_RetStop;
        }
        //  which takes care of everything, try/catch included

        // skl insert param compgraph for XDE writing 10.12.2003
        if (compgraph)
        {
            ComputeGraph(true);
        }

        return status;
    }

    /// <summary>
    /// Returns the check-list of last transfer (write) It is recorded in the FinderProcess, but it must be bound
    /// with resulting entities (in the resulting file model) rather than with original objects (in fact, their
    /// mappers)
    /// </summary>
    public Interface_CheckIterator TransferWriteCheckList()
    {
        return myTransferWriter.ResultCheckList(Model());
    }

    public XSControl_Vars? Vars()
    {
        return myVars;
    }

    public void SetVars(XSControl_Vars? theVars)
    {
        myVars = theVars;
    }

    /// <summary>
    /// Clears binders
    /// </summary>

    private XSControl_Controller? myController;
    private XSControl_TransferReader? myTransferReader;
    private XSControl_TransferWriter? myTransferWriter;
    private NCollection_DataMap<string, Standard_Transient> myContext = new NCollection_DataMap<string, Standard_Transient>();
    private XSControl_Vars? myVars;
}
