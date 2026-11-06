// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_WorkSession.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_WorkSession.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// This WorkSession completes the basic one, by adding : - use of Controller, with norm selection... -
/// management of transfers (both ways) with auxiliary classes TransferReader and TransferWriter -&gt; these
/// transfers may work with a Context List : its items are given by the user, according to the transfer to be
/// i.e. it is interpreted by the Actors Each item is accessed by a Name
/// </summary>
internal class ExchangeWorkSession : WorkSession
{
    // OCCT's process-wide mutex (GetGlobalMutex) around SelectNorm, NewModel and TransferWriteShape is the
    // controller's (Controller.SessionLock, unit 10I): a recorded controller and its actors are shared by the sessions
    // that select it (decision 1222), but most sessions have their own (decision 1264). Selecting a norm changes only
    // the session.

    public ExchangeWorkSession()
    {
        _transferReader = new TransferReader();
        _transferWriter = new TransferWriter();
        _vars = new Vars();
        _transferReader.SetStatics(Statics());
    }

    /// <summary>
    /// Creates a session working with a given parameter set (Kerf: for instance an interpreter's, or a
    /// reader's copy; decision 1181).
    /// </summary>
    public ExchangeWorkSession(StaticSet statics) : this()
    {
        SetStatics(statics);
    }

    /// <summary>
    /// Gives the session a parameter set (Kerf): its transfer reader reads it too, and the controller's
    /// statics are declared in it.
    /// </summary>
    public override void SetStatics(StaticSet statics)
    {
        base.SetStatics(statics);
        _controller?.InitStatics(Statics());
        _transferReader?.SetStatics(Statics());
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
            _transferReader.Clear(-1);
            _transferWriter.Clear(-1);
        }
        if (mode == 6 && _transferReader is not null)
        {
            _transferReader.Clear(1);
        }
        _transferReader.SetGraph(HGraph());
    }

    /// <summary>
    /// Selects a Norm defined by its name. A Norm is described and handled by a Controller Returns True if done,
    /// False if &lt;normname&gt; is unknown The current Profile for this Norm is taken.
    /// </summary>
    public bool SelectNorm(string normname)
    {
        return SelectNormLocked(normname);
    }

    private bool SelectNormLocked(string normname)
    {
        // Old norm and results
        _transferReader.Clear(-1);
        //  ????  Strictly speaking, cleanup to do in XWS: remove the items
        //        ( at the limit, why not, redo XWS entirely)

        Controller newadapt = Controller.Recorded(normname);
        if (newadapt is null)
        {
            return false;
        }
        if (newadapt == _controller)
        {
            return true;
        }
        SetController(newadapt);
        return true;
    }

    /// <summary>
    /// Selects a Norm defined by its Controller itself
    /// </summary>
    public void SetController(Controller? ctl)
    {
        _controller = ctl;

        // Kerf: the controller declares its statics in the session's parameter set (OCCT: in the process-wide
        // statics, when it is built)
        _controller.InitStatics(Statics());

        SetLibrary(_controller.WorkLibrary());
        SetProtocol(_controller.Protocol());

        ClearItems();
        ClearFinalModifiers();
        ClearShareOut(false);
        ClearFile();

        // Set worksession parameters from the controller
        ExchangeWorkSession workSession = this;
        _controller.Customise(workSession);

        _transferReader.SetController(_controller);
        _transferWriter.SetController(_controller);
    }

    /// <summary>
    /// Returns the name of the last Selected Norm. If none is defined, returns an empty string By default,
    /// returns the complete name of the norm If &lt;rsc&gt; is True, returns the short name used for resource
    /// </summary>
    public string SelectedNorm(bool rsc = false)
    {
        // JR/Hp :
        string astr = (_controller is null ? "" : _controller.Name(rsc));
        return astr;
    }

    /// <summary>
    /// Returns the norm controller itself
    /// </summary>
    public Controller? NormAdaptor()
    {
        return _controller;
    }

    /// <summary>
    /// Returns the current Context List, Null if not defined The Context is given to the TransientProcess for
    /// TransferRead
    /// </summary>
    public Dictionary<string, Transient> Context()
    {
        return _context;
    }

    /// <summary>
    /// Sets the current Context List, as a whole Sets it to the TransferReader
    /// </summary>
    public void SetAllContext(Dictionary<string, Transient> context)
    {
        global::Kerf.Engine.Exchange.Framework.TransferReader.AssignContext(_context, context); // C++'s map assignments: copies
        global::Kerf.Engine.Exchange.Framework.TransferReader.AssignContext(_transferReader.Context(), context);
    }

    /// <summary>
    /// Clears the whole current Context (nullifies it)
    /// </summary>
    public void ClearContext()
    {
        _context.Clear();
        _transferReader.Context().Clear();
    }

    /// <summary>
    /// Prints the transfer status of a transferred item, as being the Mapped n0 &lt;num&gt;, from MapWriter if
    /// &lt;wri&gt; is True, or from MapReader if &lt;wri&gt; is False Returns True when done, False else (i.e.
    /// num out of range)
    /// </summary>
    public bool PrintTransferStatus(int num, bool wri, TextWriter s)
    {
        FinderProcess fp = _transferWriter.FinderProcess();
        TransientProcess tp = _transferReader.TransientProcess();

        Binder binder = null;
        Finder finder = null;
        Transient ent = null;

        //   ***   WRITE  ***
        if (wri)
        {
            if (fp is null)
            {
                return false;
            }
            if (num == 0)
            {
                return false;
            }

            int ne = 0, nr = 0, max = fp.MappedCount, maxr = fp.RootCount;
            if (num > 0)
            {
                if (num > max)
                {
                    return false;
                }
                ne = num;
                finder = fp.Mapped(ne);
                nr = fp.RootIndex(finder);
            }
            else if (num < 0)
            {
                nr = -num;
                if (nr > maxr)
                {
                    return false;
                }
                finder = fp.Root(nr);
                ne = fp.MapIndex(finder);
            }

            s.Write("Transfer Write item n0.");
            s.Write(ne);
            s.Write(" of ");
            s.Write(max);
            if (nr > 0)
            {
                s.Write("  ** Transfer Root n0.");
                s.Write(ne);
            }
            s.Write('\n');
            ent = fp.FindTransient(finder);
            s.Write(" -> Type ");
            s.Write(finder.DynamicTypeName());
            s.Write('\n');
            fp.StartTrace(binder, finder, 0, 0); // pb sout/S
            if (ent is not null)
            {
                s.Write(" ** Transient Result, type ");
                s.Write(ent.DynamicTypeName());
                InterfaceModel model = Model();
                if (model is not null)
                {
                    s.Write(" In output Model, Entity ");
                    model.Print(ent, s);
                }
                s.Write('\n');
            }
        }

        //    ***   READ   ***
        else
        {
            if (tp is null)
            {
                return false;
            }
            InterfaceModel model = tp.Model();
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

            int ne = 0, nr = 0, max = tp.MappedCount, maxr = tp.RootCount;
            if (num > 0)
            {
                if (num > max)
                {
                    return false;
                }
                ne = num;
                ent = tp.Mapped(ne);
                nr = tp.RootIndex(finder);
            }
            else if (num < 0)
            {
                nr = -num;
                if (nr > maxr)
                {
                    return false;
                }
                ent = tp.Root(nr);
                ne = tp.MapIndex(ent);
            }

            s.Write("Transfer Read item n0.");
            s.Write(ne);
            s.Write(" of ");
            s.Write(max);
            if (nr > 0)
            {
                s.Write("  ** Transfer Root n0.");
                s.Write(ne);
            }
            s.Write('\n');
            if (model is not null)
            {
                s.Write(" In Model, Entity ");
                model.Print(ent, s);
            }
            binder = tp.MapItem(ne);
            s.Write('\n');
            tp.StartTrace(binder, ent, 0, 0);
        }

        //   ***   CHECK (common READ+WRITE)   ***
        if (binder is not null)
        {
            FrameworkCheck ch = binder.Check();
            int i, nbw = ch.WarningCount, nbf = ch.FailCount;
            if (nbw > 0)
            {
                s.Write(" - Warnings : ");
                s.Write(nbw);
                s.Write(" :\n");
                for (i = 1; i <= nbw; i++)
                {
                    s.Write(ch.CWarning(i));
                    s.Write('\n');
                }
            }
            if (nbf > 0)
            {
                s.Write(" - Fails : ");
                s.Write(nbf);
                s.Write(" :\n");
                for (i = 1; i <= nbf; i++)
                {
                    s.Write(ch.CFail(i));
                    s.Write('\n');
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
            _transferReader.Clear(-1); // full clear
        }
        if (_transferReader is null)
        {
            SetTransferReader(new TransferReader());
        }
        else
        {
            SetTransferReader(_transferReader);
        }

        // mode = 0 done by SetTransferReader following Nullify
        if (mode == 1)
        {
            if (_transferReader is not null)
            {
                _transferReader.Clear(-1);
            }
            else
            {
                SetTransferReader(new TransferReader());
            }
        }
        if (mode == 2)
        {
            TransientProcess tp = _transferReader.TransientProcess();
            if (tp is null)
            {
                tp = new TransientProcess();
                _transferReader.SetTransientProcess(tp);
                tp.SetGraph(HGraph());
            }
            NCollection_HSequence<Transient> lis =
                _transferReader.RecordedList();
            int i, nb = lis.Length();
            for (i = 1; i <= nb; i++)
            {
                tp.SetRoot(lis.Value(i));
            }
        }
        if (mode == 3)
        {
            TransientProcess tp = _transferReader.TransientProcess();
            if (tp is null)
            {
                return;
            }
            int i, nb = tp.RootCount;
            for (i = 1; i <= nb; i++)
            {
                _transferReader.RecordResult(tp.Root(i));
            }
        }
        if (mode == 4 || mode == 5)
        {
            _transferReader.BeginTransfer();
        }
    }

    /// <summary>
    /// Sets a Transfer Reader, which manages transfers on reading
    /// </summary>
    public void SetTransferReader(TransferReader? tr)
    {
        if (_transferReader != tr)
        { // i1 pdn 03.04.99 BUC60301
            _transferReader = tr;
        }
        if (tr is null)
        {
            return;
        }
        tr.SetController(_controller);
        tr.SetStatics(Statics()); // Kerf
        tr.SetGraph(HGraph());
        if (tr.TransientProcess() is not null)
        {
            return;
        }
        TransientProcess tp =
            new TransientProcess(Model() is null ? 100 : Model().EntityCount + 100);
        tp.SetGraph(HGraph());
        tp.SetErrorHandle(true);
        tr.SetTransientProcess(tp);
    }

    /// <summary>
    /// Returns the Transfer Reader, Null if not set
    /// </summary>
    public TransferReader? TransferReader()
    {
        return _transferReader;
    }

    /// <summary>
    /// Returns the TransientProcess(internal data for TransferReader)
    /// </summary>
    public TransientProcess? MapReader()
    {
        return _transferReader.TransientProcess();
    }

    /// <summary>
    /// Changes the Map Reader, i.e. considers that the new one defines the relevant read results (forgets the
    /// former ones) Returns True when done, False in case of bad definition, i.e. if Model from TP differs from
    /// that of Session
    /// </summary>
    public bool SetMapReader(TransientProcess? tp)
    {
        if (tp is null)
        {
            return false;
        }
        if (tp.Model() is null)
        {
            tp.SetModel(Model());
        }
        tp.SetGraph(HGraph());
        if (tp.Model() != Model())
        {
            return false;
        }
        //  TR must not move, it's a "hook" for signatures, selections ...
        //  On the other hand, better to reset it
        //  XSControl_TransferReader TR = new XSControl_TransferReader();
        TransferReader tr = _transferReader;
        tr.Clear(-1);

        SetTransferReader(tr); // with the same but reinitializes it
        tr.SetTransientProcess(tp); // and takes the new TP
        return true;
    }

    /// <summary>
    /// Returns the result attached to a starting entity If &lt;mode&gt; = 0, returns Final Result If
    /// &lt;mode&gt; = 1, considers Last Result If &lt;mode&gt; = 2, considers Final, else if absent, Last
    /// returns it as Transient, if result is not transient returns the Binder &lt;mode&gt; = 10,11,12 idem but
    /// returns the Binder itself (if it is not, e.g. Shape, returns the Binder) &lt;mode&gt; = 20, returns the
    /// ResultFromModel
    /// </summary>
    public Transient? Result(Transient? ent, int mode)
    {
        int ouca = (mode % 10);
        int kica = (mode / 10);

        Binder binder = null;
        ResultFromModel resu = null;

        if (ouca != 1)
        {
            resu = _transferReader.FinalResult(ent);
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
            binder = _transferReader.TransientProcess().Find(ent);
        }

        if (kica == 1)
        {
            return binder;
        }
        SimpleBinderOfTransient trb = binder as SimpleBinderOfTransient;
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
    public int TransferReadOne(Transient? ent, ProgressRange progress = default)
    {
        InterfaceModel model = Model();
        if (ent == model)
        {
            return TransferReadRoots(progress);
        }

        NCollection_HSequence<Transient> list = GiveList(ent);
        if (list.Length() == 1)
        {
            return _transferReader.TransferOne(list.Value(1), true, progress);
        }
        else
        {
            return _transferReader.TransferList(list, true, progress);
        }
    }

    /// <summary>
    /// Commands the transfer of all the root entities of the model i.e. calls TransferRoot from the
    /// TransferReader with the Graph No cumulation with former calls to TransferReadOne
    /// </summary>
    public int TransferReadRoots(ProgressRange progress = default)
    {
        return _transferReader.TransferRoots(Graph(), progress);
    }

    /// <summary>
    /// produces and returns a new Model well conditioned It is produced by the Norm Controller It can be Null
    /// (if this function is not implemented)
    /// </summary>
    public InterfaceModel? NewModel()
    {
        if (_controller is null)
        {
            return null;
        }
        lock (_controller.SessionLock)
        {
            return NewModelLocked();
        }
    }

    private InterfaceModel? NewModelLocked()
    {
        InterfaceModel newmod = null;
        if (_controller is null)
        {
            return newmod;
        }
        newmod = _controller.NewModel();

        SetModel(newmod);
        if (_transferReader.TransientProcess() is not null)
        {
            _transferReader.TransientProcess().Clear();
        }
        // clear all contains of WS
        _transferReader.Clear(3);
        _transferWriter.Clear(-1);

        return newmod;
    }

    /// <summary>
    /// Returns the Transfer Reader, Null if not set
    /// </summary>
    public TransferWriter? TransferWriter()
    {
        return _transferWriter;
    }

    /// <summary>
    /// Changes the Map Reader, i.e. considers that the new one defines the relevant read results (forgets the
    /// former ones) Returns True when done, False if &lt;FP&gt; is Null
    /// </summary>
    public bool SetMapWriter(FinderProcess? fp)
    {
        if (fp is null)
        {
            return false;
        }
        _transferWriter.SetFinderProcess(fp);
        return true;
    }

    /// <summary>
    /// Transfers a Shape from CasCade to a model of current norm, according to the last call to
    /// SetModeWriteShape Returns status :Done if OK, Fail if error during transfer, Error if transfer badly
    /// initialised
    /// </summary>
    public ReturnStatus TransferWriteShape(Shape shape, bool compgraph = true, ProgressRange progress = default)
    {
        if (_controller is null)
        {
            return ReturnStatus.RetError;
        }
        lock (_controller.SessionLock)
        {
            return TransferWriteShapeLocked(shape, compgraph, progress);
        }
    }

    private ReturnStatus TransferWriteShapeLocked(Shape shape, bool compgraph, ProgressRange progress)
    {
        ReturnStatus status;
        if (_controller is null)
        {
            return ReturnStatus.RetError;
        }
        InterfaceModel model = Model();
        if (model is null || shape.IsNull)
        {
            return ReturnStatus.RetVoid;
        }

        status = _transferWriter.TransferWriteShape(model, shape, progress);
        if (progress.UserBreak())
        {
            return ReturnStatus.RetStop;
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
    public CheckIterator TransferWriteCheckList()
    {
        return _transferWriter.ResultCheckList(Model());
    }

    public Vars? Vars()
    {
        return _vars;
    }

    public void SetVars(Vars? vars)
    {
        _vars = vars;
    }

    /// <summary>
    /// Clears binders
    /// </summary>

    private Controller? _controller;
    private TransferReader? _transferReader;
    private TransferWriter? _transferWriter;
    private Dictionary<string, Transient> _context = new Dictionary<string, Transient>();
    private Vars? _vars;
}
