// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_Reader.hxx
// Port of: src/DataExchange/TKXSBase/TransferBRep/TransferBRep_Reader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Exchange.Framework.Transfer;

/// <summary>
/// This class offers a simple, easy to call, way of transferring data from interface files to Shapes from CasCade.
/// It must be specialized according to each norm/protocol, by : - defining how to read a file (specific method with
/// protocol) - definig transfer, by providing an Actor.
/// </summary>
/// <remarks>
/// The shapes a transfer gave are published (frozen) when the transfer ends (<see cref="EndTransfer"/>), so
/// <see cref="Shapes"/>, <see cref="Shape"/> and <see cref="OneShape"/> return frozen shapes (decision 1205).
/// </remarks>
public class BrepTransferReader
{
    /// <summary>Initializes a non-specialised Reader. Typically, for each norm or protocol, is will be required to define a specific Create to load a file and transfer it.</summary>
    public BrepTransferReader()
    {
        _done = false;
        _filest = 0;
        _newpr = false;
        _shapes = new NCollection_HSequence<Shape>();
        _transi = new NCollection_HSequence<Transient>();
    }

    /// <summary>Records the protocol to be used for read and transfer roots.</summary>
    public void SetProtocol(Protocol? protocol)
    {
        _proto = protocol;
    }

    /// <summary>Returns the recorded Protocol.</summary>
    public virtual Protocol? Protocol()
    {
        return _proto;
    }

    /// <summary>Records the actor to be used for transfers.</summary>
    public void SetActor(ActorOfTransientProcess? actor)
    {
        _actor = actor;
    }

    /// <summary>Returns the recorded Actor.</summary>
    public virtual ActorOfTransientProcess? Actor()
    {
        return _actor;
    }

    /// <summary>Sets File Status to be interpreted as follows : = 0 OK, &lt; 0 file not found, &gt; 0 read error, no Model could be created.</summary>
    public void SetFileStatus(int status)
    {
        _filest = status;
    }

    /// <summary>Returns the File Status.</summary>
    public int FileStatus()
    {
        return _filest;
    }

    /// <summary>Returns True if FileStatus is for FileNotFound.</summary>
    public bool FileNotFound()
    {
        return _filest < 0;
    }

    /// <summary>Returns True if FileStatus is for Error during read (major error; for local error, see CheckModel).</summary>
    public bool SyntaxError()
    {
        return _filest > 0;
    }

    /// <summary>Specifies a Model to work on. Also clears the result and Done status.</summary>
    public void SetModel(InterfaceModel? model)
    {
        _model = model;
        Clear();
    }

    /// <summary>Returns the Model to be worked on.</summary>
    public InterfaceModel? Model()
    {
        return _model;
    }

    /// <summary>clears the result and Done status. But not the Model.</summary>
    public void Clear()
    {
        _done = false;
        _shapes.Clear();
        _transi.Clear();
    }

    /// <summary>
    /// Checks the Model. Returns True if there is NO FAIL at all (regardless Warnings). If <paramref name="withprint"/>
    /// is True, also sends Checks on standard output.
    /// </summary>
    public bool CheckStatusModel(bool withprint)
    {
        var cht = new CheckTool(_model, _proto);
        CheckIterator chl = cht.CompleteCheckList();
        if (withprint && _proc is not null && _proc.Messenger() is not null)
        {
            using Messenger.StreamBuffer buffer = _proc.Messenger().SendInfo();
            using var stream = new StringWriter(buffer.Stream());
            chl.Print(stream, _model, false);
        }
        return chl.IsEmpty(true);
    }

    /// <summary>Checks the Model (complete : syntax + semantic) and returns the produced Check List.</summary>
    public CheckIterator CheckListModel()
    {
        var cht = new CheckTool(_model, _proto);
        CheckIterator chl = cht.CompleteCheckList();
        return chl;
    }

    /// <summary>
    /// Returns (by Reference, hence can be changed) the Mode for new Transfer : True (D) means that each new Transfer
    /// produces a new TransferProcess. Else, the existing Process is cleared and reused.
    /// </summary>
    public ref bool ModeNewTransfer()
    {
        return ref _newpr;
    }

    /// <summary>
    /// Initializes the Reader for a Transfer (one,roots, or list). Also calls PrepareTransfer. Returns True when done,
    /// False if could not be done.
    /// </summary>
    public bool BeginTransfer()
    {
        _done = false;
        if (_model is null)
        {
            return false;
        }

        if (_newpr || _proc is null)
        {
            _proc = new TransientProcess(_model.EntityCount);
        }
        else
        {
            _proc.Clear();
        }
        _proc.SetErrorHandle(true);
        _proc.SetModel(_model);
        PrepareTransfer();
        _proc.SetActor(_actor);
        return true;
    }

    /// <summary>Ebds a Transfer (one, roots or list) by recording its result; the shapes are published (frozen).</summary>
    public void EndTransfer()
    {
        NCollection_HSequence<Shape> shapes = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Shapes(_proc, true);
        for (int i = 1; i <= shapes.Length(); i++)
        {
            shapes.Value(i).TShape()?.FreezeGraph(); // OCCT.NET: the reader's results are published (decision 1205)
        }
        _shapes.Append(shapes);
        int nb = _proc.RootCount;
        for (int i = 1; i <= nb; i++)
        {
            Transient? ent = _proc.Root(i);
            Transient? res = _proc.FindTransient(ent);
            if (res is not null)
            {
                _transi.Append(res);
            }
        }
        _done = true;
    }

    /// <summary>
    /// Prepares the Transfer. Also can act on the Actor or change the TransientProcess if required. Should not set the
    /// Actor into the TransientProcess, it is done by caller. The provided default does nothing.
    /// </summary>
    public virtual void PrepareTransfer()
    {
    }

    /// <summary>
    /// Transfers all Root Entities which are recognized as Geom-Topol. The result will be a list of Shapes. This
    /// method calls user redefinable PrepareTransfer. Error messages are produced as the Transfer goes on.
    /// </summary>
    public virtual void TransferRoots(ProgressRange progress = default)
    {
        Clear();
        if (!BeginTransfer())
        {
            return;
        }
        var tp = new TransferOutput(_proc, _model);

        tp.TransferRoots(_proto, progress);
        EndTransfer();
    }

    /// <summary>
    /// Transfers an Entity given its rank in the Model (Root or not). Returns True if it is recognized as Geom-Topol.
    /// (But it can have failed : see IsDone).
    /// </summary>
    public virtual bool Transfer(int num, ProgressRange progress = default)
    {
        // OCCT: the former shapes aren't cleared (TransferRoots does): they accumulate
        if (!BeginTransfer())
        {
            return false;
        }
        if (num <= 0 || num > _model.EntityCount)
        {
            return false;
        }
        Transient? ent = _model.Value(num);
        var tp = new TransferOutput(_proc, _model);

        if (_proc.TraceLevel() > 1)
        {
            using Messenger.StreamBuffer sout = _proc.Messenger().SendInfo();
            sout.Append("--  Transfer(Read) : ");
            using (var stream = new StringWriter(sout.Stream()))
            {
                _model.Print(ent, stream);
            }
            sout.Append('\n');
        }
        tp.Transfer(ent, progress);
        _proc.SetRoot(ent);
        EndTransfer();
        return true;
    }

    /// <summary>
    /// Transfers a list of Entities (only the ones also in the Model). Remark : former result is cleared (OCCT's
    /// documentation; in fact only the process is: the former shapes stay).
    /// </summary>
    public virtual void TransferList(NCollection_HSequence<Transient>? list, ProgressRange progress = default)
    {
        // OCCT: the former shapes aren't cleared, despite the documentation
        if (!BeginTransfer())
        {
            return;
        }
        if (list is null)
        {
            return;
        }
        var tp = new TransferOutput(_proc, _model);
        int i, nb = list.Length();
        using Messenger.StreamBuffer sout = _proc.Messenger().SendInfo();

        if (_proc.TraceLevel() > 1)
        {
            sout.Append("--  Transfer(Read-List) : ").Append(nb).Append(" Items").Append('\n');
        }
        using var ps = new ProgressScope(progress, null, nb);
        for (i = 1; i <= nb && ps.More(); i++)
        {
            ProgressRange range = ps.Next();
            Transient? ent = list.Value(i);
            if (_model.Number(ent) == 0)
            {
                continue;
            }

            if (_proc.TraceLevel() > 1)
            {
                sout.Append("--  Transfer(Read-List), Item ").Append(i).Append(" : ");
                using (var stream = new StringWriter(sout.Stream()))
                {
                    _model.Print(ent, stream);
                }
                sout.Append('\n');
            }
            tp.Transfer(ent, range);
            _proc.SetRoot(ent);
        }
        EndTransfer();
    }

    /// <summary>Returns True if the LAST Transfer/TransferRoots was a success.</summary>
    public bool IsDone
    {
        get
        {
            return _done;
        }
    }

    //   ######    RESULTAT : SHAPES    ######

    /// <summary>Returns the count of produced Shapes (roots).</summary>
    public int ShapeCount
    {
        get
        {
            return _shapes.Length();
        }
    }

    /// <summary>Returns the complete list of produced Shapes.</summary>
    public NCollection_HSequence<Shape> Shapes()
    {
        return _shapes;
    }

    /// <summary>Returns a Shape given its rank, by default the first one.</summary>
    public Shape Shape(int num = 1)
    {
        return _shapes.Value(num);
    }

    /// <summary>
    /// Returns a Shape produced from a given entity (if it was individually transferred or if an entity is a
    /// subpart of the one which was transferred). Returns a Null Shape if there is no result.
    /// </summary>
    public Shape ShapeResult(Transient? ent)
    {
        return global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(_proc, ent);
    }

    /// <summary>
    /// Returns a unique Shape for the result : a void shape if no result; if the result is only one shape, this
    /// shape; else, a compound of the shapes (frozen).
    /// </summary>
    public Shape OneShape()
    {
        Shape res = new Shape();
        int nb = _shapes.Length();
        if (nb == 0)
        {
            return res;
        }
        else if (nb == 1)
        {
            return _shapes.Value(1);
        }
        else
        {
            Compound c = new Compound();
            var b = new BrepBuilder();
            b.MakeCompound(ref c);
            for (int i = 1; i <= nb; i++)
            {
                b.Add(c, _shapes.Value(i));
            }
            c.TShape()?.FreezeGraph(); // OCCT.NET: published
            return c;
        }
    }

    //   ######    RESULTAT : TRANSIENTS    ######

    /// <summary>Returns the count of produced Transient Results (roots).</summary>
    public int TransientCount
    {
        get
        {
            return _transi.Length();
        }
    }

    /// <summary>Returns the complete list of produced Transient Results.</summary>
    public NCollection_HSequence<Transient> Transients()
    {
        return _transi;
    }

    /// <summary>Returns a Transient Root Result, given its rank (by default the first one).</summary>
    public Transient? Transient(int num = 1)
    {
        return _transi.Value(num);
    }

    //   ######    CHECKS    ######

    /// <summary>
    /// Checks the Result of last Transfer (individual or roots). Returns True if there is NO FAIL at all (regardless
    /// Warnings). If <paramref name="withprint"/> is True, also sends Checks on standard output.
    /// </summary>
    public bool CheckStatusResult(bool withprint)
    {
        var chl = new CheckIterator();
        if (_proc is not null)
        {
            chl = _proc.CheckList(false);
        }
        if (withprint && _proc is not null && _proc.Messenger() is not null)
        {
            using Messenger.StreamBuffer buffer = _proc.Messenger().SendInfo();
            using var stream = new StringWriter(buffer.Stream());
            chl.Print(stream, _model, false);
        }
        return chl.IsEmpty(true);
    }

    /// <summary>Checks the Result of last Transfer (individual or roots) and returns the produced list.</summary>
    public CheckIterator CheckListResult()
    {
        if (_proc is not null)
        {
            return _proc.CheckList(false);
        }
        var chbid = new CheckIterator();
        return chbid;
    }

    /// <summary>Returns the TransientProcess. It records information about the very last transfer done.</summary>
    public TransientProcess? TransientProcess()
    {
        return _proc;
    }

    protected bool _done;
    protected TransientProcess? _proc;

    private Protocol? _proto;
    private ActorOfTransientProcess? _actor;
    private InterfaceModel? _model;
    private int _filest;
    private bool _newpr;
    private readonly NCollection_HSequence<Shape> _shapes;
    private readonly NCollection_HSequence<Transient> _transi;
}
