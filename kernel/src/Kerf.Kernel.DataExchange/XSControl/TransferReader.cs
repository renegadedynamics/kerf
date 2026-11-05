// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_TransferReader.hxx
// Port of: src/DataExchange/TKXSBase/XSControl/XSControl_TransferReader.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Exchange.Framework.Selection;
using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Exchange.Framework.Selection.PrintCount;

using static Kerf.Engine.Exchange.Framework.FrameworkCheckStatus;

namespace Kerf.Engine.Exchange.Framework;

/// <summary>
/// A TransferReader performs, manages, handles results of, transfers done when reading a file (i.e. from
/// entities of an InterfaceModel, to objects for Imagine) Running is organised around basic tools :
/// TransientProcess and its Actor, results are Binders and CheckIterators. It implies control by a
/// Controller (which prepares the Actor as required) Getting results can be done directly on
/// TransientProcess, but these are immediate "last produced" results. Each transfer of an entity gives a
/// final result, but also possible intermediate data, and checks, which can be attached to sub-entities.
/// Hence, final results (which intermediates and checks) are recorded as ResultFromModel and can be queried
/// individually. Some more direct access are given for results which are Transient or Shapes
/// </summary>
internal class TransferReader : Transient
{
    //  ****    UTILITAIRE DE STATISTIQUES GENERALES

    // BinderStatus returns a value:
    // 0 Binder Null.   1 void  2 Warning only  3 Fail only
    // 11 Result OK. 12 Result+Warning. 13 Result+Fail
    // (OCCT writes the message into a char[250] buffer: here an out string)

    private static int BinderStatus(Binder? binder, out string mess)
    {
        int stat = 0;
        mess = "";
        if (binder is null)
        {
            mess = "(no data recorded)";
            return 0;
        }
        FrameworkCheckStatus cst = binder.Check().Status();
        if (cst == FrameworkCheckStatus.CheckOK)
        {
            stat = 11;
            if (binder.HasResult)
            {
                mess = Sprintf("%s", binder.ResultTypeName());
            }
            else
            {
                mess = "(no result)";
                stat = 1;
            }
        }
        else if (cst == FrameworkCheckStatus.CheckWarning)
        {
            stat = 12;
            if (binder.HasResult)
            {
                mess = Sprintf("%s  (+ warning)", binder.ResultTypeName());
            }
            else
            {
                mess = "(warning)";
                stat = 2;
            }
        }
        else if (cst == FrameworkCheckStatus.CheckFail)
        {
            stat = 13;
            if (binder.HasResult)
            {
                mess = Sprintf("%s  (+ FAIL)", binder.ResultTypeName());
            }
            else
            {
                mess = "(FAIL)";
                stat = 3;
            }
        }
        return stat;
    }

    private static void PrintPercent(Messenger sout, string mess, int nb, int nl)
    {
        if (nb <= 0 || nl == 0)
        {
            return;
        }
        using Messenger.StreamBuffer sender = sout.SendInfo();
        sender.Append("******      ").Append(mess).Append(": ");
        if (nb == nl)
        {
            sender.Append("100 %").Append('\n');
        }
        else if (nb * 100 / nl == 0)
        {
            sender.Append("< 1 %").Append('\n');
        }
        else
        {
            sender.Append(nb * 100 / nl < 10 ? "  " : " ").Append(nb * 100 / nl).Append(" %").Append('\n');
        }
    }

    /// <summary>
    /// Creates a TransferReader, empty
    /// </summary>
    public TransferReader()
    {
    }

    /// <summary>
    /// Sets a Controller. It is required to generate the Actor. Elsewhere, the Actor must be provided directly
    /// </summary>
    public void SetController(Controller? control)
    {
        _controller = control;
        _actor = null;
        Clear(-1);
    }

    /// <summary>
    /// Sets the Actor directly : this value will be used if the Controller is not set
    /// </summary>
    public void SetActor(ActorOfTransientProcess? actor)
    {
        _actor = actor;
    }

    /// <summary>
    /// Returns the Actor, determined by the Controller, or if this one is unknown, directly set. Once it has
    /// been defined, it can then be edited.
    /// </summary>
    public ActorOfTransientProcess? Actor()
    {
        if (_actor is null && _controller is not null && _model is not null)
        {
            _actor = _controller.ActorRead(_model);
        }
        return _actor;
    }

    /// <summary>
    /// Sets an InterfaceModel. This causes former results, computed from another one, to be lost (see also
    /// Clear)
    /// </summary>
    public void SetModel(InterfaceModel? model)
    {
        _model = model;
        if (_tp is not null)
        {
            _tp.SetModel(model);
        }
    }

    /// <summary>
    /// Sets a Graph and its InterfaceModel (calls SetModel)
    /// </summary>
    public void SetGraph(HGraph? graph)
    {
        if (graph is null)
        {
            _model = null;
        }
        else
        {
            _model = graph.Graph().Model();
        }

        _graph = graph;

        if (_tp is not null)
        {
            _tp.SetGraph(graph);
        }
    }

    /// <summary>
    /// Returns the currently set InterfaceModel
    /// </summary>
    public InterfaceModel? Model()
    {
        return _model;
    }

    /// <summary>
    /// Sets a Context : according to receiving appli, to be interpreted by the Actor
    /// </summary>
    public void SetContext(string name, Transient? ctx)
    {
        _context[name] = ctx;
    }

    /// <summary>
    /// Returns the Context attached to a name, if set and if it is Kind of the type, else a Null Handle Returns
    /// True if OK, False if no Context
    /// </summary>
    public bool GetContext(string name, Type? type, ref Transient? ctx)
    {
        if (_context.Count == 0)
        {
            return false;
        }
        if (!_context.TryGetValue(name, out ctx))
        {
            ctx = null;
        }
        if (ctx is null)
        {
            return false;
        }
        if (type is null)
        {
            return true;
        }
        if (!ctx.IsKind(type))
        {
            ctx = null;
        }
        return ctx is not null;
    }

    /// <summary>
    /// Returns (modifiable) the whole definition of Context Rather for internal use (ex.: preparing and setting
    /// in once)
    /// </summary>
    public Dictionary<string, Transient>? Context()
    {
        return _context;
    }

    /// <summary>
    /// Sets a new value for (loaded) file name
    /// </summary>
    /// <summary>
    /// The parameter set the reader reads its statics from (Kerf: OCCT's process-wide statics, decision
    /// 1181); set by its work session.
    /// </summary>
    public StaticSet? Statics()
    {
        return _statics;
    }

    /// <summary>Sets the parameter set the reader reads its statics from (Kerf).</summary>
    public void SetStatics(StaticSet? statics)
    {
        _statics = statics;
    }

    // C++'s assignment of a map: theTarget becomes a copy of theSource (Kerf).
    internal static void AssignContext(Dictionary<string, Transient> target,
                                       Dictionary<string, Transient> source)
    {
        if (ReferenceEquals(target, source))
        {
            return;
        }
        target.Clear();
        foreach (KeyValuePair<string, Transient> item in source)
        {
            target[item.Key] = item.Value;
        }
    }

    public void SetFileName(string name)
    {
        _fileName = name;
    }

    /// <summary>
    /// Returns actual value of file name
    /// </summary>
    public string FileName()
    {
        return _fileName;
    }

    /// <summary>
    /// Clears data, according mode : -1 all 0 nothing done +1 final results +2 working data (model, context,
    /// transfer process)
    /// </summary>
    public void Clear(int mode)
    {
        if ((mode & 1) != 0)
        {
            _results.Clear();
            _shapeResult = null;
        }
        if ((mode & 2) != 0)
        {
            _model = null;
            _graph = null;
            _tp = null;
            _actor = null;
            _fileName = "";
        }
    }

    /// <summary>
    /// Returns the currently used TransientProcess It is computed from the model by TransferReadRoots, or by
    /// BeginTransferRead
    /// </summary>
    public TransientProcess? TransientProcess()
    {
        return _tp;
    }

    /// <summary>
    /// Forces the TransientProcess Remark : it also changes the Model and the Actor, from those recorded in the
    /// new TransientProcess
    /// </summary>
    public void SetTransientProcess(TransientProcess? tp)
    {
        _tp = tp;
    }

    /// <summary>
    /// Records a final result of transferring an entity This result is recorded as a ResultFromModel, taken from
    /// the TransientProcess Returns True if a result is available, False else
    /// </summary>
    public bool RecordResult(Transient? ent)
    {
        if (_model is null || _tp is null)
        {
            return false;
        }
        int num = _model.Number(ent);
        if (num == 0)
        {
            return false;
        }
        TCollection_HAsciiString lab = _model.StringLabel(ent);

        ResultFromModel res = new ResultFromModel();
        res.Fill(_tp, ent);

        //   Case of Shape result : for main result, make HShape ...
        Binder binder = res.MainResult().Binder();
        ShapeBinder shb = binder as ShapeBinder;
        if (shb is not null)
        {
            SimpleBinderOfTransient trb = new SimpleBinderOfTransient();
            trb.SetResult(new TopoDS_HShape(shb.Result()));
            trb.Merge(binder);
            res.MainResult().SetBinder(trb);
        }

        res.SetFileName(_fileName);
        _results[num] = res;
        return true;
    }

    /// <summary>
    /// Returns True if a final result is recorded for an entity Remark that it can bring no effective result if
    /// transfer has completely failed (FinalResult brings only fail messages ...)
    /// </summary>
    public bool IsRecorded(Transient? ent)
    {
        if (_model is null)
        {
            return false;
        }
        int num = _model.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!_results.ContainsKey(num))
        {
            return false;
        }
        return (_results.GetValue(num).DynamicType() == typeof(ResultFromModel));
    }

    /// <summary>
    /// Returns True if a final result is recorded AND BRINGS AN EFFECTIVE RESULT (else, it brings only fail
    /// messages)
    /// </summary>
    public bool HasResult(Transient? ent)
    {
        if (_model is null)
        {
            return false;
        }
        int num = _model.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!_results.ContainsKey(num))
        {
            return false;
        }
        ResultFromModel fr = _results.GetValue(num) as ResultFromModel;
        if (fr is null)
        {
            return false;
        }
        return fr.HasResult;
    }

    /// <summary>
    /// Returns the list of entities to which a final result is attached (i.e. processed by RecordResult)
    /// </summary>
    public NCollection_HSequence<Transient>? RecordedList()
    {
        NCollection_HSequence<Transient> li =
            new NCollection_HSequence<Transient>();
        if (_model is null)
        {
            return li;
        }
        int i, nb = _model.EntityCount;
        for (i = 1; i <= nb; i++)
        {
            if (_results.ContainsKey(i))
            {
                if (_results.GetValue(i) is not null)
                {
                    li.Append(_model.Value(i));
                }
            }
        }
        return li;
    }

    /// <summary>
    /// Note that an entity has been required for transfer but no result at all is available (typically : case
    /// not implemented) It is not an error, but it gives a specific status : Skipped Returns True if done, False
    /// if &lt;ent&gt; is not in starting model
    /// </summary>
    public bool Skip(Transient? ent)
    {
        if (_model is null || _tp is null)
        {
            return false;
        }
        int num = _model.Number(ent);
        if (num == 0)
        {
            return false;
        }
        _results[num] = ent;
        return true;
    }

    /// <summary>
    /// Returns True if an entity is noted as skipped
    /// </summary>
    public bool IsSkipped(Transient? ent)
    {
        if (_model is null)
        {
            return false;
        }
        int num = _model.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!_results.ContainsKey(num))
        {
            return false;
        }
        return (_results.GetValue(num).DynamicType() != typeof(ResultFromModel));
    }

    /// <summary>
    /// Returns True if an entity has been asked for transfert, hence it is marked, as : Recorded (a computation
    /// has ran, with or without an effective result), or Skipped (case ignored)
    /// </summary>
    public bool IsMarked(Transient? ent)
    {
        if (_model is null)
        {
            return false;
        }
        int num = _model.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!_results.ContainsKey(num))
        {
            return false;
        }
        if (_results.GetValue(num) is null)
        {
            return false;
        }
        return true;
    }

    /// <summary>
    /// Returns the final result recorded for an entity, as such
    /// </summary>
    public ResultFromModel? FinalResult(Transient? ent)
    {
        ResultFromModel res = null;
        if (_model is null)
        {
            return res;
        }
        int num = _model.Number(ent);
        if (num == 0)
        {
            return res;
        }
        if (!_results.ContainsKey(num))
        {
            return res;
        }
        res = (_results.GetValue(num) as ResultFromModel);
        return res;
    }

    /// <summary>
    /// Returns the label attached to an entity recorded for final, or an empty string if not recorded
    /// </summary>
    public string FinalEntityLabel(Transient? ent)
    {
        ResultFromModel resu = FinalResult(ent);
        if (resu is null)
        {
            return "";
        }
        return resu.MainLabel();
    }

    /// <summary>
    /// Returns the number attached to the entity recorded for final, or zero if not recorded (looks in the
    /// ResultFromModel)
    /// </summary>
    public int FinalEntityNumber(Transient? ent)
    {
        ResultFromModel resu = FinalResult(ent);
        if (resu is null)
        {
            return 0;
        }
        return resu.MainNumber();
    }

    /// <summary>
    /// Returns the final result recorded for a NUMBER of entity (internal use). Null if out of range
    /// </summary>
    public ResultFromModel? ResultFromNumber(int num)
    {
        ResultFromModel res = null;
        if (num < 1 || num > _model.EntityCount)
        {
            return res;
        }
        if (!_results.ContainsKey(num))
        {
            return res;
        }
        res = (_results.GetValue(num) as ResultFromModel);
        return res;
    }

    /// <summary>
    /// Returns the resulting object as a Transient Null Handle if no result or result not transient
    /// </summary>
    public Transient? TransientResult(Transient? ent)
    {
        Transient tres = null;
        ResultFromModel res = FinalResult(ent);
        if (res is null)
        {
            return tres;
        }
        ResultFromTransient mres = res.MainResult();
        if (mres is null)
        {
            return tres;
        }
        SimpleBinderOfTransient bnd = mres.Binder() as SimpleBinderOfTransient;
        if (bnd is null)
        {
            return tres;
        }
        if (!bnd.HasResult)
        {
            return tres;
        }
        return bnd.Result();
    }

    /// <summary>
    /// Returns the resulting object as a Shape Null Shape if no result or result not a shape
    /// </summary>
    public Shape ShapeResult(Transient? ent)
    {
        Shape tres = default; // DOIT RESTER null
        ResultFromModel res = FinalResult(ent);
        if (res is null)
        {
            return tres;
        }
        ResultFromTransient mres = res.MainResult();
        if (mres is null)
        {
            return tres;
        }
        Utils xu = new Utils();
        Shape sh = xu.BinderShape(mres.Binder());

        //   Ooh the ugly wart
        // Kerf: the static is the reader's parameter set's (OCCT: the process-wide one; without a set, its
        // default 0.01); OCCT encodes the regularity into the recorded shape itself (ShapeFix::EncodeRegularity), as
        // the port does into a reader's working shape (unfrozen: STEPControl_ActorRead's, decision 1273), so that
        // results sharing it keep sharing it; a shape holding frozen shapes is encoded into a copy (BRepLib's
        // functional twin, decision 1224)
        double tolang = (_statics is null ? 0.01 : _statics.RVal("read.encoderegularity.angle"));
        if (tolang <= 0 || sh.IsNull)
        {
            return sh;
        }
        if (WorkingOverlay.IsWritableInPlace(sh))
        {
            ShapeFixFixer.EncodeRegularity(sh, tolang);
            return sh;
        }
        if (sh.TShape()!.IsFrozen)
        {
            sh = BrepLibrary.RegularityEncoded(sh, tolang);
            return sh;
        }
        // Kerf (unit 8G2): an unfrozen shape holding frozen ones (an assembly whose parts were published before):
        // encoded in place into its unfrozen parts, the frozen ones (results encoded when published) kept, so that the
        // shape keeps sharing its parts with the results of their entities (a copy broke STEPCAFControl_Reader's
        // search of the parts in the assembly: occt-issues.md, unit 8G2)
        EncodeRegularityAround(sh, tolang);
        return sh;
    }

    private static void EncodeRegularityAround(in Shape shape, double tolAng)
    {
        if (shape.TShape()!.IsFrozen)
        {
            return;
        }
        if (WorkingOverlay.IsWritableInPlace(shape))
        {
            ShapeFixFixer.EncodeRegularity(shape, tolAng);
            return;
        }
        for (var it = new Iterator(shape); it.More(); it.Next())
        {
            EncodeRegularityAround(it.Value(), tolAng);
        }
    }

    /// <summary>
    /// Clears recorded result for an entity, according mode &lt;mode&gt; = -1 : true, complete, clearing
    /// (erasing result) &lt;mode&gt; &gt;= 0 : simple "stripping", see ResultFromModel, in particular, 0 for
    /// simple internal strip, 10 for all but final result, 11 for all : just label, status and filename are kept
    /// Returns True when done, False if nothing was to clear
    /// </summary>
    public bool ClearResult(Transient? ent, int mode)
    {
        if (_model is null)
        {
            return false;
        }
        int num = _model.Number(ent);
        if (num == 0)
        {
            return false;
        }
        if (!_results.ContainsKey(num))
        {
            return false;
        }
        if (mode < 0)
        {
            _results.GetValueRef(num) = null;
        }
        else
        {
            ResultFromModel resu = _results.GetValue(num) as ResultFromModel;
            if (resu is null)
            {
                return false;
            }
            resu.Strip(mode);
        }
        return true;
    }

    /// <summary>
    /// Returns an entity from which a given result was produced. If &lt;mode&gt; = 0 (D), searches in last root
    /// transfers If &lt;mode&gt; = 1, searches in last (root &amp; sub) transfers If &lt;mode&gt; = 2, searches
    /// in root recorded results If &lt;mode&gt; = 3, searches in all (root &amp; sub) recordeds &lt;res&gt; can
    /// be, either a transient object (result itself) or a binder. For a binder of shape, calls
    /// EntityFromShapeResult Returns a Null Handle if &lt;res&gt; not recorded
    /// </summary>
    public Transient? EntityFromResult(Transient? res, int mode = 0)
    {
        Transient nulh = null;
        //  case of the shape
        Utils xu = new Utils();
        Shape sh = xu.BinderShape(res);
        if (!sh.IsNull)
        {
            return EntityFromShapeResult(sh, mode);
        }

        Binder abinder = null;
        Binder binder = res as Binder;
        int i, j, nb;

        if (mode == 0 || mode == 1)
        {
            //  we look in the TransientProcess (Roots or all Mappeds)
            if (_tp is not null)
            {
                nb = (mode == 0 ? _tp.RootCount : _tp.MappedCount);
                for (j = 1; j <= nb; j++)
                {
                    i = (mode == 0 ? _model.Number(_tp.Root(j)) : j);
                    if (i == 0)
                    {
                        continue;
                    }
                    abinder = _tp.MapItem(i);
                    if (abinder is null)
                    {
                        continue;
                    }
                    if (binder is not null)
                    {
                        if (binder == abinder)
                        {
                            return _tp.Mapped(i);
                        }
                        continue;
                    }
                    SimpleBinderOfTransient trb = abinder as SimpleBinderOfTransient;
                    if (trb is null)
                    {
                        continue;
                    }
                    if (trb.Result() == res)
                    {
                        return _tp.Mapped(i);
                    }
                }
            }
            return nulh; // Null
        }

        //   Search in myResults (roots)
        //     2 : Main only  3 : Main + one sub;  4 : all
        if (mode >= 2)
        {
            nb = _model.EntityCount;
            for (i = 1; i <= nb; i++)
            {
                ResultFromModel rec = ResultFromNumber(i);
                if (rec is null)
                {
                    return nulh;
                }
                NCollection_HSequence<Transient> list =
                    rec.Results(mode - 2);
                int ir, nr = list.Length();
                for (ir = 1; ir <= nr; ir++)
                {
                    ResultFromTransient rft = list.Value(ir) as ResultFromTransient;
                    if (rft is null)
                    {
                        continue;
                    }
                    if (rft.Binder() == binder)
                    {
                        return rft.Start();
                    }
                }
            }
        }

        //  other cases not yet implemented
        return nulh;
    }

    /// <summary>
    /// Returns an entity from which a given shape result was produced Returns a Null Handle if &lt;res&gt; not
    /// recorded or not a Shape
    /// </summary>
    public Transient? EntityFromShapeResult(Shape res, int mode = 0)
    {
        Transient nulh = null, samesh = null, partner = null;
        if (res.IsNull)
        {
            return nulh;
        }
        int i, j, nb;

        Utils xu = new Utils();
        if (mode == 0 || mode == 1 || mode == -1)
        {
            //  we look in the TransientProcess
            if (_tp is not null)
            {
                nb = (mode == 0 ? _tp.RootCount : _tp.MappedCount);
                for (j = 1; j <= nb; j++)
                {
                    i = (mode == 0 ? _model.Number(_tp.Root(j)) : j);
                    if (i == 0)
                    {
                        continue;
                    }
                    Transient ent = _tp.Mapped(i);
                    Shape sh = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.ShapeResult(_tp, ent);
                    if (!sh.IsNull)
                    {
                        if (sh == res)
                        {
                            return ent;
                        }
                        // lesser priorities : Same (always) or Partner (mode < 0)
                        if (sh.IsSame(res))
                        {
                            samesh = ent;
                        }
                        if (mode == -1 && sh.IsPartner(res))
                        {
                            partner = ent;
                        }
                    }
                }
            }
            //    Here, no true equality found. Lesser priorities: Same then Partner
            if (samesh is not null)
            {
                return samesh;
            }
            if (partner is not null)
            {
                return partner; // calculated if mode = -1
            }
            return nulh;
        }

        //   Search in myResults (roots)
        //     2 : Main only  3 : Main + one sub;  4 : all
        if (mode >= 2)
        {
            nb = _model.EntityCount;
            for (i = 1; i <= nb; i++)
            {
                ResultFromModel rec = ResultFromNumber(i);
                if (rec is null)
                {
                    continue;
                }

                NCollection_HSequence<Transient> list =
                    rec.Results(mode - 2);
                int ir, nr = list.Length();
                for (ir = 1; ir <= nr; ir++)
                {
                    ResultFromTransient rft = list.Value(ir) as ResultFromTransient;
                    if (rft is null)
                    {
                        continue;
                    }
                    Shape sh = xu.BinderShape(rft.Binder());
                    if (!sh.IsNull && sh == res)
                    {
                        return rft.Start();
                    }
                }
            }
        }

        return nulh;
    }

    /// <summary>
    /// Returns the list of entities from which some shapes were produced : it corresponds to a loop on
    /// EntityFromShapeResult, but is optimised
    /// </summary>
    public NCollection_HSequence<Transient>? EntitiesFromShapeList(NCollection_HSequence<Shape>? res, int mode = 0)
    {
        NCollection_HSequence<Transient> lt =
            new NCollection_HSequence<Transient>();
        if (res is null)
        {
            return lt;
        }
        HashSet<Shape> shapes = new HashSet<Shape>(ShapeHasher.Instance);

        //  We convert res to a map, for rapid presence test
        int i, j, nb = res.Length();
        if (nb == 0)
        {
            return lt;
        }
        for (i = 1; i <= nb; i++)
        {
            shapes.Add(res.Value(i));
        }

        //  Now, search and registration

        Utils xu = new Utils();
        if (mode == 0 || mode == 1)
        {
            //  we look in the TransientProcess
            if (_tp is not null)
            {
                nb = (mode == 0 ? _tp.RootCount : _tp.MappedCount);
                for (j = 1; j <= nb; j++)
                {
                    i = (mode == 0 ? _model.Number(_tp.Root(j)) : j);
                    if (i == 0)
                    {
                        continue;
                    }
                    Shape sh = xu.BinderShape(_tp.MapItem(i));
                    if (!sh.IsNull && shapes.Contains(sh))
                    {
                        lt.Append(_tp.Mapped(i));
                        j = nb; // skl (for looking for entities in checkbrep)
                    }
                }
            }
        }

        //   Search in myResults (roots)
        //     2 : Main only  3 : Main + one sub;  4 : all
        if (mode >= 2)
        {
            nb = _model.EntityCount;
            for (i = 1; i <= nb; i++)
            {
                ResultFromModel rec = ResultFromNumber(i);
                if (rec is null)
                {
                    continue;
                }

                NCollection_HSequence<Transient> list =
                    rec.Results(mode - 2);
                int ir, nr = list.Length();
                for (ir = 1; ir <= nr; ir++)
                {
                    ResultFromTransient rft = list.Value(i) as ResultFromTransient; // OCCT: i, not ir
                    if (rft is null)
                    {
                        continue;
                    }
                    Shape sh = xu.BinderShape(rft.Binder());
                    if (!sh.IsNull && shapes.Contains(sh))
                    {
                        lt.Append(rft.Start());
                    }
                }
            }
        }

        return lt;
    }

    /// <summary>
    /// Returns the CheckList resulting from transferring &lt;ent&gt;, i.e. stored in its recorded form
    /// ResultFromModel (empty if transfer successful or not recorded ...) If &lt;ent&gt; is the Model, returns
    /// the complete cumulated check-list, &lt;level&gt; is ignored If &lt;ent&gt; is an entity of the Model,
    /// &lt;level&gt; applies as follows &lt;level&gt; : -1 for &lt;ent&gt; only, LAST transfer
    /// (TransientProcess) &lt;level&gt; : 0 for &lt;ent&gt; only (D) 1 for &lt;ent&gt; and its immediate
    /// subtransfers, if any 2 for &lt;ent&gt; and subtransferts at all levels
    /// </summary>
    public CheckIterator CheckList(Transient? ent, int level = 0)
    {
        CheckIterator chl = new CheckIterator();
        if (_model is null || ent is null)
        {
            return chl;
        }
        //  Check-List COMPLETE ... the entire Model
        if (ent == _model)
        {
            int i, nb = _model.EntityCount;
            for (i = 1; i <= nb; i++)
            {
                ResultFromModel rec = ResultFromNumber(i);
                if (rec is not null)
                {
                    CheckIterator chiter = rec.CheckList(false, 2);
                    chl.Merge(chiter);
                }
            }
        }
        //  Check-List on a LIST ...
        else if (ent is NCollection_HSequence<Transient> list)
        {
            int i, nb = list.Length();
            for (i = 1; i <= nb; i++)
            {
                ResultFromModel rec = FinalResult(list.Value(i));
                if (rec is not null)
                {
                    CheckIterator chiter = rec.CheckList(false, level);
                    chl.Merge(chiter);
                }
            }
        }

        //  otherwise, Check-List on an entity: Last or FinalResult
        else if (level < 0)
        {
            if (_tp is null)
            {
                return chl;
            }
            chl.Add(_tp.Check(ent), _model.Number(ent));
        }
        else
        {
            ResultFromModel rec = FinalResult(ent);
            if (rec is null)
            {
                return chl;
            }
            chl = rec.CheckList(false, level); // manque level ...
        }
        if (ent == _model)
        {
            chl.SetName("XSControl : CheckList complete Model");
        }
        else if (level < 0)
        {
            chl.SetName("XSControl : CheckList Last");
        }
        else if (level == 0)
        {
            chl.SetName("XSControl : CheckList Final Main");
        }
        else if (level == 1)
        {
            chl.SetName("XSControl : CheckList Final Main+Subs");
        }
        else if (level >= 2)
        {
            chl.SetName("XSControl : CheckList Final Complete");
        }
        return chl;
    }

    /// <summary>
    /// Returns True if an entity (with a final result) has checks : - failsonly = False : any kind of check
    /// message - failsonly = True : fails only Returns False if &lt;ent&gt; is not recorded
    /// </summary>
    public bool HasChecks(Transient? ent, bool failsonly)
    {
        ResultFromModel resu = FinalResult(ent);
        if (resu is null)
        {
            return false;
        }
        int stat = (int)resu.ComputeCheckStatus(false);
        if (stat == 0)
        {
            return false;
        }
        if (stat > 1)
        {
            return true;
        }
        return (!failsonly);
    }

    /// <summary>
    /// Returns the list of starting entities to which a given check status is attached, IN FINAL RESULTS
    /// &lt;ent&gt; can be an entity, or the model to query all entities Below, "entities" are, either
    /// &lt;ent&gt; plus its sub-transferred, or all the entities of the model &lt;check&gt; = -2 , all entities
    /// whatever the check (see result) &lt;check&gt; = -1 , entities with no fail (warning allowed)
    /// &lt;check&gt; = 0 , entities with no check at all &lt;check&gt; = 1 , entities with warning but no fail
    /// &lt;check&gt; = 2 , entities with fail &lt;result&gt; : if True, only entities with an attached result
    /// Remark : result True and check=0 will give an empty list
    /// </summary>
    public NCollection_HSequence<Transient>? CheckedList(Transient? ent, FrameworkCheckStatus withcheck = FrameworkCheckStatus.CheckAny, bool level = true)
    {
        NCollection_HSequence<Transient> res =
            new NCollection_HSequence<Transient>();
        if (ent is null)
        {
            return res;
        }

        if (ent == _model)
        {
            int i, nb = _model.EntityCount;
            for (i = 1; i <= nb; i++)
            {
                ResultFromModel rec = ResultFromNumber(i);
                if (rec is not null)
                {
                    res.Append(rec.CheckedList(withcheck, level));
                }
            }
        }
        else if (ent is NCollection_HSequence<Transient> list)
        {
            int i, nb = list.Length();
            for (i = 1; i <= nb; i++)
            {
                ResultFromModel rec = FinalResult(list.Value(i));
                if (rec is not null)
                {
                    res.Append(rec.CheckedList(withcheck, level));
                }
            }
        }
        else
        {
            ResultFromModel rec = FinalResult(ent);
            if (rec is not null)
            {
                res = rec.CheckedList(withcheck, level);
            }
        }
        return res;
    }

    /// <summary>
    /// Defines a new TransferProcess for reading transfer Returns True if done, False if data are not properly
    /// defined (the Model, the Actor for Read)
    /// </summary>
    public bool BeginTransfer()
    {
        if (_model is null)
        {
            return false;
        }
        if (Actor() is null)
        {
            return false;
        }
        _shapeResult = null;

        if (_tp is null)
        {
            _tp = new TransientProcess(_model.EntityCount);
        }

        ActorOfTransientProcess actor = null;
        _tp.SetActor(actor); // -> Reset
        actor = Actor();
        _tp.SetActor(actor); // Set properly
        _tp.SetErrorHandle(true);
        Dictionary<string, Transient> tpContext = _tp.Context();
        AssignContext(tpContext, _context); // C++'s map assignment: a copy
        return true;
    }

    /// <summary>
    /// Tells if an entity is recognized as a valid candidate for Transfer. Calls method Recognize from the Actor
    /// (if known)
    /// </summary>
    public bool Recognize(Transient? ent)
    {
        if (_actor is null)
        {
            return false;
        }
        return _actor.Recognize(ent);
    }

    /// <summary>
    /// Commands the transfer on reading for an entity to data for Imagine, using the selected Actor for Read
    /// Returns count of transferred entities, ok or with fails (0/1) If &lt;rec&gt; is True (D), the result is
    /// recorded by RecordResult
    /// </summary>
    public int TransferOne(Transient? ent, bool rec = true, ProgressRange progress = default)
    {
        if (_actor is null || _model is null)
        {
            return 0;
        }

        if (_tp is null)
        {
            if (!BeginTransfer())
            {
                return 0;
            }
        }

        using Messenger.StreamBuffer sout = _tp.Messenger().SendInfo();
        int level = _tp.TraceLevel();

        TransferOutput tp = new TransferOutput(_tp, _model);
        if (_graph is null)
        {
            _tp.SetModel(_model);
        }
        else
        {
            _tp.SetGraph(_graph);
        }

        //  for the log-file
        if (level > 1)
        {
            int num = _model.Number(ent);
            TCollection_HAsciiString lab = _model.StringLabel(ent);
            sout.Append("\n*******************************************************************\n");
            sout.Append("******           Transferring one Entity                     ******");
            sout.Append('\n');
            if (lab is not null)
            {
                sout.Append("******    N0 in file : ");
                sout.Append(FrameworkMsg.Blanks(num, 5));
                sout.Append(num);
                sout.Append("      Ident : ");
                sout.Append(lab.ToCString());
                sout.Append(FrameworkMsg.Blanks(14 - lab.Length()));
                sout.Append("******\n");
            }
            sout.Append("******    Type : ");
            sout.Append(_model.TypeName(ent, false));
            sout.Append(FrameworkMsg.Blanks(44 - _model.TypeName(ent, false).Length));
            sout.Append("******");
            sout.Append("\n*******************************************************************\n");
        }

        //  only difference between TransferRoots and TransferOne
        int res = 0;
        Transient obj = ent;
        tp.Transfer(obj, progress);
        if (progress.UserBreak())
        {
            return res;
        }
        _tp.SetRoot(obj);

        //  Result ...
        Binder binder = _tp.Find(obj);
        if (binder is null)
        {
            return res;
        }
        if (rec)
        {
            RecordResult(obj);
        }

        if (!binder.HasResult)
        {
            return res;
        }
        res++;

        return res;
    }

    /// <summary>
    /// Commands the transfer on reading for a list of entities to data for Imagine, using the selected Actor for
    /// Read Returns count of transferred entities, ok or with fails (0/1) If &lt;rec&gt; is True (D), the
    /// results are recorded by RecordResult
    /// </summary>
    public int TransferList(NCollection_HSequence<Transient>? list, bool rec = true, ProgressRange progress = default)
    {
        if (_actor is null || _model is null)
        {
            return 0;
        }

        if (_tp is null)
        {
            if (!BeginTransfer())
            {
                return 0;
            }
        }

        int level = _tp.TraceLevel();

        TransferOutput tp = new TransferOutput(_tp, _model);
        if (_graph is null)
        {
            _tp.SetModel(_model);
        }
        else
        {
            _tp.SetGraph(_graph);
        }

        int i, nb = list.Length();

        //   For the log-file
        if (level > 0)
        {
            StringWriter sout = new StringWriter(); // OCCT: one stream buffer, sent at its end
            sout.Write("\n*******************************************************************\n");
            sout.Write("******           Transferring a list of ");
            sout.Write(FrameworkMsg.Blanks(nb, 5));
            sout.Write(" Entities       ******");
            sout.Write('\n');
            sout.Write("\n*******************************************************************\n");

            SignatureList sl = new SignatureList();
            for (i = 1; i <= nb; i++)
            {
                sl.Add(list.Value(i), _model.TypeName(list.Value(i), false));
            }
            sl.SetName("Entities to Transfer");
            sl.PrintCount(sout);
            sout.Write("\n*******************************************************************\n");
            _tp.Messenger().SendInfo().Append(sout.ToString()).Flush();
        }

        //  only difference between TransferRoots and TransferOne
        int res = 0;
        nb = list.Length();
        Transient obj = null;
        ProgressScope ps = new ProgressScope(progress, null, nb);
        for (i = 1; i <= nb && ps.More(); i++)
        {
            obj = list.Value(i);
            tp.Transfer(obj, ps.Next());
            _tp.SetRoot(obj);

            //  Result ...
            Binder binder = _tp.Find(obj);
            if (binder is null)
            {
                continue;
            }
            if (rec)
            {
                RecordResult(obj);
            }

            if (!binder.HasResult)
            {
                continue;
            }
            res++;
        }
        return res;
    }

    /// <summary>
    /// Transfers the content of the current Interface Model to data handled by Imagine, starting from its Roots
    /// (determined by the Graph &lt;G&gt;), using the selected Actor for Read Returns the count of performed
    /// root transfers (i.e. 0 if none) or -1 if no actor is defined
    /// </summary>
    public int TransferRoots(EntityGraph g, ProgressRange progress = default)
    {
        if (_model != g.Model())
        {
            return -1;
        }
        if (!BeginTransfer())
        {
            return -1;
        }
        int level = _tp.TraceLevel();

        TransferOutput tp = new TransferOutput(_tp, _model);
        if (_graph is null)
        {
            _tp.SetModel(_model);
        }
        else
        {
            _tp.SetGraph(_graph);
        }

        //   For the log-file
        if (level > 0)
        {
            EntityIterator roots = g.RootEntities();
            int nb = roots.NbEntities();
            StringWriter sout = new StringWriter(); // OCCT: one stream buffer, sent at its end
            sout.Write("\n*******************************************************************\n");
            sout.Write("******           Transferring the ");
            sout.Write(FrameworkMsg.Blanks(nb, 5));
            sout.Write(" Root Entities        ******");
            sout.Write('\n');
            sout.Write("\n*******************************************************************\n");
            SignatureList sl = new SignatureList();
            for (roots.Start(); roots.More(); roots.Next())
            {
                sl.Add(roots.Value(), _model.TypeName(roots.Value(), false));
            }
            sl.SetName("Entities to Transfer");
            sl.PrintCount(sout);
            sout.Write("\n*******************************************************************\n");
            _tp.Messenger().SendInfo().Append(sout.ToString()).Flush();
        }

        tp.TransferRoots(g, progress);
        if (progress.UserBreak())
        {
            return -1;
        }

        //  The transferred entities are noted as "asmain"
        int i, n = _tp.MappedCount;
        for (i = 1; i <= n; i++)
        {
            Transient ent = _tp.Mapped(i);
            Binder bnd = _tp.MapItem(i);
            if (bnd is null)
            {
                continue;
            }
            if (!bnd.HasResult)
            {
                continue;
            }
            RecordResult(ent);
        }

        //  Result ... we carefully note the Shapes
        _shapeResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Shapes(_tp, true);
        // ????  Et ici, il faut alimenter Imagine ...
        return _shapeResult.Length();
    }

    /// <summary>
    /// Clears the results attached to an entity if &lt;ents&gt; equates the starting model, clears all results
    /// </summary>
    public void TransferClear(Transient? ent, int level = 0)
    {
        if (_tp is null)
        {
            return;
        }
        if (ent == _model)
        {
            _tp.Clear();
            return;
        }

        _tp.RemoveResult(ent, level);
        ClearResult(ent, -1);
    }

    /// <summary>
    /// Prints statistics on current Trace File, according &lt;what&gt; and &lt;mode&gt;. See PrintStatsProcess
    /// for details
    /// </summary>
    public void PrintStats(TextWriter sout, int what, int mode = 0)
    {
        //  To be improved ... !
        sout.Write("\n*******************************************************************\n");
        sout.Write("******        Statistics on Transfer (Read)                  ******");
        sout.Write('\n');
        sout.Write("\n*******************************************************************\n");
        if (what > 10)
        {
            sout.Write(" ***  Not yet implemented");
            sout.Write('\n');
            return;
        }
        if (what < 10)
        {
            sout.Write("******        Data recorded on Last Transfer                 ******");
            sout.Write('\n');
            PrintStatsProcess(_tp, what, mode);
        }
        //  remaining what = 10: we list the roots of final results
        sout.Write("******        Final Results                                  ******");
        sout.Write('\n');
        if (_model is null)
        {
            sout.Write("****    Model unknown");
            sout.Write('\n');
            return;
        }
        NCollection_HSequence<Transient> list = RecordedList();
        int i, nb = list.Length();
        SignatureList counter = null;
        if (mode > 2)
        {
            counter = new SignatureList(mode == 6);
        }
        PrintCount pcm = PrintCount.CountByItem;
        if (mode == 6)
        {
            pcm = PrintCount.ListByItem;
        }

        sout.Write("****    Nb Recorded : ");
        sout.Write(nb);
        sout.Write(" : entities n0s : ");
        for (i = 1; i <= nb; i++)
        {
            Transient ent = list.Value(i);
            if (mode == 0)
            {
                sout.Write("  ");
                sout.Write(_model.Number(ent));
                continue;
            }
            if (mode == 1 || mode == 2)
            {
                sout.Write("[ ");
                sout.Write(FrameworkMsg.Blanks(i, 6));
                sout.Write(" ]:");
                _model.Print(ent, sout);
                sout.Write("  Type:");
                sout.Write(_model.TypeName(ent, false));
            }
            if (mode >= 3 && mode <= 6)
            {
                counter.Add(ent, _model.TypeName(ent, false));
            }
        }
        if (counter is not null)
        {
            counter.PrintList(sout, _model, pcm);
        }

        sout.Write('\n');
    }

    /// <summary>
    /// Returns the CheckList resulting from last TransferRead i.e. from TransientProcess itself, recorded from
    /// last Clear
    /// </summary>
    public CheckIterator LastCheckList()
    {
        CheckIterator chl = new CheckIterator();
        if (_tp is not null)
        {
            chl = _tp.CheckList(false);
        }
        return chl;
    }

    /// <summary>
    /// Returns the list of entities recorded as lastly transferred i.e. from TransientProcess itself, recorded
    /// from last Clear If &lt;roots&gt; is True , considers only roots of transfer If &lt;roots&gt; is False,
    /// considers all entities bound with result
    /// </summary>
    public NCollection_HSequence<Transient>? LastTransferList(bool roots)
    {
        NCollection_HSequence<Transient> li =
            new NCollection_HSequence<Transient>();
        if (_tp is null)
        {
            return li;
        }
        int i, j, nb = (roots ? _tp.RootCount : _tp.MappedCount);
        for (j = 1; j <= nb; j++)
        {
            i = (roots ? _model.Number(_tp.Root(j)) : j);
            Binder bnd = _tp.MapItem(i);
            if (bnd is null)
            {
                continue;
            }
            if (!bnd.HasResult)
            {
                continue;
            }
            li.Append(_tp.Mapped(i));
        }
        return li;
    }

    /// <summary>
    /// Returns a list of result Shapes If &lt;rec&gt; is True , sees RecordedList If &lt;rec&gt; is False, sees
    /// LastTransferList (last ROOT transfers) For each one, if it is a Shape, it is cumulated to the list If no
    /// Shape is found, returns an empty Sequence
    /// </summary>
    public NCollection_HSequence<Shape>? ShapeResultList(bool rec)
    {
        if (!rec)
        {
            if (_shapeResult is null)
            {
                _shapeResult = global::Kerf.Engine.Exchange.Framework.Transfer.TransferBRep.Shapes(_tp, true);
            }
            if (_shapeResult is null)
            {
                _shapeResult = new NCollection_HSequence<Shape>();
            }
        }
        else
        {
            if (_shapeResult is null)
            {
                _shapeResult = new NCollection_HSequence<Shape>();
            }
            if (_model is null)
            {
                return _shapeResult;
            }
            NCollection_HSequence<Transient> li = RecordedList();
            _shapeResult = new NCollection_HSequence<Shape>();
            int i, nb = _model.EntityCount;
            Shape sh;
            for (i = 1; i <= nb; i++)
            {
                sh = ShapeResult(_model.Value(i));
                if (!sh.IsNull)
                {
                    _shapeResult.Append(sh);
                }
            }
        }
        return _shapeResult;
    }

    /// <summary>
    /// This routines prints statistics about a TransientProcess It can be called, by a TransferReader, or
    /// isolately Prints are done on the default trace file &lt;what&gt; defines what kind of statistics are to
    /// be printed : 0 : basic figures 1 : root results 2 : all recorded (roots, intermediate, checked entities)
    /// 3 : abnormal records 4 : check messages (warnings and fails) 5 : fail messages &lt;mode&gt; is used
    /// according &lt;what&gt; : &lt;what&gt; = 0 : &lt;mode&gt; is ignored &lt;what&gt; = 1,2,3 : &lt;mode&gt;
    /// as follows : 0 (D) : just lists numbers of concerned entities in the model 1 : for each entity, gives
    /// number,label, type and result type and/or status (fail/warning...) 2 : for each entity, gives maximal
    /// information (i.e. checks) 3 : counts per type of starting entity (class type) 4 : counts per result type
    /// and/or status 5 : counts per couple (starting type / result type/status) 6 : idem plus gives for each
    /// item, the list of numbers of entities in the starting model &lt;what&gt; = 4,5 : modes relays on an enum
    /// PrintCount : 0 (D) : ItemsByEntity (sequential list by entity) 1 : CountByItem 2 : ShortByItem (count + 5
    /// first numbers) 3 : ListByItem (count + entity numbers) 4 : EntitiesByItem (count + entity numbers and
    /// labels)
    /// </summary>
    public static void PrintStatsProcess(TransientProcess? tp, int what, int mode = 0)
    {
        NCollection_HSequence<Transient> list = null; // null
        TransferReader.PrintStatsOnList(tp, list, what, mode);
    }

    /// <summary>
    /// Works as PrintStatsProcess, but displays data only on the entities which are in &lt;list&gt; (filter)
    /// </summary>
    public static void PrintStatsOnList(TransientProcess? tp, NCollection_HSequence<Transient>? list, int what, int mode = 0)
    {
        // OCCT: one stream buffer of TP's messenger (made before TP is tested), sent when the function returns
        if (tp is null)
        {
            return;
        }
        StringWriter sout = new StringWriter();
        try
        {
            PrintStatsOnList(tp, list, what, mode, sout);
        }
        finally
        {
            if (sout.GetStringBuilder().Length > 0)
            {
                tp.Messenger().SendInfo().Append(sout.ToString()).Flush();
            }
        }
    }

    internal static void PrintStatsOnList(TransientProcess tp, NCollection_HSequence<Transient>? list,
                                         int what, int mode, TextWriter sout)
    {
        string mess = "";
        if (what == 0)
        {
            tp.PrintStats(0, sout);
            return;
        }

        sout.Write("\n*******************************************************************\n");
        sout.Write("******        Statistics on Transfer Process (Read)          ******");
        sout.Write('\n');
        if (what == 1)
        {
            sout.Write("******        Individual Transfers  (Roots)                  ******\n");
        }
        if (what == 2)
        {
            sout.Write("******        All recorded data about Transfer               ******\n");
        }
        if (what == 3)
        {
            sout.Write("******        Abnormal records                               ******\n");
        }
        if (what == 1 || what == 2 || what == 3)
        {
            if (mode == 0)
            {
                sout.Write("******        (n0s of recorded entities)                     ******\n");
            }
            if (mode == 1)
            {
                sout.Write("******        (per entity : type + result)                   ******\n");
            }
            if (mode == 2)
            {
                sout.Write("******        (per entity : type + result/status)            ******\n");
            }
            if (mode == 3)
            {
                sout.Write("******        (count per type of entity)                     ******\n");
            }
            if (mode == 4)
            {
                sout.Write("******        (count per type of result)                     ******\n");
            }
            if (mode == 5)
            {
                sout.Write("******   (count per couple entity-type / result-type/status) ******\n");
            }
            if (mode == 6)
            {
                sout.Write("******   (list per couple entity-type / result-type/status)  ******\n");
            }
        }
        if (what == 4)
        {
            sout.Write("******        Check messages                                 ******\n");
        }
        if (what == 5)
        {
            sout.Write("******        Fail  messages                                 ******\n");
        }
        sout.Write("*******************************************************************\n");

        //  Case what = 1,2,3: content of TP (binders)

        bool nolist = list is null;
        InterfaceModel model = tp.Model();
        if (what >= 1 && what <= 3)
        {

            int stat;
            int nbw = 0, nbf = 0, nbr = 0, nbrw = 0, nbrf = 0, nbnr = 0, nbi = 0;
            IteratorOfProcessForTransient itrp = new IteratorOfProcessForTransient(true);
            if (what == 1)
            {
                itrp = tp.RootResult(true);
            }
            if (what == 2)
            {
                itrp = tp.CompleteResult(true);
            }
            if (what == 3)
            {
                itrp = tp.AbnormalResult();
            }
            int i = 0, nb = itrp.Number();
            if (!nolist)
            {
                itrp.Filter(list);
            }
            int nl = itrp.Number(); // after filtering
            SignatureList counter = null;
            if (mode > 2)
            {
                counter = new SignatureList(mode == 6);
            }
            bool                notrec = (!nolist && mode > 2); // note the "no record"
            PrintCount pcm = PrintCount.CountByItem;
            if (mode == 6)
            {
                pcm = PrintCount.ListByItem;
            }

            sout.Write("****        Entities in Model   : ");
            sout.Write(model.EntityCount);
            sout.Write('\n');
            sout.Write("****        Nb Items (Transfer) : ");
            sout.Write(nb);
            sout.Write('\n');
            if (!nolist)
            {
                sout.Write("****        Nb Items (Listed)   : ");
                sout.Write(nl);
                sout.Write('\n');
            }

            for (itrp.Start(); itrp.More(); itrp.Next())
            {
                nbi++;
                Binder binder = itrp.Value();
                Transient ent = itrp.Starting();
                if (binder is null)
                {
                    nbnr++;
                    if (notrec)
                    {
                        counter.Add(ent, "(not recorded)");
                    }
                    else if (mode == 1 || mode == 2)
                    {
                        sout.Write("[");
                        sout.Write(FrameworkMsg.Blanks(nbi, 4));
                        sout.Write(nbi);
                        sout.Write(" ]:");
                        model.Print(ent, sout);
                        sout.Write("   ");
                        sout.Write(model.TypeName(ent, false));
                        sout.Write("  (not recorded)");
                        sout.Write('\n');
                        continue;
                    }
                }
                if (mode == 0)
                {
                    sout.Write("  ");
                    sout.Write(model.Number(ent));
                    continue;
                }
                if (mode != 3)
                {
                    stat = BinderStatus(binder, out mess);
                    // 0 Binder Null.   1 void  2 Warning only  3 Fail only
                    // 11 Result OK. 12 Result+Warning. 13 Result+Fail
                    if (stat == 2)
                    {
                        nbw++;
                    }
                    if (stat == 3)
                    {
                        nbf++;
                    }
                    if (stat == 11)
                    {
                        nbr++;
                    }
                    if (stat == 12)
                    {
                        nbrw++;
                    }
                    if (stat == 13)
                    {
                        nbrf++;
                    }
                }

                //  mode : 0 list num;  1 : num+label + type + result (abrege);  2 : complet
                if (mode == 1 || mode == 2)
                {
                    sout.Write("[");
                    sout.Write(FrameworkMsg.Blanks(i, 4));
                    sout.Write(i);
                    sout.Write(" ]:");
                    model.Print(ent, sout);
                    sout.Write("   ");
                    sout.Write(model.TypeName(ent, false));
                    sout.Write("\tResult:");
                    sout.Write(mess);
                    sout.Write('\n');
                    if (mode == 1)
                    {
                        continue;
                    }

                    FrameworkCheck ch = binder.Check();
                    int newi, newnbw = ch.WarningCount, newnbf = ch.FailCount;

                    if (newnbw > 0)
                    {
                        sout.Write(" - Warnings : ");
                        sout.Write(newnbw);
                        sout.Write(":\n");
                        for (newi = 1; newi <= newnbw; newi++)
                        {
                            sout.Write(ch.CWarning(newi));
                            sout.Write('\n');
                        }
                    }
                    if (newnbf > 0)
                    {
                        sout.Write(" - Fails : ");
                        sout.Write(newnbf);
                        sout.Write(":\n");
                        for (newi = 1; newi <= newnbf; newi++)
                        {
                            sout.Write(ch.CFail(newi));
                            sout.Write('\n');
                        }
                    }
                    continue;
                }

                //  mode : 3, counts per type of starting entity (class type)
                //         4 : counts per result type and/or status
                //         5 : counts per couple (starting type / result type/status)
                //         6 : idem plus gives for each item, the list of numbers of
                //                  entities in the starting model
                if (mode >= 3 && mode <= 6)
                {
                    // IFSelect_PrintCount newpcm = IFSelect_CountByItem;
                    // if (mode == 6) newpcm = IFSelect_ListByItem;
                    if (mode == 3)
                    {
                        counter.Add(ent, model.TypeName(ent, false));
                    }
                    if (mode == 4)
                    {
                        counter.Add(ent, mess);
                    }
                    if (mode >= 5)
                    {
                        string mest = model.TypeName(ent, false);
                        mest += "\t-> ";
                        mest += mess;
                        // Sprintf(mest,"%s\t-> %s",model.TypeName(ent,false),mess);
                        counter.Add(ent, mest);
                    }
                }

                //    End of iteration
            }
            if (counter is not null)
            {
                counter.PrintList(sout, model, pcm);
            }
            else
            {
                sout.Write('\n');
            }
            //    Pourcentages
            if (mode != 3 && nbi > 0)
            {
                sout.Write("******        Percentages according Transfer Status          ******");
                sout.Write('\n');
                PrintPercent(tp.Messenger(), "Result          ", nbr + nbrw, nl);
                PrintPercent(tp.Messenger(), "Result + FAIL   ", nbrf, nl);
                PrintPercent(tp.Messenger(), "FAIL, no Result ", nbf, nl);
                PrintPercent(tp.Messenger(), "Just Warning    ", nbw, nl);
                PrintPercent(tp.Messenger(), "Nothing Recorded", nbnr, nl);
                // (OCCT's former percentages, commented out, left out)
            }
            return;
        }

        //  Case what = 4,5: check-list

        if (what == 4 || what == 5)
        {

            CheckIterator chl = tp.CheckList(false);
            chl.SetName("** TRANSFER READ CHECK **");
            if (mode == 0)
            {
                chl.Print(sout, model, (what == 5));
            }
            else
            {
                PrintCount pcm = PrintCount.CountByItem;
                if (mode == 2)
                {
                    pcm = PrintCount.ListByItem;
                }
                CheckCounter counter = new CheckCounter(true);
                counter.Analyse(chl, model, true, (what == 5));
                counter.PrintList(sout, model, pcm);
            }
        }
    }

    private Controller? _controller;
    private string _fileName = "";
    private InterfaceModel? _model;
    private HGraph? _graph;
    private Dictionary<string, Transient> _context = new Dictionary<string, Transient>();
    private ActorOfTransientProcess? _actor;
    private TransientProcess? _tp;
    private Dictionary<int, Transient> _results = new Dictionary<int, Transient>();
    private StaticSet? _statics;
    private NCollection_HSequence<Shape>? _shapeResult;
}
