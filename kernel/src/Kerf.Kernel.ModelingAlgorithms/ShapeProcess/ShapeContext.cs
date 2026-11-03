// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_ShapeContext.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_ShapeContext.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Process;

/// <summary>
/// The context for shape processing: the shape, the result of the operators run so far, the history from the shape's
/// sub-shapes (down to the detalisation level) to the result's, and the messages attached to the shape's sub-shapes.
/// </summary>
/// <remarks>
/// <para>
/// <b>The working copy</b> (decision 1022). OCCT's operators write into the shape they process: SetTolerance and
/// SameParameter change tolerances and pcurves in place, the modifiers run BRepTools_Modifier in its mutable-input
/// mode, which updates the input's vertices. A shape a caller holds is frozen, so while operators run
/// (<see cref="ShapeProcess"/>'s Perform, or an operator run on its own) the context's result is an exact copy
/// (<c>BRepTools_CopyAtEntry</c>) of a result holding frozen shapes, the operators work on it in place as OCCT's do,
/// and the history keeps OCCT's keys: a sub-shape of <see cref="Shape"/> not yet in the map stands for its copy. At
/// the end the copy's sub-shapes left unchanged are the caller's own again, in the result and in the map's values,
/// so a shape that needed nothing comes back itself. The map has OCCT's entries: a sub-shape changed in place (a
/// tolerance raised, a vertex updated) has none, as natively, where it is the same TShape; in the port the result
/// holds a changed copy of it.
/// </para>
/// <para>
/// <see cref="Result"/> and <see cref="Map"/> are publication points: they freeze the result and the map's values.
/// Ported operators read <see cref="InternalResult"/>.
/// </para>
/// </remarks>
public class ShapeContext : ProcessContext
{
    private Shape _shape;
    private Shape _result;
    private readonly NCollection_DataMap<Shape, Shape> _map = new(ShapeHasher.Instance);
    private MsgRegistrator? _msg;
    private ShapeType _until;
    private bool _nonManifold;

    // OCCT.NET: the working copy of the result while operators run (decision 1022)
    private CopyAtEntry? _entry;
    private int _writeDepth;

    /// <summary>Creates a context with the resource file <paramref name="file"/> and the scope <paramref name="seq"/>.</summary>
    public ShapeContext(string file, string seq = "", string? defaultsDirectory = null,
                                     string? userDefaultsDirectory = null)
        : base(file, seq, defaultsDirectory, userDefaultsDirectory)
    {
        _until = ShapeType.Face;
        _nonManifold = false;
    }

    /// <summary>
    /// Creates a context with the shape <paramref name="s"/>, the resource file <paramref name="file"/> and the scope
    /// <paramref name="seq"/>.
    /// </summary>
    public ShapeContext(in Shape s, string file, string seq = "", string? defaultsDirectory = null,
                                     string? userDefaultsDirectory = null)
        : base(file, seq, defaultsDirectory, userDefaultsDirectory)
    {
        _until = ShapeType.Face;
        _nonManifold = false;
        Init(s);
    }

    /// <summary>Initialises the context with the shape: the result is the shape, the map and the messages are cleared.</summary>
    public void Init(in Shape s)
    {
        if (_writeDepth > 0)
        {
            throw new ProgramErrorException("ShapeProcess_ShapeContext::Init called while operators run");
        }
        _map.Clear();
        _msg = new MsgRegistrator();
        _shape = s;
        _result = _shape; //.Nullify();
    }

    /// <summary>The shape being processed.</summary>
    public Shape Shape() => _shape;

    /// <summary>The current result, frozen.</summary>
    public Shape Result()
    {
        _result.TShape()?.FreezeGraph();
        return _result;
    }

    /// <summary><see cref="Result"/> without freezing it, for the operators.</summary>
    internal Shape InternalResult() => _result;

    /// <summary>
    /// The history: each sub-shape of <see cref="Shape"/> (down to the detalisation level) that was modified, with
    /// what it became (null if removed). Its values are frozen.
    /// </summary>
    public NCollection_DataMap<Shape, Shape> Map()
    {
        foreach (KeyValuePair<Shape, Shape> item in _map)
        {
            item.Value.TShape()?.FreezeGraph();
        }
        return _map;
    }

    /// <summary><see cref="Map"/> without freezing its values, for ported code.</summary>
    internal NCollection_DataMap<Shape, Shape> InternalMap() => _map;

    /// <summary>The messages attached to the shape's sub-shapes (null if messages are off).</summary>
    public MsgRegistrator? Messages() => _msg;

    /// <summary>Sets the registrator of messages (null switches them off).</summary>
    public void SetMessages(MsgRegistrator? messages)
    {
        // OCCT's Messages() returns a modifiable reference
        _msg = messages;
    }

    /// <summary>Sets the level down to which the history is recorded (TopAbs_SHAPE: all sub-shapes).</summary>
    public void SetDetalisation(ShapeType level)
    {
        _until = level;
    }

    /// <summary>The level down to which the history is recorded.</summary>
    public ShapeType GetDetalisation() => _until;

    /// <summary>Sets the result, and records it as what the shape became.</summary>
    public void SetResult(in Shape s)
    {
        _result = s;
        _map.Bind(_shape, _result);
    }

    /// <summary>Sets the non-manifold flag, which FixShape passes to ShapeFix_Shell.</summary>
    public void SetNonManifold(bool nonManifold)
    {
        _nonManifold = nonManifold;
    }

    /// <summary>The non-manifold flag.</summary>
    public bool IsNonManifold => _nonManifold;

    // ---------------------------------------------------------------------------------------------------------------
    // OCCT.NET: the working copy (decision 1022)

    /// <summary>
    /// Starts a run of operators: at the outermost call, a result holding frozen shapes is replaced by an exact copy,
    /// and the map's values by their copies.
    /// </summary>
    internal void InternalBeginWrite()
    {
        if (_writeDepth++ > 0)
        {
            InternalEnsureWritable();
            return;
        }
        BeginCopy();
    }

    /// <summary>
    /// Ends a run of operators: at the outermost call, the copy's sub-shapes left unchanged are the caller's own again,
    /// in the result and in the map's values.
    /// </summary>
    internal void InternalEndWrite()
    {
        if (--_writeDepth > 0)
        {
            return;
        }
        EndCopy();
    }

    /// <summary>
    /// Inside a run of operators, takes a new copy if the result holds frozen shapes again (an operator published it
    /// through <see cref="Result"/>), so that the next operator can write into it.
    /// </summary>
    internal void InternalEnsureWritable()
    {
        if (_writeDepth > 0 && !_result.IsNull && BrepTools.HoldsFrozen(_result))
        {
            EndCopy();
            BeginCopy();
        }
    }

    private void BeginCopy()
    {
        _entry = null;
        if (_result.IsNull || !BrepTools.HoldsFrozen(_result))
        {
            return;
        }

        _entry = new CopyAtEntry(_result, null, true);
        _result = _entry.Shape();
        var keys = new List<Shape>(_map.Keys);
        foreach (Shape key in keys)
        {
            _map.Bind(key, Copied(_map.Find(key)));
        }
    }

    private void EndCopy()
    {
        CopyAtEntry? entry = _entry;
        _entry = null;
        if (entry is null)
        {
            return;
        }

        var values = new List<Shape>();
        foreach (KeyValuePair<Shape, Shape> item in _map)
        {
            values.Add(item.Value);
        }
        _result = entry.Finish(_result, values);
        var keys = new List<Shape>(_map.Keys);
        foreach (Shape key in keys)
        {
            _map.Bind(key, entry.Final(_map.Find(key)));
        }
    }

    // The copy of a shape of the result (a compound made of them, as RecModif makes for a split shape, rebuilt on
    // their copies; any other shape is itself).
    private Shape Copied(in Shape shape)
    {
        if (shape.IsNull || _entry is null)
        {
            return shape;
        }
        Shape copy = _entry.Copied(shape);
        if (!ReferenceEquals(copy.TShape(), shape.TShape()) || shape.ShapeType() != ShapeType.Compound)
        {
            return copy;
        }

        bool isChanged = false;
        var children = new List<Shape>();
        for (var it = new Iterator(shape, false, false); it.More(); it.Next())
        {
            Shape child = Copied(it.Value());
            isChanged |= !ReferenceEquals(child.TShape(), it.Value().TShape());
            children.Add(child);
        }
        if (!isChanged)
        {
            return shape;
        }

        Shape result = shape.EmptyCopied();
        result.Orientation(Forward);
        var b = new BrepBuilder();
        foreach (Shape child in children)
        {
            b.Add(result, child);
        }
        result.Orientation(shape.Orientation());
        result.Location(shape.Location(), false);
        return result;
    }

    // What a sub-shape of the shape not yet in the map stands for: its working copy (itself when there is none)
    private Shape Working(in Shape shape) => _entry is null ? shape : _entry.Copied(shape);

    // ---------------------------------------------------------------------------------------------------------------

    private static void RecModif(in Shape s,
                                 NCollection_DataMap<Shape, Shape> repl,
                                 MsgRegistrator? msg,
                                 NCollection_DataMap<Shape, Shape> map,
                                 MsgRegistrator? myMsg,
                                 ShapeType until,
                                 ShapeContext context)
    {
        Shape r = s;
        // gka  -modification to keep history for shape with location (OCC21617)
        Placement shLoc = s.Location();
        var nullLoc = new Placement();
        r.Location(nullLoc);

        if (map.IsBound(r))
        {
            r = map.Find(r);
        }
        else
        {
            r = context.Working(r); // OCCT.NET: the working copy (decision 1022)
        }
        if (!r.IsNull)
        {
            Shape res = r;

            if (repl.IsBound(r.Located(shLoc, false)))
            {
                res = repl.Find(r.Located(shLoc, false));
                // it is supposed that map is created for r having FORWARD orientation
                // hence, if it is reversed, result should be reversed too
                // INTERNAL or EXTERNAL orientations are not allowed
                if (r.Orientation() != Forward)
                {
                    res.Reverse();
                }
            }
            // Treat special case: if S was split, r will be a compound of
            // resulting shapes, each to be checked separately
            // It is supposed that repl does not contain such splitting
            else if (r.ShapeType() < s.ShapeType())
            {
                Shape result = r.EmptyCopied();
                result.Orientation(Forward); // protect against INTERNAL or EXTERNAL shapes
                bool modif = false;
                var b = new BrepBuilder();
                for (var it = new Iterator(r, false); it.More(); it.Next())
                {
                    Shape sh = it.Value();
                    if (repl.IsBound(sh))
                    {
                        Shape newsh = repl.Find(sh);
                        if (!newsh.IsNull)
                        {
                            b.Add(result, newsh);
                        }
                        modif = true;
                    }
                    else
                    {
                        b.Add(result, sh);
                    }
                }
                if (modif)
                {
                    if (result.ShapeType() == ShapeType.Wire || result.ShapeType() == ShapeType.Shell)
                    {
                        result.Closed(Tool.IsClosed(result));
                    }
                    res = result;
                }
            }

            if (res != r)
            {
                map.Bind(s.Located(nullLoc), res);
            }
        }

        // update messages (messages must be taken from each level in the substitution map)
        if (!r.IsNull && myMsg is not null && msg is not null && msg.MapShape().Extent() > 0)
        {
            NCollection_DataMap<Shape, NCollection_List<Msg>> msgmap = msg.MapShape();
            if (msgmap.IsBound(r))
            {
                NCollection_List<Msg> msglist = msgmap.Find(r);
                for (var iter = new NCollection_List<Msg>.Iterator(msglist); iter.More(); iter.Next())
                {
                    myMsg.Send(s, iter.Value(), Warning);
                }
            }
            else if (msgmap.IsBound(context.Working(s))) // OCCT.NET: S as the operator saw it (decision 1022)
            {
                NCollection_List<Msg> msglist = msgmap.Find(context.Working(s));
                for (var iter = new NCollection_List<Msg>.Iterator(msglist); iter.More(); iter.Next())
                {
                    myMsg.Send(s, iter.Value(), Warning);
                }
            }
        }

        if (until == global::Kerf.Topology.ShapeType.Shape || s.ShapeType() >= until)
        {
            return;
        }

        for (var it = new Iterator(s); it.More(); it.Next())
        {
            RecModif(it.Value(), repl, msg, map, myMsg, until, context);
        }
    }

    /// <summary>
    /// Records the modifications <paramref name="repl"/> (from the shapes the operator worked on to what they became)
    /// in the history, for every sub-shape of the shape down to the detalisation level, with their messages.
    /// </summary>
    public void RecordModification(NCollection_DataMap<Shape, Shape> repl, MsgRegistrator? msg = null)
    {
        if (repl.Extent() <= 0)
        {
            return;
        }
        RecModif(_shape, repl, msg, _map, _msg, _until, this);
        if (_map.IsBound(_shape))
        {
            _result = _map.Find(_shape);
        }
        // OCCT_DEBUG: the map dumped, left out
    }

    private static void RecModif(in Shape s,
                                 ReShapeBuilder repl,
                                 MsgRegistrator? msg,
                                 NCollection_DataMap<Shape, Shape> map,
                                 MsgRegistrator? myMsg,
                                 ShapeType until,
                                 ShapeContext context)
    {
        if (s.IsNull)
        {
            return;
        }
        // gka  -modification to keep history for shape with location (OCC21617)
        var nullLoc = new Placement();
        Shape aS = s.Located(nullLoc);
        Shape r = aS;

        if (map.IsBound(r))
        {
            r = map.Find(r);
        }
        else
        {
            r = context.Working(r); // OCCT.NET: the working copy (decision 1022)
        }
        if (!r.IsNull)
        {
            if (repl.InternalStatus(r, out Shape res, true) != 0 && res != r)
            {
                map.Bind(aS, res);

                // Treat special case: if S was split, r will be a compound of
                // resulting shapes, recursive procedure should be applied
            }
            else if (r.ShapeType() < s.ShapeType())
            {
                res = repl.InternalApply(r, (ShapeType)((int)s.ShapeType() + 1));
                if (res != r)
                {
                    map.Bind(aS, res);
                }
            }
        }

        // update messages (messages must be taken from each level in the substitution map)
        if (!r.IsNull && myMsg is not null && msg is not null && msg.MapShape().Extent() > 0)
        {
            Shape cur, next = r;
            NCollection_DataMap<Shape, NCollection_List<Msg>> msgmap = msg.MapShape();
            Shape workS = context.Working(s); // OCCT.NET: S as the operator saw it (decision 1022)
            if (msgmap.IsBound(workS))
            {
                next = workS;
            }
            do
            {
                cur = next;
                if (msgmap.IsBound(cur))
                {
                    NCollection_List<Msg> msglist = msgmap.Find(cur);
                    for (var iter = new NCollection_List<Msg>.Iterator(msglist); iter.More(); iter.Next())
                    {
                        myMsg.Send(s, iter.Value(), Warning);
                    }
                }
                next = repl.InternalValue(cur);
            } while (!next.IsNull && cur != next);
        }

        if (until == global::Kerf.Topology.ShapeType.Shape || s.ShapeType() >= until)
        {
            return;
        }

        for (var it = new Iterator(s, false /*,false*/); it.More(); it.Next())
        {
            RecModif(it.Value(), repl, msg, map, myMsg, until, context);
        }
    }

    /// <summary>
    /// Records the modifications a ShapeBuild_ReShape made (the requests recorded in it) in the history, for every
    /// sub-shape of the shape down to the detalisation level, with their messages.
    /// </summary>
    public void RecordModification(ReShapeBuilder repl, MsgRegistrator? msg)
    {
        RecModif(_shape, repl, msg, _map, _msg, _until, this);
        if (_map.IsBound(_shape))
        {
            _result = _map.Find(_shape);
            _result.Location(_shape.Location(), false);
        }
        // OCCT_DEBUG: the map dumped, left out
    }

    /// <summary>Records the modifications a ShapeBuild_ReShape made, without messages.</summary>
    public void RecordModification(ReShapeBuilder repl)
    {
        RecordModification(repl, null);
    }

    /// <summary>Adds a message for the sub-shape <paramref name="s"/> (if messages are on).</summary>
    public void AddMessage(in Shape s, Msg msg, Gravity grv = Warning)
    {
        _msg?.Send(s, msg, grv);
    }

    private static void ExplodeModifier(in Shape s,
                                        Modifier repl,
                                        NCollection_DataMap<Shape, Shape> map,
                                        ShapeType until)
    {
        Shape res = repl.InternalModifiedShape(s);

        if (res != s)
        {
            map.Bind(s, res);
        }
        if (until == global::Kerf.Topology.ShapeType.Shape || s.ShapeType() >= until)
        {
            return;
        }
        for (var it = new Iterator(s); it.More(); it.Next())
        {
            ExplodeModifier(it.Value(), repl, map, until);
        }
    }

    /// <summary>
    /// Records the modifications a BRepTools_Modifier made on <paramref name="s"/> in the history, down to the
    /// detalisation level, with their messages.
    /// </summary>
    public void RecordModification(in Shape s, Modifier repl, MsgRegistrator? msg)
    {
        var map = new NCollection_DataMap<Shape, Shape>(ShapeHasher.Instance);
        ExplodeModifier(s, repl, map, _until);
        RecordModification(map, msg);
    }

    /// <summary>The value of a continuity parameter (C0, G1, C1, G2, C2, C3, CN); false if it isn't one.</summary>
    public bool GetContinuity(string param, ref Continuity cont)
    {
        string str = "";
        if (!GetString(param, ref str))
        {
            return false;
        }

        str = str.Trim(' ', '\t', '\n', '\v', '\f', '\r').ToUpperInvariant();

        switch (str)
        {
            case "C0":
                cont = Continuity.C0;
                break;
            case "G1":
                cont = Continuity.G1;
                break;
            case "C1":
                cont = Continuity.C1;
                break;
            case "G2":
                cont = Continuity.G2;
                break;
            case "C2":
                cont = Continuity.C2;
                break;
            case "C3":
                cont = Continuity.C3;
                break;
            case "CN":
                cont = Continuity.CN;
                break;
            default:
                return false;
        }
        return true;
    }

    /// <summary>The value of a continuity parameter, or <paramref name="def"/>.</summary>
    public Continuity ContinuityVal(string param, Continuity def)
    {
        Continuity val = def;
        return GetContinuity(param, ref val) ? val : def;
    }

    /// <summary>Prints statistics on the history (shells and faces mapped, the preparation ratio) to the messenger.</summary>
    public void PrintStatistics()
    {
        PrintPreparation(_map, Messenger());
    }

    // The statistics ShapeProcess_ShapeContext::PrintStatistics and ShapeProcessAPI_ApplySequence::PrintPreparationResult
    // print (the same code twice in OCCT)
    internal static void PrintPreparation(NCollection_DataMap<Shape, Shape> map, Messenger aMessenger)
    {
        int ss = 0, sn = 0, ff = 0, fs = 0, fn = 0;
        foreach (KeyValuePair<Shape, Shape> it in map)
        {
            Shape keyshape = it.Key, valueshape = it.Value;
            if (keyshape.ShapeType() == ShapeType.Shell)
            {
                if (valueshape.IsNull)
                {
                    sn++;
                }
                else
                {
                    ss++;
                }
            }
            else if (keyshape.ShapeType() == ShapeType.Face)
            {
                if (valueshape.IsNull)
                {
                    fn++;
                }
                else if (valueshape.ShapeType() == ShapeType.Shell)
                {
                    fs++;
                }
                else
                {
                    ff++;
                }
            }
        }

        // mapping
        var epmsg100 = new Msg("PrResult.Print.MSG100"); // Mapping:
        aMessenger.Send(epmsg100.Get(), Info);
        var tpmsg50 = new Msg("PrResult.Print.MSG50"); //  Shells:
        aMessenger.Send(tpmsg50.Get(), Info);
        var epmsg110 = new Msg("PrResult.Print.MSG110"); //    Result is Shell                 : %d
        epmsg110.Arg(ss);
        aMessenger.Send(epmsg110.Get(), Info);
        var epmsg150 = new Msg("PrResult.Print.MSG150"); //    No Result                       : %d
        epmsg150.Arg(sn);
        aMessenger.Send(epmsg150.Get(), Info);

        string tmp110 = epmsg110.Original(), tmp150 = epmsg150.Original();
        epmsg110.Set(tmp110);
        epmsg150.Set(tmp150);

        var tpmsg55 = new Msg("PrResult.Print.MSG55"); //  Faces:
        aMessenger.Send(tpmsg55.Get(), Info);
        var epmsg115 = new Msg("PrResult.Print.MSG115"); //    Result is Face                  : %d
        epmsg115.Arg(ff);
        aMessenger.Send(epmsg115.Get(), Info);
        epmsg110.Arg(fs);
        aMessenger.Send(epmsg110.Get(), Info);
        epmsg150.Arg(fn);
        aMessenger.Send(epmsg150.Get(), Info);

        // preparation ratio
        double spr = 1, fpr = 1;
        int sTotalR = ss, fTotalR = ff + fs;
        int nbS = sTotalR + sn, nbF = fTotalR + fn;
        if (nbS > 0)
        {
            spr = 1.0 * (nbS - sn) / nbS;
        }
        if (nbF > 0)
        {
            fpr = 1.0 * (nbF - fn) / nbF;
        }
        var pmsg200 = new Msg("PrResult.Print.MSG200"); // Preparation ratio:
        aMessenger.Send(pmsg200.Get(), Info);
        var pmsg205 = new Msg("PrResult.Print.MSG205"); //  Shells: %d per cent
        pmsg205.Arg((int)(100.0 * spr));
        aMessenger.Send(pmsg205.Get(), Info);
        var pmsg210 = new Msg("PrResult.Print.MSG210"); //  Faces : %d per cent
        pmsg210.Arg((int)(100.0 * fpr));
        aMessenger.Send(pmsg210.Get(), Info);
    }
}
