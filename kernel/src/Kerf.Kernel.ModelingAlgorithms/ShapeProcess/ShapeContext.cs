// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_ShapeContext.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_ShapeContext.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
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
/// <b>The working overlay</b> (decision 1022; Kerf unit 10H4). OCCT's operators write into the shape they process:
/// SetTolerance and SameParameter change tolerances and pcurves in place, the modifiers run BRepTools_Modifier in its
/// mutable-input mode, which updates the input's vertices. A shape a caller holds is frozen, so while operators run
/// (<see cref="ShapeProcess"/>'s Perform, or an operator run on its own) they run on the context's result in a working
/// overlay that takes those writes, as OCCT's run in place, and the history keeps OCCT's keys, the caller's own
/// sub-shapes. At the end the result and the map's values hold new TShapes for what changed and the caller's own
/// sub-shapes for the rest, so a shape that needed nothing comes back itself. The map has OCCT's entries: a sub-shape
/// changed in place (a tolerance raised, a vertex updated) has none, as natively, where it is the same TShape; in Kerf
/// the result holds a new TShape for it. Inside another operation's overlay the operators run in place in that one.
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
    private readonly Dictionary<Shape, Shape> _map = new(ShapeHasher.Instance);
    private MsgRegistrator? _msg;
    private ShapeType _until;
    private bool _nonManifold;

    // Kerf: the working overlay of the result while operators run (decision 1022, unit 10H4); null when they run in
    // another operation's
    private WorkingOverlay? _overlay;
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
    public Dictionary<Shape, Shape> Map()
    {
        foreach (KeyValuePair<Shape, Shape> item in _map)
        {
            item.Value.TShape()?.FreezeGraph();
        }
        return _map;
    }

    /// <summary><see cref="Map"/> without freezing its values, for ported code.</summary>
    internal Dictionary<Shape, Shape> InternalMap() => _map;

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
        _map[_shape] = _result;
    }

    /// <summary>Sets the non-manifold flag, which FixShape passes to ShapeFix_Shell.</summary>
    public void SetNonManifold(bool nonManifold)
    {
        _nonManifold = nonManifold;
    }

    /// <summary>The non-manifold flag.</summary>
    public bool IsNonManifold => _nonManifold;

    // ---------------------------------------------------------------------------------------------------------------
    // OCCT.NET: the working copy (decision 1022), a working overlay since Kerf unit 10H4

    /// <summary>
    /// Starts a run of operators: at the outermost call, opens a working overlay over the result (unless the run is
    /// inside another operation's overlay, which then takes the operators' writes).
    /// </summary>
    internal void InternalBeginWrite()
    {
        if (_writeDepth++ > 0)
        {
            return;
        }
        _overlay = _result.IsNull ? null : WorkingOverlay.BeginUnlessWriting(_result);
    }

    /// <summary>
    /// Ends a run of operators: at the outermost call, finishes the overlay: the result and the map's values hold new
    /// TShapes for what the operators changed and the caller's own sub-shapes for the rest.
    /// </summary>
    internal void InternalEndWrite()
    {
        if (--_writeDepth > 0)
        {
            return;
        }
        WorkingOverlay? overlay = _overlay;
        _overlay = null;
        if (overlay is null)
        {
            return;
        }

        using (overlay)
        {
            var values = new List<Shape>();
            foreach (KeyValuePair<Shape, Shape> item in _map)
            {
                values.Add(item.Value);
            }
            _result = overlay.Finish(_result, values);
            var keys = new List<Shape>(_map.Keys);
            foreach (Shape key in keys)
            {
                _map[key] = overlay.Final(_map.GetValue(key));
            }
        }
    }

    // ---------------------------------------------------------------------------------------------------------------

    private static void RecModif(in Shape s,
                                 Dictionary<Shape, Shape> repl,
                                 MsgRegistrator? msg,
                                 Dictionary<Shape, Shape> map,
                                 MsgRegistrator? myMsg,
                                 ShapeType until)
    {
        Shape r = s;
        // gka  -modification to keep history for shape with location (OCC21617)
        Placement shLoc = s.Placement;
        var nullLoc = new Placement();
        r.Location(nullLoc);

        if (map.ContainsKey(r))
        {
            r = map.GetValue(r);
        }
        if (!r.IsNull)
        {
            Shape res = r;

            if (repl.ContainsKey(r.Located(shLoc, false)))
            {
                res = repl.GetValue(r.Located(shLoc, false));
                // it is supposed that map is created for r having FORWARD orientation
                // hence, if it is reversed, result should be reversed too
                // INTERNAL or EXTERNAL orientations are not allowed
                if (r.Orientation != Forward)
                {
                    res.Reverse();
                }
            }
            // Treat special case: if S was split, r will be a compound of
            // resulting shapes, each to be checked separately
            // It is supposed that repl does not contain such splitting
            else if (r.Type < s.Type)
            {
                Shape result = r.EmptyCopied();
                result.Orient(Forward); // protect against INTERNAL or EXTERNAL shapes
                bool modif = false;
                var b = new BrepBuilder();
                for (var it = new Iterator(r, false); it.More(); it.Next())
                {
                    Shape sh = it.Value();
                    if (repl.ContainsKey(sh))
                    {
                        Shape newsh = repl.GetValue(sh);
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
                    if (result.Type == ShapeType.Wire || result.Type == ShapeType.Shell)
                    {
                        result.Closed(Tool.IsClosed(result));
                    }
                    res = result;
                }
            }

            if (res != r)
            {
                map[s.Located(nullLoc)] = res;
            }
        }

        // update messages (messages must be taken from each level in the substitution map)
        if (!r.IsNull && myMsg is not null && msg is not null && msg.MapShape().Count > 0)
        {
            Dictionary<Shape, NCollection_List<Msg>> msgmap = msg.MapShape();
            if (msgmap.ContainsKey(r))
            {
                NCollection_List<Msg> msglist = msgmap.GetValue(r);
                for (var iter = new NCollection_List<Msg>.Iterator(msglist); iter.More(); iter.Next())
                {
                    myMsg.Send(s, iter.Value(), Warning);
                }
            }
            else if (msgmap.ContainsKey(s))
            {
                NCollection_List<Msg> msglist = msgmap.GetValue(s);
                for (var iter = new NCollection_List<Msg>.Iterator(msglist); iter.More(); iter.Next())
                {
                    myMsg.Send(s, iter.Value(), Warning);
                }
            }
        }

        if (until == global::Kerf.Topology.ShapeType.Shape || s.Type >= until)
        {
            return;
        }

        for (var it = new Iterator(s); it.More(); it.Next())
        {
            RecModif(it.Value(), repl, msg, map, myMsg, until);
        }
    }

    /// <summary>
    /// Records the modifications <paramref name="repl"/> (from the shapes the operator worked on to what they became)
    /// in the history, for every sub-shape of the shape down to the detalisation level, with their messages.
    /// </summary>
    public void RecordModification(Dictionary<Shape, Shape> repl, MsgRegistrator? msg = null)
    {
        if (repl.Count <= 0)
        {
            return;
        }
        RecModif(_shape, repl, msg, _map, _msg, _until);
        if (_map.ContainsKey(_shape))
        {
            _result = _map.GetValue(_shape);
        }
        // OCCT_DEBUG: the map dumped, left out
    }

    private static void RecModif(in Shape s,
                                 ReShapeBuilder repl,
                                 MsgRegistrator? msg,
                                 Dictionary<Shape, Shape> map,
                                 MsgRegistrator? myMsg,
                                 ShapeType until)
    {
        if (s.IsNull)
        {
            return;
        }
        // gka  -modification to keep history for shape with location (OCC21617)
        var nullLoc = new Placement();
        Shape aS = s.Located(nullLoc);
        Shape r = aS;

        if (map.ContainsKey(r))
        {
            r = map.GetValue(r);
        }
        if (!r.IsNull)
        {
            if (repl.InternalStatus(r, out Shape res, true) != 0 && res != r)
            {
                map[aS] = res;

                // Treat special case: if S was split, r will be a compound of
                // resulting shapes, recursive procedure should be applied
            }
            else if (r.Type < s.Type)
            {
                res = repl.InternalApply(r, (ShapeType)((int)s.Type + 1));
                if (res != r)
                {
                    map[aS] = res;
                }
            }
        }

        // update messages (messages must be taken from each level in the substitution map)
        if (!r.IsNull && myMsg is not null && msg is not null && msg.MapShape().Count > 0)
        {
            Shape cur, next = r;
            Dictionary<Shape, NCollection_List<Msg>> msgmap = msg.MapShape();
            if (msgmap.ContainsKey(s))
            {
                next = s;
            }
            do
            {
                cur = next;
                if (msgmap.ContainsKey(cur))
                {
                    NCollection_List<Msg> msglist = msgmap.GetValue(cur);
                    for (var iter = new NCollection_List<Msg>.Iterator(msglist); iter.More(); iter.Next())
                    {
                        myMsg.Send(s, iter.Value(), Warning);
                    }
                }
                next = repl.InternalValue(cur);
            } while (!next.IsNull && cur != next);
        }

        if (until == global::Kerf.Topology.ShapeType.Shape || s.Type >= until)
        {
            return;
        }

        for (var it = new Iterator(s, false /*,false*/); it.More(); it.Next())
        {
            RecModif(it.Value(), repl, msg, map, myMsg, until);
        }
    }

    /// <summary>
    /// Records the modifications a ShapeBuild_ReShape made (the requests recorded in it) in the history, for every
    /// sub-shape of the shape down to the detalisation level, with their messages.
    /// </summary>
    public void RecordModification(ReShapeBuilder repl, MsgRegistrator? msg)
    {
        RecModif(_shape, repl, msg, _map, _msg, _until);
        if (_map.ContainsKey(_shape))
        {
            _result = _map.GetValue(_shape);
            _result.Location(_shape.Placement, false);
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
                                        Dictionary<Shape, Shape> map,
                                        ShapeType until)
    {
        Shape res = repl.InternalModifiedShape(s);

        if (res != s)
        {
            map[s] = res;
        }
        if (until == global::Kerf.Topology.ShapeType.Shape || s.Type >= until)
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
        var map = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
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
    internal static void PrintPreparation(Dictionary<Shape, Shape> map, Messenger aMessenger)
    {
        int ss = 0, sn = 0, ff = 0, fs = 0, fn = 0;
        foreach (KeyValuePair<Shape, Shape> it in map)
        {
            Shape keyshape = it.Key, valueshape = it.Value;
            if (keyshape.Type == ShapeType.Shell)
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
            else if (keyshape.Type == ShapeType.Face)
            {
                if (valueshape.IsNull)
                {
                    fn++;
                }
                else if (valueshape.Type == ShapeType.Shell)
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
