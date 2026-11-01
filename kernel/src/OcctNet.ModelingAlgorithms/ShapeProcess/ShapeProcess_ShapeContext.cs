// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_ShapeContext.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeProcess/ShapeProcess_ShapeContext.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The context for shape processing: the shape, the result of the operators run so far, the history from the shape's
/// sub-shapes (down to the detalisation level) to the result's, and the messages attached to the shape's sub-shapes.
/// </summary>
/// <remarks>
/// <para>
/// <b>The working copy</b> (decision 8702). OCCT's operators write into the shape they process: SetTolerance and
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
public class ShapeProcess_ShapeContext : ShapeProcess_Context
{
    private TopoDS_Shape myShape;
    private TopoDS_Shape myResult;
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myMap = new(TopTools_ShapeMapHasher.Instance);
    private ShapeExtend_MsgRegistrator? myMsg;
    private TopAbs_ShapeEnum myUntil;
    private bool myNonManifold;

    // OCCT.NET: the working copy of the result while operators run (decision 8702)
    private BRepTools_CopyAtEntry? myEntry;
    private int myWriteDepth;

    /// <summary>Creates a context with the resource file <paramref name="file"/> and the scope <paramref name="seq"/>.</summary>
    public ShapeProcess_ShapeContext(string file, string seq = "", string? theDefaultsDirectory = null,
                                     string? theUserDefaultsDirectory = null)
        : base(file, seq, theDefaultsDirectory, theUserDefaultsDirectory)
    {
        myUntil = TopAbs_FACE;
        myNonManifold = false;
    }

    /// <summary>
    /// Creates a context with the shape <paramref name="S"/>, the resource file <paramref name="file"/> and the scope
    /// <paramref name="seq"/>.
    /// </summary>
    public ShapeProcess_ShapeContext(in TopoDS_Shape S, string file, string seq = "", string? theDefaultsDirectory = null,
                                     string? theUserDefaultsDirectory = null)
        : base(file, seq, theDefaultsDirectory, theUserDefaultsDirectory)
    {
        myUntil = TopAbs_FACE;
        myNonManifold = false;
        Init(S);
    }

    /// <summary>Initialises the context with the shape: the result is the shape, the map and the messages are cleared.</summary>
    public void Init(in TopoDS_Shape S)
    {
        if (myWriteDepth > 0)
        {
            throw new Standard_ProgramError("ShapeProcess_ShapeContext::Init called while operators run");
        }
        myMap.Clear();
        myMsg = new ShapeExtend_MsgRegistrator();
        myShape = S;
        myResult = myShape; //.Nullify();
    }

    /// <summary>The shape being processed.</summary>
    public TopoDS_Shape Shape() => myShape;

    /// <summary>The current result, frozen.</summary>
    public TopoDS_Shape Result()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="Result"/> without freezing it, for the operators.</summary>
    internal TopoDS_Shape InternalResult() => myResult;

    /// <summary>
    /// The history: each sub-shape of <see cref="Shape"/> (down to the detalisation level) that was modified, with
    /// what it became (null if removed). Its values are frozen.
    /// </summary>
    public NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Map()
    {
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anItem in myMap)
        {
            anItem.Value.TShape()?.FreezeGraph();
        }
        return myMap;
    }

    /// <summary><see cref="Map"/> without freezing its values, for ported code.</summary>
    internal NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> InternalMap() => myMap;

    /// <summary>The messages attached to the shape's sub-shapes (null if messages are off).</summary>
    public ShapeExtend_MsgRegistrator? Messages() => myMsg;

    /// <summary>Sets the registrator of messages (null switches them off).</summary>
    public void SetMessages(ShapeExtend_MsgRegistrator? theMessages)
    {
        // OCCT's Messages() returns a modifiable reference
        myMsg = theMessages;
    }

    /// <summary>Sets the level down to which the history is recorded (TopAbs_SHAPE: all sub-shapes).</summary>
    public void SetDetalisation(TopAbs_ShapeEnum level)
    {
        myUntil = level;
    }

    /// <summary>The level down to which the history is recorded.</summary>
    public TopAbs_ShapeEnum GetDetalisation() => myUntil;

    /// <summary>Sets the result, and records it as what the shape became.</summary>
    public void SetResult(in TopoDS_Shape S)
    {
        myResult = S;
        myMap.Bind(myShape, myResult);
    }

    /// <summary>Sets the non-manifold flag, which FixShape passes to ShapeFix_Shell.</summary>
    public void SetNonManifold(bool theNonManifold)
    {
        myNonManifold = theNonManifold;
    }

    /// <summary>The non-manifold flag.</summary>
    public bool IsNonManifold() => myNonManifold;

    // ---------------------------------------------------------------------------------------------------------------
    // OCCT.NET: the working copy (decision 8702)

    /// <summary>
    /// Starts a run of operators: at the outermost call, a result holding frozen shapes is replaced by an exact copy,
    /// and the map's values by their copies.
    /// </summary>
    internal void InternalBeginWrite()
    {
        if (myWriteDepth++ > 0)
        {
            InternalEnsureWritable();
            return;
        }
        beginCopy();
    }

    /// <summary>
    /// Ends a run of operators: at the outermost call, the copy's sub-shapes left unchanged are the caller's own again,
    /// in the result and in the map's values.
    /// </summary>
    internal void InternalEndWrite()
    {
        if (--myWriteDepth > 0)
        {
            return;
        }
        endCopy();
    }

    /// <summary>
    /// Inside a run of operators, takes a new copy if the result holds frozen shapes again (an operator published it
    /// through <see cref="Result"/>), so that the next operator can write into it.
    /// </summary>
    internal void InternalEnsureWritable()
    {
        if (myWriteDepth > 0 && !myResult.IsNull() && BRepTools.HoldsFrozen(myResult))
        {
            endCopy();
            beginCopy();
        }
    }

    private void beginCopy()
    {
        myEntry = null;
        if (myResult.IsNull() || !BRepTools.HoldsFrozen(myResult))
        {
            return;
        }

        myEntry = new BRepTools_CopyAtEntry(myResult, null, true);
        myResult = myEntry.Shape();
        var aKeys = new List<TopoDS_Shape>(myMap.Keys);
        foreach (TopoDS_Shape aKey in aKeys)
        {
            myMap.Bind(aKey, copied(myMap.Find(aKey)));
        }
    }

    private void endCopy()
    {
        BRepTools_CopyAtEntry? anEntry = myEntry;
        myEntry = null;
        if (anEntry is null)
        {
            return;
        }

        var aValues = new List<TopoDS_Shape>();
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> anItem in myMap)
        {
            aValues.Add(anItem.Value);
        }
        myResult = anEntry.Finish(myResult, aValues);
        var aKeys = new List<TopoDS_Shape>(myMap.Keys);
        foreach (TopoDS_Shape aKey in aKeys)
        {
            myMap.Bind(aKey, anEntry.Final(myMap.Find(aKey)));
        }
    }

    // The copy of a shape of the result (a compound made of them, as RecModif makes for a split shape, rebuilt on
    // their copies; any other shape is itself).
    private TopoDS_Shape copied(in TopoDS_Shape theShape)
    {
        if (theShape.IsNull() || myEntry is null)
        {
            return theShape;
        }
        TopoDS_Shape aCopy = myEntry.Copied(theShape);
        if (!ReferenceEquals(aCopy.TShape(), theShape.TShape()) || theShape.ShapeType() != TopAbs_COMPOUND)
        {
            return aCopy;
        }

        bool isChanged = false;
        var aChildren = new List<TopoDS_Shape>();
        for (var it = new TopoDS_Iterator(theShape, false, false); it.More(); it.Next())
        {
            TopoDS_Shape aChild = copied(it.Value());
            isChanged |= !ReferenceEquals(aChild.TShape(), it.Value().TShape());
            aChildren.Add(aChild);
        }
        if (!isChanged)
        {
            return theShape;
        }

        TopoDS_Shape aResult = theShape.EmptyCopied();
        aResult.Orientation(TopAbs_FORWARD);
        var B = new BRep_Builder();
        foreach (TopoDS_Shape aChild in aChildren)
        {
            B.Add(aResult, aChild);
        }
        aResult.Orientation(theShape.Orientation());
        aResult.Location(theShape.Location(), false);
        return aResult;
    }

    // What a sub-shape of the shape not yet in the map stands for: its working copy (itself when there is none)
    private TopoDS_Shape working(in TopoDS_Shape theShape) => myEntry is null ? theShape : myEntry.Copied(theShape);

    // ---------------------------------------------------------------------------------------------------------------

    private static void RecModif(in TopoDS_Shape S,
                                 NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> repl,
                                 ShapeExtend_MsgRegistrator? msg,
                                 NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> map,
                                 ShapeExtend_MsgRegistrator? myMsg,
                                 TopAbs_ShapeEnum until,
                                 ShapeProcess_ShapeContext theContext)
    {
        TopoDS_Shape r = S;
        // gka  -modification to keep history for shape with location (OCC21617)
        TopLoc_Location aShLoc = S.Location();
        var aNullLoc = new TopLoc_Location();
        r.Location(aNullLoc);

        if (map.IsBound(r))
        {
            r = map.Find(r);
        }
        else
        {
            r = theContext.working(r); // OCCT.NET: the working copy (decision 8702)
        }
        if (!r.IsNull())
        {
            TopoDS_Shape res = r;

            if (repl.IsBound(r.Located(aShLoc, false)))
            {
                res = repl.Find(r.Located(aShLoc, false));
                // it is supposed that map is created for r having FORWARD orientation
                // hence, if it is reversed, result should be reversed too
                // INTERNAL or EXTERNAL orientations are not allowed
                if (r.Orientation() != TopAbs_FORWARD)
                {
                    res.Reverse();
                }
            }
            // Treat special case: if S was split, r will be a compound of
            // resulting shapes, each to be checked separately
            // It is supposed that repl does not contain such splitting
            else if (r.ShapeType() < S.ShapeType())
            {
                TopoDS_Shape result = r.EmptyCopied();
                result.Orientation(TopAbs_FORWARD); // protect against INTERNAL or EXTERNAL shapes
                bool modif = false;
                var B = new BRep_Builder();
                for (var it = new TopoDS_Iterator(r, false); it.More(); it.Next())
                {
                    TopoDS_Shape sh = it.Value();
                    if (repl.IsBound(sh))
                    {
                        TopoDS_Shape newsh = repl.Find(sh);
                        if (!newsh.IsNull())
                        {
                            B.Add(result, newsh);
                        }
                        modif = true;
                    }
                    else
                    {
                        B.Add(result, sh);
                    }
                }
                if (modif)
                {
                    if (result.ShapeType() == TopAbs_WIRE || result.ShapeType() == TopAbs_SHELL)
                    {
                        result.Closed(BRep_Tool.IsClosed(result));
                    }
                    res = result;
                }
            }

            if (res != r)
            {
                map.Bind(S.Located(aNullLoc), res);
            }
        }

        // update messages (messages must be taken from each level in the substitution map)
        if (!r.IsNull() && myMsg is not null && msg is not null && msg.MapShape().Extent() > 0)
        {
            NCollection_DataMap<TopoDS_Shape, NCollection_List<Message_Msg>> msgmap = msg.MapShape();
            if (msgmap.IsBound(r))
            {
                NCollection_List<Message_Msg> msglist = msgmap.Find(r);
                for (var iter = new NCollection_List<Message_Msg>.Iterator(msglist); iter.More(); iter.Next())
                {
                    myMsg.Send(S, iter.Value(), Message_Warning);
                }
            }
            else if (msgmap.IsBound(theContext.working(S))) // OCCT.NET: S as the operator saw it (decision 8702)
            {
                NCollection_List<Message_Msg> msglist = msgmap.Find(theContext.working(S));
                for (var iter = new NCollection_List<Message_Msg>.Iterator(msglist); iter.More(); iter.Next())
                {
                    myMsg.Send(S, iter.Value(), Message_Warning);
                }
            }
        }

        if (until == TopAbs_SHAPE || S.ShapeType() >= until)
        {
            return;
        }

        for (var it = new TopoDS_Iterator(S); it.More(); it.Next())
        {
            RecModif(it.Value(), repl, msg, map, myMsg, until, theContext);
        }
    }

    /// <summary>
    /// Records the modifications <paramref name="repl"/> (from the shapes the operator worked on to what they became)
    /// in the history, for every sub-shape of the shape down to the detalisation level, with their messages.
    /// </summary>
    public void RecordModification(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> repl, ShapeExtend_MsgRegistrator? msg = null)
    {
        if (repl.Extent() <= 0)
        {
            return;
        }
        RecModif(myShape, repl, msg, myMap, myMsg, myUntil, this);
        if (myMap.IsBound(myShape))
        {
            myResult = myMap.Find(myShape);
        }
        // OCCT_DEBUG: the map dumped, left out
    }

    private static void RecModif(in TopoDS_Shape S,
                                 ShapeBuild_ReShape repl,
                                 ShapeExtend_MsgRegistrator? msg,
                                 NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> map,
                                 ShapeExtend_MsgRegistrator? myMsg,
                                 TopAbs_ShapeEnum until,
                                 ShapeProcess_ShapeContext theContext)
    {
        if (S.IsNull())
        {
            return;
        }
        // gka  -modification to keep history for shape with location (OCC21617)
        var aNullLoc = new TopLoc_Location();
        TopoDS_Shape aS = S.Located(aNullLoc);
        TopoDS_Shape r = aS;

        if (map.IsBound(r))
        {
            r = map.Find(r);
        }
        else
        {
            r = theContext.working(r); // OCCT.NET: the working copy (decision 8702)
        }
        if (!r.IsNull())
        {
            if (repl.InternalStatus(r, out TopoDS_Shape res, true) != 0 && res != r)
            {
                map.Bind(aS, res);

                // Treat special case: if S was split, r will be a compound of
                // resulting shapes, recursive procedure should be applied
            }
            else if (r.ShapeType() < S.ShapeType())
            {
                res = repl.InternalApply(r, (TopAbs_ShapeEnum)((int)S.ShapeType() + 1));
                if (res != r)
                {
                    map.Bind(aS, res);
                }
            }
        }

        // update messages (messages must be taken from each level in the substitution map)
        if (!r.IsNull() && myMsg is not null && msg is not null && msg.MapShape().Extent() > 0)
        {
            TopoDS_Shape cur, next = r;
            NCollection_DataMap<TopoDS_Shape, NCollection_List<Message_Msg>> msgmap = msg.MapShape();
            TopoDS_Shape aWorkS = theContext.working(S); // OCCT.NET: S as the operator saw it (decision 8702)
            if (msgmap.IsBound(aWorkS))
            {
                next = aWorkS;
            }
            do
            {
                cur = next;
                if (msgmap.IsBound(cur))
                {
                    NCollection_List<Message_Msg> msglist = msgmap.Find(cur);
                    for (var iter = new NCollection_List<Message_Msg>.Iterator(msglist); iter.More(); iter.Next())
                    {
                        myMsg.Send(S, iter.Value(), Message_Warning);
                    }
                }
                next = repl.InternalValue(cur);
            } while (!next.IsNull() && cur != next);
        }

        if (until == TopAbs_SHAPE || S.ShapeType() >= until)
        {
            return;
        }

        for (var it = new TopoDS_Iterator(S, false /*,false*/); it.More(); it.Next())
        {
            RecModif(it.Value(), repl, msg, map, myMsg, until, theContext);
        }
    }

    /// <summary>
    /// Records the modifications a ShapeBuild_ReShape made (the requests recorded in it) in the history, for every
    /// sub-shape of the shape down to the detalisation level, with their messages.
    /// </summary>
    public void RecordModification(ShapeBuild_ReShape repl, ShapeExtend_MsgRegistrator? msg)
    {
        RecModif(myShape, repl, msg, myMap, myMsg, myUntil, this);
        if (myMap.IsBound(myShape))
        {
            myResult = myMap.Find(myShape);
            myResult.Location(myShape.Location(), false);
        }
        // OCCT_DEBUG: the map dumped, left out
    }

    /// <summary>Records the modifications a ShapeBuild_ReShape made, without messages.</summary>
    public void RecordModification(ShapeBuild_ReShape repl)
    {
        RecordModification(repl, null);
    }

    /// <summary>Adds a message for the sub-shape <paramref name="S"/> (if messages are on).</summary>
    public void AddMessage(in TopoDS_Shape S, Message_Msg msg, Message_Gravity grv = Message_Warning)
    {
        myMsg?.Send(S, msg, grv);
    }

    private static void ExplodeModifier(in TopoDS_Shape S,
                                        BRepTools_Modifier repl,
                                        NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> map,
                                        TopAbs_ShapeEnum until)
    {
        TopoDS_Shape res = repl.InternalModifiedShape(S);

        if (res != S)
        {
            map.Bind(S, res);
        }
        if (until == TopAbs_SHAPE || S.ShapeType() >= until)
        {
            return;
        }
        for (var it = new TopoDS_Iterator(S); it.More(); it.Next())
        {
            ExplodeModifier(it.Value(), repl, map, until);
        }
    }

    /// <summary>
    /// Records the modifications a BRepTools_Modifier made on <paramref name="S"/> in the history, down to the
    /// detalisation level, with their messages.
    /// </summary>
    public void RecordModification(in TopoDS_Shape S, BRepTools_Modifier repl, ShapeExtend_MsgRegistrator? msg)
    {
        var map = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        ExplodeModifier(S, repl, map, myUntil);
        RecordModification(map, msg);
    }

    /// <summary>The value of a continuity parameter (C0, G1, C1, G2, C2, C3, CN); false if it isn't one.</summary>
    public bool GetContinuity(string param, ref GeomAbs_Shape cont)
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
                cont = GeomAbs_Shape.GeomAbs_C0;
                break;
            case "G1":
                cont = GeomAbs_Shape.GeomAbs_G1;
                break;
            case "C1":
                cont = GeomAbs_Shape.GeomAbs_C1;
                break;
            case "G2":
                cont = GeomAbs_Shape.GeomAbs_G2;
                break;
            case "C2":
                cont = GeomAbs_Shape.GeomAbs_C2;
                break;
            case "C3":
                cont = GeomAbs_Shape.GeomAbs_C3;
                break;
            case "CN":
                cont = GeomAbs_Shape.GeomAbs_CN;
                break;
            default:
                return false;
        }
        return true;
    }

    /// <summary>The value of a continuity parameter, or <paramref name="def"/>.</summary>
    public GeomAbs_Shape ContinuityVal(string param, GeomAbs_Shape def)
    {
        GeomAbs_Shape val = def;
        return GetContinuity(param, ref val) ? val : def;
    }

    /// <summary>Prints statistics on the history (shells and faces mapped, the preparation ratio) to the messenger.</summary>
    public void PrintStatistics()
    {
        PrintPreparation(myMap, Messenger());
    }

    // The statistics ShapeProcess_ShapeContext::PrintStatistics and ShapeProcessAPI_ApplySequence::PrintPreparationResult
    // print (the same code twice in OCCT)
    internal static void PrintPreparation(NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theMap, Message_Messenger aMessenger)
    {
        int SS = 0, SN = 0, FF = 0, FS = 0, FN = 0;
        foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> It in theMap)
        {
            TopoDS_Shape keyshape = It.Key, valueshape = It.Value;
            if (keyshape.ShapeType() == TopAbs_SHELL)
            {
                if (valueshape.IsNull())
                {
                    SN++;
                }
                else
                {
                    SS++;
                }
            }
            else if (keyshape.ShapeType() == TopAbs_FACE)
            {
                if (valueshape.IsNull())
                {
                    FN++;
                }
                else if (valueshape.ShapeType() == TopAbs_SHELL)
                {
                    FS++;
                }
                else
                {
                    FF++;
                }
            }
        }

        // mapping
        var EPMSG100 = new Message_Msg("PrResult.Print.MSG100"); // Mapping:
        aMessenger.Send(EPMSG100.Get(), Message_Info);
        var TPMSG50 = new Message_Msg("PrResult.Print.MSG50"); //  Shells:
        aMessenger.Send(TPMSG50.Get(), Message_Info);
        var EPMSG110 = new Message_Msg("PrResult.Print.MSG110"); //    Result is Shell                 : %d
        EPMSG110.Arg(SS);
        aMessenger.Send(EPMSG110.Get(), Message_Info);
        var EPMSG150 = new Message_Msg("PrResult.Print.MSG150"); //    No Result                       : %d
        EPMSG150.Arg(SN);
        aMessenger.Send(EPMSG150.Get(), Message_Info);

        string tmp110 = EPMSG110.Original(), tmp150 = EPMSG150.Original();
        EPMSG110.Set(tmp110);
        EPMSG150.Set(tmp150);

        var TPMSG55 = new Message_Msg("PrResult.Print.MSG55"); //  Faces:
        aMessenger.Send(TPMSG55.Get(), Message_Info);
        var EPMSG115 = new Message_Msg("PrResult.Print.MSG115"); //    Result is Face                  : %d
        EPMSG115.Arg(FF);
        aMessenger.Send(EPMSG115.Get(), Message_Info);
        EPMSG110.Arg(FS);
        aMessenger.Send(EPMSG110.Get(), Message_Info);
        EPMSG150.Arg(FN);
        aMessenger.Send(EPMSG150.Get(), Message_Info);

        // preparation ratio
        double SPR = 1, FPR = 1;
        int STotalR = SS, FTotalR = FF + FS;
        int NbS = STotalR + SN, NbF = FTotalR + FN;
        if (NbS > 0)
        {
            SPR = 1.0 * (NbS - SN) / NbS;
        }
        if (NbF > 0)
        {
            FPR = 1.0 * (NbF - FN) / NbF;
        }
        var PMSG200 = new Message_Msg("PrResult.Print.MSG200"); // Preparation ratio:
        aMessenger.Send(PMSG200.Get(), Message_Info);
        var PMSG205 = new Message_Msg("PrResult.Print.MSG205"); //  Shells: %d per cent
        PMSG205.Arg((int)(100.0 * SPR));
        aMessenger.Send(PMSG205.Get(), Message_Info);
        var PMSG210 = new Message_Msg("PrResult.Print.MSG210"); //  Faces : %d per cent
        PMSG210.Arg((int)(100.0 * FPR));
        aMessenger.Send(PMSG210.Get(), Message_Info);
    }
}
