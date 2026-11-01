// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;
using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Divides all the faces of a shape (and its free wires and edges) with the split face tool's criteria, recording
/// the changes in a context (<see cref="ShapeBuild_ReShape"/>). The compounds' children are processed one by one, to
/// keep the sharing in assemblies.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). The dividers write into the shape in place (ShapeFix_ComposeShell splits the edges
/// of the faces it rebuilds, ShapeConvertToBezier sets ranges and curves), so a shape holding frozen shapes is copied
/// at the entry and the caller's own sub-shapes are given back where nothing changed, the context re-keyed to them
/// (decision 999); the shape given is never modified.
/// </para>
/// <para>
/// <see cref="Result"/> freezes what it returns; ported callers (ShapeProcess's operators) read
/// <see cref="InternalResult"/>. The context is the tool's working object: its <c>Value</c> and <c>Apply</c> freeze
/// what they return, its internal forms don't.
/// </para>
/// </remarks>
public class ShapeUpgrade_ShapeDivide
{
    /// <summary>The context.</summary>
    protected ShapeBuild_ReShape? myContext;
    /// <summary>The message registrator.</summary>
    protected ShapeExtend_BasicMsgRegistrator? myMsgReg;
    /// <summary>The shape.</summary>
    protected TopoDS_Shape myShape;
    /// <summary>The result.</summary>
    protected TopoDS_Shape myResult;
    /// <summary>The precision.</summary>
    protected double myPrecision;
    /// <summary>The minimal tolerance.</summary>
    protected double myMinTol;
    /// <summary>The maximal tolerance.</summary>
    protected double myMaxTol;
    /// <summary>The mode for trimming (segmenting) surfaces by the wires' UV bounds.</summary>
    protected bool mySegmentMode;
    /// <summary>The status.</summary>
    protected int myStatus;
    /// <summary>The mode for splitting 3D curves of edges.</summary>
    protected int myEdgeMode;

    private ShapeUpgrade_FaceDivide? mySplitFaceTool;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_ShapeDivide()
    {
        myShape = new TopoDS_Shape();
        myResult = new TopoDS_Shape();
        myStatus = 0;
        myPrecision = myMinTol = Occt.Precision.Confusion();
        myMaxTol = 1; // Precision::Infinite() ?? pdn
        mySplitFaceTool = new ShapeUpgrade_FaceDivide();
        myContext = new ShapeBuild_ReShape();
        // myMsgReg = new ShapeExtend_BasicMsgRegistrator;
        mySegmentMode = true;
        myEdgeMode = 2;
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeUpgrade_ShapeDivide(in TopoDS_Shape S) : this()
    {
        Init(S);
    }

    /// <summary>Initializes by a shape.</summary>
    public void Init(in TopoDS_Shape S)
    {
        myShape = S;
    }

    /// <summary>Defines the spatial precision used for splitting.</summary>
    public void SetPrecision(double Prec)
    {
        myPrecision = Prec;
    }

    /// <summary>Sets the maximal allowed tolerance.</summary>
    public void SetMaxTolerance(double maxtol)
    {
        myMaxTol = maxtol;
    }

    /// <summary>Sets the minimal allowed tolerance.</summary>
    public void SetMinTolerance(double mintol)
    {
        myMinTol = mintol;
    }

    /// <summary>Sets the mode for trimming (segmenting) surfaces by the wires' UV bounds.</summary>
    public void SetSurfaceSegmentMode(bool Segment)
    {
        mySegmentMode = Segment;
    }

    /// <summary>
    /// Performs the splitting and computes the resulting shape. With <paramref name="newContext"/> (the default) the
    /// context is cleared at the start, else the previous substitutions apply. True if the result differs from the
    /// shape.
    /// </summary>
    public virtual bool Perform(bool newContext = true) => PerformAtEntry(newContext, performBody);

    // OCCT.NET: the copy at the entry (decision 999). The face dividers' ShapeFix_ComposeShell edits the edges it splits
    // in place (as do ShapeConvertToBezier's last step and the dividers called on a compound's children), and the
    // edges shared by other faces see those edits natively; so the outermost Perform runs on an exact copy of a shape
    // holding frozen shapes, and gives back the caller's own sub-shapes where nothing changed (decision 583). Nested
    // calls (a compound's children, ShapeConvertToBezier's passes) run on the copy.
    private int myEntryDepth;

    /// <summary>
    /// Runs <paramref name="theBody"/> on a copy of the shape when it holds frozen shapes and this is the outermost
    /// call, then gives back the caller's unchanged sub-shapes in the result, in the shape left in <c>myShape</c> and
    /// in the context.
    /// </summary>
    protected bool PerformAtEntry(bool newContext, Func<bool, bool> theBody)
    {
        if (myEntryDepth > 0 || myShape.IsNull())
        {
            return theBody(newContext);
        }

        // the requests already recorded in a kept context move onto the copies
        var anEntry = new BRepTools_CopyAtEntry(myShape, newContext ? null : myContext);
        myShape = anEntry.Shape();
        TopoDS_Shape aCopy = myShape;
        bool aRes;
        myEntryDepth++;
        try
        {
            aRes = theBody(newContext);
        }
        finally
        {
            myEntryDepth--;
        }

        anEntry.AttachContext(myContext);
        // ShapeConvertToBezier leaves myShape on the intermediate result of its surface conversion
        bool isIntermediate = !myShape.IsNull() && !ReferenceEquals(myShape.TShape(), aCopy.TShape());
        myResult = anEntry.Finish(myResult, isIntermediate ? [myShape] : null);
        myShape = anEntry.Final(myShape);
        return aRes;
    }

    private bool performBody(bool newContext)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (myShape.IsNull())
        {
            myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL1);
            return false;
        }

        if (newContext || myContext is null)
        {
            myContext = new ShapeBuild_ReShape();
        }

        // Process COMPOUNDs separately in order to handle sharing in assemblies
        // NOTE: not optimized: subshape can be processed twice (second time - no modif)
        if (myShape.ShapeType() == TopAbs_COMPOUND)
        {
            int locStatus = myStatus;
            var C = new TopoDS_Compound();
            var B = new BRep_Builder();
            B.MakeCompound(ref C);
            TopoDS_Shape savShape = myShape;
            for (var it = new TopoDS_Iterator(savShape, false); it.More(); it.Next())
            {
                TopoDS_Shape shape = it.Value();
                TopLoc_Location L = shape.Location();
                if (myContext.ModeConsiderLocation())
                {
                    var nullLoc = new TopLoc_Location();
                    shape.Location(nullLoc);
                }
                myShape = myContext.InternalApply(shape);
                Perform(false);
                if (myContext.ModeConsiderLocation())
                {
                    myResult.Location(L);
                }
                myResult.Orientation(TopAbs.Compose(myResult.Orientation(), savShape.Orientation()));
                B.Add(C, myResult);
                locStatus |= myStatus;
            }

            myShape = savShape;
            myStatus = locStatus;
            if (Status(ShapeExtend_DONE))
            {
                myResult = myContext.InternalApply(C, TopAbs_SHAPE);
                myContext.Replace(myShape, myResult);
                return true;
            }
            myResult = myShape;
            return false;
        }

        // Process FACEs
        ShapeUpgrade_FaceDivide? SplitFace = GetSplitFaceTool();
        if (SplitFace is not null)
        {
            SplitFace.SetPrecision(myPrecision);
            SplitFace.SetMaxTolerance(myMaxTol);
            SplitFace.SetSurfaceSegmentMode(mySegmentMode);
            ShapeUpgrade_WireDivide? SplitWireTool = SplitFace.GetWireDivideTool();
            if (SplitWireTool is not null)
            {
                SplitWireTool.SetMinTolerance(myMinTol);
                SplitWireTool.SetEdgeMode(myEdgeMode);
            }
            Message_Msg doneMsg = GetFaceMsg();

            for (var exp = new TopExp_Explorer(myShape, TopAbs_FACE); exp.More(); exp.Next())
            {
                TopoDS_Shape tmpF = exp.Current().Oriented(TopAbs_FORWARD);
                TopoDS_Face F = TopoDS.Face(tmpF); // protection against INTERNAL shapes: cts20105a.rle
                TopoDS_Shape sh = myContext.InternalApply(F, TopAbs_SHAPE);
                for (var exp2 = new TopExp_Explorer(sh, TopAbs_FACE); exp2.More(); exp2.Next())
                {
                    try
                    {
                        for (; exp2.More(); exp2.Next())
                        {
                            TopoDS_Face face = TopoDS.Face(exp2.Current());
                            SplitFace.Init(face);
                            SplitFace.SetContext(myContext);
                            SplitFace.Perform();
                            if (SplitFace.Status(ShapeExtend_FAIL))
                            {
                                myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                            }
                            if (SplitFace.Status(ShapeExtend_DONE))
                            {
                                myContext.Replace(face, SplitFace.InternalResult());
                                SendMsg(face, doneMsg, Message_Info);
                                if (SplitFace.Status(ShapeExtend_DONE1))
                                {
                                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                                }
                                if (SplitFace.Status(ShapeExtend_DONE2))
                                {
                                    myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE2);
                                }
                            }
                        }
                    }
                    catch (Exception anException) when (ShapeUpgrade.IsCaughtFailure(anException))
                    {
                        // OCCT_DEBUG output left out: exception in ShapeUpgrade_FaceDivide::Perform()
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL2);
                    }
                    if (!exp2.More())
                    {
                        break;
                    }
                }
            }
        }

        // Process free WIREs
        // (OCCT dereferences the split face tool here without checking it)
        ShapeUpgrade_WireDivide? SplitWire = SplitFace!.GetWireDivideTool();
        if (SplitWire is not null)
        {
            SplitWire.SetFace(new TopoDS_Face());
            SplitWire.SetPrecision(myPrecision);
            SplitWire.SetMaxTolerance(myMaxTol);
            SplitWire.SetMinTolerance(myMinTol);
            SplitWire.SetEdgeMode(myEdgeMode);
            Message_Msg doneMsg = GetWireMsg();

            var exp = new TopExp_Explorer(); // svv Jan 10 2000 : porting on DEC
            for (exp.Init(myShape, TopAbs_WIRE, TopAbs_FACE); exp.More(); exp.Next())
            {
                // smh#8
                TopoDS_Shape tmpW = exp.Current().Oriented(TopAbs_FORWARD);
                TopoDS_Wire W = TopoDS.Wire(tmpW); // protection against INTERNAL shapes
                TopoDS_Shape sh = myContext.InternalApply(W, TopAbs_SHAPE);
                for (var exp2 = new TopExp_Explorer(sh, TopAbs_WIRE); exp2.More(); exp2.Next())
                {
                    TopoDS_Wire wire = TopoDS.Wire(exp2.Current());
                    SplitWire.Load(wire);
                    SplitWire.SetContext(myContext);
                    SplitWire.Perform();
                    if (SplitWire.Status(ShapeExtend_FAIL))
                    {
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
                    }
                    if (SplitWire.Status(ShapeExtend_DONE))
                    {
                        myContext.Replace(wire, SplitWire.InternalWire());
                        SendMsg(wire, doneMsg, Message_Info);
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                    }
                }
            }

            // Process free EDGEs
            Message_Msg edgeDoneMsg = GetEdgeMsg();
            for (exp.Init(myShape, TopAbs_EDGE, TopAbs_WIRE); exp.More(); exp.Next())
            {
                // smh#8
                TopoDS_Shape tmpE = exp.Current().Oriented(TopAbs_FORWARD);
                TopoDS_Edge E = TopoDS.Edge(tmpE); // protection against INTERNAL shapes
                TopExp.Vertices(E, out TopoDS_Vertex V2, out TopoDS_Vertex V1);
                if (V1.IsNull() && V2.IsNull())
                {
                    continue; // skl 27.10.2004 for OCC5624
                }
                TopoDS_Shape sh = myContext.InternalApply(E, TopAbs_SHAPE);
                for (var exp2 = new TopExp_Explorer(sh, TopAbs_EDGE); exp2.More(); exp2.Next())
                {
                    TopoDS_Edge edge = TopoDS.Edge(exp2.Current());
                    SplitWire.Load(edge);
                    SplitWire.SetContext(myContext);
                    SplitWire.Perform();
                    if (SplitWire.Status(ShapeExtend_FAIL))
                    {
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_FAIL3);
                    }
                    if (SplitWire.Status(ShapeExtend_DONE))
                    {
                        myContext.Replace(edge, SplitWire.InternalWire());
                        SendMsg(edge, edgeDoneMsg, Message_Info);
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                    }
                }
            }
        }
        myResult = myContext.InternalApply(myShape, TopAbs_SHAPE);
        return !myResult.IsSame(myShape);
    }

    /// <summary>Gives the resulting shape (null if not done), frozen.</summary>
    public TopoDS_Shape Result()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="Result"/> without freezing it, for ported callers (ShapeProcess).</summary>
    internal TopoDS_Shape InternalResult() => myResult;

    /// <summary>Returns the context with all the modifications made during the last call(s) to Perform recorded.</summary>
    public ShapeBuild_ReShape? GetContext() => myContext;

    /// <summary>Sets the context with recorded modifications to apply during the next call(s) to Perform(false).</summary>
    public void SetContext(ShapeBuild_ReShape? context)
    {
        myContext = context;
    }

    /// <summary>Sets the message registrator.</summary>
    public virtual void SetMsgRegistrator(ShapeExtend_BasicMsgRegistrator? msgreg)
    {
        myMsgReg = msgreg;
    }

    /// <summary>Returns the message registrator.</summary>
    public ShapeExtend_BasicMsgRegistrator? MsgRegistrator() => myMsgReg;

    /// <summary>Sends a message attached to the shape, through the message registrator.</summary>
    public void SendMsg(in TopoDS_Shape shape, Message_Msg message, Message_Gravity gravity = Message_Info)
    {
        myMsgReg?.Send(shape, message, gravity);
    }

    /// <summary>
    /// Queries the status of the last call to Perform: OK (no splitting done, or no call), DONE1 (some edges split),
    /// DONE2 (a surface split), FAIL1 (errors).
    /// </summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Sets the tool for splitting faces.</summary>
    public void SetSplitFaceTool(ShapeUpgrade_FaceDivide? splitFaceTool)
    {
        mySplitFaceTool = splitFaceTool;
    }

    /// <summary>Sets the mode for splitting 3D curves of edges: 0 free edges only, 1 shared edges only, 2 all.</summary>
    public void SetEdgeMode(int aEdgeMode)
    {
        myEdgeMode = aEdgeMode;
    }

    /// <summary>Returns the tool for splitting faces.</summary>
    protected virtual ShapeUpgrade_FaceDivide? GetSplitFaceTool() => mySplitFaceTool;

    /// <summary>The message sent for a divided face.</summary>
    protected virtual Message_Msg GetFaceMsg() => new("ShapeDivide.FaceDivide.MSG0");

    /// <summary>The message sent for a divided free wire.</summary>
    protected virtual Message_Msg GetWireMsg() => new("ShapeDivide.WireDivide.MSG0");

    /// <summary>The message sent for a divided free edge.</summary>
    protected virtual Message_Msg GetEdgeMsg() => new("ShapeDivide.EdgeDivide.MSG0");
}
