// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using static Kerf.Engine.Foundation.Gravity;
using static Occt.ShapeExtend_Status;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

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
    protected Shape myShape;
    /// <summary>The result.</summary>
    protected Shape myResult;
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
        myShape = new Shape();
        myResult = new Shape();
        myStatus = 0;
        myPrecision = myMinTol = Kerf.Engine.Foundation.Precision.Confusion();
        myMaxTol = 1; // Precision::Infinite() ?? pdn
        mySplitFaceTool = new ShapeUpgrade_FaceDivide();
        myContext = new ShapeBuild_ReShape();
        // myMsgReg = new ShapeExtend_BasicMsgRegistrator;
        mySegmentMode = true;
        myEdgeMode = 2;
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeUpgrade_ShapeDivide(in Shape S) : this()
    {
        Init(S);
    }

    /// <summary>Initializes by a shape.</summary>
    public void Init(in Shape S)
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
        if (myEntryDepth > 0 || myShape.IsNull)
        {
            return theBody(newContext);
        }

        // the requests already recorded in a kept context move onto the copies
        var anEntry = new CopyAtEntry(myShape, newContext ? null : myContext);
        myShape = anEntry.Shape();
        Shape aCopy = myShape;
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
        bool isIntermediate = !myShape.IsNull && !ReferenceEquals(myShape.TShape(), aCopy.TShape());
        myResult = anEntry.Finish(myResult, isIntermediate ? [myShape] : null);
        myShape = anEntry.Final(myShape);
        return aRes;
    }

    private bool performBody(bool newContext)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        if (myShape.IsNull)
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
        if (myShape.ShapeType() == ShapeType.Compound)
        {
            int locStatus = myStatus;
            var C = new Compound();
            var B = new BrepBuilder();
            B.MakeCompound(ref C);
            Shape savShape = myShape;
            for (var it = new Iterator(savShape, false); it.More(); it.Next())
            {
                Shape shape = it.Value();
                Placement L = shape.Location();
                if (myContext.ModeConsiderLocation())
                {
                    var nullLoc = new Placement();
                    shape.Location(nullLoc);
                }
                myShape = myContext.InternalApply(shape);
                Perform(false);
                if (myContext.ModeConsiderLocation())
                {
                    myResult.Location(L);
                }
                myResult.Orientation(global::Kerf.Engine.Topology.TopAbs.Compose(myResult.Orientation(), savShape.Orientation()));
                B.Add(C, myResult);
                locStatus |= myStatus;
            }

            myShape = savShape;
            myStatus = locStatus;
            if (Status(ShapeExtend_DONE))
            {
                myResult = myContext.InternalApply(C, ShapeType.Shape);
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
            Msg doneMsg = GetFaceMsg();

            for (var exp = new Explorer(myShape, ShapeType.Face); exp.More(); exp.Next())
            {
                Shape tmpF = exp.Current().Oriented(Forward);
                Face F = ShapeCasts.Face(tmpF); // protection against INTERNAL shapes: cts20105a.rle
                Shape sh = myContext.InternalApply(F, ShapeType.Shape);
                for (var exp2 = new Explorer(sh, ShapeType.Face); exp2.More(); exp2.Next())
                {
                    try
                    {
                        for (; exp2.More(); exp2.Next())
                        {
                            Face face = ShapeCasts.Face(exp2.Current());
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
                                SendMsg(face, doneMsg, Info);
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
            SplitWire.SetFace(new Face());
            SplitWire.SetPrecision(myPrecision);
            SplitWire.SetMaxTolerance(myMaxTol);
            SplitWire.SetMinTolerance(myMinTol);
            SplitWire.SetEdgeMode(myEdgeMode);
            Msg doneMsg = GetWireMsg();

            var exp = new Explorer(); // svv Jan 10 2000 : porting on DEC
            for (exp.Init(myShape, ShapeType.Wire, ShapeType.Face); exp.More(); exp.Next())
            {
                // smh#8
                Shape tmpW = exp.Current().Oriented(Forward);
                Wire W = ShapeCasts.Wire(tmpW); // protection against INTERNAL shapes
                Shape sh = myContext.InternalApply(W, ShapeType.Shape);
                for (var exp2 = new Explorer(sh, ShapeType.Wire); exp2.More(); exp2.Next())
                {
                    Wire wire = ShapeCasts.Wire(exp2.Current());
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
                        SendMsg(wire, doneMsg, Info);
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                    }
                }
            }

            // Process free EDGEs
            Msg edgeDoneMsg = GetEdgeMsg();
            for (exp.Init(myShape, ShapeType.Edge, ShapeType.Wire); exp.More(); exp.Next())
            {
                // smh#8
                Shape tmpE = exp.Current().Oriented(Forward);
                Edge E = ShapeCasts.Edge(tmpE); // protection against INTERNAL shapes
                Exploration.Vertices(E, out Vertex V2, out Vertex V1);
                if (V1.IsNull && V2.IsNull)
                {
                    continue; // skl 27.10.2004 for OCC5624
                }
                Shape sh = myContext.InternalApply(E, ShapeType.Shape);
                for (var exp2 = new Explorer(sh, ShapeType.Edge); exp2.More(); exp2.Next())
                {
                    Edge edge = ShapeCasts.Edge(exp2.Current());
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
                        SendMsg(edge, edgeDoneMsg, Info);
                        myStatus |= ShapeExtend.EncodeStatus(ShapeExtend_DONE1);
                    }
                }
            }
        }
        myResult = myContext.InternalApply(myShape, ShapeType.Shape);
        return !myResult.IsSame(myShape);
    }

    /// <summary>Gives the resulting shape (null if not done), frozen.</summary>
    public Shape Result()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="Result"/> without freezing it, for ported callers (ShapeProcess).</summary>
    internal Shape InternalResult() => myResult;

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
    public void SendMsg(in Shape shape, Msg message, Gravity gravity = Info)
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
    protected virtual Msg GetFaceMsg() => new("ShapeDivide.FaceDivide.MSG0");

    /// <summary>The message sent for a divided free wire.</summary>
    protected virtual Msg GetWireMsg() => new("ShapeDivide.WireDivide.MSG0");

    /// <summary>The message sent for a divided free edge.</summary>
    protected virtual Msg GetEdgeMsg() => new("ShapeDivide.EdgeDivide.MSG0");
}
