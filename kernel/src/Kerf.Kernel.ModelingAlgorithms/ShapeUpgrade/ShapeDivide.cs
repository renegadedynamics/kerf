// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Divides all the faces of a shape (and its free wires and edges) with the split face tool's criteria, recording
/// the changes in a context (<see cref="ReShapeBuilder"/>). The compounds' children are processed one by one, to
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
public class ShapeDivide
{
    /// <summary>The context.</summary>
    protected ReShapeBuilder? _context;
    /// <summary>The message registrator.</summary>
    protected BasicMsgRegistrator? _msgReg;
    /// <summary>The shape.</summary>
    protected Shape _shape;
    /// <summary>The result.</summary>
    protected Shape _result;
    /// <summary>The precision.</summary>
    protected double _precision;
    /// <summary>The minimal tolerance.</summary>
    protected double _minTol;
    /// <summary>The maximal tolerance.</summary>
    protected double _maxTol;
    /// <summary>The mode for trimming (segmenting) surfaces by the wires' UV bounds.</summary>
    protected bool _segmentMode;
    /// <summary>The status.</summary>
    protected int _status;
    /// <summary>The mode for splitting 3D curves of edges.</summary>
    protected int _edgeMode;

    private FaceDivide? _splitFaceTool;

    /// <summary>Empty constructor.</summary>
    public ShapeDivide()
    {
        _shape = new Shape();
        _result = new Shape();
        _status = 0;
        _precision = _minTol = Kerf.Engine.Foundation.Precision.Confusion();
        _maxTol = 1; // Precision::Infinite() ?? pdn
        _splitFaceTool = new FaceDivide();
        _context = new ReShapeBuilder();
        // myMsgReg = new ShapeExtend_BasicMsgRegistrator;
        _segmentMode = true;
        _edgeMode = 2;
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeDivide(in Shape s) : this()
    {
        Init(s);
    }

    /// <summary>Initializes by a shape.</summary>
    public void Init(in Shape s)
    {
        _shape = s;
    }

    /// <summary>Defines the spatial precision used for splitting.</summary>
    public void SetPrecision(double prec)
    {
        _precision = prec;
    }

    /// <summary>Sets the maximal allowed tolerance.</summary>
    public void SetMaxTolerance(double maxtol)
    {
        _maxTol = maxtol;
    }

    /// <summary>Sets the minimal allowed tolerance.</summary>
    public void SetMinTolerance(double mintol)
    {
        _minTol = mintol;
    }

    /// <summary>Sets the mode for trimming (segmenting) surfaces by the wires' UV bounds.</summary>
    public void SetSurfaceSegmentMode(bool segment)
    {
        _segmentMode = segment;
    }

    /// <summary>
    /// Performs the splitting and computes the resulting shape. With <paramref name="newContext"/> (the default) the
    /// context is cleared at the start, else the previous substitutions apply. True if the result differs from the
    /// shape.
    /// </summary>
    public virtual bool Perform(bool newContext = true) => PerformAtEntry(newContext, PerformBody);

    // OCCT.NET: the copy at the entry (decision 999), a working overlay since Kerf unit 10H4. The face dividers'
    // ShapeFix_ComposeShell edits the edges it splits in place (as do ShapeConvertToBezier's last step and the dividers
    // called on a compound's children), and the edges shared by other faces see those edits natively; so the outermost
    // Perform runs on the caller's shape in an overlay that takes those writes, and the result holds new TShapes for
    // what changed and the caller's own for the rest (decision 583). Nested calls (a compound's children,
    // ShapeConvertToBezier's passes) and a Perform inside another operation's overlay run in place in that one.

    /// <summary>
    /// Runs <paramref name="body"/> in a working overlay unless one is open here, then assembles the result, the shape
    /// left in <c>myShape</c> and the context's replacements, with the caller's own sub-shapes wherever they are
    /// unchanged.
    /// </summary>
    protected bool PerformAtEntry(bool newContext, Func<bool, bool> body)
    {
        if (_shape.IsNull)
        {
            return body(newContext);
        }

        // a kept context's requests are over the caller's sub-shapes, as the overlay's
        using WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(_shape, newContext ? null : _context);
        if (overlay is null)
        {
            return body(newContext);
        }
        Shape input = _shape;
        bool res = body(newContext);

        overlay.AttachContext(_context);
        // ShapeConvertToBezier leaves myShape on the intermediate result of its surface conversion
        bool isIntermediate = !_shape.IsNull && !ReferenceEquals(_shape.TShape(), input.TShape());
        _result = overlay.Finish(_result, isIntermediate ? [_shape] : null);
        _shape = overlay.Final(_shape);
        return res;
    }

    private bool PerformBody(bool newContext)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (_shape.IsNull)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
            return false;
        }

        if (newContext || _context is null)
        {
            _context = new ReShapeBuilder();
        }

        // Process COMPOUNDs separately in order to handle sharing in assemblies
        // NOTE: not optimized: subshape can be processed twice (second time - no modif)
        if (_shape.Type == ShapeType.Compound)
        {
            int locStatus = _status;
            var c = new Compound();
            var b = new BrepBuilder();
            b.MakeCompound(ref c);
            Shape savShape = _shape;
            for (var it = new Iterator(savShape, false); it.More(); it.Next())
            {
                Shape shape = it.Value();
                Placement l = shape.Placement;
                if (_context.ModeConsiderLocation())
                {
                    var nullLoc = new Placement();
                    shape.Location(nullLoc);
                }
                _shape = _context.InternalApply(shape);
                Perform(false);
                if (_context.ModeConsiderLocation())
                {
                    _result.Location(l);
                }
                _result.Orient(global::Kerf.Engine.Topology.TopAbs.Compose(_result.Orientation, savShape.Orientation));
                b.Add(c, _result);
                locStatus |= _status;
            }

            _shape = savShape;
            _status = locStatus;
            if (Status(ExtendStatus.Done))
            {
                _result = _context.InternalApply(c, ShapeType.Shape);
                _context.Replace(_shape, _result);
                return true;
            }
            _result = _shape;
            return false;
        }

        // Process FACEs
        FaceDivide? splitFace = GetSplitFaceTool();
        if (splitFace is not null)
        {
            splitFace.SetPrecision(_precision);
            splitFace.SetMaxTolerance(_maxTol);
            splitFace.SetSurfaceSegmentMode(_segmentMode);
            WireDivide? splitWireTool = splitFace.GetWireDivideTool();
            if (splitWireTool is not null)
            {
                splitWireTool.SetMinTolerance(_minTol);
                splitWireTool.SetEdgeMode(_edgeMode);
            }
            Msg doneMsg = GetFaceMsg();

            for (var exp = new Explorer(_shape, ShapeType.Face); exp.More(); exp.Next())
            {
                Shape tmpF = exp.Current().Oriented(Forward);
                Face f = ShapeCasts.Face(tmpF); // protection against INTERNAL shapes: cts20105a.rle
                Shape sh = _context.InternalApply(f, ShapeType.Shape);
                for (var exp2 = new Explorer(sh, ShapeType.Face); exp2.More(); exp2.Next())
                {
                    try
                    {
                        for (; exp2.More(); exp2.Next())
                        {
                            Face face = ShapeCasts.Face(exp2.Current());
                            splitFace.Init(face);
                            splitFace.SetContext(_context);
                            splitFace.Perform();
                            if (splitFace.Status(ExtendStatus.Fail))
                            {
                                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
                            }
                            if (splitFace.Status(ExtendStatus.Done))
                            {
                                _context.Replace(face, splitFace.InternalResult());
                                SendMsg(face, doneMsg, Info);
                                if (splitFace.Status(ExtendStatus.DONE1))
                                {
                                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                                }
                                if (splitFace.Status(ExtendStatus.DONE2))
                                {
                                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                                }
                            }
                        }
                    }
                    catch (Exception exception) when (global::Kerf.Engine.Healing.Upgrade.ShapeUpgrade.IsCaughtFailure(exception))
                    {
                        // OCCT_DEBUG output left out: exception in ShapeUpgrade_FaceDivide::Perform()
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
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
        WireDivide? splitWire = splitFace!.GetWireDivideTool();
        if (splitWire is not null)
        {
            splitWire.SetFace(new Face());
            splitWire.SetPrecision(_precision);
            splitWire.SetMaxTolerance(_maxTol);
            splitWire.SetMinTolerance(_minTol);
            splitWire.SetEdgeMode(_edgeMode);
            Msg doneMsg = GetWireMsg();

            var exp = new Explorer(); // svv Jan 10 2000 : porting on DEC
            for (exp.Init(_shape, ShapeType.Wire, ShapeType.Face); exp.More(); exp.Next())
            {
                // smh#8
                Shape tmpW = exp.Current().Oriented(Forward);
                Wire w = ShapeCasts.Wire(tmpW); // protection against INTERNAL shapes
                Shape sh = _context.InternalApply(w, ShapeType.Shape);
                for (var exp2 = new Explorer(sh, ShapeType.Wire); exp2.More(); exp2.Next())
                {
                    Wire wire = ShapeCasts.Wire(exp2.Current());
                    splitWire.Load(wire);
                    splitWire.SetContext(_context);
                    splitWire.Perform();
                    if (splitWire.Status(ExtendStatus.Fail))
                    {
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
                    }
                    if (splitWire.Status(ExtendStatus.Done))
                    {
                        _context.Replace(wire, splitWire.InternalWire());
                        SendMsg(wire, doneMsg, Info);
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                    }
                }
            }

            // Process free EDGEs
            Msg edgeDoneMsg = GetEdgeMsg();
            for (exp.Init(_shape, ShapeType.Edge, ShapeType.Wire); exp.More(); exp.Next())
            {
                // smh#8
                Shape tmpE = exp.Current().Oriented(Forward);
                Edge e = ShapeCasts.Edge(tmpE); // protection against INTERNAL shapes
                Exploration.Vertices(e, out Vertex v2, out Vertex v1);
                if (v1.IsNull && v2.IsNull)
                {
                    continue; // skl 27.10.2004 for OCC5624
                }
                Shape sh = _context.InternalApply(e, ShapeType.Shape);
                for (var exp2 = new Explorer(sh, ShapeType.Edge); exp2.More(); exp2.Next())
                {
                    Edge edge = ShapeCasts.Edge(exp2.Current());
                    splitWire.Load(edge);
                    splitWire.SetContext(_context);
                    splitWire.Perform();
                    if (splitWire.Status(ExtendStatus.Fail))
                    {
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
                    }
                    if (splitWire.Status(ExtendStatus.Done))
                    {
                        _context.Replace(edge, splitWire.InternalWire());
                        SendMsg(edge, edgeDoneMsg, Info);
                        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                    }
                }
            }
        }
        _result = _context.InternalApply(_shape, ShapeType.Shape);
        return !_result.IsSame(_shape);
    }

    /// <summary>Gives the resulting shape (null if not done), frozen.</summary>
    public Shape Result()
    {
        _result.TShape()?.FreezeGraph();
        return _result;
    }

    /// <summary><see cref="Result"/> without freezing it, for ported callers (ShapeProcess).</summary>
    internal Shape InternalResult() => _result;

    /// <summary>Returns the context with all the modifications made during the last call(s) to Perform recorded.</summary>
    public ReShapeBuilder? GetContext() => _context;

    /// <summary>Sets the context with recorded modifications to apply during the next call(s) to Perform(false).</summary>
    public void SetContext(ReShapeBuilder? context)
    {
        _context = context;
    }

    /// <summary>Sets the message registrator.</summary>
    public virtual void SetMsgRegistrator(BasicMsgRegistrator? msgreg)
    {
        _msgReg = msgreg;
    }

    /// <summary>Returns the message registrator.</summary>
    public BasicMsgRegistrator? MsgRegistrator() => _msgReg;

    /// <summary>Sends a message attached to the shape, through the message registrator.</summary>
    public void SendMsg(in Shape shape, Msg message, Gravity gravity = Info)
    {
        _msgReg?.Send(shape, message, gravity);
    }

    /// <summary>
    /// Queries the status of the last call to Perform: OK (no splitting done, or no call), DONE1 (some edges split),
    /// DONE2 (a surface split), FAIL1 (errors).
    /// </summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Sets the tool for splitting faces.</summary>
    public void SetSplitFaceTool(FaceDivide? splitFaceTool)
    {
        _splitFaceTool = splitFaceTool;
    }

    /// <summary>Sets the mode for splitting 3D curves of edges: 0 free edges only, 1 shared edges only, 2 all.</summary>
    public void SetEdgeMode(int aEdgeMode)
    {
        _edgeMode = aEdgeMode;
    }

    /// <summary>Returns the tool for splitting faces.</summary>
    protected virtual FaceDivide? GetSplitFaceTool() => _splitFaceTool;

    /// <summary>The message sent for a divided face.</summary>
    protected virtual Msg GetFaceMsg() => new("ShapeDivide.FaceDivide.MSG0");

    /// <summary>The message sent for a divided free wire.</summary>
    protected virtual Msg GetWireMsg() => new("ShapeDivide.WireDivide.MSG0");

    /// <summary>The message sent for a divided free edge.</summary>
    protected virtual Msg GetEdgeMsg() => new("ShapeDivide.EdgeDivide.MSG0");
}
