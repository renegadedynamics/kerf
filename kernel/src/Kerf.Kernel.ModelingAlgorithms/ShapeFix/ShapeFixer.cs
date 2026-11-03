// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shape.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shape.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_Shape.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>
/// Fixes a shape of any type: dispatches it, by type, to ShapeFix_Solid, ShapeFix_Shell, ShapeFix_Face, ShapeFix_Wire
/// or ShapeFix_Edge (going down a compound or a compsolid), then makes the edges SameParameter and checks the
/// tolerances of the vertices.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). <see cref="Init"/> records the shape (and makes a context considering locations if
/// none was set). <see cref="Perform"/> edits the shape it works on in place, as OCCT's: a shape holding frozen shapes
/// (any shape a caller holds) is copied first (<see cref="CopyAtEntry"/>, decision 583), so the caller's
/// shape never changes. At the end of <see cref="Perform"/> the copies the fixes left unchanged are the caller's own
/// again, in the result and in the context, and the context records the sub-shapes changed in place, so it answers for
/// the caller's shapes. A shape needing nothing gives back the input itself.
/// </para>
/// <para>
/// <see cref="Shape"/> freezes the result; ported code fixing a shape it owns reads <see cref="InternalShape"/>. The
/// tools (<see cref="FixSolidTool"/> down to <see cref="FixEdgeTool"/>) are this tool's, as OCCT's: set their modes
/// before <see cref="Perform"/>.
/// </para>
/// </remarks>
public class ShapeFixer : RootFixer
{
    /// <summary>The result.</summary>
    protected Shape                  _result;
    /// <summary>The solid fixing tool (holding the shell, face, wire and edge tools).</summary>
    protected SolidFixer                _fixSolid;
    /// <summary>The shapes (without location) already fixed by this tool.</summary>
    protected HashSet<Shape> _mapFixingShape;
    /// <summary>The mode of the solid fixes.</summary>
    protected int                           _fixSolidMode;
    /// <summary>The mode of the fixes of free shells.</summary>
    protected int                           _fixShellMode;
    /// <summary>The mode of the fixes of free faces.</summary>
    protected int                           _fixFaceMode;
    /// <summary>The mode of the fixes of free wires.</summary>
    protected int                           _fixWireMode;
    /// <summary>The mode of the SameParameter fix.</summary>
    protected int                           _fixSameParameterMode;
    /// <summary>The mode of the fix of the vertex positions.</summary>
    protected int                           _fixVertexPositionMode;
    /// <summary>The mode of the fix of the vertex tolerances.</summary>
    protected int                           _fixVertexTolMode;
    /// <summary>The status.</summary>
    protected int                           _status;

    // The shapes added to myMapFixingShape during the current Perform (keys over the working copy).
    private List<Shape>? _addedKeys;

    /// <summary>Empty constructor.</summary>
    public ShapeFixer()
    {
        _result                = new Shape();
        _mapFixingShape        = new HashSet<Shape>(ShapeHasher.Instance);
        _status                = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _fixSolidMode          = -1;
        _fixShellMode          = -1;
        _fixFaceMode           = -1;
        _fixWireMode           = -1;
        _fixSameParameterMode  = -1;
        _fixVertexPositionMode = 0;
        _fixVertexTolMode      = -1;
        _fixSolid              = new SolidFixer();
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeFixer(in Shape shape)
    {
        _result                = new Shape();
        _mapFixingShape        = new HashSet<Shape>(ShapeHasher.Instance);
        _status                = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        _fixSolidMode          = -1;
        _fixShellMode          = -1;
        _fixFaceMode           = -1;
        _fixWireMode           = -1;
        _fixSameParameterMode  = -1;
        _fixSolid              = new SolidFixer();
        _fixVertexPositionMode = 0;
        _fixVertexTolMode      = -1;
        Init(shape);
    }

    /// <summary>Initializes by a shape; makes a context considering locations if none was set.</summary>
    public void Init(in Shape shape)
    {
        _shape = shape;
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
            Context()!.ModeConsiderLocation(true);
        }
        _result = _shape;
    }

    /// <summary>
    /// Fixes the shape (by type, see the class), then makes its edges SameParameter and checks the tolerances of its
    /// vertices, as the modes allow. Returns true if something was done.
    /// </summary>
    public bool Perform(ProgressRange progress = default)
    {
        // OCCT's early return (a shape this tool already fixed, or one the context made) runs on the caller's shape
        Shape shapeNullLoc = _shape.Located(new Placement());
        if (Context()!.IsNewShape(_shape) || _mapFixingShape.Contains(shapeNullLoc))
        {
            return PerformCore(progress);
        }

        var          entry = new CopyAtEntry(_shape, Context());
        Shape input = _shape;
        _shape     = entry.Shape();
        _addedKeys = [];
        bool status = PerformCore(progress);
        _shape  = input;
        _result = entry.Finish(_result);

        // the shapes fixed, as the caller's (a further Perform on them takes OCCT's early return)
        if (entry.IsCopied)
        {
            foreach (Shape key in _addedKeys)
            {
                _mapFixingShape.Remove(key);
                _mapFixingShape.Add(entry.Original(key));
            }
        }
        _addedKeys = null;
        return status;
    }

    // ShapeFix_Shape::Perform on the working shape (recursive over compounds)
    private bool PerformCore(ProgressRange progress)
    {
        ReShapeBuilder context                = Context()!;
        int                savFixSmallAreaWireMode = 0;
        int                savFixVertexTolMode     = _fixVertexTolMode;
        FaceFixer      fft                     = FixFaceTool();
        savFixSmallAreaWireMode = fft.FixSmallAreaWireMode();
        if (savFixSmallAreaWireMode == -1 && _shape.ShapeType() == ShapeType.Face)
        {
            fft.FixSmallAreaWireMode() = 1;
        }

        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        bool             status = false;
        ShapeType st;

        // gka fix for sharing assembly
        Placement nullLoc = new(), l;
        l = _shape.Location();
        Shape shapeNullLoc = _shape;
        bool         isRecorded   = context.IsNewShape(_shape);
        shapeNullLoc.Location(nullLoc);
        if (isRecorded || _mapFixingShape.Contains(shapeNullLoc))
        {
            _shape.Location(l, false);
            _result = context.InternalApply(_shape);
            status   = true;
            return status;
        }
        _mapFixingShape.Add(shapeNullLoc);
        _addedKeys?.Add(shapeNullLoc);
        //---------------------------------------
        _shape.Location(l, false);
        Shape s = context.InternalApply(_shape);
        if (NeedFix(_fixVertexPositionMode))
        {
            ShapeFixFixer.FixVertexPosition(ref s, Precision(), context);
        }

        st = s.ShapeType();

        // Open progress indication scope for the following fix stages:
        // - Fix on Solid or Shell;
        // - Fix same parameterization;
        using var ps = new ProgressScope(progress, "Fixing stage", 2);

        switch (st)
        {
            case ShapeType.Compound:
            case ShapeType.CompSolid:
            {
                Shape shape                   = _shape;
                int          savFixSameParameterMode = _fixSameParameterMode;
                _fixSameParameterMode = 0;
                _fixVertexTolMode     = 0;
                int shapesNb          = s.ChildCount;

                // Open progress indication scope for sub-shape fixing
                using var psSubShape = new ProgressScope(ps.Next(), "Fixing sub-shape", shapesNb);
                for (var iter = new Iterator(s); iter.More() && psSubShape.More(); iter.Next())
                {
                    _shape = iter.Value();
                    if (PerformCore(psSubShape.Next()))
                    {
                        status = true;
                    }
                }
                if (!psSubShape.More())
                {
                    return false; // aborted execution
                }
                _fixSameParameterMode = savFixSameParameterMode;
                _fixVertexTolMode     = savFixVertexTolMode;
                _shape                = shape;
                break;
            }
            case ShapeType.Solid:
            {
                if (!NeedFix(_fixSolidMode))
                {
                    break;
                }
                _fixSolid.Init(ShapeCasts.Solid(s));
                _fixSolid.SetContext(context);

                if (_fixSolid.Perform(ps.Next()))
                {
                    status = true;
                }

                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                break;
            }
            case ShapeType.Shell:
            {
                if (!NeedFix(_fixShellMode))
                {
                    break;
                }
                ShellFixer sfsh = FixShellTool();
                sfsh.Init(ShapeCasts.Shell(s));
                sfsh.SetContext(context);

                if (sfsh.Perform(ps.Next()))
                {
                    status = true;
                }

                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE4);
                break;
            }
            case ShapeType.Face:
            {
                if (!NeedFix(_fixFaceMode))
                {
                    break;
                }
                FaceFixer sff         = FixFaceTool();
                bool          savTopoMode = sff.FixWireTool().ModifyTopologyMode();
                sff.FixWireTool().ModifyTopologyMode() = true;
                sff.Init(ShapeCasts.Face(s));
                sff.SetContext(context);

                if (sff.Perform())
                {
                    status = true;
                }
                sff.FixWireTool().ModifyTopologyMode() = savTopoMode;

                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE3);
                break;
            }
            case ShapeType.Wire:
            {
                if (!NeedFix(_fixWireMode))
                {
                    break;
                }
                WireFixer sfw           = FixWireTool();
                bool          savTopoMode   = sfw.ModifyTopologyMode();
                bool          savClosedMode = sfw.ClosedWireMode();
                sfw.ModifyTopologyMode() = true;
                if (!s.Closed())
                {
                    sfw.ClosedWireMode() = false;
                }
                sfw.SetFace(new Face());
                sfw.Load(ShapeCasts.Wire(s));
                sfw.SetContext(context);
                if (sfw.Perform())
                {
                    status = true;
                    context.Replace(s, sfw.InternalWire()); // replace for wire only
                }
                sfw.ModifyTopologyMode() = savTopoMode;
                sfw.ClosedWireMode()     = savClosedMode;
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
                break;
            }
            case ShapeType.Edge:
            {
                EdgeFixer sfe = FixEdgeTool();
                sfe.SetContext(context);
                if (sfe.FixVertexTolerance(ShapeCasts.Edge(s)))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                }
                break;
            }
            case ShapeType.Vertex:
            case global::Kerf.Topology.ShapeType.Shape:
            default:
                break;
        }
        if (!ps.More())
        {
            return false; // aborted execution
        }

        _result = context.InternalApply(s);

        if (NeedFix(_fixSameParameterMode))
        {
            SameParameter(_result, false, ps.Next());

            if (!ps.More())
            {
                return false; // aborted execution
            }
        }
        if (NeedFix(_fixVertexTolMode))
        {
            int nbF    = 0;
            var expF = new Explorer(_result, ShapeType.Face);
            for (; expF.More() && nbF <= 1; expF.Next())
            {
                nbF++;
            }
            if (nbF > 1)
            {
                // fix for bug  0025455
                //  for case when vertex belong to the different faces it is necessary to check vertices
                //  tolerances
                // after all fixes.
                // This fix it should be performed for example for case when cutting edge was performed.
                EdgeFixer sfe = FixEdgeTool();
                for (expF.ReInit(); expF.More(); expF.Next())
                {
                    Face f     = ShapeCasts.Face(expF.Current());
                    var         expE = new Explorer(f, ShapeType.Edge);
                    for (; expE.More(); expE.Next())
                    {
                        sfe.FixVertexTolerance(ShapeCasts.Edge(expE.Current()), f);
                    }
                }
            }
        }

        _result = context.InternalApply(_result);

        fft.FixSmallAreaWireMode() = savFixSmallAreaWireMode;

        return status;
    }

    /// <summary>Fixes SameParameter of the edges of the shape (ShapeFix::SameParameter). Writes into the shape.</summary>
    protected virtual void SameParameter(in Shape sh, bool enforce, ProgressRange progress = default)
    {
        ShapeFixFixer.SameParameter(sh, enforce, 0.0, progress);
    }

    /// <summary>Returns the resulting shape, frozen.</summary>
    public Shape Shape()
    {
        _result.TShape()?.FreezeGraph();
        return _result;
    }

    /// <summary>Returns the resulting shape, unfrozen, for ported callers fixing a shape they own.</summary>
    internal Shape InternalShape() => _result;

    /// <summary>Returns the solid fixing tool.</summary>
    public SolidFixer FixSolidTool() => _fixSolid;

    /// <summary>Returns the shell fixing tool.</summary>
    public ShellFixer FixShellTool() => _fixSolid.FixShellTool();

    /// <summary>Returns the face fixing tool.</summary>
    public FaceFixer FixFaceTool() => _fixSolid.FixShellTool().FixFaceTool();

    /// <summary>Returns the wire fixing tool.</summary>
    public WireFixer FixWireTool() => _fixSolid.FixShellTool().FixFaceTool().FixWireTool();

    /// <summary>Returns the edge fixing tool.</summary>
    public EdgeFixer FixEdgeTool() => _fixSolid.FixShellTool().FixFaceTool().FixWireTool().FixEdgeTool();

    /// <summary>
    /// Returns the status: OK nothing done; DONE1 vertex tolerances fixed on a free edge; DONE2 a free wire fixed;
    /// DONE3 a free face fixed; DONE4 a shell or a solid fixed.
    /// </summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Sets the message registrator, for this tool and its tools.</summary>
    public override void SetMsgRegistrator(BasicMsgRegistrator? msgreg)
    {
        base.SetMsgRegistrator(msgreg);
        _fixSolid.SetMsgRegistrator(msgreg);
    }

    /// <summary>Sets the basic precision, for this tool and its tools.</summary>
    public override void SetPrecision(double preci)
    {
        base.SetPrecision(preci);
        _fixSolid.SetPrecision(preci);
    }

    /// <summary>Sets the minimal tolerance, for this tool and its tools.</summary>
    public override void SetMinTolerance(double mintol)
    {
        base.SetMinTolerance(mintol);
        _fixSolid.SetMinTolerance(mintol);
    }

    /// <summary>Sets the maximal tolerance, for this tool and its tools.</summary>
    public override void SetMaxTolerance(double maxtol)
    {
        base.SetMaxTolerance(maxtol);
        _fixSolid.SetMaxTolerance(maxtol);
    }

    /// <summary>The mode of the solid fixes (-1 default, 0 off, 1 on).</summary>
    public ref int FixSolidMode() => ref _fixSolidMode;

    /// <summary>The mode of the fixes of free shells (-1 default, 0 off, 1 on).</summary>
    public ref int FixFreeShellMode() => ref _fixShellMode;

    /// <summary>The mode of the fixes of free faces (-1 default, 0 off, 1 on).</summary>
    public ref int FixFreeFaceMode() => ref _fixFaceMode;

    /// <summary>The mode of the fixes of free wires (-1 default, 0 off, 1 on).</summary>
    public ref int FixFreeWireMode() => ref _fixWireMode;

    /// <summary>The mode of the SameParameter fix (-1 default, 0 off, 1 on).</summary>
    public ref int FixSameParameterMode() => ref _fixSameParameterMode;

    /// <summary>The mode of the fix of the vertex positions (0 by default: off).</summary>
    public ref int FixVertexPositionMode() => ref _fixVertexPositionMode;

    /// <summary>The mode of the check of the vertex tolerances after all fixes (-1 default, 0 off, 1 on).</summary>
    public ref int FixVertexTolMode() => ref _fixVertexTolMode;
}
