// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Divides a face (both its surface and its boundary) with criteria: the surface is split with the split surface tool
/// and the face rebuilt on the patches with ShapeFix_ComposeShell (giving a shell), then the edges of the resulting
/// faces are divided with the wire divide tool. The changes are recorded in the context (created if none is set).
/// </summary>
/// <remarks>
/// A working object (decision 21). The face given is not modified (decision 999): ShapeFix_ComposeShell, which edits
/// the face's edges, works on a copy of a frozen face (decision 583) and gives back the caller's unchanged
/// sub-shapes; the wire divider only builds new shapes. <see cref="Result"/> freezes what it returns; ported callers
/// read <see cref="InternalResult"/>.
/// </remarks>
internal class FaceDivide : UpgradeTool
{
    /// <summary>The face.</summary>
    protected Face _face;
    /// <summary>The result (a face or a shell).</summary>
    protected Shape _result;
    /// <summary>The mode for trimming (segmenting) the surface by the wire's UV bounds.</summary>
    protected bool _segmentMode;
    /// <summary>The status.</summary>
    protected int _status;

    private SplitSurface? _splitSurfaceTool;
    private WireDivide? _wireDivideTool;

    /// <summary>Empty constructor.</summary>
    public FaceDivide()
    {
        _face = new Face();
        _result = new Shape();
        _status = 0;
        _segmentMode = true;
        _splitSurfaceTool = new SplitSurface();
        _wireDivideTool = new WireDivide();
    }

    /// <summary>Initializes by a face.</summary>
    public FaceDivide(in Face f) : this()
    {
        Init(f);
    }

    /// <summary>Initializes by a face.</summary>
    public void Init(in Face f)
    {
        _result = _face = f;
    }

    /// <summary>Sets the mode for trimming (segmenting) the surface by the wire's UV bounds.</summary>
    public void SetSurfaceSegmentMode(bool segment)
    {
        _segmentMode = segment;
    }

    /// <summary>
    /// Performs the splitting and computes the resulting shell (the context keeps track of former splittings); true if
    /// something was done.
    /// </summary>
    public virtual bool Perform(double area = 0.0)
    {
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
        if (_face.IsNull)
        {
            return false;
        }
        if (Context() is null)
        {
            SetContext(new ReShapeBuilder());
        }
        _result = _face;
        SplitSurface(area);
        SplitCurves();
        return Status(ExtendStatus.Done);
    }

    /// <summary>
    /// Splits the surface and rebuilds the face on the patches (with ShapeFix_ComposeShell); true if the surface was
    /// split.
    /// </summary>
    public virtual bool SplitSurface(double area = 0.0)
    {
        SplitSurface? splitSurf = GetSplitSurfaceTool();
        if (splitSurf is null)
        {
            return false;
        }

        // myResult should be face; else return with FAIL
        if (_result.IsNull || _result.Type != ShapeType.Face)
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
            return false;
        }
        Face face = ShapeCasts.Face(_result);

        Surface surf = Tool.Surface(face, out Placement l)!;

        //  BRepTools::UVBounds(myFace,Uf,Ul,Vf,Vl);
        ShapeAnalysisAnalysis.GetFaceUVBounds(face, out double uf, out double ul, out double vf, out double vl);
        if (Kerf.Engine.Foundation.Precision.IsInfinite(uf) || Kerf.Engine.Foundation.Precision.IsInfinite(ul) || Kerf.Engine.Foundation.Precision.IsInfinite(vf)
            || Kerf.Engine.Foundation.Precision.IsInfinite(vl))
        {
            return false;
        }

        // make little extension to ensure all pcurves fit inside new surface bounds
        surf.ParameterBounds(out double sUf, out double sUl, out double sVf, out double sVl);
        if (!surf.IsUPeriodic)
        {
            double dU = (ul - uf) * 0.01;
            if (uf > sUf)
            {
                uf -= Math.Min(dU, uf - sUf);
            }
            if (ul < sUl)
            {
                ul += Math.Min(dU, sUl - ul);
            }
        }
        if (!surf.IsVPeriodic)
        {
            double dV = (vl - vf) * 0.01;
            if (vf > sVf)
            {
                vf -= Math.Min(dV, vf - sVf);
            }
            if (vl < sVl)
            {
                vl += Math.Min(dV, sVl - vl);
            }
        }

        splitSurf.Init(surf, uf, ul, vf, vl, area);
        splitSurf.Perform(_segmentMode);

        // If surface was neither split nor modified, do nothing
        if (!splitSurf.Status(ExtendStatus.Done))
        {
            return false;
        }

        // if surface was modified, force copying all vertices (and edges as consequence)
        // to protect original shape from increasing tolerance after SameParameter
        if (splitSurf.Status(ExtendStatus.DONE3))
        {
            for (var exp = new Explorer(face, ShapeType.Vertex); exp.More(); exp.Next())
            {
                if (Context()!.IsRecorded(exp.Current()))
                {
                    continue;
                }
                Shape emptyCopied = exp.Current().EmptyCopied();
                Vertex v = ShapeCasts.Vertex(emptyCopied);
                Context()!.Replace(exp.Current(), v);
            }
        }

        CompositeSurface grid = splitSurf.ResSurfaces()!;

        var compShell = new ComposeShellFixer();
        compShell.Init(grid, l, face, Precision());
        compShell.SetContext(Context());
        compShell.SetMaxTolerance(MaxTolerance());
        WireDivide? splitWire = GetWireDivideTool();
        if (splitWire is not null)
        {
            compShell.SetTransferParamTool(GetWireDivideTool()!.GetTransferParamTool()!);
        }

        compShell.Perform();
        if (compShell.Status(ExtendStatus.Fail) || !compShell.Status(ExtendStatus.Done))
        {
            _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL2);
        }

        // Kerf: the result unfrozen, the caller's sub-shapes given back where the face was copied (decision 999)
        _result = compShell.InternalFinishedResult();
        _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE2);
        return true;
    }

    /// <summary>Divides the edges of the faces of the result with the wire divide tool; true if something was done.</summary>
    public virtual bool SplitCurves()
    {
        WireDivide? splitWire = GetWireDivideTool();
        if (splitWire is null)
        {
            return false;
        }

        splitWire.SetMaxTolerance(MaxTolerance());
        for (var explf = new Explorer(_result, ShapeType.Face); explf.More(); explf.Next())
        {
            Shape s = Context()!.InternalApply(explf.Current(), ShapeType.Shape);

            // S should be face; else return with FAIL
            if (s.IsNull || s.Type != ShapeType.Face)
            {
                _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL3);
                return false;
            }

            Face f = ShapeCasts.Face(s);
            splitWire.SetFace(f);
            for (var wi = new Iterator(f, false); wi.More(); wi.Next())
            {
                // TopoDS_Wire wire = TopoDS::Wire ( wi.Value() );
                //  modifications already defined in context are to be applied inside SplitWire
                if (wi.Value().Type != ShapeType.Wire)
                {
                    continue;
                }

                Wire wire = ShapeCasts.Wire(wi.Value());
                splitWire.Load(wire);
                splitWire.SetContext(Context());
                splitWire.Perform();
                if (splitWire.Status(ExtendStatus.Fail))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.FAIL1);
                }
                if (splitWire.Status(ExtendStatus.Done))
                {
                    _status |= global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.DONE1);
                    Context()!.Replace(wire, splitWire.InternalWire());
                }
            }
        }
        _result = Context()!.InternalApply(_result);
        return Status(ExtendStatus.Done);
    }

    /// <summary>Gives the resulting shell or face (the initial face if not done, null if not initialized), frozen.</summary>
    public Shape Result()
    {
        _result.TShape()?.FreezeGraph();
        return _result;
    }

    /// <summary><see cref="Result"/> without freezing it, for ported callers.</summary>
    internal Shape InternalResult() => _result;

    /// <summary>
    /// Queries the status of the last <see cref="Perform"/>: OK (no splitting done), DONE1 (some edges split), DONE2
    /// (the surface split), FAIL1 (errors in splitting edges), FAIL2 (errors in splitting the surface), FAIL3 (the
    /// result isn't a face).
    /// </summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Sets the tool for splitting the surface.</summary>
    public void SetSplitSurfaceTool(SplitSurface? splitSurfaceTool)
    {
        _splitSurfaceTool = splitSurfaceTool;
    }

    /// <summary>Sets the tool for dividing the edges (none: no division of the edges).</summary>
    public void SetWireDivideTool(WireDivide? wireDivideTool)
    {
        _wireDivideTool = wireDivideTool;
    }

    /// <summary>Returns the tool for splitting the surface.</summary>
    public virtual SplitSurface? GetSplitSurfaceTool() => _splitSurfaceTool;

    /// <summary>Returns the tool for dividing the edges.</summary>
    public virtual WireDivide? GetWireDivideTool() => _wireDivideTool;
}
