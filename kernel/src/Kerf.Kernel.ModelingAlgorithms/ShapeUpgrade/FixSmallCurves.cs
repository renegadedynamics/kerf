// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallCurves.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Extend;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// The root of the tools that replace the small segments a split leaves at an edge's end by one approximated
/// segment; the base class approximates nothing (<see cref="Approx"/> returns false).
/// </summary>
/// <remarks>A working object (decision 21). It only reads the edge.</remarks>
internal class FixSmallCurves : UpgradeTool
{
    /// <summary>The status.</summary>
    protected int _status;
    /// <summary>The tool splitting 3D curves.</summary>
    protected SplitCurve3d? _splitCurve3dTool;
    /// <summary>The tool splitting pcurves.</summary>
    protected SplitCurve2d? _splitCurve2dTool;
    /// <summary>The edge.</summary>
    protected Edge _edge;
    /// <summary>The face.</summary>
    protected Face _face;

    /// <summary>Empty constructor.</summary>
    public FixSmallCurves()
    {
        _edge = new Edge();
        _face = new Face();
        _status = global::Kerf.Engine.Healing.Extend.ShapeExtend.EncodeStatus(ExtendStatus.OK);
    }

    /// <summary>Initializes with the edge and the face.</summary>
    public void Init(in Edge edge, in Face face)
    {
        _edge = edge;
        _face = face;
    }

    /// <summary>
    /// Approximates the edge's curves between <paramref name="first"/> and <paramref name="last"/> by single segments;
    /// false if not done (always, in the base class).
    /// </summary>
    public virtual bool Approx(ref Curve3D? curve3d,
                               ref Curve2D? curve2d,
                               ref Curve2D? curve2dR,
                               ref double first,
                               ref double last)
    {
        return false;
    }

    /// <summary>Sets the tool for splitting 3D curves.</summary>
    public void SetSplitCurve3dTool(SplitCurve3d? splitCurve3dTool)
    {
        _splitCurve3dTool = splitCurve3dTool;
    }

    /// <summary>Sets the tool for splitting pcurves.</summary>
    public void SetSplitCurve2dTool(SplitCurve2d? splitCurve2dTool)
    {
        _splitCurve2dTool = splitCurve2dTool;
    }

    /// <summary>Queries the status of the last call.</summary>
    public bool Status(ExtendStatus status) => global::Kerf.Engine.Healing.Extend.ShapeExtend.DecodeStatus(_status, status);

    /// <summary>Returns the tool for splitting 3D curves.</summary>
    protected virtual SplitCurve3d? GetSplitCurve3dTool() => _splitCurve3dTool;

    /// <summary>Returns the tool for splitting pcurves.</summary>
    protected virtual SplitCurve2d? GetSplitCurve2dTool() => _splitCurve2dTool;
}
