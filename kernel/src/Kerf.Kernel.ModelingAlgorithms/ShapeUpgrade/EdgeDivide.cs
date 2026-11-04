// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_EdgeDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_EdgeDivide.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_EdgeDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Healing.Extend.ExtendStatus;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Computes the splitting values of an edge: on its 3D curve and on its pcurve on the face, with the split tools.
/// </summary>
/// <remarks>A working object (decision 21). It only reads the edge.</remarks>
internal class EdgeDivide : UpgradeTool
{
    /// <summary>The face the pcurve is taken on (none for free edges).</summary>
    protected Face _face;
    /// <summary>True if the edge has a pcurve on the face.</summary>
    protected bool _hasCurve2d;
    /// <summary>True if the edge has a 3D curve.</summary>
    protected bool _hasCurve3d;
    /// <summary>The splitting values on the pcurve.</summary>
    protected NCollection_HSequence<double>? _knots2d;
    /// <summary>The splitting values on the 3D curve.</summary>
    protected NCollection_HSequence<double>? _knots3d;

    private SplitCurve3d? _splitCurve3dTool;
    private SplitCurve2d? _splitCurve2dTool;

    /// <summary>Empty constructor.</summary>
    public EdgeDivide()
    {
        _face = new Face();
        _splitCurve3dTool = new SplitCurve3d();
        _splitCurve2dTool = new SplitCurve2d();
    }

    /// <summary>Clears the computed values.</summary>
    public void Clear()
    {
        _knots3d = null;
        _knots2d = null;
        _hasCurve3d = false;
        _hasCurve2d = false;
    }

    /// <summary>Sets the supporting face.</summary>
    public void SetFace(in Face f)
    {
        _face = f;
    }

    /// <summary>Computes the splitting values of the edge; true if it has to be split.</summary>
    public virtual bool Compute(in Edge anEdge)
    {
        Clear();

        Curve3D? curve3d = Tool.Curve(anEdge, out double f, out double l);
        _hasCurve3d = curve3d is not null;

        SplitCurve3d theSplit3dTool = GetSplitCurve3dTool()!;
        if (_hasCurve3d)
        {
            theSplit3dTool.Init(curve3d!, f, l);
            theSplit3dTool.Compute();
            _knots3d = theSplit3dTool.SplitValues();
        }

        // on pcurve(s): all knots
        // assume that if seam-edge, its pcurve1 and pcurve2 has the same split knots !!!
        double f2d = 0.0, l2d = 0.0;
        Curve2D? pcurve1 = null;
        if (!_face.IsNull)
        { // process free edges
            var sae = new EdgeAnalysis();
            sae.PCurve(anEdge, _face, out pcurve1, out f2d, out l2d, false);
        }
        _hasCurve2d = pcurve1 is not null;

        SplitCurve2d theSplit2dTool = GetSplitCurve2dTool()!;
        if (_hasCurve2d)
        {
            theSplit2dTool.Init(pcurve1!, f2d, l2d);
            theSplit2dTool.Compute();
            _knots2d = theSplit2dTool.SplitValues();
        }
        return theSplit3dTool.Status(ExtendStatus.Done) || theSplit2dTool.Status(ExtendStatus.Done);
    }

    /// <summary>True if the edge has a pcurve on the face.</summary>
    public bool HasCurve2d => _hasCurve2d;

    /// <summary>True if the edge has a 3D curve.</summary>
    public bool HasCurve3d => _hasCurve3d;

    /// <summary>The splitting values on the pcurve.</summary>
    public NCollection_HSequence<double>? Knots2d() => _knots2d;

    /// <summary>The splitting values on the 3D curve.</summary>
    public NCollection_HSequence<double>? Knots3d() => _knots3d;

    /// <summary>Sets the tool for splitting pcurves.</summary>
    public void SetSplitCurve2dTool(SplitCurve2d? splitCurve2dTool)
    {
        _splitCurve2dTool = splitCurve2dTool;
    }

    /// <summary>Sets the tool for splitting 3D curves.</summary>
    public void SetSplitCurve3dTool(SplitCurve3d? splitCurve3dTool)
    {
        _splitCurve3dTool = splitCurve3dTool;
    }

    /// <summary>Returns the tool for splitting pcurves.</summary>
    public virtual SplitCurve2d? GetSplitCurve2dTool() => _splitCurve2dTool;

    /// <summary>Returns the tool for splitting 3D curves.</summary>
    public virtual SplitCurve3d? GetSplitCurve3dTool() => _splitCurve3dTool;
}
