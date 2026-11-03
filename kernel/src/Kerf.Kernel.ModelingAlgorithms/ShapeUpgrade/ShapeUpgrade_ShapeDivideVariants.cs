// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideAngle.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideAngle.cxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideArea.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideArea.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideArea.cxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideClosed.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideClosed.cxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideClosedEdges.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideClosedEdges.cxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideContinuity.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_ShapeDivideContinuity.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Splits all the faces of a shape on surfaces of revolution, cylinders, cones, spheres and tori into segments no
/// longer than the maximal angle in U (<see cref="SplitSurfaceAngle"/>); the edges aren't divided.
/// </summary>
/// <remarks>A working object (decision 21); see <see cref="ShapeDivide"/>.</remarks>
public class ShapeDivideAngle : ShapeDivide
{
    /// <summary>Creates the tool with the maximal angle.</summary>
    public ShapeDivideAngle(double maxAngle)
    {
        InitTool(maxAngle);
    }

    /// <summary>Creates the tool with the maximal angle and the shape.</summary>
    public ShapeDivideAngle(double maxAngle, in Shape s) : base(s)
    {
        InitTool(maxAngle);
    }

    /// <summary>Resets the tool for splitting faces with the given angle.</summary>
    public void InitTool(double maxAngle)
    {
        FaceDivide tool = GetSplitFaceTool()!;
        tool.SetSplitSurfaceTool(new SplitSurfaceAngle(maxAngle));
        tool.SetWireDivideTool(null); // no splitting of wire
        SetSplitFaceTool(tool);
    }

    /// <summary>Sets the maximal angle (calls <see cref="InitTool"/>).</summary>
    public void SetMaxAngle(double maxAngle)
    {
        InitTool(maxAngle);
    }

    /// <summary>Returns the maximal angle.</summary>
    public double MaxAngle()
    {
        FaceDivide? faceTool = GetSplitFaceTool();
        if (faceTool is null)
        {
            return 0.0;
        }
        var tool = faceTool.GetSplitSurfaceTool() as SplitSurfaceAngle;
        return (tool is null ? 0.0 : tool.MaxAngle());
    }
}

/// <summary>
/// Divides all the faces of a shape into parts of an area no greater than the maximal area, or into a number of
/// parts (<see cref="FaceDivideArea"/>).
/// </summary>
/// <remarks>
/// A working object (decision 21); see <see cref="ShapeDivide"/>. OCCT's modifiable references
/// (<see cref="MaxArea"/>, <see cref="NbParts"/>) return <c>ref</c>.
/// </remarks>
public class ShapeDivideArea : ShapeDivide
{
    private double _maxArea;
    private int _nbParts;
    private int _unbSplit;
    private int _vnbSplit;
    private bool _isSplittingByNumber;

    /// <summary>Empty constructor (no maximal area).</summary>
    public ShapeDivideArea()
    {
        _maxArea = Kerf.Engine.Foundation.Precision.Infinite();
        _nbParts = 0;
        _unbSplit = _vnbSplit = -1;
        _isSplittingByNumber = false;
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeDivideArea(in Shape s) : base(s)
    {
        _maxArea = Kerf.Engine.Foundation.Precision.Infinite();
        _nbParts = 0;
        _unbSplit = _vnbSplit = -1;
        _isSplittingByNumber = false;
    }

    /// <summary>The maximal area allowed for the faces.</summary>
    public ref double MaxArea() => ref _maxArea;

    /// <summary>The number of parts expected when splitting by number.</summary>
    public ref int NbParts() => ref _nbParts;

    /// <summary>Sets fixed numbers of splits in U and V (splitting by number).</summary>
    public void SetNumbersUVSplits(int nbUsplits, int nbVsplits)
    {
        _isSplittingByNumber = true;
        _maxArea = -1;
        _unbSplit = nbUsplits;
        _vnbSplit = nbVsplits;
        if (_unbSplit > 0 && _vnbSplit > 0)
        {
            _nbParts = _unbSplit * _vnbSplit;
        }
    }

    /// <summary>
    /// Sets the splitting mode: splitting by number splits each face into about <see cref="NbParts"/> parts close to
    /// squares in 2D.
    /// </summary>
    public void SetSplittingByNumber(bool isSplittingByNumber)
    {
        _isSplittingByNumber = isSplittingByNumber;
        if (_isSplittingByNumber)
        {
            _maxArea = -1;
        }
    }

    /// <summary>Returns a new tool for splitting faces with the current parameters.</summary>
    protected override FaceDivide? GetSplitFaceTool()
    {
        var faceTool = new FaceDivideArea();
        faceTool.MaxArea() = _maxArea;
        faceTool.NbParts() = _nbParts;
        faceTool.SetNumbersUVSplits(_unbSplit, _vnbSplit);
        faceTool.SetSplittingByNumber(_isSplittingByNumber);
        return faceTool;
    }
}

/// <summary>
/// Divides all the closed faces of a shape into the given number of parts plus one
/// (<see cref="ClosedFaceDivide"/>); the edges aren't divided.
/// </summary>
/// <remarks>A working object (decision 21); see <see cref="ShapeDivide"/>.</remarks>
public class ShapeDivideClosed : ShapeDivide
{
    /// <summary>Initializes the tool with the shape and one cut.</summary>
    public ShapeDivideClosed(in Shape s) : base(s)
    {
        SetNbSplitPoints(1);
    }

    /// <summary>Sets the number of cuts applied to divide closed faces (the resulting faces are num + 1).</summary>
    public void SetNbSplitPoints(int num)
    {
        var tool = new ClosedFaceDivide();
        tool.SetNbSplitPoints(num);
        tool.SetWireDivideTool(null); // no splitting of wire
        SetSplitFaceTool(tool);
    }
}

/// <summary>
/// Divides all the closed edges of a shape in two (<see cref="ClosedEdgeDivide"/>); the surfaces aren't
/// split.
/// </summary>
/// <remarks>A working object (decision 21); see <see cref="ShapeDivide"/>.</remarks>
public class ShapeDivideClosedEdges : ShapeDivide
{
    /// <summary>Initializes the tool with the shape and one cut.</summary>
    public ShapeDivideClosedEdges(in Shape s) : base(s)
    {
        SetNbSplitPoints(1);
    }

    /// <summary>Sets the number of cuts applied to divide closed edges (unused: each closed edge is cut once).</summary>
    public void SetNbSplitPoints(int num)
    {
        var tool = new ClosedEdgeDivide();
        var wtool = new WireDivide();
        wtool.SetEdgeDivideTool(tool);
        var ftool = new FaceDivide();
        ftool.SetWireDivideTool(wtool);
        ftool.SetSplitSurfaceTool(null);
        SetSplitFaceTool(ftool);
    }
}

/// <summary>
/// Divides the faces and edges of a shape at their discontinuities below the criteria (C1 by default for the 3D
/// curves, the pcurves and the surfaces), removing knots within the tolerances where possible.
/// </summary>
/// <remarks>A working object (decision 21); see <see cref="ShapeDivide"/>.</remarks>
public class ShapeDivideContinuity : ShapeDivide
{
    private Continuity _curve3dCriterion;
    private Continuity _curve2dCriterion;
    private Continuity _surfaceCriterion;
    private double _tolerance3d;
    private double _tolerance2d;

    /// <summary>Empty constructor (criteria C1, tolerances Confusion and PConfusion).</summary>
    public ShapeDivideContinuity()
    {
        _curve3dCriterion = C1;
        _curve2dCriterion = C1;
        _surfaceCriterion = C1;
        _tolerance3d = Kerf.Engine.Foundation.Precision.Confusion();
        _tolerance2d = Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeDivideContinuity(in Shape s) : base(s)
    {
        _curve3dCriterion = C1;
        _curve2dCriterion = C1;
        _surfaceCriterion = C1;
        _tolerance3d = Kerf.Engine.Foundation.Precision.Confusion();
        _tolerance2d = Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>Sets the 3D tolerance (for the 3D curves and the surfaces).</summary>
    public void SetTolerance(double tol)
    {
        _tolerance3d = tol;
    }

    /// <summary>Sets the 2D tolerance (for the pcurves).</summary>
    public void SetTolerance2d(double tol)
    {
        _tolerance2d = tol;
    }

    /// <summary>Defines the criterion of continuity for the 3D curves (C1 by default; G1 and G2 aren't authorized).</summary>
    public void SetBoundaryCriterion(Continuity criterion = C1)
    {
        _curve3dCriterion = criterion;
    }

    /// <summary>Defines the criterion of continuity for the pcurves (C1 by default; G1 and G2 aren't authorized).</summary>
    public void SetPCurveCriterion(Continuity criterion = C1)
    {
        _curve2dCriterion = criterion;
    }

    /// <summary>Defines the criterion of continuity for the surfaces (C1 by default; G1 and G2 aren't authorized).</summary>
    public void SetSurfaceCriterion(Continuity criterion = C1)
    {
        _surfaceCriterion = criterion;
    }

    /// <summary>Returns a new tool for dividing faces with the current criteria.</summary>
    protected override FaceDivide? GetSplitFaceTool()
    {
        var theSplitCurve2dTool = new SplitCurve2dContinuity();
        var theSplitCurve3dTool = new SplitCurve3dContinuity();
        var theSplitSurfaceTool = new SplitSurfaceContinuity();
        theSplitCurve2dTool.SetCriterion(_curve2dCriterion);
        theSplitCurve3dTool.SetCriterion(_curve3dCriterion);
        theSplitSurfaceTool.SetCriterion(_surfaceCriterion);
        theSplitCurve2dTool.SetTolerance(_tolerance2d);
        theSplitCurve3dTool.SetTolerance(_tolerance3d);
        theSplitSurfaceTool.SetTolerance(_tolerance3d);
        var theSplitWireTool = new WireDivide();
        theSplitWireTool.SetSplitCurve2dTool(theSplitCurve2dTool);
        theSplitWireTool.SetSplitCurve3dTool(theSplitCurve3dTool);
        var theSplitFaceTool = new FaceDivide();
        theSplitFaceTool.SetSplitSurfaceTool(theSplitSurfaceTool);
        theSplitFaceTool.SetWireDivideTool(theSplitWireTool);
        return theSplitFaceTool;
    }
}
