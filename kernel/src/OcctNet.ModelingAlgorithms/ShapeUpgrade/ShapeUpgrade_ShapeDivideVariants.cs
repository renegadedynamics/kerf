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
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Splits all the faces of a shape on surfaces of revolution, cylinders, cones, spheres and tori into segments no
/// longer than the maximal angle in U (<see cref="ShapeUpgrade_SplitSurfaceAngle"/>); the edges aren't divided.
/// </summary>
/// <remarks>A working object (decision 21); see <see cref="ShapeUpgrade_ShapeDivide"/>.</remarks>
public class ShapeUpgrade_ShapeDivideAngle : ShapeUpgrade_ShapeDivide
{
    /// <summary>Creates the tool with the maximal angle.</summary>
    public ShapeUpgrade_ShapeDivideAngle(double MaxAngle)
    {
        InitTool(MaxAngle);
    }

    /// <summary>Creates the tool with the maximal angle and the shape.</summary>
    public ShapeUpgrade_ShapeDivideAngle(double MaxAngle, in Shape S) : base(S)
    {
        InitTool(MaxAngle);
    }

    /// <summary>Resets the tool for splitting faces with the given angle.</summary>
    public void InitTool(double MaxAngle)
    {
        ShapeUpgrade_FaceDivide tool = GetSplitFaceTool()!;
        tool.SetSplitSurfaceTool(new ShapeUpgrade_SplitSurfaceAngle(MaxAngle));
        tool.SetWireDivideTool(null); // no splitting of wire
        SetSplitFaceTool(tool);
    }

    /// <summary>Sets the maximal angle (calls <see cref="InitTool"/>).</summary>
    public void SetMaxAngle(double MaxAngle)
    {
        InitTool(MaxAngle);
    }

    /// <summary>Returns the maximal angle.</summary>
    public double MaxAngle()
    {
        ShapeUpgrade_FaceDivide? faceTool = GetSplitFaceTool();
        if (faceTool is null)
        {
            return 0.0;
        }
        var tool = faceTool.GetSplitSurfaceTool() as ShapeUpgrade_SplitSurfaceAngle;
        return (tool is null ? 0.0 : tool.MaxAngle());
    }
}

/// <summary>
/// Divides all the faces of a shape into parts of an area no greater than the maximal area, or into a number of
/// parts (<see cref="ShapeUpgrade_FaceDivideArea"/>).
/// </summary>
/// <remarks>
/// A working object (decision 21); see <see cref="ShapeUpgrade_ShapeDivide"/>. OCCT's modifiable references
/// (<see cref="MaxArea"/>, <see cref="NbParts"/>) return <c>ref</c>.
/// </remarks>
public class ShapeUpgrade_ShapeDivideArea : ShapeUpgrade_ShapeDivide
{
    private double myMaxArea;
    private int myNbParts;
    private int myUnbSplit;
    private int myVnbSplit;
    private bool myIsSplittingByNumber;

    /// <summary>Empty constructor (no maximal area).</summary>
    public ShapeUpgrade_ShapeDivideArea()
    {
        myMaxArea = Kerf.Engine.Foundation.Precision.Infinite();
        myNbParts = 0;
        myUnbSplit = myVnbSplit = -1;
        myIsSplittingByNumber = false;
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeUpgrade_ShapeDivideArea(in Shape S) : base(S)
    {
        myMaxArea = Kerf.Engine.Foundation.Precision.Infinite();
        myNbParts = 0;
        myUnbSplit = myVnbSplit = -1;
        myIsSplittingByNumber = false;
    }

    /// <summary>The maximal area allowed for the faces.</summary>
    public ref double MaxArea() => ref myMaxArea;

    /// <summary>The number of parts expected when splitting by number.</summary>
    public ref int NbParts() => ref myNbParts;

    /// <summary>Sets fixed numbers of splits in U and V (splitting by number).</summary>
    public void SetNumbersUVSplits(int theNbUsplits, int theNbVsplits)
    {
        myIsSplittingByNumber = true;
        myMaxArea = -1;
        myUnbSplit = theNbUsplits;
        myVnbSplit = theNbVsplits;
        if (myUnbSplit > 0 && myVnbSplit > 0)
        {
            myNbParts = myUnbSplit * myVnbSplit;
        }
    }

    /// <summary>
    /// Sets the splitting mode: splitting by number splits each face into about <see cref="NbParts"/> parts close to
    /// squares in 2D.
    /// </summary>
    public void SetSplittingByNumber(bool theIsSplittingByNumber)
    {
        myIsSplittingByNumber = theIsSplittingByNumber;
        if (myIsSplittingByNumber)
        {
            myMaxArea = -1;
        }
    }

    /// <summary>Returns a new tool for splitting faces with the current parameters.</summary>
    protected override ShapeUpgrade_FaceDivide? GetSplitFaceTool()
    {
        var aFaceTool = new ShapeUpgrade_FaceDivideArea();
        aFaceTool.MaxArea() = myMaxArea;
        aFaceTool.NbParts() = myNbParts;
        aFaceTool.SetNumbersUVSplits(myUnbSplit, myVnbSplit);
        aFaceTool.SetSplittingByNumber(myIsSplittingByNumber);
        return aFaceTool;
    }
}

/// <summary>
/// Divides all the closed faces of a shape into the given number of parts plus one
/// (<see cref="ShapeUpgrade_ClosedFaceDivide"/>); the edges aren't divided.
/// </summary>
/// <remarks>A working object (decision 21); see <see cref="ShapeUpgrade_ShapeDivide"/>.</remarks>
public class ShapeUpgrade_ShapeDivideClosed : ShapeUpgrade_ShapeDivide
{
    /// <summary>Initializes the tool with the shape and one cut.</summary>
    public ShapeUpgrade_ShapeDivideClosed(in Shape S) : base(S)
    {
        SetNbSplitPoints(1);
    }

    /// <summary>Sets the number of cuts applied to divide closed faces (the resulting faces are num + 1).</summary>
    public void SetNbSplitPoints(int num)
    {
        var tool = new ShapeUpgrade_ClosedFaceDivide();
        tool.SetNbSplitPoints(num);
        tool.SetWireDivideTool(null); // no splitting of wire
        SetSplitFaceTool(tool);
    }
}

/// <summary>
/// Divides all the closed edges of a shape in two (<see cref="ShapeUpgrade_ClosedEdgeDivide"/>); the surfaces aren't
/// split.
/// </summary>
/// <remarks>A working object (decision 21); see <see cref="ShapeUpgrade_ShapeDivide"/>.</remarks>
public class ShapeUpgrade_ShapeDivideClosedEdges : ShapeUpgrade_ShapeDivide
{
    /// <summary>Initializes the tool with the shape and one cut.</summary>
    public ShapeUpgrade_ShapeDivideClosedEdges(in Shape S) : base(S)
    {
        SetNbSplitPoints(1);
    }

    /// <summary>Sets the number of cuts applied to divide closed edges (unused: each closed edge is cut once).</summary>
    public void SetNbSplitPoints(int num)
    {
        var tool = new ShapeUpgrade_ClosedEdgeDivide();
        var wtool = new ShapeUpgrade_WireDivide();
        wtool.SetEdgeDivideTool(tool);
        var ftool = new ShapeUpgrade_FaceDivide();
        ftool.SetWireDivideTool(wtool);
        ftool.SetSplitSurfaceTool(null);
        SetSplitFaceTool(ftool);
    }
}

/// <summary>
/// Divides the faces and edges of a shape at their discontinuities below the criteria (C1 by default for the 3D
/// curves, the pcurves and the surfaces), removing knots within the tolerances where possible.
/// </summary>
/// <remarks>A working object (decision 21); see <see cref="ShapeUpgrade_ShapeDivide"/>.</remarks>
public class ShapeUpgrade_ShapeDivideContinuity : ShapeUpgrade_ShapeDivide
{
    private Continuity myCurve3dCriterion;
    private Continuity myCurve2dCriterion;
    private Continuity mySurfaceCriterion;
    private double myTolerance3d;
    private double myTolerance2d;

    /// <summary>Empty constructor (criteria C1, tolerances Confusion and PConfusion).</summary>
    public ShapeUpgrade_ShapeDivideContinuity()
    {
        myCurve3dCriterion = C1;
        myCurve2dCriterion = C1;
        mySurfaceCriterion = C1;
        myTolerance3d = Kerf.Engine.Foundation.Precision.Confusion();
        myTolerance2d = Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>Initializes by a shape.</summary>
    public ShapeUpgrade_ShapeDivideContinuity(in Shape S) : base(S)
    {
        myCurve3dCriterion = C1;
        myCurve2dCriterion = C1;
        mySurfaceCriterion = C1;
        myTolerance3d = Kerf.Engine.Foundation.Precision.Confusion();
        myTolerance2d = Kerf.Engine.Foundation.Precision.PConfusion();
    }

    /// <summary>Sets the 3D tolerance (for the 3D curves and the surfaces).</summary>
    public void SetTolerance(double Tol)
    {
        myTolerance3d = Tol;
    }

    /// <summary>Sets the 2D tolerance (for the pcurves).</summary>
    public void SetTolerance2d(double Tol)
    {
        myTolerance2d = Tol;
    }

    /// <summary>Defines the criterion of continuity for the 3D curves (C1 by default; G1 and G2 aren't authorized).</summary>
    public void SetBoundaryCriterion(Continuity Criterion = C1)
    {
        myCurve3dCriterion = Criterion;
    }

    /// <summary>Defines the criterion of continuity for the pcurves (C1 by default; G1 and G2 aren't authorized).</summary>
    public void SetPCurveCriterion(Continuity Criterion = C1)
    {
        myCurve2dCriterion = Criterion;
    }

    /// <summary>Defines the criterion of continuity for the surfaces (C1 by default; G1 and G2 aren't authorized).</summary>
    public void SetSurfaceCriterion(Continuity Criterion = C1)
    {
        mySurfaceCriterion = Criterion;
    }

    /// <summary>Returns a new tool for dividing faces with the current criteria.</summary>
    protected override ShapeUpgrade_FaceDivide? GetSplitFaceTool()
    {
        var theSplitCurve2dTool = new ShapeUpgrade_SplitCurve2dContinuity();
        var theSplitCurve3dTool = new ShapeUpgrade_SplitCurve3dContinuity();
        var theSplitSurfaceTool = new ShapeUpgrade_SplitSurfaceContinuity();
        theSplitCurve2dTool.SetCriterion(myCurve2dCriterion);
        theSplitCurve3dTool.SetCriterion(myCurve3dCriterion);
        theSplitSurfaceTool.SetCriterion(mySurfaceCriterion);
        theSplitCurve2dTool.SetTolerance(myTolerance2d);
        theSplitCurve3dTool.SetTolerance(myTolerance3d);
        theSplitSurfaceTool.SetTolerance(myTolerance3d);
        var theSplitWireTool = new ShapeUpgrade_WireDivide();
        theSplitWireTool.SetSplitCurve2dTool(theSplitCurve2dTool);
        theSplitWireTool.SetSplitCurve3dTool(theSplitCurve3dTool);
        var theSplitFaceTool = new ShapeUpgrade_FaceDivide();
        theSplitFaceTool.SetSplitSurfaceTool(theSplitSurfaceTool);
        theSplitFaceTool.SetWireDivideTool(theSplitWireTool);
        return theSplitFaceTool;
    }
}
