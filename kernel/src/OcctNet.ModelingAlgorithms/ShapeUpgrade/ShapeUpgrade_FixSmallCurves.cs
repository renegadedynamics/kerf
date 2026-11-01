// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallCurves.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FixSmallCurves.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// The root of the tools that replace the small segments a split leaves at an edge's end by one approximated
/// segment; the base class approximates nothing (<see cref="Approx"/> returns false).
/// </summary>
/// <remarks>A working object (decision 21). It only reads the edge.</remarks>
public class ShapeUpgrade_FixSmallCurves : ShapeUpgrade_Tool
{
    /// <summary>The status.</summary>
    protected int myStatus;
    /// <summary>The tool splitting 3D curves.</summary>
    protected ShapeUpgrade_SplitCurve3d? mySplitCurve3dTool;
    /// <summary>The tool splitting pcurves.</summary>
    protected ShapeUpgrade_SplitCurve2d? mySplitCurve2dTool;
    /// <summary>The edge.</summary>
    protected TopoDS_Edge myEdge;
    /// <summary>The face.</summary>
    protected TopoDS_Face myFace;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_FixSmallCurves()
    {
        myEdge = new TopoDS_Edge();
        myFace = new TopoDS_Face();
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
    }

    /// <summary>Initializes with the edge and the face.</summary>
    public void Init(in TopoDS_Edge theEdge, in TopoDS_Face theFace)
    {
        myEdge = theEdge;
        myFace = theFace;
    }

    /// <summary>
    /// Approximates the edge's curves between <paramref name="First"/> and <paramref name="Last"/> by single segments;
    /// false if not done (always, in the base class).
    /// </summary>
    public virtual bool Approx(ref Geom_Curve? Curve3d,
                               ref Geom2d_Curve? Curve2d,
                               ref Geom2d_Curve? Curve2dR,
                               ref double First,
                               ref double Last)
    {
        return false;
    }

    /// <summary>Sets the tool for splitting 3D curves.</summary>
    public void SetSplitCurve3dTool(ShapeUpgrade_SplitCurve3d? splitCurve3dTool)
    {
        mySplitCurve3dTool = splitCurve3dTool;
    }

    /// <summary>Sets the tool for splitting pcurves.</summary>
    public void SetSplitCurve2dTool(ShapeUpgrade_SplitCurve2d? splitCurve2dTool)
    {
        mySplitCurve2dTool = splitCurve2dTool;
    }

    /// <summary>Queries the status of the last call.</summary>
    public bool Status(ShapeExtend_Status status) => ShapeExtend.DecodeStatus(myStatus, status);

    /// <summary>Returns the tool for splitting 3D curves.</summary>
    protected virtual ShapeUpgrade_SplitCurve3d? GetSplitCurve3dTool() => mySplitCurve3dTool;

    /// <summary>Returns the tool for splitting pcurves.</summary>
    protected virtual ShapeUpgrade_SplitCurve2d? GetSplitCurve2dTool() => mySplitCurve2dTool;
}
