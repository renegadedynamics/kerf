// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_EdgeDivide.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_EdgeDivide.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_EdgeDivide.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2000-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;

namespace Occt;

/// <summary>
/// Computes the splitting values of an edge: on its 3D curve and on its pcurve on the face, with the split tools.
/// </summary>
/// <remarks>A working object (decision 21). It only reads the edge.</remarks>
public class ShapeUpgrade_EdgeDivide : ShapeUpgrade_Tool
{
    /// <summary>The face the pcurve is taken on (none for free edges).</summary>
    protected TopoDS_Face myFace;
    /// <summary>True if the edge has a pcurve on the face.</summary>
    protected bool myHasCurve2d;
    /// <summary>True if the edge has a 3D curve.</summary>
    protected bool myHasCurve3d;
    /// <summary>The splitting values on the pcurve.</summary>
    protected NCollection_HSequence<double>? myKnots2d;
    /// <summary>The splitting values on the 3D curve.</summary>
    protected NCollection_HSequence<double>? myKnots3d;

    private ShapeUpgrade_SplitCurve3d? mySplitCurve3dTool;
    private ShapeUpgrade_SplitCurve2d? mySplitCurve2dTool;

    /// <summary>Empty constructor.</summary>
    public ShapeUpgrade_EdgeDivide()
    {
        myFace = new TopoDS_Face();
        mySplitCurve3dTool = new ShapeUpgrade_SplitCurve3d();
        mySplitCurve2dTool = new ShapeUpgrade_SplitCurve2d();
    }

    /// <summary>Clears the computed values.</summary>
    public void Clear()
    {
        myKnots3d = null;
        myKnots2d = null;
        myHasCurve3d = false;
        myHasCurve2d = false;
    }

    /// <summary>Sets the supporting face.</summary>
    public void SetFace(in TopoDS_Face F)
    {
        myFace = F;
    }

    /// <summary>Computes the splitting values of the edge; true if it has to be split.</summary>
    public virtual bool Compute(in TopoDS_Edge anEdge)
    {
        Clear();

        Geom_Curve? curve3d = BRep_Tool.Curve(anEdge, out double f, out double l);
        myHasCurve3d = curve3d is not null;

        ShapeUpgrade_SplitCurve3d theSplit3dTool = GetSplitCurve3dTool()!;
        if (myHasCurve3d)
        {
            theSplit3dTool.Init(curve3d!, f, l);
            theSplit3dTool.Compute();
            myKnots3d = theSplit3dTool.SplitValues();
        }

        // on pcurve(s): all knots
        // assume that if seam-edge, its pcurve1 and pcurve2 has the same split knots !!!
        double f2d = 0.0, l2d = 0.0;
        Geom2d_Curve? pcurve1 = null;
        if (!myFace.IsNull())
        { // process free edges
            var sae = new ShapeAnalysis_Edge();
            sae.PCurve(anEdge, myFace, out pcurve1, out f2d, out l2d, false);
        }
        myHasCurve2d = pcurve1 is not null;

        ShapeUpgrade_SplitCurve2d theSplit2dTool = GetSplitCurve2dTool()!;
        if (myHasCurve2d)
        {
            theSplit2dTool.Init(pcurve1!, f2d, l2d);
            theSplit2dTool.Compute();
            myKnots2d = theSplit2dTool.SplitValues();
        }
        return theSplit3dTool.Status(ShapeExtend_DONE) || theSplit2dTool.Status(ShapeExtend_DONE);
    }

    /// <summary>True if the edge has a pcurve on the face.</summary>
    public bool HasCurve2d() => myHasCurve2d;

    /// <summary>True if the edge has a 3D curve.</summary>
    public bool HasCurve3d() => myHasCurve3d;

    /// <summary>The splitting values on the pcurve.</summary>
    public NCollection_HSequence<double>? Knots2d() => myKnots2d;

    /// <summary>The splitting values on the 3D curve.</summary>
    public NCollection_HSequence<double>? Knots3d() => myKnots3d;

    /// <summary>Sets the tool for splitting pcurves.</summary>
    public void SetSplitCurve2dTool(ShapeUpgrade_SplitCurve2d? splitCurve2dTool)
    {
        mySplitCurve2dTool = splitCurve2dTool;
    }

    /// <summary>Sets the tool for splitting 3D curves.</summary>
    public void SetSplitCurve3dTool(ShapeUpgrade_SplitCurve3d? splitCurve3dTool)
    {
        mySplitCurve3dTool = splitCurve3dTool;
    }

    /// <summary>Returns the tool for splitting pcurves.</summary>
    public virtual ShapeUpgrade_SplitCurve2d? GetSplitCurve2dTool() => mySplitCurve2dTool;

    /// <summary>Returns the tool for splitting 3D curves.</summary>
    public virtual ShapeUpgrade_SplitCurve3d? GetSplitCurve3dTool() => mySplitCurve3dTool;
}
