// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivideArea.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivideArea.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_FaceDivideArea.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.ShapeExtend_Status;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// Divides a face into parts of an area no greater than the maximal area, or into a number of parts (splitting by
/// number, optionally with fixed numbers of U and V splits), with <see cref="ShapeUpgrade_SplitSurfaceArea"/>; when
/// splitting by area the parts still too large are divided again.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's modifiable references (<see cref="MaxArea"/>, <see cref="NbParts"/>) return
/// <c>ref</c>. The face given is not modified (see <see cref="ShapeUpgrade_FaceDivide"/>).
/// </remarks>
public class ShapeUpgrade_FaceDivideArea : ShapeUpgrade_FaceDivide
{
    private double myMaxArea;
    private int myNbParts;
    private int myUnbSplit;
    private int myVnbSplit;
    private bool myIsSplittingByNumber;

    /// <summary>Empty constructor (precision 1e-5, no maximal area).</summary>
    public ShapeUpgrade_FaceDivideArea()
    {
        myMaxArea = Occt.Precision.Infinite();
        myNbParts = 0;
        myUnbSplit = myVnbSplit = -1;
        myIsSplittingByNumber = false;
        SetPrecision(1.0e-5);
        SetSplitSurfaceTool(new ShapeUpgrade_SplitSurfaceArea());
    }

    /// <summary>Initializes by a face.</summary>
    public ShapeUpgrade_FaceDivideArea(in TopoDS_Face F) : this()
    {
        Init(F);
    }

    /// <summary>The maximal area allowed for the faces (-1 when splitting by number).</summary>
    public ref double MaxArea() => ref myMaxArea;

    /// <summary>The number of parts expected when splitting by number.</summary>
    public ref int NbParts() => ref myNbParts;

    /// <summary>Sets fixed numbers of splits in U and V (only when splitting by number).</summary>
    public void SetNumbersUVSplits(int theNbUsplits, int theNbVsplits)
    {
        myUnbSplit = theNbUsplits;
        myVnbSplit = theNbVsplits;
        if (myUnbSplit > 0 && myVnbSplit > 0)
        {
            myNbParts = myUnbSplit * myVnbSplit;
        }
    }

    /// <summary>
    /// Sets the splitting mode: splitting by number splits the face into about <see cref="NbParts"/> parts close to
    /// squares in 2D.
    /// </summary>
    public void SetSplittingByNumber(bool theIsSplittingByNumber)
    {
        myIsSplittingByNumber = theIsSplittingByNumber;
    }

    /// <summary>Performs the splitting and computes the resulting shell; true if something was done.</summary>
    public override bool Perform(double theArea = 0.0)
    {
        myStatus = ShapeExtend.EncodeStatus(ShapeExtend_OK);
        var aGprop = new GProp_GProps();

        BRepGProp.SurfaceProperties(myFace, aGprop, Precision());
        double anArea = aGprop.Mass();

        int anbParts = 0;
        if (myMaxArea == -1)
        {
            anbParts = myNbParts;
            myMaxArea = anArea / anbParts;
        }

        if ((anArea - myMaxArea) < Occt.Precision.Confusion())
        {
            return false;
        }

        if (anbParts == 0)
        {
            anbParts = RealToInt(Math.Ceiling(anArea / myMaxArea));
        }

        var aSurfTool = GetSplitSurfaceTool() as ShapeUpgrade_SplitSurfaceArea;
        if (aSurfTool is null)
        {
            return false;
        }
        aSurfTool.NbParts() = anbParts;
        if (myIsSplittingByNumber)
        {
            aSurfTool.SetSplittingIntoSquares(true);
            aSurfTool.SetNumbersUVSplits(myUnbSplit, myVnbSplit);
        }
        if (!base.Perform(anArea))
        {
            return false;
        }

        TopoDS_Shape aResult = InternalResult();
        if (aResult.ShapeType() == TopAbs_FACE)
        {
            return false;
        }
        int aStatus = myStatus;

        if (!myIsSplittingByNumber)
        {
            var aExpF = new TopExp_Explorer(aResult, TopAbs_FACE);
            TopoDS_Shape aCopyRes = aResult.EmptyCopied();

            bool isModified = false;
            for (; aExpF.More(); aExpF.Next())
            {
                TopoDS_Shape aSh = Context()!.InternalApply(aExpF.Current());
                TopoDS_Face aFace = TopoDS.Face(aSh);
                Init(aFace);
                var aB = new BRep_Builder();
                if (Perform())
                {
                    isModified = true;
                    TopoDS_Shape aRes = InternalResult();
                    var aExpR = new TopExp_Explorer(aRes, TopAbs_FACE);
                    for (; aExpR.More(); aExpR.Next())
                    {
                        aB.Add(aCopyRes, aExpR.Current());
                    }
                }
                else
                {
                    aB.Add(aCopyRes, aFace);
                }
            }
            if (isModified)
            {
                if (aCopyRes.ShapeType() == TopAbs_WIRE || aCopyRes.ShapeType() == TopAbs_SHELL)
                {
                    aCopyRes.Closed(BRep_Tool.IsClosed(aCopyRes));
                }
                Context()!.Replace(aResult, aCopyRes);
            }
        }

        myStatus |= aStatus;
        myResult = Context()!.InternalApply(aResult);
        return Status(ShapeExtend_DONE);
    }
}
