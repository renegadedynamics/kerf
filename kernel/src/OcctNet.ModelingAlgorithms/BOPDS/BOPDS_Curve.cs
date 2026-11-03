// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Curve.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Curve.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

namespace Occt;

/// <summary>
/// An intersection curve of two faces: the curve, its box, its pave blocks, its technological vertices and its
/// tolerance.
/// </summary>
/// <remarks>
/// A class, a working object held in <see cref="BOPDS_InterfFF.ChangeCurves"/> and changed in place (decision
/// 6151). OCCT's by-value copies are the copy constructor, which copies the lists (sharing the pave blocks, as the
/// C++ copy shares the handles).
/// </remarks>
public class BOPDS_Curve
{
    protected IntTools_Curve myCurve = new();
    protected NCollection_List<BOPDS_PaveBlock> myPaveBlocks = new();
    protected NCollection_List<int> myTechnoVertices = new();
    protected Box myBox = new();
    protected double myTolerance;

    public BOPDS_Curve()
    {
        myTolerance = 0.0;
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's implicit copy constructor).</summary>
    public BOPDS_Curve(BOPDS_Curve theOther)
    {
        myCurve = theOther.myCurve;
        myPaveBlocks = new NCollection_List<BOPDS_PaveBlock>(theOther.myPaveBlocks);
        myTechnoVertices = new NCollection_List<int>(theOther.myTechnoVertices);
        myBox = theOther.myBox;
        myTolerance = theOther.myTolerance;
    }

    internal void SetCurve(in IntTools_Curve theCurve) => myCurve = theCurve;

    public IntTools_Curve Curve() => myCurve;

    internal void SetBox(in Box theBox) => myBox = theBox;

    public Box Box() => myBox;

    internal ref Box ChangeBox() => ref myBox;

    /// <summary>Replaces the pave blocks by those of the list.</summary>
    internal void SetPaveBlocks(NCollection_List<BOPDS_PaveBlock> theLPB)
    {
        myPaveBlocks.Clear();
        for (NCollection_List<BOPDS_PaveBlock>.Iterator aIt = new(theLPB); aIt.More(); aIt.Next())
        {
            myPaveBlocks.Append(aIt.Value());
        }
    }

    /// <summary>The pave blocks of the curve (the curve's own list; read only for callers).</summary>
    public NCollection_List<BOPDS_PaveBlock> PaveBlocks() => myPaveBlocks;

    internal NCollection_List<BOPDS_PaveBlock> ChangePaveBlocks() => myPaveBlocks;

    /// <summary>Creates the initial pave block of the curve, if it has none.</summary>
    internal void InitPaveBlock1()
    {
        if (myPaveBlocks.Extent() == 0)
        {
            BOPDS_PaveBlock aPB = new();
            myPaveBlocks.Append(aPB);
        }
    }

    /// <summary>The initial pave block, as a reference into the list (OCCT returns a reference to the handle).</summary>
    internal ref BOPDS_PaveBlock ChangePaveBlock1() => ref myPaveBlocks.ChangeFirst();

    /// <summary>The indices of the technological vertices of the curve (read only for callers).</summary>
    public NCollection_List<int> TechnoVertices() => myTechnoVertices;

    internal NCollection_List<int> ChangeTechnoVertices() => myTechnoVertices;

    /// <summary>True if a pave block of the curve has an edge.</summary>
    public bool HasEdge()
    {
        bool bFlag;
        //
        bFlag = false;
        for (NCollection_List<BOPDS_PaveBlock>.Iterator aItPB = new(myPaveBlocks); aItPB.More(); aItPB.Next())
        {
            BOPDS_PaveBlock aPB = aItPB.Value();
            bFlag = aPB.HasEdge();
            if (bFlag)
            {
                break;
            }
        }
        return bFlag;
    }

    internal void SetTolerance(double theTol) => myTolerance = theTol;

    public double Tolerance() => myTolerance;

    /// <summary>The tangential tolerance of the curve.</summary>
    public double TangentialTolerance() => myCurve.TangentialTolerance();
}
