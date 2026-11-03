// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Section.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Section.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BOPAlgo_Operation;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The section of two shapes (or planes, or surfaces): the edges and vertices where they meet, as a compound.
/// </summary>
/// <remarks>
/// The options: <see cref="Approximation"/> (approximate the section curves as B-splines),
/// <see cref="ComputePCurveOn1"/> and <see cref="ComputePCurveOn2"/> (attach p-curves on the first or second
/// argument's faces). A plane or surface argument becomes a face (C2 or smoother surfaces) or a shell
/// (BRepBuilderAPI_MakeShell), as in OCCT.
/// </remarks>
public class BRepAlgoAPI_Section : BRepAlgoAPI_BooleanOperation
{
    private bool myApprox;
    private bool myComputePCurve1;
    private bool myComputePCurve2;

    /// <summary>An empty section.</summary>
    public BRepAlgoAPI_Section()
    {
        Init(false);
    }

    /// <summary>A section using the intersections already computed by <paramref name="aPF"/>.</summary>
    public BRepAlgoAPI_Section(BOPAlgo_PaveFiller aPF)
        : base(aPF)
    {
        Init(false);
    }

    /// <summary>The section of two shapes, built at once unless <paramref name="PerformNow"/> is false.</summary>
    public BRepAlgoAPI_Section(in Shape Sh1, in Shape Sh2, bool PerformNow = true)
        : base(Sh1, Sh2, BOPAlgo_SECTION)
    {
        Init(PerformNow);
    }

    /// <summary>The section of two shapes with the intersections already computed by <paramref name="aDSF"/>.</summary>
    public BRepAlgoAPI_Section(in Shape aS1, in Shape aS2, BOPAlgo_PaveFiller aDSF, bool PerformNow = true)
        : base(aS1, aS2, aDSF, BOPAlgo_SECTION)
    {
        Init(PerformNow);
    }

    /// <summary>The section of a shape by a plane.</summary>
    public BRepAlgoAPI_Section(in Shape Sh, in Plane3D Pl, bool PerformNow = true)
        : base(Sh, MakeShape(new PlaneSurface(Pl)), BOPAlgo_SECTION)
    {
        Init(PerformNow);
    }

    /// <summary>The section of a shape by a surface.</summary>
    public BRepAlgoAPI_Section(in Shape Sh, Surface Sf, bool PerformNow = true)
        : base(Sh, MakeShape(Sf), BOPAlgo_SECTION)
    {
        Init(PerformNow);
    }

    /// <summary>The section of a surface by a shape.</summary>
    public BRepAlgoAPI_Section(Surface Sf, in Shape Sh, bool PerformNow = true)
        : base(MakeShape(Sf), Sh, BOPAlgo_SECTION)
    {
        Init(PerformNow);
    }

    /// <summary>The section of two surfaces.</summary>
    public BRepAlgoAPI_Section(Surface Sf1, Surface Sf2, bool PerformNow = true)
        : base(MakeShape(Sf1), MakeShape(Sf2), BOPAlgo_SECTION)
    {
        Init(PerformNow);
    }

    /// <summary>Sets the operation and the default options, and builds if asked.</summary>
    protected void Init(bool bFlag)
    {
        myOperation = BOPAlgo_SECTION;
        myApprox = false;
        myComputePCurve1 = false;
        myComputePCurve2 = false;
        //
        if (bFlag)
        {
            Build();
        }
    }

    /// <summary>Sets the first argument (replacing the objects).</summary>
    public void Init1(in Shape S1)
    {
        myArguments.Clear();
        myArguments.Append(S1);
    }

    /// <summary>Sets the first argument to a plane.</summary>
    public void Init1(in Plane3D Pl) => Init1(MakeShape(new PlaneSurface(Pl)));

    /// <summary>Sets the first argument to a surface.</summary>
    public void Init1(Surface Sf) => Init1(MakeShape(Sf));

    /// <summary>Sets the second argument (replacing the tools).</summary>
    public void Init2(in Shape S2)
    {
        myTools.Clear();
        myTools.Append(S2);
    }

    /// <summary>Sets the second argument to a plane.</summary>
    public void Init2(in Plane3D Pl) => Init2(MakeShape(new PlaneSurface(Pl)));

    /// <summary>Sets the second argument to a surface.</summary>
    public void Init2(Surface Sf) => Init2(MakeShape(Sf));

    /// <summary>Approximates the section curves as B-splines.</summary>
    public void Approximation(bool B) => myApprox = B;

    /// <summary>Attaches to the section edges their p-curves on the faces of the first argument.</summary>
    public void ComputePCurveOn1(bool B) => myComputePCurve1 = B;

    /// <summary>Attaches to the section edges their p-curves on the faces of the second argument.</summary>
    public void ComputePCurveOn2(bool B) => myComputePCurve2 = B;

    /// <summary>Gives the section options to the intersection tool.</summary>
    protected override void SetAttributes()
    {
        BOPAlgo_SectionAttribute theSecAttr = new(myApprox, myComputePCurve1, myComputePCurve2);
        myDSFiller!.SetSectionAttribute(theSecAttr);
    }

    /// <summary>Performs the section.</summary>
    public override void Build(ProgressRange theRange = default) => base.Build(theRange);

    /// <summary>
    /// The face of the first argument on which the section edge <paramref name="E"/> lies, and true; false (and a null
    /// shape) if there is none. OCCT leaves the output untouched when it returns false. The face is published (frozen).
    /// </summary>
    public bool HasAncestorFaceOn1(in Shape E, out Shape F)
    {
        bool bRes;
        F = new Shape();
        bRes = HasAncestorFace(myDSFiller, 1, E, ref F);
        F.TShape()?.FreezeGraph();
        return bRes;
    }

    /// <summary>The face of the second argument on which the section edge <paramref name="E"/> lies (see the first form).</summary>
    public bool HasAncestorFaceOn2(in Shape E, out Shape F)
    {
        bool bRes;
        F = new Shape();
        bRes = HasAncestorFace(myDSFiller, 2, E, ref F);
        F.TShape()?.FreezeGraph();
        return bRes;
    }

    private static bool HasAncestorFace(BOPAlgo_PaveFiller? pPF, int aIndex, in Shape aE, ref Shape aF)
    {
        bool bRes;
        //
        bRes = false;
        if (aE.IsNull)
        {
            return bRes;
        }
        if (aE.ShapeType() != ShapeType.Edge)
        {
            return bRes;
        }
        //
        Shape aF1 = new(), aF2 = new();
        //
        bRes = HasAncestorFaces(pPF!, aE, ref aF1, ref aF2);
        if (!bRes)
        {
            return bRes;
        }
        //
        aF = (aIndex == 1) ? aF1 : aF2;
        return bRes;
    }

    private static bool HasAncestorFaces(BOPAlgo_PaveFiller pPF, in Shape aEx, ref Shape aF1, ref Shape aF2)
    {
        int aNbFF, i, j, nE, aNbVC;
        //
        BOPDS_DS pDS = pPF.PDS()!;
        NCollection_DynamicArray<BOPDS_InterfFF> aFFs = pDS.InterfFF();
        //
        aNbFF = aFFs.Length();
        for (i = 0; i < aNbFF; ++i)
        {
            BOPDS_InterfFF aFFi = aFFs[i];
            aFFi.Indices(out int nF1, out int nF2);
            //
            NCollection_DynamicArray<BOPDS_Curve> aVC = aFFi.Curves();
            aNbVC = aVC.Length();
            for (j = 0; j < aNbVC; j++)
            {
                BOPDS_Curve aBC = aVC[j];
                //
                for (NCollection_List<BOPDS_PaveBlock>.Iterator aItLPB = new(aBC.PaveBlocks()); aItLPB.More(); aItLPB.Next())
                {
                    BOPDS_PaveBlock aPB = aItLPB.Value();
                    nE = aPB.Edge();
                    if (nE < 0)
                    {
                        continue;
                    }
                    //
                    Shape aE = pDS.Shape(nE);
                    if (aEx.IsSame(aE))
                    {
                        aF1 = pDS.Shape(nF1);
                        aF2 = pDS.Shape(nF2);
                        return true;
                    }
                }
            }
        }
        return false;
    }

    // A face (C2 or smoother surfaces) or a shell on the whole surface, unfrozen (made here, as in OCCT).
    private static Shape MakeShape(Surface S)
    {
        Continuity c = S.Continuity();
        if (c >= C2)
        {
            return new BRepBuilderAPI_MakeFace(S, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalShape();
        }
        return new BRepBuilderAPI_MakeShell(S).InternalShape();
    }
}
