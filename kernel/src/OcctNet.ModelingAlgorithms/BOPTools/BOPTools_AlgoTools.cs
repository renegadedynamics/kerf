// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools.cxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_AlgoTools_2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>The topological tools of the Boolean component.</summary>
/// <remarks>
/// Unit 6A ports the three statics BOPDS_DS calls (<see cref="PointOnEdge"/>, <see cref="IsOpenShell"/>,
/// <see cref="IsInvertedSolid"/>); unit 6B ports the rest of the class into this file (decision 617).
/// </remarks>
public static partial class BOPTools_AlgoTools
{
    /// <summary>The point of the edge's 3D curve at the parameter.</summary>
    public static void PointOnEdge(in TopoDS_Edge aE, double aParm, out gp_Pnt aPnt)
    {
        Geom_Curve C1 = BRep_Tool.Curve(aE, out _, out _)!;
        C1.D0(aParm, out aPnt);
    }

    /// <summary>True if the shell has a free edge: a non-degenerated edge in only one forward or reversed face.</summary>
    public static bool IsOpenShell(in TopoDS_Shell aSh)
    {
        bool bRet;
        int i, aNbE, aNbF;
        TopAbs_Orientation aOrF;
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        //
        bRet = false;
        //
        TopExp.MapShapesAndAncestors(aSh, TopAbs_EDGE, TopAbs_FACE, aMEF);
        //
        aNbE = aMEF.Extent();
        for (i = 1; i <= aNbE; ++i)
        {
            TopoDS_Edge aE = TopoDS.Edge(aMEF.FindKey(i));
            if (BRep_Tool.Degenerated(aE))
            {
                continue;
            }
            //
            aNbF = 0;
            NCollection_List<TopoDS_Shape> aLF = aMEF.FindFromIndex(i);
            for (NCollection_List<TopoDS_Shape>.Iterator aItLS = new(aLF); aItLS.More(); aItLS.Next())
            {
                TopoDS_Shape aF = aItLS.Value();
                aOrF = aF.Orientation();
                if (aOrF == TopAbs_INTERNAL || aOrF == TopAbs_EXTERNAL)
                {
                    continue;
                }
                ++aNbF;
            }
            //
            if (aNbF == 1)
            {
                bRet = !bRet; // True
                break;
            }
        }
        //
        return bRet;
    }

    /// <summary>True if the solid is inverted: the infinite point classifies inside it.</summary>
    public static bool IsInvertedSolid(in TopoDS_Solid aSolid)
    {
        double aTolS;
        TopAbs_State aState;
        BRepClass3d_SolidClassifier aSC = new(aSolid);
        //
        aTolS = 1.0e-7;
        aSC.PerformInfinitePoint(aTolS);
        aState = aSC.State();
        return aState == TopAbs_IN;
    }
}
