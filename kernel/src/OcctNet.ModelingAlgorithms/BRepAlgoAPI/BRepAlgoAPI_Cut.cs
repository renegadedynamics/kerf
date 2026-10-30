// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Cut.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Cut.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BOPAlgo_Operation;

namespace Occt;

/// <summary>The objects minus the tools.</summary>
public class BRepAlgoAPI_Cut : BRepAlgoAPI_BooleanOperation
{
    /// <summary>An empty operation: set the arguments and tools, then <see cref="BRepAlgoAPI_BooleanOperation.Build"/>.</summary>
    public BRepAlgoAPI_Cut()
    {
        myOperation = BOPAlgo_CUT;
    }

    /// <summary>An operation using the intersections already computed by <paramref name="aPF"/>.</summary>
    public BRepAlgoAPI_Cut(BOPAlgo_PaveFiller aPF)
        : base(aPF)
    {
        myOperation = BOPAlgo_CUT;
    }

    /// <summary>S1 minus S2 (built at once).</summary>
    public BRepAlgoAPI_Cut(in TopoDS_Shape S1, in TopoDS_Shape S2, Message_ProgressRange theRange = default)
        : base(S1, S2, BOPAlgo_CUT)
    {
        Build(theRange);
    }

    /// <summary>
    /// S1 minus S2, with the intersections already computed by <paramref name="aDSF"/> (built at once); with
    /// <paramref name="bFWD"/> false, <paramref name="S2"/> minus <paramref name="S1"/> (CUT21).
    /// </summary>
    public BRepAlgoAPI_Cut(in TopoDS_Shape S1, in TopoDS_Shape S2, BOPAlgo_PaveFiller aDSF, bool bFWD = true,
                           Message_ProgressRange theRange = default)
        : base(S1, S2, aDSF, bFWD ? BOPAlgo_CUT : BOPAlgo_CUT21)
    {
        Build(theRange);
    }
}
