// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Fuse.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Fuse.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using static Occt.BOPAlgo_Operation;

namespace Occt;

/// <summary>The union of the objects and the tools.</summary>
public class BRepAlgoAPI_Fuse : BRepAlgoAPI_BooleanOperation
{
    /// <summary>An empty operation: set the arguments and tools, then <see cref="BRepAlgoAPI_BooleanOperation.Build"/>.</summary>
    public BRepAlgoAPI_Fuse()
    {
        myOperation = BOPAlgo_FUSE;
    }

    /// <summary>An operation using the intersections already computed by <paramref name="aPF"/>.</summary>
    public BRepAlgoAPI_Fuse(BOPAlgo_PaveFiller aPF)
        : base(aPF)
    {
        myOperation = BOPAlgo_FUSE;
    }

    /// <summary>The union of S1 and S2 (built at once).</summary>
    public BRepAlgoAPI_Fuse(in TopoDS_Shape S1, in TopoDS_Shape S2, ProgressRange theRange = default)
        : base(S1, S2, BOPAlgo_FUSE)
    {
        Build(theRange);
    }

    /// <summary>The union of S1 and S2, with the intersections already computed by <paramref name="aDSF"/> (built at once).</summary>
    public BRepAlgoAPI_Fuse(in TopoDS_Shape S1, in TopoDS_Shape S2, BOPAlgo_PaveFiller aDSF, ProgressRange theRange = default)
        : base(S1, S2, aDSF, BOPAlgo_FUSE)
    {
        Build(theRange);
    }
}
