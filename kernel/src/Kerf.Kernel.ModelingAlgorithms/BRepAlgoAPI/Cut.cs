// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Cut.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Cut.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Operation;

namespace Kerf.Engine.Booleans.Api;

/// <summary>The objects minus the tools.</summary>
public class Cut : BooleanOperation
{
    /// <summary>An empty operation: set the arguments and tools, then <see cref="BooleanOperation.Build"/>.</summary>
    public Cut()
    {
        _operation = global::Kerf.Engine.Booleans.Operation.Cut;
    }

    /// <summary>An operation using the intersections already computed by <paramref name="aPF"/>.</summary>
    public Cut(PaveFiller aPF)
        : base(aPF)
    {
        _operation = global::Kerf.Engine.Booleans.Operation.Cut;
    }

    /// <summary>S1 minus S2 (built at once).</summary>
    public Cut(in Shape s1, in Shape s2, ProgressRange range = default)
        : base(s1, s2, global::Kerf.Engine.Booleans.Operation.Cut)
    {
        Build(range);
    }

    /// <summary>
    /// S1 minus S2, with the intersections already computed by <paramref name="aDSF"/> (built at once); with
    /// <paramref name="bFWD"/> false, <paramref name="s2"/> minus <paramref name="s1"/> (CUT21).
    /// </summary>
    public Cut(in Shape s1, in Shape s2, PaveFiller aDSF, bool bFWD = true,
                           ProgressRange range = default)
        : base(s1, s2, aDSF, bFWD ? global::Kerf.Engine.Booleans.Operation.Cut : global::Kerf.Engine.Booleans.Operation.CUT21)
    {
        Build(range);
    }
}
