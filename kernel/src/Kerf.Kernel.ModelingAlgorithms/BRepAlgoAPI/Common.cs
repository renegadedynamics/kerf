// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Common.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Common.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Booleans.Operation;

namespace Kerf.Engine.Booleans.Api;

/// <summary>The common part of the objects and the tools.</summary>
internal class Common : BooleanOperation
{
    /// <summary>An empty operation: set the arguments and tools, then <see cref="BooleanOperation.Build"/>.</summary>
    public Common()
    {
        _operation = global::Kerf.Engine.Booleans.Operation.Common;
    }

    /// <summary>An operation using the intersections already computed by <paramref name="aPF"/>.</summary>
    public Common(PaveFiller aPF)
        : base(aPF)
    {
        _operation = global::Kerf.Engine.Booleans.Operation.Common;
    }

    /// <summary>The common part of S1 and S2 (built at once).</summary>
    public Common(in Shape s1, in Shape s2, ProgressRange range = default)
        : base(s1, s2, global::Kerf.Engine.Booleans.Operation.Common)
    {
        Build(range);
    }

    /// <summary>The common part of S1 and S2, with the intersections already computed by <paramref name="aDSF"/> (built at once).</summary>
    public Common(in Shape s1, in Shape s2, PaveFiller aDSF, ProgressRange range = default)
        : base(s1, s2, aDSF, global::Kerf.Engine.Booleans.Operation.Common)
    {
        Build(range);
    }
}
