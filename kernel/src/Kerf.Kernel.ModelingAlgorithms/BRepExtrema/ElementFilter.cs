// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepExtrema/BRepExtrema_ElementFilter.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2015-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Kerf.Engine.Measurement.Distance;

/// <summary>
/// Decides whether two mesh elements should be tested for overlapping or intersection (used to tell correct shared
/// edges and vertices from incorrect ones). The default checks every pair.
/// </summary>
internal class ElementFilter
{
    /// <summary>The result of filtering.</summary>
    public enum FilterResult
    {
        NoCheck,
        Overlap,
        DoCheck,
    }

    /// <summary>Whether the two mesh elements should be tested for overlapping or intersection.</summary>
    public virtual FilterResult PreCheckElements(int index1, int index2) => FilterResult.DoCheck;
}
