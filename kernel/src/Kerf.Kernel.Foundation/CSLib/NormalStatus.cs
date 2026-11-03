// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_NormalStatus.hxx


using Occt;

namespace Kerf.Engine.Geometry;

/// <summary>The result of computing a surface normal at a possibly singular point.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.CSLib_NormalStatus;</c>.</remarks>
public enum NormalStatus
{
    /// <summary>Surface is singular at the point (normal undefined).</summary>
    Singular,
    /// <summary>Normal is well-defined and computed successfully.</summary>
    Defined,
    /// <summary>Infinite normals possible.</summary>
    InfinityOfSolutions,
    /// <summary>DN/du has null length: ||DN/du|| &lt;= Resolution.</summary>
    D1NuIsNull,
    /// <summary>DN/dv has null length: ||DN/dv|| &lt;= Resolution.</summary>
    D1NvIsNull,
    /// <summary>Both normal derivatives are null.</summary>
    D1NIsNull,
    /// <summary>DN/du is negligible compared to DN/dv.</summary>
    D1NuNvRatioIsNull,
    /// <summary>DN/dv is negligible compared to DN/du.</summary>
    D1NvNuRatioIsNull,
    /// <summary>DN/du and DN/dv are parallel.</summary>
    D1NuIsParallelD1Nv,
}
