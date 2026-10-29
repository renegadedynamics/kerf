// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_NormalStatus.hxx


namespace Occt;

/// <summary>The result of computing a surface normal at a possibly singular point.</summary>
/// <remarks>An unscoped enum in C++; ported code imports it with <c>using static Occt.CSLib_NormalStatus;</c>.</remarks>
public enum CSLib_NormalStatus
{
    /// <summary>Surface is singular at the point (normal undefined).</summary>
    CSLib_Singular,
    /// <summary>Normal is well-defined and computed successfully.</summary>
    CSLib_Defined,
    /// <summary>Infinite normals possible.</summary>
    CSLib_InfinityOfSolutions,
    /// <summary>DN/du has null length: ||DN/du|| &lt;= Resolution.</summary>
    CSLib_D1NuIsNull,
    /// <summary>DN/dv has null length: ||DN/dv|| &lt;= Resolution.</summary>
    CSLib_D1NvIsNull,
    /// <summary>Both normal derivatives are null.</summary>
    CSLib_D1NIsNull,
    /// <summary>DN/du is negligible compared to DN/dv.</summary>
    CSLib_D1NuNvRatioIsNull,
    /// <summary>DN/dv is negligible compared to DN/du.</summary>
    CSLib_D1NvNuRatioIsNull,
    /// <summary>DN/du and DN/dv are parallel.</summary>
    CSLib_D1NuIsParallelD1Nv,
}
