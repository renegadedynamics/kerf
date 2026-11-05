// Port of: src/FoundationClasses/TKMath/CSLib/CSLib_DerivativeStatus.hxx


using Kerf.Engine;

namespace Kerf.Engine.Geometry;

/// <summary>The result of computing a surface normal from the first derivatives D1U and D1V.</summary>
internal enum DerivativeStatus
{
    /// <summary>Normal computed successfully.</summary>
    Done,
    /// <summary>D1U has null length: ||D1U|| &lt;= Resolution.</summary>
    D1uIsNull,
    /// <summary>D1V has null length: ||D1V|| &lt;= Resolution.</summary>
    D1vIsNull,
    /// <summary>Both derivatives are null.</summary>
    D1IsNull,
    /// <summary>D1U is negligible: ||D1U|| / ||D1V|| &lt;= RealEpsilon.</summary>
    D1uD1vRatioIsNull,
    /// <summary>D1V is negligible: ||D1V|| / ||D1U|| &lt;= RealEpsilon.</summary>
    D1vD1uRatioIsNull,
    /// <summary>D1U and D1V are parallel.</summary>
    D1uIsParallelD1v,
}
