// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Algo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BRepAlgoAPI/BRepAlgoAPI_Algo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Topology;

namespace Occt;

/// <summary>The root of the Boolean API algorithms.</summary>
/// <remarks>
/// <para>
/// OCCT derives from BRepBuilderAPI_MakeShape and, protected, from BOPAlgo_Options, and re-exposes the options with
/// <c>using</c>. C# has one base class, so the algorithm holds a <see cref="BOPAlgo_Options"/> (decisions 627,
/// 6182) and forwards OCCT's re-exposed members to it; the derived classes reach the fields through
/// <see cref="myOptions"/>.
/// </para>
/// <para>
/// The result is published by <see cref="BRepBuilderAPI_MakeShape.Shape"/>, which freezes it (decision 454). OCCT's
/// override of <c>Shape()</c>, which doesn't check <c>IsDone</c>, is <see cref="InternalShape"/>.
/// </para>
/// </remarks>
public abstract class BRepAlgoAPI_Algo : BRepBuilderAPI_MakeShape
{
    /// <summary>OCCT's protected base class BOPAlgo_Options, held.</summary>
    private protected readonly BOPAlgo_Options myOptions = new();

    protected BRepAlgoAPI_Algo()
    {
    }

    /// <summary>The result, not checked for <c>IsDone</c> and not frozen (OCCT's <c>Shape()</c>).</summary>
    internal override Shape InternalShape() => myShape;

    // ---- BOPAlgo_Options' members, which OCCT re-exposes with "using"

    /// <summary>Clears the warnings and errors and the data of the previous run; the options stay.</summary>
    public virtual void Clear() => myOptions.Clear();

    /// <summary>Clears the warnings.</summary>
    public void ClearWarnings() => myOptions.ClearWarnings();

    /// <summary>Writes the errors' message keys, one per line.</summary>
    public void DumpErrors(TextWriter theOS) => myOptions.DumpErrors(theOS);

    /// <summary>Writes the warnings' message keys, one per line.</summary>
    public void DumpWarnings(TextWriter theOS) => myOptions.DumpWarnings(theOS);

    /// <summary>The additional tolerance of the operation.</summary>
    public double FuzzyValue() => myOptions.FuzzyValue();

    /// <summary>Sets the additional tolerance (at least <see cref="Precision.Confusion"/>).</summary>
    public void SetFuzzyValue(double theFuzz) => myOptions.SetFuzzyValue(theFuzz);

    /// <summary>The report of warnings and errors.</summary>
    public Report GetReport() => myOptions.GetReport();

    /// <summary>True if the algorithm failed with an error of exactly this type.</summary>
    public bool HasError(Type theType) => myOptions.HasError(theType);

    /// <summary>True if the algorithm failed.</summary>
    public bool HasErrors() => myOptions.HasErrors();

    /// <summary>True if the algorithm gave a warning of exactly this type.</summary>
    public bool HasWarning(Type theType) => myOptions.HasWarning(theType);

    /// <summary>True if the algorithm gave warnings.</summary>
    public bool HasWarnings() => myOptions.HasWarnings();

    /// <summary>The parallel mode of this algorithm.</summary>
    public bool RunParallel() => myOptions.RunParallel();

    /// <summary>Switches the parallel mode of this algorithm on or off.</summary>
    public void SetRunParallel(bool theFlag) => myOptions.SetRunParallel(theFlag);

    /// <summary>Enables or disables the oriented bounding boxes to filter the intersections.</summary>
    public void SetUseOBB(bool theUseOBB) => myOptions.SetUseOBB(theUseOBB);

    /// <summary>True if the oriented bounding boxes filter the intersections.</summary>
    public bool UseOBB() => myOptions.UseOBB();

    /// <summary>Adds an error (BOPAlgo_Options' <c>AddError</c>, for the derived algorithms).</summary>
    private protected void AddError(Alert theAlert) => myOptions.AddError(theAlert);

    /// <summary>Adds a warning.</summary>
    private protected void AddWarning(Alert theAlert) => myOptions.AddWarning(theAlert);
}
