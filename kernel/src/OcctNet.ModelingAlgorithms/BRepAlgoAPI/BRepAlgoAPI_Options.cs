// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// The options of a Boolean operation for the functional twins of <see cref="BRepAlgoAPI"/> (decision 579): OCCT's
/// options of BRepAlgoAPI_BuilderAlgo under OCCT's names, with OCCT's defaults.
/// </summary>
/// <remarks>
/// <para>
/// An OCCT.NET addition (OCCT sets the options on the algorithm object), as <see cref="IMeshTools_Parameters"/>
/// (decision 528): a struct with public fields, written
/// <c>new BRepAlgoAPI_Options { FuzzyValue = 1e-5, RunParallel = true }</c>. It is a value, equatable field by field
/// (doubles as <c>double.Equals</c> does, so NaN equals itself), so <c>(arguments, tools, options)</c> can key a
/// cache of results. No option is added or reinterpreted (decision 582).
/// </para>
/// <para>
/// <c>new BRepAlgoAPI_Options()</c> has OCCT's defaults. <c>default(BRepAlgoAPI_Options)</c> doesn't: its fuzzy value is
/// 0 (raised to <see cref="Precision.Confusion"/> by the algorithm, as OCCT's <c>SetFuzzyValue</c> does) and
/// <see cref="CheckInverted"/> is false.
/// </para>
/// </remarks>
public struct BRepAlgoAPI_Options : IEquatable<BRepAlgoAPI_Options>
{
    /// <summary>OCCT's defaults.</summary>
    public BRepAlgoAPI_Options()
    {
        FuzzyValue = Precision.Confusion();
        Glue = BOPAlgo_GlueEnum.BOPAlgo_GlueOff;
        UseOBB = false;
        CheckInverted = true;
        RunParallel = false;
        NonDestructive = false;
        Simplify = false;
    }

    /// <summary>
    /// The additional tolerance of the operation (<c>SetFuzzyValue</c>; at least <see cref="Precision.Confusion"/>, the
    /// default).
    /// </summary>
    public double FuzzyValue;

    /// <summary>The glue option (<c>SetGlue</c>; off by default).</summary>
    public BOPAlgo_GlueEnum Glue;

    /// <summary>Filters the intersections with oriented bounding boxes (<c>SetUseOBB</c>; false by default).</summary>
    public bool UseOBB;

    /// <summary>Checks the input solids for being inverted (<c>SetCheckInverted</c>; true by default).</summary>
    public bool CheckInverted;

    /// <summary>
    /// Runs the operation's loops in parallel (<c>SetRunParallel</c>). False by default: OCCT's default is the global
    /// <see cref="BOPAlgo_Options.GetParallelMode"/>, which is false, and a value mustn't depend on process state. The
    /// result is the same either way (decision 580).
    /// </summary>
    public bool RunParallel;

    /// <summary>
    /// The non-destructive mode (<c>SetNonDestructive</c>; false by default). The twins freeze their arguments, so the
    /// operation is non-destructive whatever this says (decision 577); it is kept for OCCT's other effects.
    /// </summary>
    public bool NonDestructive;

    /// <summary>
    /// Simplifies the result with ShapeUpgrade_UnifySameDomain (<c>SimplifyResult()</c> with its defaults: edges and
    /// faces, <see cref="Precision.Angular"/>; the fuzzy value as the linear tolerance; false by default).
    /// </summary>
    public bool Simplify;

    public readonly bool Equals(BRepAlgoAPI_Options theOther) =>
        FuzzyValue.Equals(theOther.FuzzyValue) && Glue == theOther.Glue && UseOBB == theOther.UseOBB
        && CheckInverted == theOther.CheckInverted && RunParallel == theOther.RunParallel
        && NonDestructive == theOther.NonDestructive && Simplify == theOther.Simplify;

    public override readonly bool Equals(object? theObj) => theObj is BRepAlgoAPI_Options anOther && Equals(anOther);

    public override readonly int GetHashCode() =>
        HashCode.Combine(FuzzyValue, Glue, UseOBB, CheckInverted, RunParallel, NonDestructive, Simplify);

    public static bool operator ==(BRepAlgoAPI_Options theLeft, BRepAlgoAPI_Options theRight) => theLeft.Equals(theRight);

    public static bool operator !=(BRepAlgoAPI_Options theLeft, BRepAlgoAPI_Options theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        string.Create(CultureInfo.InvariantCulture,
                      $"BRepAlgoAPI_Options(FuzzyValue={FuzzyValue:R}, Glue={Glue}, UseOBB={UseOBB}, CheckInverted={CheckInverted}, RunParallel={RunParallel}, NonDestructive={NonDestructive}, Simplify={Simplify})");
}
