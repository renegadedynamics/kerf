// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Offsets;

/// <summary>
/// Whether an offset of a solid has the size its input allows (decision 1918). An offset can give a shape every check calls
/// valid and that's nonsense: a cone offset inwards by 1, with its apex, had a volume of 3.7e101 and reported success.
/// </summary>
/// <remarks>
/// Moving every face inwards can only remove material, and outwards can only add it, so the result's volume is on one side of
/// the input's. Its box lies inside the input's box grown by the distance, by a factor that allows for a loose box and for the
/// point of a mitre: an apex of half-angle 0.1 degrees moves 573 times the distance. There's no rule for what isn't a solid.
/// </remarks>
internal static class OffsetVolumes
{
    // As BooleanVolumes measures, for the same reason (decision 1802)
    private static readonly MassOptions Measured = new() { Tolerance = 1e-6 };

    // How far past a rule, for the input's volume, a result may be: integration's error, as decision 1861 measured it for Booleans
    private const double MarginOfVolume = 1e-4;

    // A box of curved faces can be larger than they are, by this much of its diagonal
    private const double LooseBox = 0.25;

    // Round joins stay within the distance, 
    private const double GrowthOfArc = 3;

    // Sharp joins go as far as their mitre
    private const double GrowthOfMitre = 1000;

    /// <summary>The error if <paramref name="result"/> can't be an offset of <paramref name="input"/>, or null.</summary>
    /// <param name="input">The shape.</param>
    /// <param name="distance">How far: outwards when positive.</param>
    /// <param name="join">The join, which says how far a corner can go.</param>
    /// <param name="result">What came of it.</param>
    /// <param name="what">What to call it.</param>
    /// <param name="isWall">A hollow solid: it holds less than a whole offset, so outwards there is no lower limit; inwards it must hold less than its input.</param>
    public static Diagnostic? Check(Shape input, double distance, JoinKind join, Shape result, string what, bool isWall = false)
    {
        if (input.Type != ShapeType.Solid && input.AllSolids().Length == 0)
        {
            return null;
        }

        if (result.AllSolids().Length == 0)
        {
            return Wrong(what, FormattableString.Invariant($"from a solid it made a {result.Type}, with no solid in it"));
        }

        BoundingBox inputBox = input.BoundingBox(), resultBox = result.BoundingBox();
        if (inputBox.IsEmpty || resultBox.IsEmpty)
        {
            return null;
        }

        double before = input.IntegrateVolume(Measured), after = result.IntegrateVolume(Measured);
        if (!double.IsFinite(before) || before <= 0)
        {
            return null;
        }

        double margin = MarginOfVolume * before;
        if (!double.IsFinite(after) || after < -margin)
        {
            return Wrong(what, FormattableString.Invariant($"its volume is {after:G4}"));
        }

        if (distance < 0 && after > before + margin)
        {
            return Wrong(what, FormattableString.Invariant($"moved inwards it can't hold more, and its volume is {after:G6} from {before:G6}"));
        }

        // A wall made inwards is cut from the input, so something is taken out (decision 1939)
        if (distance < 0 && isWall && after > before - margin)
        {
            return Wrong(what, FormattableString.Invariant($"hollowed inwards it must hold less, and its volume is {after:G6} from {before:G6}"));
        }

        if (distance > 0 && !isWall && after < before - margin)
        {
            return Wrong(what, FormattableString.Invariant($"moved outwards it can't hold less, and its volume is {after:G6} from {before:G6}"));
        }

        double reach = LooseBox * inputBox.Min.DistanceTo(inputBox.Max)
               + Math.Abs(distance) * (distance < 0 ? 1 : join == JoinKind.Arc ? GrowthOfArc : GrowthOfMitre);
        if (resultBox.Min.X < inputBox.Min.X - reach || resultBox.Min.Y < inputBox.Min.Y - reach || resultBox.Min.Z < inputBox.Min.Z - reach
            || resultBox.Max.X > inputBox.Max.X + reach || resultBox.Max.Y > inputBox.Max.Y + reach || resultBox.Max.Z > inputBox.Max.Z + reach)
        {
            return Wrong(what, string.Create(CultureInfo.InvariantCulture, $"it reaches {reach:G4} past its input's box and more"));
        }

        return null;
    }

    private static Diagnostic Wrong(string what, string why) => Diagnostic.Error(DiagnosticCode.InvalidResult, $"{what} is wrong: {why}.");
}
