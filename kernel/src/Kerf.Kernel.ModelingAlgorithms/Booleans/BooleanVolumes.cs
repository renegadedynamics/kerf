// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using Kerf.Engine.Healing.Analysis;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Booleans;

/// <summary>The Booleans whose result's volume can be checked against their inputs' (<see cref="BooleanVolumes.Check"/>).</summary>
public enum BooleanVolumeRule
{
    /// <summary>The result is no larger than the smaller input.</summary>
    Common,

    /// <summary>The result is no larger than the object, and no smaller than the object less the tools.</summary>
    Cut,

    /// <summary>The result is no smaller than the largest input, and no larger than the inputs together.</summary>
    Fuse,
}

/// <summary>
/// Whether a Boolean's result has a volume its inputs allow (decisions 21301 to 21306). A silent wrong Boolean can give a valid
/// shape, which the validity check can't catch; breaking one of these rules proves the result wrong.
/// </summary>
/// <remarks>
/// The rules are for solids. If anything in the inputs or the result isn't a solid, or a compound of them, or an input's
/// volume isn't positive, there's no rule and the result passes: so faces, shells and half-spaces. A compound's solids may
/// overlap, so its sum is only the most it can hold, and its largest solid the least. Before a rule is said to be broken, the
/// inputs must be valid and bounded, as a volume means nothing otherwise, and the margin grows with their tolerances.
/// </remarks>
public static class BooleanVolumes
{
    // As Kerf.Modeling measures: the default integration can be 3.7% off on swept and B-spline faces (decision 1802), which is
    // more than most of what a rule could find, and on #1659 and #1643 it reported a Cut larger than its object where there was none
    private static readonly MassOptions Measured = new() { Tolerance = 1e-6 };

    // How far past a rule, for the size of the inputs, a result may be (decision 21302). No good result in the corpus came within
    // a third of it, once the tolerances are allowed for
    private const double MarginOfSize = 1e-4;

    // A tolerance makes a shell of its width over a shape's surface, about 6 tolerances over the shape's size of its volume;
    // the corpus's worst good result is 0.67 of a fuzzy value on that measure
    private const double MarginOfTolerance = 10;

    // Volume is a cube, so anything this small for the shape's box is integration's noise (a half-space's, or two shells' that cancel)
    private const double SmallestOfBox = 1e-9;

    /// <summary>
    /// The error if <paramref name="result"/> can't be what <paramref name="rule"/> makes of <paramref name="arguments"/> and
    /// <paramref name="tools"/>: <c>result.invalid</c>, naming the rule and the figures. Null if it may be right, or if no rule
    /// applies to these shapes.
    /// </summary>
    /// <param name="rule">The operation.</param>
    /// <param name="arguments">Its objects.</param>
    /// <param name="tools">Its tools.</param>
    /// <param name="result">What it made.</param>
    /// <param name="fuzzyValue">Its fuzzy value, which widens the margin.</param>
    public static Diagnostic? Check(BooleanVolumeRule rule, IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, Shape result, double fuzzyValue = 0)
    {
        ArgumentNullException.ThrowIfNull(arguments);
        ArgumentNullException.ThrowIfNull(tools);
        // Look at what the shapes are before measuring anything: a Boolean of faces costs nothing here
        if (!AllSolids(arguments) || !AllSolids(tools) || !AllSolids([result]) || !HasSolid(arguments) || !HasSolid(tools))
        {
            return null;
        }
        var outcome = new Measure();
        outcome.Add(result);
        // A common that made nothing breaks no rule, so nothing need be measured
        if (rule == BooleanVolumeRule.Common && outcome.Count == 0)
        {
            return null;
        }
        var a = new Measure();
        foreach (Shape shape in arguments)
        {
            a.Add(shape);
        }
        var b = new Measure();
        foreach (Shape shape in tools)
        {
            b.Add(shape);
        }
        if (a.Smallest <= 0 || b.Smallest <= 0 || !double.IsFinite(a.Sum) || !double.IsFinite(b.Sum) || !double.IsFinite(outcome.Sum))
        {
            return null;
        }

        double size = Math.Max(a.Sum, b.Sum);
        (double least, double most) = Bounds(rule, a, b);
        double r = outcome.Sum;
        double margin = size * MarginOfSize;
        bool negative = outcome.Count > 0 && outcome.Smallest < -margin;
        if (r >= least - margin && r <= most + margin && !negative)
        {
            return null;
        }

        // Past a rule, which is rare, so this is when the dearer questions are worth asking: is it proved?
        margin += size * MarginOfTolerance * (LargestTolerance(arguments, tools, result) + fuzzyValue) / Math.Cbrt(size);
        negative = outcome.Count > 0 && outcome.Smallest < -margin;
        if ((r >= least - margin && r <= most + margin && !negative) || !Trustworthy(arguments) || !Trustworthy(tools))
        {
            return null;
        }
        return Diagnostic.Error(DiagnosticCode.InvalidResult, Message(rule, a, b, outcome, negative, least, most));
    }

    private static (double Least, double Most) Bounds(BooleanVolumeRule rule, in Measure a, in Measure b) => rule switch
    {
        BooleanVolumeRule.Common => (0, Math.Min(a.Sum, b.Sum)),
        // A tool's overlaps only make its sum larger, so this bound stays true; the object's largest solid is what's sure of it
        BooleanVolumeRule.Cut => (a.Largest - b.Sum, a.Sum),
        BooleanVolumeRule.Fuse => (Math.Max(a.Largest, b.Largest), a.Sum + b.Sum),
        _ => throw new ArgumentOutOfRangeException(nameof(rule)),
    };

    private static string Message(BooleanVolumeRule rule, in Measure a, in Measure b, in Measure result, bool negative, double least, double most)
    {
        string name = rule switch
        {
            BooleanVolumeRule.Common => "common",
            BooleanVolumeRule.Cut => "cut",
            _ => "fuse",
        };
        string figures = $"the object's volume is {Figure(a.Sum)}, the tool's {Figure(b.Sum)} and the result's {Figure(result.Sum)}";
        if (negative)
        {
            return $"The {name}'s result is wrong: one of its solids is inside out, with a volume of {Figure(result.Smallest)}; {figures}.";
        }
        string rule1 = rule switch
        {
            BooleanVolumeRule.Common => "a common can't be larger than the smaller input",
            BooleanVolumeRule.Cut when result.Sum > most => "a cut can't be larger than its object",
            BooleanVolumeRule.Cut => "a cut can't remove more than its tool",
            BooleanVolumeRule.Fuse when result.Sum < least => "a fuse can't be smaller than its largest input",
            _ => "a fuse can't be larger than its inputs together",
        };
        return $"The {name}'s result is wrong: {rule1}; {figures}.";
    }

    private static string Figure(double volume) => volume.ToString("G8", CultureInfo.InvariantCulture);

    private static bool AllSolids(IReadOnlyList<Shape> shapes)
    {
        foreach (Shape shape in shapes)
        {
            if (shape.IsNull || !AllSolids(shape))
            {
                return false;
            }
        }
        return true;
    }

    private static bool AllSolids(Shape shape)
    {
        switch (shape.Type)
        {
            case ShapeType.Solid:
                return true;
            case ShapeType.Compound or ShapeType.CompSolid:
                foreach (Shape child in shape.Children)
                {
                    if (!AllSolids(child))
                    {
                        return false;
                    }
                }
                return true;
            default:
                return false;
        }
    }

    private static bool HasSolid(IReadOnlyList<Shape> shapes)
    {
        foreach (Shape shape in shapes)
        {
            if (shape.Explore<Solid>().Any())
            {
                return true;
            }
        }
        return false;
    }

    private static double LargestTolerance(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, Shape result)
    {
        var analysis = new ShapeToleranceAnalysis();
        double largest = analysis.Tolerance(result, 1);
        foreach (Shape shape in arguments.Concat(tools))
        {
            largest = Math.Max(largest, analysis.Tolerance(shape, 1));
        }
        return largest;
    }

    // An input's volume means something: it's valid, it has a bounded box, and its volume isn't lost in that box
    private static bool Trustworthy(IReadOnlyList<Shape> shapes)
    {
        foreach (Shape shape in shapes)
        {
            if (!shape.Validate().IsValid)
            {
                return false;
            }
            foreach (Solid solid in shape.Explore<Solid>())
            {
                BoundingBox box = solid.BoundingBox();
                Vector3D size = box.Max - box.Min;
                double room = Math.Abs(size.X * size.Y * size.Z);
                if (!double.IsFinite(room) || room >= 1e60 || solid.IntegrateVolume(Measured) < room * SmallestOfBox)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // What a side's solids hold
    private struct Measure
    {
        public int Count;
        public double Sum;
        public double Largest;
        public double Smallest;

        public void Add(Shape shape)
        {
            if (shape.Type == ShapeType.Solid)
            {
                double volume = shape.IntegrateVolume(Measured);
                Smallest = Count == 0 ? volume : Math.Min(Smallest, volume);
                Largest = Count == 0 ? volume : Math.Max(Largest, volume);
                Sum += volume;
                Count++;
                return;
            }
            foreach (Shape child in shape.Children)
            {
                Add(child);
            }
        }
    }
}
