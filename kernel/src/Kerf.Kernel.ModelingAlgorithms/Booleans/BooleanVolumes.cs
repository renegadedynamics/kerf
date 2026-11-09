// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.CompilerServices;
using Kerf.Engine.Classification;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Tools;
using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Booleans;

/// <summary>The Booleans whose result's volume can be checked against their inputs' (<see cref="BooleanVolumes.Check"/>).</summary>
public enum BooleanVolumeRule
{
    /// <summary>The result is no larger than the smaller input, and empty only if no points lie well inside both.</summary>
    Common,

    /// <summary>The result is no larger than the object, no smaller than the object less the tools, and has removed something
    /// if points lie well inside both.</summary>
    Cut,

    /// <summary>The result is no smaller than the largest input, and no larger than the inputs together.</summary>
    Fuse,
}

/// <summary>
/// Whether a Boolean's result has a volume its inputs allow (decisions 1860 to 1868). A silent wrong Boolean can give a valid
/// shape, which the validity check can't catch; breaking one of these rules proves the result wrong.
/// </summary>
/// <remarks>
/// The rules are for solids. If anything in the inputs or the result isn't a solid, or a compound of them, or an input's
/// volume isn't positive, there's no rule of volume: so faces, shells and half-spaces. Faces or shells against solids are tried at points instead. A compound's solids may
/// overlap, so its sum is only the most it can hold, and its largest solid the least. Before a rule is said to be broken, the
/// inputs must be valid and bounded, as a volume means nothing otherwise, and the margin grows with their tolerances. An empty
/// common, or a cut that removed nothing, breaks no rule of volume, so points well inside both inputs are looked for instead.
/// </remarks>
public static class BooleanVolumes
{
    // As Kerf.Modeling measures: the default integration can be 3.7% off on swept and B-spline faces (decision 1802), which is
    // more than most of what a rule could find, and on #1659 and #1643 it reported a Cut larger than its object where there was none
    private static readonly MassOptions Measured = new() { Tolerance = 1e-6 };

    // The volumes measured of frozen shapes, kept with their TShapes (unit 12Y). A chain of Booleans checks each result, and the
    // next one measures it again as its object: about half of what the check integrated on ordinary parts
    private static readonly ConditionalWeakTable<TShape, KeptVolume> Kept = new();

    // At most this many placements and orientations of one TShape: a solid moved about as a tool has one a move
    private const int KeptPerShape = 4;

    // How far past a rule, for the size of the inputs, a result may be (decision 1861). The corpus's worst good result is 6.8e-5
    // of its larger input, with tolerances of 0.6, and otherwise 6.7e-6
    private const double MarginOfSize = 1e-4;

    // A tolerance makes a shell of its width over a shape's surface, about 6 tolerances over the shape's size of its volume;
    // the corpus's worst good result is 0.67 of a fuzzy value on that measure
    private const double MarginOfTolerance = 10;

    // Volume is a cube, so anything this small for the shape's box is integration's noise (a half-space's, or two shells' that cancel)
    private const double SmallestOfBox = 1e-9;

    // The points tried in each direction, in the overlap of the inputs' boxes: 343 at most, most of them one classification
    private const int OverlapGrid = 7;

    // The points tried in each direction on a face of a result that isn't solid, and how many faces it may have for that: most of a
    // result's faces come from one or two faces of an input, and this keeps the classifications to a few thousand
    private const int FaceGrid = 9;
    private const int FacesLooked = 64;

    // A face larger than this isn't a part, but a plane or the like with no bounds (Precision::Infinite is 2e100)
    private const double UnboundedFace = 1e7;

    // How far inside both inputs a point must be, for their size, to prove they overlap
    private const double OverlapDepth = 1e-3;

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
        KernelArgumentNullException.ThrowIfNull(arguments);
        KernelArgumentNullException.ThrowIfNull(tools);
        // Look at what the shapes are before measuring anything: a Boolean of faces costs nothing here
        if (!AllSolids(arguments) || !AllSolids(tools) || !AllSolids([result]) || !HasSolid(arguments) || !HasSolid(tools))
        {
            try
            {
                return FacesAgainstSolids(rule, arguments, tools, result, fuzzyValue);
            }
            catch (Exception e) when (Refusal.IsFailure(e))
            {
                // A point that can't be judged proves nothing, and the Boolean itself worked
                return null;
            }
        }
        var outcome = new Measure();
        outcome.Add(result);
        // A common that made nothing breaks no rule of volume, so nothing need be measured: only a point in both can prove it wrong
        if (rule == BooleanVolumeRule.Common && outcome.Count == 0)
        {
            return ProvedToOverlap(rule, arguments, tools, result, fuzzyValue, 0, -1);
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
            // Within the rules, but a cut that removed nothing, or a fuse that lost nothing, can still be proved wrong
            return rule switch
            {
                BooleanVolumeRule.Cut when r >= a.Sum * (1 - MarginOfSize) => ProvedToOverlap(rule, arguments, tools, result, fuzzyValue, 0, Math.Min(a.Sum, b.Sum)),
                BooleanVolumeRule.Fuse when r <= least + margin && ProvedLost(arguments, tools, result, fuzzyValue, Math.Min(a.Sum, b.Sum)) is { } lost => lost,
                BooleanVolumeRule.Fuse when r >= most - margin => ProvedToOverlap(rule, arguments, tools, result, fuzzyValue, Math.Max(0, most - r), Math.Min(a.Sum, b.Sum)),
                _ => null,
            };
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

    // A common that made nothing, or a cut that removed nothing, is wrong if there are points well inside both inputs, and for a
    // cut still in its result (decisions 1866 to 1868). The rules of volume can't see this, and it's what #1659, #1643 and #245 do.
    // Only for one input on each side, as what several mean together isn't plain; a grid of points in the boxes' overlap can miss
    // a small overlap, and then nothing is said. Two points are asked for, as a classifier can be wrong about one.
    // A fuse is as large as its inputs together only if they barely overlap: the points found stand for a volume each, and to prove
    // it wrong must stand for over twice what it lost, as a pin in a large block can be within the margin and right (decision 1881).
    private static Diagnostic? ProvedToOverlap(BooleanVolumeRule rule, IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, Shape result, double fuzzyValue, double lost, double smallerVolume)
    {
        if (arguments.Count != 1 || tools.Count != 1)
        {
            return null;
        }
        Shape a = arguments[0], b = tools[0];
        BoundingBox inA = a.BoundingBox(), inB = b.BoundingBox();
        if (!inA.Intersects(inB))
        {
            return null;
        }
        double fromX = Math.Max(inA.Min.X, inB.Min.X), toX = Math.Min(inA.Max.X, inB.Max.X);
        double fromY = Math.Max(inA.Min.Y, inB.Min.Y), toY = Math.Min(inA.Max.Y, inB.Max.Y);
        double fromZ = Math.Max(inA.Min.Z, inB.Min.Z), toZ = Math.Min(inA.Max.Z, inB.Max.Z);
        if (!double.IsFinite(fromX + toX + fromY + toY + fromZ + toZ) || toX <= fromX || toY <= fromY || toZ <= fromZ)
        {
            return null;
        }

        // A point deep inside both has a ball of that radius inside both, and so inside the overlap of their boxes. Inputs that only
        // touch have a slab of their tolerances for an overlap, which no point can be deep in, so nothing need be classified: a
        // bottle's neck on its body took 343 points of each, a quarter of its bytes (decision 1887)
        if (smallerVolume < 0)
        {
            smallerVolume = Math.Min(VolumeOf(a), VolumeOf(b));
        }
        double size = Math.Cbrt(smallerVolume);
        if (Math.Min(toX - fromX, Math.Min(toY - fromY, toZ - fromZ)) < 2 * OverlapDepth * size)
        {
            return null;
        }

        var classifyA = new SolidClassifier(a);
        var classifyB = new SolidClassifier(b);
        SolidClassifier? classifyResult = rule == BooleanVolumeRule.Cut ? new SolidClassifier(result) : null;
        double depth = -1;
        int found = 0;
        Point3D first = default;
        double cell = (toX - fromX) * (toY - fromY) * (toZ - fromZ) / Math.Pow(OverlapGrid + 1, 3);
        double needed = Math.Max(2, 2 * lost / cell + 1);
        for (int i = 1; i <= OverlapGrid; i++)
        {
            for (int j = 1; j <= OverlapGrid; j++)
            {
                for (int k = 1; k <= OverlapGrid; k++)
                {
                    var point = new Point3D(fromX + (toX - fromX) * i / (OverlapGrid + 1), fromY + (toY - fromY) * j / (OverlapGrid + 1),
                                            fromZ + (toZ - fromZ) * k / (OverlapGrid + 1));
                    if (!Inside(classifyA, point) || !Inside(classifyB, point))
                    {
                        continue;
                    }
                    // Inside both and gone from the result: the cut did remove something, however small (a small hole in a large block)
                    if (classifyResult is not null && !Inside(classifyResult, point))
                    {
                        return null;
                    }
                    if (depth < 0)
                    {
                        if (!Trustworthy(arguments) || !Trustworthy(tools))
                        {
                            return null;
                        }
                        depth = OverlapDepth * size + MarginOfTolerance * (LargestTolerance(arguments, tools, result) + fuzzyValue);
                    }
                    // Deep inside both, so it can't be tolerance or a touch
                    if (Deep(point, a, depth) && Deep(point, b, depth))
                    {
                        if (found++ == 0)
                        {
                            first = point;
                        }
                        if (found >= needed)
                        {
                            string at = $"({Figure(first.X)}, {Figure(first.Y)}, {Figure(first.Z)})";
                            return Diagnostic.Error(DiagnosticCode.InvalidResult, rule switch
                            {
                                BooleanVolumeRule.Common => $"The common's result is wrong: it's empty, but the point {at} and others are inside both inputs.",
                                BooleanVolumeRule.Cut => $"The cut's result is wrong: it removed nothing, but the point {at} and others are inside the tool and still in the result.",
                                _ => $"The fuse's result is wrong: it's as large as its inputs together, but the point {at} and others are inside both of them.",
                            });
                        }
                    }
                }
            }
        }
        return null;
    }

    // Faces or shells against solids have no volume to go by, so their result's faces are looked at (decision 1932): what a common
    // keeps must not be outside the solids, and what a cut keeps must not be inside them. OCCT #1039 keeps 86898 of a slice through a
    // model, with an edge of the section lost, where the slice beside it keeps 70013. A point counts only if it's further from the
    // solids' shells than tolerance could explain, and two are asked for, as a classifier can be wrong about one. The inputs' faces
    // are looked at too, for what the result ought to have kept. A result whose wrong part falls between the points is passed.
    private static Diagnostic? FacesAgainstSolids(BooleanVolumeRule rule, IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, Shape result, double fuzzyValue)
    {
        if (rule == BooleanVolumeRule.Fuse)
        {
            return null;
        }

        IReadOnlyList<Shape> solids, others;
        if (AllSolids(tools) && HasSolid(tools) && !HasSolid(arguments))
        {
            (solids, others) = (tools, arguments);
        }
        else if (rule == BooleanVolumeRule.Common && AllSolids(arguments) && HasSolid(arguments) && !HasSolid(tools))
        {
            (solids, others) = (arguments, tools);
        }
        else
        {
            return null;
        }

        if (others.Any(static o => o.IsNull) || result.AllFaces().Length > FacesLooked
            || others.Sum(static o => o.AllFaces().Length) is 0 or > FacesLooked || !Trustworthy(solids))
        {
            return null;
        }

        List<SolidClassifier> classifiers = [.. solids.SelectMany(static s => s.AllSolids()).Select(static s => new SolidClassifier(s))];
        double size = Math.Cbrt(solids.Sum(static s => VolumeOf(s)));
        double depth = OverlapDepth * size + MarginOfTolerance * (LargestTolerance(arguments, tools, result) + fuzzyValue);
        int found = 0;
        Point3D first = default;
        foreach (Face face in result.AllFaces().Concat(others.SelectMany(static o => o.AllFaces())))
        {
            bool ofInput = found >= 0 && !result.AllFaces().Contains(face);
            // Its own bounds, from its wires; a face with none, such as an infinite plane, has nothing to sample
            if (face.BoundingBox() is var faceBox && (faceBox.IsEmpty || !(faceBox.Min.DistanceTo(faceBox.Max) <= UnboundedFace)))
            {
                continue;
            }

            var surface = new FaceSurfaceAdaptor(face);
            BrepTools.UVBounds(face, out double fromU, out double toU, out double fromV, out double toV);
            if (!double.IsFinite(fromU + toU + fromV + toV))
            {
                continue;
            }

            for (int i = 1; i <= FaceGrid; i++)
            {
                for (int j = 1; j <= FaceGrid; j++)
                {
                    double u = fromU + (toU - fromU) * i / (FaceGrid + 1), v = fromV + (toV - fromV) * j / (FaceGrid + 1);
                    if (face.Classify(new Point2D(u, v)) != Containment.In)
                    {
                        continue;
                    }

                    Point3D point = surface.Value(u, v);
                    bool inside = classifiers.Any(c => Inside(c, point));
                    bool kept = inside == (rule == BooleanVolumeRule.Common);
                    // Of the result: it mustn't be what the operation drops. Of the input: what the operation keeps must be in the result
                    if (kept != ofInput || !solids.All(s => Deep(point, s, depth))
                        || (ofInput && (!result.Distance(Vertex.At(point)).TryGetValue(out DistanceResult? gap) || gap.Value < depth)))
                    {
                        continue;
                    }

                    if (found++ == 0)
                    {
                        first = point;
                    }

                    if (found >= 2)
                    {
                        string at = $"({Figure(first.X)}, {Figure(first.Y)}, {Figure(first.Z)})";
                        return Diagnostic.Error(DiagnosticCode.InvalidResult, rule == BooleanVolumeRule.Common
                            ? $"The common's result isn't the part of its faces inside the solid, at the point {at} and others. An intersection was probably lost."
                            : $"The cut's result isn't the part of its faces outside the solid, at the point {at} and others. An intersection was probably lost.");
                    }
                }
            }
        }

        return null;
    }

    // A fuse no larger than its larger input holds the smaller one inside it, or has lost part of it: OCCT #1643 gives the body alone,
    // with the lip's part outside it gone, which no rule of volume can see (decision 1937). Two points on the smaller input's faces,
    // well outside the result, prove the second; the volume lost can be too small for a grid inside it to reach.
    private static Diagnostic? ProvedLost(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, Shape result, double fuzzyValue, double smallerVolume)
    {
        if (arguments.Count != 1 || tools.Count != 1 || smallerVolume <= 0)
        {
            return null;
        }

        bool argumentIsSmaller = VolumeOf(arguments[0]) <= VolumeOf(tools[0]);
        Shape smaller = argumentIsSmaller ? arguments[0] : tools[0], larger = argumentIsSmaller ? tools[0] : arguments[0];
        if (smaller.AllFaces().Length > FacesLooked)
        {
            return null;
        }

        var inResult = new SolidClassifier(result);
        var inLarger = new SolidClassifier(larger);
        double depth = -1;
        int found = 0;
        Point3D first = default;
        foreach (Face face in smaller.AllFaces())
        {
            // Its own bounds, from its wires; a face with none, such as an infinite plane, has nothing to sample
            if (face.BoundingBox() is var faceBox && (faceBox.IsEmpty || !(faceBox.Min.DistanceTo(faceBox.Max) <= UnboundedFace)))
            {
                continue;
            }

            var surface = new FaceSurfaceAdaptor(face);
            BrepTools.UVBounds(face, out double fromU, out double toU, out double fromV, out double toV);
            if (!double.IsFinite(fromU + toU + fromV + toV))
            {
                continue;
            }

            for (int i = 1; i <= FaceGrid; i++)
            {
                for (int j = 1; j <= FaceGrid; j++)
                {
                    double u = fromU + (toU - fromU) * i / (FaceGrid + 1), v = fromV + (toV - fromV) * j / (FaceGrid + 1);
                    if (face.Classify(new Point2D(u, v)) != Containment.In)
                    {
                        continue;
                    }

                    Point3D point = surface.Value(u, v);
                    // What's inside the larger input is in a fuse whatever its classifier says of the result
                    if (Inside(inResult, point) || Inside(inLarger, point))
                    {
                        continue;
                    }

                    if (depth < 0)
                    {
                        if (!Trustworthy(arguments) || !Trustworthy(tools))
                        {
                            return null;
                        }
                        depth = OverlapDepth * Math.Cbrt(smallerVolume) + MarginOfTolerance * (LargestTolerance(arguments, tools, result) + fuzzyValue);
                    }

                    if (Deep(point, result, depth) && Deep(point, larger, depth))
                    {
                        if (found++ == 0)
                        {
                            first = point;
                        }
                        if (found >= 2)
                        {
                            return Diagnostic.Error(DiagnosticCode.InvalidResult,
                                $"The fuse's result is wrong: it lost part of an input, as the point ({Figure(first.X)}, {Figure(first.Y)}, {Figure(first.Z)}) and others are on that input and outside the result.");
                        }
                    }
                }
            }
        }
        return null;
    }

    private static bool Inside(SolidClassifier classifier, in Point3D point)
    {
        classifier.Perform(point, Tolerances.Confusion);
        return classifier.State() == Containment.In;
    }

    private static bool Deep(in Point3D point, Shape shape, double depth)
    {
        Vertex vertex = Vertex.At(point);
        foreach (Shell shell in shape.Explore<Shell>())
        {
            if (!shell.Distance(vertex).TryGetValue(out DistanceResult? distance) || distance.Value < depth)
            {
                return false;
            }
        }
        return true;
    }

    private static (double Least, double Most) Bounds(BooleanVolumeRule rule, in Measure a, in Measure b) => rule switch
    {
        BooleanVolumeRule.Common => (0, Math.Min(a.Sum, b.Sum)),
        // A tool's overlaps only make its sum larger, so this bound stays true; the object's largest solid is what's sure of it
        BooleanVolumeRule.Cut => (a.Largest - b.Sum, a.Sum),
        BooleanVolumeRule.Fuse => (Math.Max(a.Largest, b.Largest), a.Sum + b.Sum),
        _ => throw new KernelArgumentOutOfRangeException(nameof(rule)),
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
                if (!double.IsFinite(room) || room >= 1e60 || VolumeOf(solid) < room * SmallestOfBox)
                {
                    return false;
                }
            }
        }
        return true;
    }

    // A shape's volume as Measured integrates it. A frozen TShape, its children and their geometry never change again, and
    // the volume is a function of them, the placement (by its datums, which fix its transformation) and the orientation, so
    // one measured before is the same bits. Not inside an open working overlay, where a frozen TShape may read as its working
    // copy; nothing is kept from there either
    internal static double VolumeOf(Shape shape)
    {
        if (shape.TShape() is not { IsFrozen: true } tShape || WorkingOverlay.Current is not null)
        {
            return shape.IntegrateVolume(Measured);
        }
        KeptVolume kept = Kept.GetValue(tShape, static _ => new KeptVolume());
        if (kept.TryGet(shape, out double volume))
        {
            return volume;
        }
        volume = shape.IntegrateVolume(Measured);
        kept.Add(shape, volume);
        return volume;
    }

    // A TShape's measured volumes: a list whose nodes never change, read and published with Volatile, so a reader sees whole
    // nodes; two threads keeping one at once can lose one, which is only a volume measured again
    private sealed class KeptVolume
    {
        private Node? _first;

        public bool TryGet(in Shape shape, out double volume)
        {
            for (Node? node = Volatile.Read(ref _first); node is not null; node = node.Next)
            {
                if (node.Orientation == shape.Orientation && node.Placement.IsEqual(shape.Placement))
                {
                    volume = node.Volume;
                    return true;
                }
            }
            volume = 0;
            return false;
        }

        public void Add(in Shape shape, double volume)
        {
            Node? first = Volatile.Read(ref _first);
            if ((first?.Count ?? 0) < KeptPerShape)
            {
                Volatile.Write(ref _first, new Node(shape.Placement, shape.Orientation, volume, first));
            }
        }

        private sealed class Node(Placement placement, Orientation orientation, double volume, Node? next)
        {
            public readonly Placement Placement = placement;
            public readonly Orientation Orientation = orientation;
            public readonly double Volume = volume;
            public readonly Node? Next = next;
            public readonly int Count = (next?.Count ?? 0) + 1;
        }
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
                double volume = VolumeOf(shape);
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
