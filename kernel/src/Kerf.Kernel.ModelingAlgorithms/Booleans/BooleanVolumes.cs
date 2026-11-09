// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Runtime.CompilerServices;
using Kerf.Engine.Classification;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
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

    // Above this tolerance in a result, surfaces nearly touched: a hundred times the usual (decision 1924)
    private const double NearlyTouching = 1e-5;

    // More faces than this in a result or an input and decision 1924's points aren't asked: a corpus case took 8 s
    private const int MostFacesAsked = 64;

    // The points tried in each direction, in the inputs' whole box, where a result's tolerance is that large
    private const int PointsGrid = 4;

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
        var shared = new Shared(arguments, tools, result);
        // A common that made nothing breaks no rule of volume, so nothing need be measured: only a point in both can prove it wrong
        if (rule == BooleanVolumeRule.Common && outcome.Count == 0)
        {
            return ProvedToOverlap(rule, shared, fuzzyValue, 0, -1);
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
            // Within the rules, but a cut that removed nothing, or a fuse that lost nothing, can still be proved wrong; and a
            // result whose tolerances say surfaces nearly touched is asked at points (decision 1924)
            return (rule switch
            {
                BooleanVolumeRule.Cut when r >= a.Sum * (1 - MarginOfSize) => ProvedToOverlap(rule, shared, fuzzyValue, 0, Math.Min(a.Sum, b.Sum)),
                BooleanVolumeRule.Fuse when r <= least + margin && ProvedLost(shared, fuzzyValue, Math.Min(a.Sum, b.Sum)) is { } lost => lost,
                BooleanVolumeRule.Fuse when r >= most - margin => ProvedToOverlap(rule, shared, fuzzyValue, Math.Max(0, most - r), Math.Min(a.Sum, b.Sum)),
                _ => null,
            }) ?? ProvedByPoints(rule, shared, fuzzyValue, Math.Min(a.Sum, b.Sum));
        }

        // Past a rule, which is rare, so this is when the dearer questions are worth asking: is it proved?
        margin += size * MarginOfTolerance * (shared.LargestTolerance + fuzzyValue) / Math.Cbrt(size);
        negative = outcome.Count > 0 && outcome.Smallest < -margin;
        if ((r >= least - margin && r <= most + margin && !negative) || !shared.Trustworthy)
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
    private static Diagnostic? ProvedToOverlap(BooleanVolumeRule rule, Shared shared, double fuzzyValue, double lost, double smallerVolume)
    {
        if (!shared.OneEach)
        {
            return null;
        }
        Shape a = shared.Argument.Shape, b = shared.Tool.Shape;
        BoundingBox inA = shared.Argument.Box, inB = shared.Tool.Box;
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

        Classified classifyA = shared.Argument, classifyB = shared.Tool;
        Classified? classifyResult = rule == BooleanVolumeRule.Cut ? shared.Result : null;
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
                    if (!classifyA.Inside(point) || !classifyB.Inside(point))
                    {
                        continue;
                    }
                    // Inside both and gone from the result: the cut did remove something, however small (a small hole in a large block)
                    if (classifyResult is not null && !classifyResult.Inside(point))
                    {
                        return null;
                    }
                    if (depth < 0)
                    {
                        if (!shared.Trustworthy)
                        {
                            return null;
                        }
                        depth = OverlapDepth * size + MarginOfTolerance * (shared.LargestTolerance + fuzzyValue);
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

            // One classifier of the face for its points, not one each (unit 13A)
            var inFace = new FaceClassifier2D(face, Tolerances.Confusion);
            for (int i = 1; i <= FaceGrid; i++)
            {
                for (int j = 1; j <= FaceGrid; j++)
                {
                    double u = fromU + (toU - fromU) * i / (FaceGrid + 1), v = fromV + (toV - fromV) * j / (FaceGrid + 1);
                    if (inFace.Perform(new Point2D(u, v)) != Containment.In)
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
    private static Diagnostic? ProvedLost(Shared shared, double fuzzyValue, double smallerVolume)
    {
        if (!shared.OneEach || smallerVolume <= 0)
        {
            return null;
        }

        Shape result = shared.Result.Shape;
        bool argumentIsSmaller = VolumeOf(shared.Argument.Shape) <= VolumeOf(shared.Tool.Shape);
        Classified larger = argumentIsSmaller ? shared.Tool : shared.Argument;
        Shape smaller = argumentIsSmaller ? shared.Argument.Shape : shared.Tool.Shape;
        if (smaller.AllFaces().Length > FacesLooked)
        {
            return null;
        }

        // A point can prove something only if it's well outside the result, so a point on the result's boundary is passed over
        // before it's classified (unit 13A): a face of the smaller input that's one of the result's lies on it all over, and a
        // point inside a face of the result on the same surface, placed the same, is on it too, as is one within a tolerance of
        // a plane of the result and inside its face. A small part fused onto a large one keeps most of its faces, or pieces of
        // them, or lies on the large one's, and they were most of what this classified: a bump of 1.5 by 0.5 on a door 1,688
        // classifications and distances, 311 MB, after a Boolean of 8 MB
        var resultFaces = new HashSet<Shape>(ShapeHasher.Instance);
        var onSurface = new Dictionary<Surface, List<(Placement Placement, InFace Classifier)>>(ReferenceEqualityComparer.Instance);
        var planes = new List<ResultPlane>();
        foreach (Face face in result.AllFaces())
        {
            resultFaces.Add(face);
            if (Tool.Surface(face, out Placement placement) is { } surface)
            {
                if (!onSurface.TryGetValue(surface, out var faces))
                {
                    onSurface[surface] = faces = [];
                }
                var classifier = new InFace(face);
                faces.Add((placement, classifier));
                if (surface is PlaneSurface plane)
                {
                    planes.Add(new ResultPlane(new FaceSurfaceAdaptor(face).Plane(), plane.Plane, placement, classifier, Math.Max(Tool.Tolerance(face), Tolerances.Confusion)));
                }
            }
        }

        // The classifiers are made when a point needs them: a classifier's first point fills its trees, about 3 ms on the door,
        // and a boss inside the door never asks the door's
        Classified inResult = shared.Result;
        double depth = -1;
        int found = 0;
        Point3D first = default;
        foreach (Face face in smaller.AllFaces())
        {
            // Its own bounds, from its wires; a face with none, such as an infinite plane, has nothing to sample
            if (resultFaces.Contains(face)
                || (face.BoundingBox() is var faceBox && (faceBox.IsEmpty || !(faceBox.Min.DistanceTo(faceBox.Max) <= UnboundedFace))))
            {
                continue;
            }

            var surface = new FaceSurfaceAdaptor(face);
            BrepTools.UVBounds(face, out double fromU, out double toU, out double fromV, out double toV);
            if (!double.IsFinite(fromU + toU + fromV + toV))
            {
                continue;
            }

            var inFace = new FaceClassifier2D(face, Tolerances.Confusion);
            List<(Placement Placement, InFace Classifier)> resultFacesHere = [];
            List<ResultPlane> planesHere = [];
            if (Tool.Surface(face, out Placement here) is { } own)
            {
                if (onSurface.TryGetValue(own, out var sharing))
                {
                    resultFacesHere.AddRange(sharing.Where(s => s.Placement.IsEqual(here)));
                }
                if (own is PlaneSurface)
                {
                    Direction3D normal = surface.Plane().Axis.Direction;
                    planesHere.AddRange(planes.Where(p => p.World.Axis.Direction.IsParallel(normal, 1e-9)));
                }
            }

            for (int i = 1; i <= FaceGrid; i++)
            {
                for (int j = 1; j <= FaceGrid; j++)
                {
                    double u = fromU + (toU - fromU) * i / (FaceGrid + 1), v = fromV + (toV - fromV) * j / (FaceGrid + 1);
                    var uv = new Point2D(u, v);
                    if (inFace.Perform(uv) != Containment.In || resultFacesHere.Any(s => s.Classifier.Holds(uv)))
                    {
                        continue;
                    }

                    Point3D point = surface.Value(u, v);
                    if (planesHere.Any(p => p.Holds(point)))
                    {
                        continue;
                    }
                    // What's inside the larger input is in a fuse whatever its classifier says of the result; and a point on the
                    // boundary of either is within its tolerance of it, so can't be deep outside it
                    if (inResult.StateOf(point) is Containment.In or Containment.On || larger.StateOf(point) is Containment.In or Containment.On)
                    {
                        continue;
                    }

                    if (depth < 0)
                    {
                        if (!shared.Trustworthy)
                        {
                            return null;
                        }
                        depth = OverlapDepth * Math.Cbrt(smallerVolume) + MarginOfTolerance * (shared.LargestTolerance + fuzzyValue);
                    }

                    if (Deep(point, result, depth) && Deep(point, larger.Shape, depth))
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

    // Within the rules of volume, a result can still have lost a part, as a fuse of crossing cylinders lost their common part, 16.5%
    // of it, valid. So where the result's tolerances say surfaces nearly touched, which is where this happens, points through the
    // inputs' box are asked what the operation says of them, and two that disagree, deep in or out of all three, prove it wrong
    // (decision 1924). A point outside a shape's box is outside it, and further from it than its distance to the box, so neither
    // needs a classifier; and of the inputs only what decides the operation's answer is asked (unit 13K)
    private static Diagnostic? ProvedByPoints(BooleanVolumeRule rule, Shared shared, double fuzzyValue, double smallerVolume)
    {
        if (!shared.OneEach || shared.ResultTolerance <= NearlyTouching || shared.Result.Shape.AllFaces().Length > MostFacesAsked
            || shared.Argument.Shape.AllFaces().Length > MostFacesAsked || shared.Tool.Shape.AllFaces().Length > MostFacesAsked)
        {
            return null;
        }

        s_pointChecks++;
        BoundingBox inA = shared.Argument.Box, inB = shared.Tool.Box;
        if (inA.IsEmpty || inB.IsEmpty)
        {
            return null;
        }
        BoundingBox all = inA.Union(inB);
        Point3D from = all.Min, to = all.Max;
        if (!double.IsFinite(from.X + from.Y + from.Z + to.X + to.Y + to.Z))
        {
            return null;
        }

        double depth = -1;
        int found = 0;
        Point3D first = default;
        bool should = false;
        for (int i = 1; i <= PointsGrid; i++)
        {
            for (int j = 1; j <= PointsGrid; j++)
            {
                for (int k = 1; k <= PointsGrid; k++)
                {
                    var point = new Point3D(from.X + (to.X - from.X) * i / (PointsGrid + 1), from.Y + (to.Y - from.Y) * j / (PointsGrid + 1),
                                            from.Z + (to.Z - from.Z) * k / (PointsGrid + 1));
                    // Only what the answer needs: a common's point outside the object is outside it whatever the tool says
                    Containment inArgument = shared.Argument.Quick(point), inTool = Containment.Unknown;
                    should = rule switch
                    {
                        BooleanVolumeRule.Common => inArgument == Containment.In && (inTool = shared.Tool.Quick(point)) == Containment.In,
                        BooleanVolumeRule.Cut => inArgument == Containment.In && (inTool = shared.Tool.Quick(point)) != Containment.In,
                        _ => inArgument == Containment.In || (inTool = shared.Tool.Quick(point)) == Containment.In,
                    };
                    Containment inResult = shared.Result.Quick(point);
                    if (should == (inResult == Containment.In))
                    {
                        continue;
                    }

                    // A point on the boundary of any of the three is within its tolerance of it, so can't be deep
                    if (inTool == Containment.Unknown)
                    {
                        inTool = shared.Tool.Quick(point);
                    }
                    if (inArgument == Containment.On || inTool == Containment.On || inResult == Containment.On)
                    {
                        continue;
                    }
                    if (depth < 0)
                    {
                        if (!shared.Trustworthy)
                        {
                            return null;
                        }
                        depth = OverlapDepth * Math.Cbrt(smallerVolume) + MarginOfTolerance * (shared.LargestTolerance + fuzzyValue);
                    }
                    if (shared.Argument.Deep(point, depth) && shared.Tool.Deep(point, depth) && shared.Result.Deep(point, depth) && found++ == 0)
                    {
                        first = point;
                    }
                    if (found >= 2)
                    {
                        string name = rule switch
                        {
                            BooleanVolumeRule.Common => "common",
                            BooleanVolumeRule.Cut => "cut",
                            _ => "fuse",
                        };
                        string at = $"({Figure(first.X)}, {Figure(first.Y)}, {Figure(first.Z)})";
                        return Diagnostic.Error(DiagnosticCode.InvalidResult, should
                            ? $"The {name}'s result is wrong: the point {at} and others should be in it and aren't."
                            : $"The {name}'s result is wrong: the point {at} and others shouldn't be in it and are.");
                    }
                }
            }
        }
        return null;
    }

    // A planar face of a result: a point within its tolerance of the plane, and inside the face, is on the result's boundary
    private sealed record ResultPlane(Plane3D World, Plane3D Own, Placement Placement, InFace Classifier, double Tolerance)
    {
        public bool Holds(in Point3D point)
        {
            if (World.DistanceTo(point) > Tolerance)
            {
                return false;
            }
            Engine.Geometry.ElSLib.Parameters(Own, point.Transformed(Placement.Transformation.Inverted()), out double u, out double v);
            return Classifier.Holds(new Point2D(u, v));
        }
    }

    // A face's 2D classifier, made when a point first asks it: most of a result's faces are never asked, and making one for
    // each of the door's 52 was about 2.5 ms
    private sealed class InFace(Face face)
    {
        private FaceClassifier2D? _classifier;

        public bool Holds(in Point2D uv) => (_classifier ??= new FaceClassifier2D(face, Tolerances.Confusion)).Perform(uv) == Containment.In;
    }

    // What one check has learnt of its shapes, each part made when a question first needs it and kept for the questions after
    // (unit 13K): a classifier fills its trees on its first point, about 3 ms on the door, and validating an input is a check of
    // the whole shape, so a check asking several questions of one input and one tool makes each once. One check's, on its thread
    private sealed class Shared(IReadOnlyList<Shape> arguments, IReadOnlyList<Shape> tools, Shape result)
    {
        private Classified? _argument, _tool, _result;
        private double _resultTolerance = -1, _largestTolerance = -1;
        private bool? _trustworthy;

        // What the point questions are asked of: one object and one tool, as what several mean together isn't plain
        public bool OneEach => arguments.Count == 1 && tools.Count == 1;

        public Classified Argument => _argument ??= new Classified(arguments[0]);

        public Classified Tool => _tool ??= new Classified(tools[0]);

        public Classified Result => _result ??= new Classified(result);

        public double ResultTolerance => _resultTolerance >= 0 ? _resultTolerance : _resultTolerance = new ShapeToleranceAnalysis().Tolerance(result, 1);

        public double LargestTolerance =>_largestTolerance >= 0 ? _largestTolerance : _largestTolerance = BooleanVolumes.LargestTolerance(arguments, tools, result);

        public bool Trustworthy => _trustworthy ??= BooleanVolumes.Trustworthy(arguments) && BooleanVolumes.Trustworthy(tools);
    }

    // A shape a check asks points of: its classifier and its box, each made when first asked
    private sealed class Classified(Shape shape)
    {
        private SolidClassifier? _classifier;
        private BoundingBox? _box, _widened;

        public Shape Shape => shape;

        public BoundingBox Box => _box ??= shape.BoundingBox();

        public Containment StateOf(in Point3D point) => BooleanVolumes.StateOf(_classifier ??= new SolidClassifier(shape), point);

        public bool Inside(in Point3D point) => StateOf(point) == Containment.In;

        // Out without a classifier for a point outside the box, which holds the shape and its tolerances, widened by a
        // thousandth of its diagonal (decision 1924's points, unit 13K)
        public Containment Quick(in Point3D point)
        {
            if (_widened is not { } widened)
            {
                BoundingBox box = Box;
                _widened = widened = box.IsEmpty ? box : box.Enlarged(OverlapDepth * box.Min.DistanceTo(box.Max));
            }
            return widened.IsEmpty || widened.Contains(point) ? StateOf(point) : Containment.Out;
        }

        // Further than the depth from each of the shape's shells: a point at least that far from the box is, without a distance
        public bool Deep(in Point3D point, double depth)
        {
            BoundingBox box = Box;
            if (!box.IsEmpty)
            {
                Point3D low = box.Min, high = box.Max;
                double x = Math.Max(0, Math.Max(low.X - point.X, point.X - high.X));
                double y = Math.Max(0, Math.Max(low.Y - point.Y, point.Y - high.Y));
                double z = Math.Max(0, Math.Max(low.Z - point.Z, point.Z - high.Z));
                if (Math.Sqrt(x * x + y * y + z * z) >= depth)
                {
                    return true;
                }
            }
            return BooleanVolumes.Deep(point, shape, depth);
        }
    }

    // The points this thread's checks have classified or measured the distance of, for the tests that hold the checks' cost down
    [ThreadStatic]
    internal static int s_pointsAsked;

    // The checks of decision 1924's points this thread has made, past the tolerance and the faces that let them be asked
    [ThreadStatic]
    internal static int s_pointChecks;

    private static bool Inside(SolidClassifier classifier, in Point3D point) => StateOf(classifier, point) == Containment.In;

    private static Containment StateOf(SolidClassifier classifier, in Point3D point)
    {
        s_pointsAsked++;
        classifier.Perform(point, Tolerances.Confusion);
        return classifier.State();
    }

    private static bool Deep(in Point3D point, Shape shape, double depth)
    {
        s_pointsAsked++;
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
