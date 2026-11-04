// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Features;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Features.Draft;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Features;

/// <summary>Where <see cref="FeatureOperations.ExtrudeUntil"/> stops.</summary>
public enum ExtrudeLimit
{
    /// <summary>At <see cref="ExtrudeUntilOptions.Height"/> from the sketch face.</summary>
    Height,

    /// <summary>At <see cref="ExtrudeUntilOptions.Face"/>, a face of the shape.</summary>
    Face,

    /// <summary>At the shape's last face in the direction (OCCT's <c>PerformUntilEnd</c>).</summary>
    End,

    /// <summary>Through the whole shape, both ways (OCCT's <c>PerformThruAll</c>).</summary>
    ThroughAll,
}

/// <summary>How <see cref="FeatureOperations.ExtrudeUntil"/> extrudes (OCCT's <c>BRepFeat_MakeDPrism</c>).</summary>
public sealed record ExtrudeUntilOptions
{
    /// <summary>Where it stops.</summary>
    public ExtrudeLimit Until { get; init; } = ExtrudeLimit.End;

    /// <summary>For <see cref="ExtrudeLimit.Height"/>: the height.</summary>
    public double Height { get; init; }

    /// <summary>For <see cref="ExtrudeLimit.Face"/>: the face it stops at.</summary>
    public Face? Face { get; init; }

    /// <summary>Add the extrusion to the shape (a boss), or take it away (a pocket).</summary>
    public bool Fuse { get; init; } = true;
}

/// <summary>
/// Features: local edits of a shape (docs/api/offsets-features.md; OCCT's <c>BRepOffsetAPI_DraftAngle</c> and
/// <c>BRepFeat</c>), as static methods and extension members. A failure is the engine's status as a diagnostic
/// (<c>draft.face-failed</c> with the face, <c>feature.no-intersect-u</c>…).
/// </summary>
public static class FeatureOperations
{
    /// <summary>A face can't be drafted: <c>draft.face-failed</c>, with the face.</summary>
    public static DiagnosticCode DraftFaceFailed { get; } = new("draft.face-failed");

    /// <summary>The feature failed without saying why: <c>feature.failed</c>.</summary>
    public static DiagnosticCode Failed { get; } = new("feature.failed");

    private static readonly Diagnostic Failure = Diagnostic.Error(Failed, "The feature couldn't be built.");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>The shape with <paramref name="faces"/> tilted by <paramref name="angle"/> (radians) from
        /// <paramref name="pull"/>, about their intersection with <paramref name="neutralPlane"/>.</summary>
        public Result<Built<Shape>> Draft(IEnumerable<Face> faces, Direction3D pull, double angle, in Plane3D neutralPlane)
        {
            ArgumentNullException.ThrowIfNull(faces);
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            MakerOutcome.Valid(pull, nameof(pull));
            Face[] drafted = Own(s, faces, nameof(faces));
            Plane3D plane = neutralPlane;
            return EngineOperation.Run<Shape>(range =>
            {
                var draft = new DraftAngle(s);
                foreach (Face face in drafted)
                {
                    draft.Add(face, pull, angle, plane);
                    if (!draft.AddDone())
                    {
                        return new EngineOutcome(false, static () => default,
                            Failure: Diagnostic.Error(DraftFaceFailed, "A face can't be drafted at this angle.").WithShapes(draft.ProblematicShape()));
                    }
                }
                return MakerOutcome.Of(draft, range, [s], () => draft.Status() == DraftErrorStatus.NoError
                    ? null
                    : Diagnostic.Error(DraftFaceFailed, $"The draft failed ({draft.Status()}).").WithShapes(draft.ProblematicShape()));
            }, Failure);
        }

        /// <summary>The shape with <paramref name="profile"/>, a face on <paramref name="sketchFace"/> (a face of the
        /// shape), extruded with a taper of <paramref name="taper"/> (radians) and added or taken away.</summary>
        public Result<Built<Shape>> ExtrudeUntil(Face profile, Face sketchFace, double taper, ExtrudeUntilOptions options)
        {
            ArgumentNullException.ThrowIfNull(options);
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            MakerOutcome.NotNull(profile, nameof(profile));
            MakerOutcome.NotNull(sketchFace, nameof(sketchFace));
            Own(s, [sketchFace], nameof(sketchFace));
            Face? until = null;
            switch (options.Until)
            {
                case ExtrudeLimit.Height:
                    MakerOutcome.Positive(Math.Abs(options.Height), nameof(options));
                    break;
                case ExtrudeLimit.Face:
                    until = options.Face is { IsNull: false } f ? f : throw new ArgumentException("ExtrudeLimit.Face needs the face.", nameof(options));
                    break;
                case ExtrudeLimit.End or ExtrudeLimit.ThroughAll:
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(options), options.Until, "An unknown limit.");
            }
            return EngineOperation.Run<Shape>(_ =>
            {
                var prism = new MakeDraftPrism(s, profile, sketchFace, taper, options.Fuse ? 1 : 0, true);
                switch (options.Until)
                {
                    case ExtrudeLimit.Height:
                        prism.Perform(options.Height);
                        break;
                    case ExtrudeLimit.Face:
                        prism.Perform(until!.Value);
                        break;
                    case ExtrudeLimit.End:
                        prism.PerformUntilEnd();
                        break;
                    default:
                        prism.PerformThruAll();
                        break;
                }
                return MakerOutcome.Done(prism, [s, profile], () => StatusDiagnostics.ToDiagnostic(prism.CurrentStatusError()));
            }, Failure);
        }

        /// <summary>The shape with faces split by wires or edges lying on them (OCCT's <c>BRepFeat_SplitShape</c>).</summary>
        public Result<Built<Shape>> SplitFaces(IEnumerable<(Face Face, Shape Splitter)> splits)
        {
            ArgumentNullException.ThrowIfNull(splits);
            Shape s = shape.AsShape();
            MakerOutcome.NotNull(s, nameof(shape));
            (Face Face, Shape Splitter)[] list = [.. splits];
            Own(s, list.Select(p => p.Face), nameof(splits));
            foreach ((_, Shape splitter) in list)
            {
                MakerOutcome.NotNull(splitter, nameof(splits));
                if (splitter.Type is not (ShapeType.Wire or ShapeType.Edge or ShapeType.Compound))
                {
                    throw new ArgumentException($"A splitter must be a wire, an edge or a compound of them, not a {splitter.Type}.", nameof(splits));
                }
            }
            return EngineOperation.Run<Shape>(range =>
            {
                var split = new SplitShape(s);
                foreach ((Face face, Shape splitter) in list)
                {
                    switch (splitter.Type)
                    {
                        case ShapeType.Wire:
                            split.Add((Wire)splitter, face);
                            break;
                        case ShapeType.Edge:
                            split.Add((Edge)splitter, face);
                            break;
                        default:
                            split.Add((Compound)splitter, face);
                            break;
                    }
                }
                return MakerOutcome.Of(split, range, [s]);
            }, Failure);
        }
    }

    private static Face[] Own(Shape shape, IEnumerable<Face> faces, string name)
    {
        Face[] list = [.. faces];
        if (list.Length == 0)
        {
            throw new ArgumentException("There are no faces.", name);
        }
        var own = shape.Faces();
        if (list.Any(f => f.IsNull || !own.Any(o => o.IsSame(f))))
        {
            throw new ArgumentException("A face isn't one of the shape's.", name);
        }
        return list;
    }
}
