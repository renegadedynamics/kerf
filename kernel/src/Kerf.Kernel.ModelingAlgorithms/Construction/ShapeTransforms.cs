// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Construction;

/// <summary>
/// Shapes transformed into new shapes (docs/api/construction.md; OCCT's <c>BRepBuilderAPI_Transform</c> and
/// <c>GTransform</c>), as extension members: <c>shape.Transformed(t)</c>. A rigid move only changes the placement; a
/// scale, a mirror or a general transformation copies the geometry. For a placement alone, Topology's <c>Moved</c>.
/// </summary>
public static class ShapeTransforms
{
    /// <summary>The transformation couldn't be applied: <c>construction.transform-failed</c>.</summary>
    public static DiagnosticCode TransformFailed { get; } = new("construction.transform-failed");

    private static readonly Diagnostic Failure = Diagnostic.Error(TransformFailed, "The shape couldn't be transformed.");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>The shape transformed by <paramref name="transform"/>. With <paramref name="copyGeometry"/>, a rigid move
        /// too makes new geometry (OCCT's <c>BRepBuilderAPI_Transform</c> with <c>Copy</c>), where without it only the
        /// placement changes.</summary>
        public Result<Built<Shape>> Transformed(in Transform3D transform, bool copyGeometry = false)
        {
            MakerOutcome.NotNull(shape.AsShape(), nameof(shape));
            Shape s = shape.AsShape();
            Transform3D t = transform;
            return EngineOperation.Run<Shape>(range => MakerOutcome.Of(new Transform(s, t, copyGeometry), range, [s]), Failure);
        }

        /// <summary>The shape transformed by a general (affine) transformation, such as a scale by axis; its geometry
        /// is converted to B-splines where it must be. With <paramref name="copyGeometry"/>, geometry the transformation
        /// leaves as it is is copied too (OCCT's <c>BRepBuilderAPI_GTransform</c> with <c>Copy</c>).</summary>
        public Result<Built<Shape>> Transformed(in GeneralTransform3D transform, bool copyGeometry = false)
        {
            MakerOutcome.NotNull(shape.AsShape(), nameof(shape));
            Shape s = shape.AsShape();
            GeneralTransform3D t = transform;
            return EngineOperation.Run<Shape>(range => MakerOutcome.Of(new GTransform(s, t, copyGeometry), range, [s]), Failure, checkedAs: "The transformed shape");
        }

        /// <summary>The shape mirrored in the plane of <paramref name="plane"/> (its origin and its normal, the main
        /// direction).</summary>
        public Result<Built<Shape>> Mirrored(in Frame3D plane)
        {
            MakerOutcome.Valid(plane, nameof(plane));
            return shape.Transformed(Transform3D.Mirror(plane));
        }

        /// <summary>The shape scaled by <paramref name="factor"/> about <paramref name="centre"/>.</summary>
        public Result<Built<Shape>> Scaled(Point3D centre, double factor)
        {
            if (!(Math.Abs(factor) > Tolerances.Confusion) || double.IsInfinity(factor))
            {
                throw new KernelArgumentOutOfRangeException(nameof(factor), factor, "The scale factor must be a non-zero number.");
            }
            return shape.Transformed(Transform3D.Scale(centre, factor));
        }
    }
}

/// <summary>Wires projected onto shapes (docs/api/construction.md; OCCT's <c>BRepProj_Projection</c>).</summary>
public static class ShapeProjections
{
    /// <summary>The wire's projection doesn't meet the shape: <c>construction.projection-empty</c>.</summary>
    public static DiagnosticCode ProjectionEmpty { get; } = new("construction.projection-empty");

    /// <summary><paramref name="wire"/> projected onto <paramref name="target"/> along <paramref name="direction"/>
    /// (a cylindrical projection): the wires it makes on the shape's faces.</summary>
    public static Result<ImmutableArray<Wire>> Project(Wire wire, Shape target, Direction3D direction)
    {
        MakerOutcome.NotNull(wire, nameof(wire));
        MakerOutcome.NotNull(target, nameof(target));
        MakerOutcome.Valid(direction, nameof(direction));
        return Wires(() => new Projection(wire, target, direction));
    }

    /// <summary><paramref name="edge"/> projected onto <paramref name="target"/> along <paramref name="direction"/>, as a
    /// wire's is.</summary>
    public static Result<ImmutableArray<Wire>> Project(Edge edge, Shape target, Direction3D direction)
    {
        MakerOutcome.NotNull(edge, nameof(edge));
        MakerOutcome.NotNull(target, nameof(target));
        MakerOutcome.Valid(direction, nameof(direction));
        return Wires(() => new Projection(edge, target, direction));
    }

    /// <summary><paramref name="edge"/> projected onto <paramref name="target"/> from <paramref name="eye"/>, as a wire's is.</summary>
    public static Result<ImmutableArray<Wire>> Project(Edge edge, Shape target, Point3D eye)
    {
        MakerOutcome.NotNull(edge, nameof(edge));
        MakerOutcome.NotNull(target, nameof(target));
        return Wires(() => new Projection(edge, target, eye));
    }

    /// <summary><paramref name="wire"/> projected onto <paramref name="target"/> from <paramref name="eye"/> (a conical
    /// projection).</summary>
    public static Result<ImmutableArray<Wire>> Project(Wire wire, Shape target, Point3D eye)
    {
        MakerOutcome.NotNull(wire, nameof(wire));
        MakerOutcome.NotNull(target, nameof(target));
        return Wires(() => new Projection(wire, target, eye));
    }

    private static Result<ImmutableArray<Wire>> Wires(Func<Projection> project)
    {
        try
        {
            Projection projection = project();
            var wires = ImmutableArray.CreateBuilder<Wire>();
            for (projection.Init(); projection.More(); projection.Next())
            {
                wires.Add(projection.Current());
            }
            return wires.Count > 0
                ? wires.ToImmutable()
                : Result<ImmutableArray<Wire>>.Failure(Diagnostic.Error(ProjectionEmpty, "The wire's projection doesn't meet the shape."));
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            return Result<ImmutableArray<Wire>>.Failure(FailureDiagnostics.ToDiagnostic(e));
        }
    }
}
