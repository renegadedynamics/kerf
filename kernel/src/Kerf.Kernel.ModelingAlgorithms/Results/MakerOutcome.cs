// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Foundation;

/// <summary>
/// What a <see cref="MakeShape"/> algorithm left, for <see cref="EngineOperation.Run{TShape}"/> (unit 10L2): it builds
/// the algorithm, and its history of the inputs' vertices, edges, faces and solids (OCCT's <c>BRepTools_History</c>
/// from the algorithm's <c>Modified</c>, <c>Generated</c> and <c>IsDeleted</c>), only once it is done.
/// </summary>
internal static class MakerOutcome
{
    /// <summary>Builds <paramref name="maker"/> on <paramref name="range"/> and says what it left; an algorithm that isn't
    /// done fails with <paramref name="status"/>'s diagnostic (a status enum's), if it gives one.</summary>
    public static EngineOutcome Of(MakeShape maker, ProgressRange range, ReadOnlySpan<Shape> inputs, Func<Diagnostic?>? status = null)
    {
        maker.Build(range);
        return Done(maker, inputs, status);
    }

    /// <summary>What an already built <paramref name="maker"/> left.</summary>
    public static EngineOutcome Done(MakeShape maker, ReadOnlySpan<Shape> inputs, Func<Diagnostic?>? status = null)
    {
        if (!maker.IsDone)
        {
            return new EngineOutcome(false, static () => default, Failure: status?.Invoke());
        }

        return new EngineOutcome(true, maker.Shape, HistoryOf(maker, inputs));
    }

    /// <summary>The history of <paramref name="algorithm"/> for the sub-shapes of <paramref name="inputs"/>.</summary>
    public static History HistoryOf(History.IAlgorithm algorithm, ReadOnlySpan<Shape> inputs)
    {
        var arguments = new NCollection_List<Shape>();
        foreach (Shape input in inputs)
        {
            arguments.Append(input);
        }
        return new History(arguments, algorithm);
    }

    /// <summary>A shape made without an algorithm of its own (a ruled face): done, with an empty history.</summary>
    public static EngineOutcome Made(Shape shape) => new(!shape.IsNull, () => shape);

    /// <summary>Misuse: a null shape, or the wrong kind of shape, is an argument error.</summary>
    public static void NotNull(Shape shape, string name)
    {
        if (shape.IsNull)
        {
            throw new ArgumentException("The shape is null.", name);
        }
    }

    /// <summary>Misuse: a size that isn't a positive number.</summary>
    public static void Positive(double value, string name)
    {
        if (!(value > 0) || double.IsInfinity(value))
        {
            throw new ArgumentOutOfRangeException(name, value, "The value must be a positive number.");
        }
    }

    /// <summary>Misuse: a default (zero) frame, axis or direction.</summary>
    public static void Valid(in Frame3D frame, string name)
    {
        if (frame.Direction.Equals(default(Direction3D)))
        {
            throw new ArgumentException("The frame is default (zero); use Frame3D.World or a frame of your own.", name);
        }
    }

    /// <inheritdoc cref="Valid(in Frame3D, string)"/>
    public static void Valid(in Axis3D axis, string name)
    {
        if (axis.Direction.Equals(default(Direction3D)))
        {
            throw new ArgumentException("The axis is default (zero).", name);
        }
    }

    /// <inheritdoc cref="Valid(in Frame3D, string)"/>
    public static void Valid(Direction3D direction, string name)
    {
        if (direction.Equals(default(Direction3D)))
        {
            throw new ArgumentException("The direction is default (zero).", name);
        }
    }
}
