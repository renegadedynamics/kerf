// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Geometry;

namespace Kerf.Fonts;

/// <summary>What a segment of a glyph's outline is.</summary>
public enum OutlineSegmentKind
{
    /// <summary>A straight line from <see cref="OutlineSegment.Start"/> to <see cref="OutlineSegment.End"/>.</summary>
    Line,

    /// <summary>A quadratic Bézier curve (TrueType's), with <see cref="OutlineSegment.Control1"/> its control point.</summary>
    Quadratic,

    /// <summary>A cubic Bézier curve (CFF's), with <see cref="OutlineSegment.Control1"/> and <see cref="OutlineSegment.Control2"/>.</summary>
    Cubic,
}

/// <summary>
/// A segment of a glyph's outline, in the font's units (<see cref="Font.UnitsPerEm"/> to the em), y up. A segment's start
/// is the previous one's end, and a contour's last segment ends where its first starts.
/// </summary>
public readonly struct OutlineSegment : IEquatable<OutlineSegment>
{
    internal OutlineSegment(OutlineSegmentKind kind, Point2D start, Point2D control1, Point2D control2, Point2D end)
    {
        Kind = kind;
        Start = start;
        Control1 = control1;
        Control2 = control2;
        End = end;
    }

    /// <summary>A line, a quadratic or a cubic.</summary>
    public OutlineSegmentKind Kind { get; }

    /// <summary>Where it starts.</summary>
    public Point2D Start { get; }

    /// <summary>The first control point of a curve; a line's start.</summary>
    public Point2D Control1 { get; }

    /// <summary>A cubic's second control point; a quadratic's only one, a line's end.</summary>
    public Point2D Control2 { get; }

    /// <summary>Where it ends.</summary>
    public Point2D End { get; }

    /// <summary>Exact equality of the kind and the points.</summary>
    public bool Equals(OutlineSegment other) =>
        Kind == other.Kind && Start == other.Start && Control1 == other.Control1 && Control2 == other.Control2 && End == other.End;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is OutlineSegment other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(Kind, Start, Control1, Control2, End);

    /// <summary>Exact equality.</summary>
    public static bool operator ==(OutlineSegment left, OutlineSegment right) => left.Equals(right);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(OutlineSegment left, OutlineSegment right) => !left.Equals(right);

    /// <inheritdoc/>
    public override string ToString() => Kind switch
    {
        OutlineSegmentKind.Line => $"line {Start} {End}",
        OutlineSegmentKind.Quadratic => $"quadratic {Start} {Control1} {End}",
        _ => $"cubic {Start} {Control1} {Control2} {End}",
    };
}

/// <summary>
/// A glyph's outline: closed contours of segments in the font's units, as the font draws them. TrueType's outer contours
/// run clockwise and CFF's counter-clockwise; a contour inside another, and opposite to it, is a hole.
/// </summary>
public sealed class GlyphOutline
{
    internal GlyphOutline(int glyph, ImmutableArray<Kerf.Engine.Fonts.Contour> contours)
    {
        Glyph = glyph;
        Raw = contours;
        Contours = contours.Select(c => c.Segments).ToImmutableArray();
    }

    /// <summary>The contours with what a single-stroke font needs: whether each was closed by a line.</summary>
    internal ImmutableArray<Kerf.Engine.Fonts.Contour> Raw { get; }

    /// <summary>The glyph's index in its font.</summary>
    public int Glyph { get; }

    /// <summary>The contours, each its segments in order; none for a blank glyph such as a space.</summary>
    public IReadOnlyList<ImmutableArray<OutlineSegment>> Contours { get; }
}
