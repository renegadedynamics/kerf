// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Fonts;
using Kerf.Geometry;

namespace Kerf.Engine.Fonts;

/// <summary>
/// One contour's segments as a decomposition draws them: a move, lines and curves, and the close, which joins the last
/// point to the first with a line unless they are the same. A line of no length is left out, as FreeType's outlines leave
/// no such point.
/// </summary>
internal sealed class SegmentSink
{
    private readonly ImmutableArray<OutlineSegment>.Builder _segments = ImmutableArray.CreateBuilder<OutlineSegment>();
    private Point2D _start;
    private Point2D _current;

    public Point2D Current => _current;

    public void MoveTo(Point2D point)
    {
        _start = point;
        _current = point;
    }

    public void LineTo(Point2D point)
    {
        if (point != _current)
        {
            _segments.Add(new OutlineSegment(OutlineSegmentKind.Line, _current, _current, point, point));
        }

        _current = point;
    }

    public void QuadTo(Point2D control, Point2D point)
    {
        _segments.Add(new OutlineSegment(OutlineSegmentKind.Quadratic, _current, control, control, point));
        _current = point;
    }

    public void CubicTo(Point2D control1, Point2D control2, Point2D point)
    {
        _segments.Add(new OutlineSegment(OutlineSegmentKind.Cubic, _current, control1, control2, point));
        _current = point;
    }

    /// <summary>Closes the contour; true if that took a line back to its start (which a single-stroke font doesn't draw).</summary>
    public bool Close()
    {
        bool line = _current != _start;
        LineTo(_start);
        return line;
    }

    public ImmutableArray<OutlineSegment> ToSegments() => _segments.ToImmutable();

    /// <summary>Takes the segments drawn so far and starts again.</summary>
    public ImmutableArray<OutlineSegment> Drain()
    {
        ImmutableArray<OutlineSegment> segments = _segments.ToImmutable();
        _segments.Clear();
        return segments;
    }
}
