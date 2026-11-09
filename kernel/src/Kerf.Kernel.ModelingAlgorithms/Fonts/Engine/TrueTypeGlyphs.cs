// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Fonts;
using Kerf.Geometry;

namespace Kerf.Engine.Fonts;

/// <summary>
/// TrueType's outlines: the <c>glyf</c> table through <c>loca</c>, simple glyphs of quadratic contours and composite glyphs
/// of transformed components, without hinting (as OCCT loads them, <c>FT_LOAD_NO_HINTING</c>). A contour is decomposed as
/// FreeType's <c>FT_Outline_Decompose</c> decomposes it: two off-curve points in a row imply an on-curve point midway, and
/// a contour starting off the curve starts at its last point, or midway to it.
/// </summary>
internal sealed class TrueTypeGlyphs
{
    private const int FlagOnCurve = 0x01, FlagXShort = 0x02, FlagYShort = 0x04, FlagRepeat = 0x08, FlagXSame = 0x10, FlagYSame = 0x20;
    private const int ArgsAreWords = 0x0001, ArgsAreXY = 0x0002, HaveScale = 0x0008, MoreComponents = 0x0020;
    private const int HaveXYScale = 0x0040, HaveTwoByTwo = 0x0080, UseMyMetrics = 0x0200, ScaledOffset = 0x0800, UnscaledOffset = 0x1000;

    private readonly FontData _glyf;
    private readonly FontData _loca;
    private readonly bool _longOffsets;
    private readonly int _glyphCount;
    private readonly FontReadLimits _limits;
    private readonly Func<int, int?> _leftSideBearing;

    public TrueTypeGlyphs(FontData glyf, FontData loca, bool longOffsets, int glyphCount, FontReadLimits limits, Func<int, int?> leftSideBearing)
    {
        _leftSideBearing = leftSideBearing;
        _glyf = glyf;
        _loca = loca;
        _longOffsets = longOffsets;
        _limits = limits;
        // A loca shorter than the glyphs (which FreeType accepts) leaves the rest without outlines
        _glyphCount = Math.Min(glyphCount, Math.Max(0, loca.Length / (longOffsets ? 4 : 2) - 1));
    }

    public ImmutableArray<Contour> Outline(int glyph)
    {
        var points = new PointList();
        // As FreeType places a glyph: its left side bearing point (its first phantom point, xMin less the bearing hmtx gives,
        // or a USE_MY_METRICS component's) at the origin, which moves it where a font's bearing and bounding box disagree
        // (Arial's napostrophe, by one unit)
        double origin = Load(glyph, points, 0);
        points.Translate(0, -origin, 0);
        return points.Decompose();
    }

    private FontData GlyphData(int glyph)
    {
        if (glyph >= _glyphCount)
        {
            return default;
        }

        long start = _longOffsets ? _loca.U32(4 * glyph) : 2L * _loca.U16(2 * glyph);
        long end = _longOffsets ? _loca.U32(4 * glyph + 4) : 2L * _loca.U16(2 * glyph + 2);
        if (end < start)
        {
            throw FontData.Malformed($"glyph {glyph}'s location ends ({end}) before it starts ({start})");
        }

        return _glyf.Slice(start, end - start, $"glyph {glyph}");
    }

    // Loads the glyph's points, and returns the x of its first phantom point (FreeType's pp1)
    private double Load(int glyph, PointList points, int depth)
    {
        if (depth > _limits.MaxCompositeDepth)
        {
            throw new FontLimitException(
                $"its composite glyphs nest deeper than {nameof(FontReadLimits.MaxCompositeDepth)} ({_limits.MaxCompositeDepth}), as a loop of components would");
        }

        Cancellation.ThrowIfRequested();
        System.Runtime.CompilerServices.RuntimeHelpers.EnsureSufficientExecutionStack();
        points.CountLoad(_limits);
        FontData data = GlyphData(glyph);
        if (data.Length == 0)
        {
            return 0; // a blank glyph
        }

        int contours = data.I16(0);
        if (contours >= 0)
        {
            LoadSimple(glyph, data, contours, points);
            return Origin(glyph, data);
        }

        return LoadComposite(glyph, data, points, depth);
    }

    private double Origin(int glyph, FontData data) => _leftSideBearing(glyph) is int bearing ? data.I16(2) - bearing : 0;

    private void LoadSimple(int glyph, FontData data, int contours, PointList points)
    {
        data.CheckArray(10, contours, 2, $"glyph {glyph}'s contour ends");
        int count = 0, previous = -1;
        for (int c = 0; c < contours; c++)
        {
            int end = data.U16(10 + 2 * c);
            if (end <= previous)
            {
                throw FontData.Malformed($"glyph {glyph}'s contours aren't in order");
            }

            previous = end;
            count = end + 1;
        }

        points.Reserve(count, _limits);
        int at = 10 + 2 * contours;
        int instructions = data.U16(at);
        at += 2 + instructions;
        var flags = new byte[count];
        for (int i = 0; i < count;)
        {
            byte flag = data.U8(at++);
            flags[i++] = flag;
            if ((flag & FlagRepeat) != 0)
            {
                int repeat = data.U8(at++);
                if (repeat > count - i)
                {
                    throw FontData.Malformed($"glyph {glyph}'s flags repeat past its points");
                }

                flags.AsSpan(i, repeat).Fill(flag);
                i += repeat;
            }
        }

        var xs = new int[count];
        int x = 0;
        for (int i = 0; i < count; i++)
        {
            int flag = flags[i];
            if ((flag & FlagXShort) != 0)
            {
                int d = data.U8(at++);
                x += (flag & FlagXSame) != 0 ? d : -d;
            }
            else if ((flag & FlagXSame) == 0)
            {
                x += data.I16(at);
                at += 2;
            }

            xs[i] = x;
        }

        int y = 0;
        int start = points.Count;
        for (int i = 0; i < count; i++)
        {
            int flag = flags[i];
            if ((flag & FlagYShort) != 0)
            {
                int d = data.U8(at++);
                y += (flag & FlagYSame) != 0 ? d : -d;
            }
            else if ((flag & FlagYSame) == 0)
            {
                y += data.I16(at);
                at += 2;
            }

            points.Add(xs[i], y, (flags[i] & FlagOnCurve) != 0);
        }

        for (int c = 0; c < contours; c++)
        {
            points.EndContour(start + data.U16(10 + 2 * c));
        }
    }

    private double LoadComposite(int glyph, FontData data, PointList points, int depth)
    {
        double origin = Origin(glyph, data);
        int at = 10;
        int flags;
        int compositeStart = points.Count;
        do
        {
            flags = data.U16(at);
            int child = data.U16(at + 2);
            at += 4;
            int arg1, arg2;
            if ((flags & ArgsAreWords) != 0)
            {
                arg1 = (flags & ArgsAreXY) != 0 ? data.I16(at) : data.U16(at);
                arg2 = (flags & ArgsAreXY) != 0 ? data.I16(at + 2) : data.U16(at + 2);
                at += 4;
            }
            else
            {
                arg1 = (flags & ArgsAreXY) != 0 ? data.I8(at) : data.U8(at);
                arg2 = (flags & ArgsAreXY) != 0 ? data.I8(at + 1) : data.U8(at + 1);
                at += 2;
            }

            double xx = 1, yx = 0, xy = 0, yy = 1;
            bool scaled = true;
            if ((flags & HaveScale) != 0)
            {
                xx = yy = data.F2Dot14(at);
                at += 2;
            }
            else if ((flags & HaveXYScale) != 0)
            {
                xx = data.F2Dot14(at);
                yy = data.F2Dot14(at + 2);
                at += 4;
            }
            else if ((flags & HaveTwoByTwo) != 0)
            {
                xx = data.F2Dot14(at);
                yx = data.F2Dot14(at + 2);
                xy = data.F2Dot14(at + 4);
                yy = data.F2Dot14(at + 6);
                at += 8;
            }
            else
            {
                scaled = false;
            }

            int first = points.Count;
            double childOrigin = Load(child, points, depth + 1);
            if (scaled)
            {
                points.Transform(first, xx, xy, yx, yy);
            }

            double dx, dy;
            if ((flags & ArgsAreXY) != 0)
            {
                (dx, dy) = (arg1, arg2);
                // FreeType's default: the offset is scaled only when the component says so
                if (scaled && (flags & ScaledOffset) != 0 && (flags & UnscaledOffset) == 0)
                {
                    dx *= Math.Sqrt(xx * xx + xy * xy);
                    dy *= Math.Sqrt(yy * yy + yx * yx);
                }
            }
            else
            {
                // Matching points, as FreeType matches them: the composite's point arg1 (of its components so far) meets the
                // component's point arg2
                int parent = compositeStart + arg1, own = first + arg2;
                if (parent >= first || own >= points.Count)
                {
                    throw FontData.Malformed($"a component matches point {arg1} to {arg2}, past the points it has");
                }

                (dx, dy) = (points.X(parent) - points.X(own), points.Y(parent) - points.Y(own));
            }

            points.Translate(first, dx, dy);
            // The component's phantom points stand for the composite's, untransformed, as FreeType keeps them
            if ((flags & UseMyMetrics) != 0)
            {
                origin = childOrigin;
            }
        }
        while ((flags & MoreComponents) != 0);

        return origin;
    }

    /// <summary>The points of a glyph as they load, with the ends of its contours, then decomposed into segments.</summary>
    private sealed class PointList
    {
        private readonly List<double> _x = [];
        private readonly List<double> _y = [];
        private readonly List<bool> _on = [];
        private readonly List<int> _ends = [];

        public int Count => _x.Count;

        public int ContourCount => _ends.Count;

        public double X(int i) => _x[i];

        public double Y(int i) => _y[i];

        public void Reserve(int more, FontReadLimits limits)
        {
            if ((long)_x.Count + more > limits.MaxOutlinePoints)
            {
                throw new FontLimitException(
                    $"a glyph's outline has more than {nameof(FontReadLimits.MaxOutlinePoints)} ({limits.MaxOutlinePoints}) points with its components");
            }
        }

        private int _loads;

        // Each glyph loaded counts as a point, so that components of blank glyphs can't multiply without end
        public void CountLoad(FontReadLimits limits)
        {
            if (++_loads > limits.MaxOutlinePoints)
            {
                throw new FontLimitException(
                    $"a glyph has more than {nameof(FontReadLimits.MaxOutlinePoints)} ({limits.MaxOutlinePoints}) components");
            }
        }

        public void Add(double x, double y, bool on)
        {
            _x.Add(x);
            _y.Add(y);
            _on.Add(on);
        }

        public void EndContour(int last) => _ends.Add(last);

        public void Transform(int from, double xx, double xy, double yx, double yy)
        {
            for (int i = from; i < _x.Count; i++)
            {
                double x = _x[i], y = _y[i];
                _x[i] = xx * x + xy * y;
                _y[i] = yx * x + yy * y;
            }
        }

        public void Translate(int from, double dx, double dy)
        {
            if (dx == 0 && dy == 0)
            {
                return;
            }

            for (int i = from; i < _x.Count; i++)
            {
                _x[i] += dx;
                _y[i] += dy;
            }
        }

        public ImmutableArray<Contour> Decompose()
        {
            var contours = ImmutableArray.CreateBuilder<Contour>(_ends.Count);
            int first = 0;
            foreach (int last in _ends)
            {
                Contour contour = DecomposeContour(first, last);
                if (contour.Segments.Length > 0)
                {
                    contours.Add(contour);
                }

                first = last + 1;
            }

            return contours.ToImmutable();
        }

        private Point2D P(int i) => new(_x[i], _y[i]);

        private static Point2D Mid(Point2D a, Point2D b) => new((a.X + b.X) / 2, (a.Y + b.Y) / 2);

        // FT_Outline_Decompose for a contour of on-curve and quadratic off-curve points
        private Contour DecomposeContour(int first, int last)
        {
            var sink = new SegmentSink();
            Point2D start = P(first);
            int point = first;
            int limit = last;
            if (!_on[first])
            {
                if (_on[last])
                {
                    start = P(last);
                    limit--;
                }
                else
                {
                    start = Mid(start, P(last));
                }

                point--;
            }

            sink.MoveTo(start);
            while (point < limit)
            {
                point++;
                if (_on[point])
                {
                    sink.LineTo(P(point));
                    continue;
                }

                Point2D control = P(point);
                bool closed = false;
                while (true)
                {
                    if (point < limit)
                    {
                        point++;
                        Point2D next = P(point);
                        if (_on[point])
                        {
                            sink.QuadTo(control, next);
                            break;
                        }

                        sink.QuadTo(control, Mid(control, next));
                        control = next;
                        continue;
                    }

                    sink.QuadTo(control, start);
                    closed = true;
                    break;
                }

                if (closed)
                {
                    break;
                }
            }

            bool line = sink.Close();
            return new Contour(sink.ToSegments(), line);
        }
    }
}
