// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Fonts;
using Kerf.Geometry;

namespace Kerf.Engine.Fonts;

/// <summary>
/// OpenType's CFF outlines (the <c>CFF </c> table, Adobe's Compact Font Format 1 with Type 2 charstrings): the INDEXes and
/// DICTs read and checked when the font is read, each glyph's charstring run when it is asked for, with its local and
/// global subroutines. CID-keyed fonts choose a glyph's private DICT through FDSelect. Hints are read past, not applied.
/// Not read: CFF2 (variable fonts), and <c>seac</c>, the deprecated accent composition of <c>endchar</c>.
/// </summary>
internal sealed class CffGlyphs
{
    private const int MaxStack = 48;       // the Type 2 argument stack (Adobe's technical note 5177)
    private const int MaxSubrDepth = 10;   // the subroutine nesting the note allows
    private const int TransientSize = 32;

    private readonly CffIndex _charStrings;
    private readonly CffIndex _globalSubrs;
    private readonly CffIndex[] _localSubrs;     // per private DICT (one, or each FD of a CID font)
    private readonly FontData _fdSelect;         // empty for a font that isn't CID-keyed
    private readonly int _fdSelectFormat;
    private readonly int _fdCount;
    private readonly double[]? _matrix;          // the font matrix scaled by units per em, when it isn't the identity
    private readonly FontReadLimits _limits;

    private CffGlyphs(CffIndex charStrings, CffIndex globalSubrs, CffIndex[] localSubrs, FontData fdSelect, int fdSelectFormat,
        int fdCount, double[]? matrix, FontReadLimits limits)
    {
        _charStrings = charStrings;
        _globalSubrs = globalSubrs;
        _localSubrs = localSubrs;
        _fdSelect = fdSelect;
        _fdSelectFormat = fdSelectFormat;
        _fdCount = fdCount;
        _matrix = matrix;
        _limits = limits;
    }

    public int GlyphCount => _charStrings.Count;

    public static CffGlyphs Read(FontData cff, int unitsPerEm, FontReadLimits limits)
    {
        int major = cff.U8(0);
        if (major != 1)
        {
            throw new FontUnsupportedException($"its CFF table is version {major}, which isn't read (CFF2 is for variable fonts)");
        }

        int headerSize = cff.U8(2);
        CffIndex names = CffIndex.Read(cff, headerSize, "the CFF name INDEX");
        CffIndex topDicts = CffIndex.Read(cff, names.End, "the CFF top DICT INDEX");
        CffIndex strings = CffIndex.Read(cff, topDicts.End, "the CFF string INDEX");
        CffIndex globalSubrs = CffIndex.Read(cff, strings.End, "the CFF global subroutines");
        if (topDicts.Count < 1)
        {
            throw FontData.Malformed("its CFF table has no top DICT");
        }

        var top = CffDict.Read(topDicts.Item(0), "the CFF top DICT");
        int type = (int)top.Number(CffDict.Escape(6), 2);
        if (type != 2)
        {
            throw new FontUnsupportedException($"its CFF charstrings are type {type}, not type 2");
        }

        CffIndex charStrings = CffIndex.Read(cff, top.Offset(17, "the CFF charstrings"), "the CFF charstrings");
        double[]? matrix = null;
        if (top.TryGet(CffDict.Escape(7), out double[] m) && m.Length == 6)
        {
            double expected = 1.0 / unitsPerEm;
            if (m[0] != expected || m[1] != 0 || m[2] != 0 || m[3] != expected || m[4] != 0 || m[5] != 0)
            {
                matrix = [m[0] * unitsPerEm, m[1] * unitsPerEm, m[2] * unitsPerEm, m[3] * unitsPerEm, m[4] * unitsPerEm, m[5] * unitsPerEm];
            }
        }

        CffIndex[] locals;
        FontData fdSelect = default;
        int fdFormat = 0, fdCount = 0;
        if (top.TryGet(CffDict.Escape(36), out double[] fdArrayAt))
        {
            CffIndex fdArray = CffIndex.Read(cff, CffDict.ToOffset(fdArrayAt, "the CFF FDArray"), "the CFF FDArray");
            fdCount = fdArray.Count;
            locals = new CffIndex[fdCount];
            for (int i = 0; i < fdCount; i++)
            {
                var fd = CffDict.Read(fdArray.Item(i), "a CFF font DICT");
                locals[i] = LocalSubrs(cff, fd);
            }

            int fdSelectAt = top.Offset(CffDict.Escape(37), "the CFF FDSelect");
            fdSelect = cff.From(fdSelectAt, "the CFF FDSelect");
            fdFormat = fdSelect.U8(0);
            if (fdFormat == 0)
            {
                fdSelect.CheckArray(1, charStrings.Count, 1, "the CFF FDSelect");
            }
            else if (fdFormat == 3)
            {
                int ranges = fdSelect.U16(1);
                fdSelect.CheckArray(3, ranges, 3, "the CFF FDSelect's ranges");
                fdSelect.U16(3 + 3 * ranges); // the sentinel
                if (ranges == 0)
                {
                    throw FontData.Malformed("its CFF FDSelect has no ranges");
                }
            }
            else
            {
                throw FontData.Malformed($"its CFF FDSelect has format {fdFormat}");
            }
        }
        else
        {
            locals = [LocalSubrs(cff, top)];
        }

        return new CffGlyphs(charStrings, globalSubrs, locals, fdSelect, fdFormat, fdCount, matrix, limits);
    }

    private static CffIndex LocalSubrs(FontData cff, CffDict dict)
    {
        if (!dict.TryGet(18, out double[] priv) || priv.Length != 2)
        {
            return CffIndex.Empty;
        }

        int size = CffDict.ToOffset([priv[0]], "a CFF private DICT's size");
        int at = CffDict.ToOffset([priv[1]], "a CFF private DICT");
        var privateDict = CffDict.Read(cff.Slice(at, size, "a CFF private DICT"), "a CFF private DICT");
        if (!privateDict.TryGet(19, out double[] subrs))
        {
            return CffIndex.Empty;
        }

        long subrsAt = (long)at + CffDict.ToOffset(subrs, "a CFF private DICT's subroutines");
        return CffIndex.Read(cff, subrsAt, "a CFF private DICT's subroutines");
    }

    public ImmutableArray<Contour> Outline(int glyph)
    {
        if (glyph >= _charStrings.Count)
        {
            return [];
        }

        CffIndex locals = _localSubrs.Length == 1 ? _localSubrs[0] : _localSubrs[FontDictOf(glyph)];
        var run = new Interpreter(this, locals);
        run.Execute(_charStrings.Item(glyph), 0);
        run.Finish();
        ImmutableArray<Contour> contours = run.Contours.ToImmutable();
        return _matrix is null ? contours : Transformed(contours, _matrix);
    }

    private int FontDictOf(int glyph)
    {
        int fd;
        if (_fdSelectFormat == 0)
        {
            fd = _fdSelect.U8(1 + glyph);
        }
        else
        {
            int ranges = _fdSelect.U16(1);
            fd = -1;
            int lo = 0, hi = ranges - 1;
            while (lo <= hi)
            {
                int mid = (lo + hi) >>> 1;
                int first = _fdSelect.U16(3 + 3 * mid);
                int next = _fdSelect.U16(3 + 3 * (mid + 1));
                if (glyph < first)
                {
                    hi = mid - 1;
                }
                else if (glyph >= next)
                {
                    lo = mid + 1;
                }
                else
                {
                    fd = _fdSelect.U8(5 + 3 * mid);
                    break;
                }
            }
        }

        if (fd < 0 || fd >= _fdCount)
        {
            throw FontData.Malformed($"glyph {glyph}'s FDSelect entry ({fd}) isn't one of the {_fdCount} font DICTs");
        }

        return fd;
    }

    private static ImmutableArray<Contour> Transformed(ImmutableArray<Contour> contours, double[] m)
    {
        Point2D T(Point2D p) => new(m[0] * p.X + m[2] * p.Y + m[4], m[1] * p.X + m[3] * p.Y + m[5]);
        return [.. contours.Select(c => c with
        {
            Segments = [.. c.Segments.Select(s => new OutlineSegment(s.Kind, T(s.Start), T(s.Control1), T(s.Control2), T(s.End)))],
        })];
    }

    private static int Bias(int count) => count < 1240 ? 107 : count < 33900 ? 1131 : 32768;

    /// <summary>The Type 2 charstring interpreter for one glyph.</summary>
    private sealed class Interpreter(CffGlyphs font, CffIndex locals)
    {
        private readonly double[] _stack = new double[MaxStack];
        private readonly double[] _transient = new double[TransientSize];
        private readonly SegmentSink _sink = new();
        private int _sp;
        private int _stems;
        private bool _widthDone;
        private bool _open;
        private bool _ended;
        private int _operations;
        private double _x, _y;

        public ImmutableArray<Contour>.Builder Contours { get; } = ImmutableArray.CreateBuilder<Contour>();

        public void Finish() => CloseContour();

        public void Execute(FontData code, int depth)
        {
            if (depth > MaxSubrDepth)
            {
                throw FontData.Malformed($"its charstrings call subroutines deeper than {MaxSubrDepth}, as a loop would");
            }

            Cancellation.ThrowIfRequested();

            int at = 0;
            while (at < code.Length && !_ended)
            {
                if (++_operations > font._limits.MaxCharStringOperations)
                {
                    throw new FontLimitException(
                        $"a glyph's charstring runs more than {nameof(FontReadLimits.MaxCharStringOperations)} ({font._limits.MaxCharStringOperations}) operations");
                }

                int b0 = code.U8(at++);
                if (b0 >= 32 || b0 == 28)
                {
                    double value;
                    if (b0 == 28)
                    {
                        value = code.I16(at);
                        at += 2;
                    }
                    else if (b0 <= 246)
                    {
                        value = b0 - 139;
                    }
                    else if (b0 <= 250)
                    {
                        value = (b0 - 247) * 256 + code.U8(at++) + 108;
                    }
                    else if (b0 <= 254)
                    {
                        value = -(b0 - 251) * 256 - code.U8(at++) - 108;
                    }
                    else
                    {
                        value = code.I32(at) / 65536.0;
                        at += 4;
                    }

                    Push(value);
                    continue;
                }

                switch (b0)
                {
                    case 1: // hstem
                    case 3: // vstem
                    case 18: // hstemhm
                    case 23: // vstemhm
                        Width(_sp % 2 == 1);
                        _stems += _sp / 2;
                        _sp = 0;
                        break;
                    case 19: // hintmask
                    case 20: // cntrmask
                        Width(_sp % 2 == 1);
                        _stems += _sp / 2;
                        _sp = 0;
                        at += (_stems + 7) / 8;
                        break;
                    case 21: // rmoveto
                        Width(_sp > 2);
                        MoveTo(_x + Arg(0), _y + Arg(1));
                        break;
                    case 22: // hmoveto
                        Width(_sp > 1);
                        MoveTo(_x + Arg(0), _y);
                        break;
                    case 4: // vmoveto
                        Width(_sp > 1);
                        MoveTo(_x, _y + Arg(0));
                        break;
                    case 5: // rlineto
                        for (int i = 0; i + 1 < _sp; i += 2)
                        {
                            LineTo(_x + _stack[i], _y + _stack[i + 1]);
                        }

                        _sp = 0;
                        break;
                    case 6: // hlineto
                    case 7: // vlineto
                    {
                        bool horizontal = b0 == 6;
                        for (int i = 0; i < _sp; i++, horizontal = !horizontal)
                        {
                            if (horizontal)
                            {
                                LineTo(_x + _stack[i], _y);
                            }
                            else
                            {
                                LineTo(_x, _y + _stack[i]);
                            }
                        }

                        _sp = 0;
                        break;
                    }

                    case 8: // rrcurveto
                        for (int i = 0; i + 5 < _sp; i += 6)
                        {
                            Curve(i);
                        }

                        _sp = 0;
                        break;
                    case 24: // rcurveline
                    {
                        int i = 0;
                        for (; i + 7 < _sp; i += 6)
                        {
                            Curve(i);
                        }

                        if (i + 1 < _sp)
                        {
                            LineTo(_x + _stack[i], _y + _stack[i + 1]);
                        }

                        _sp = 0;
                        break;
                    }

                    case 25: // rlinecurve
                    {
                        int i = 0;
                        for (; i + 7 < _sp; i += 2)
                        {
                            LineTo(_x + _stack[i], _y + _stack[i + 1]);
                        }

                        if (i + 5 < _sp)
                        {
                            Curve(i);
                        }

                        _sp = 0;
                        break;
                    }

                    case 26: // vvcurveto
                    {
                        int i = 0;
                        double dx1 = 0;
                        if (_sp % 2 == 1)
                        {
                            dx1 = _stack[0];
                            i = 1;
                        }

                        for (; i + 3 < _sp; i += 4)
                        {
                            double c1x = _x + dx1, c1y = _y + _stack[i];
                            double c2x = c1x + _stack[i + 1], c2y = c1y + _stack[i + 2];
                            CurveTo(c1x, c1y, c2x, c2y, c2x, c2y + _stack[i + 3]);
                            dx1 = 0;
                        }

                        _sp = 0;
                        break;
                    }

                    case 27: // hhcurveto
                    {
                        int i = 0;
                        double dy1 = 0;
                        if (_sp % 2 == 1)
                        {
                            dy1 = _stack[0];
                            i = 1;
                        }

                        for (; i + 3 < _sp; i += 4)
                        {
                            double c1x = _x + _stack[i], c1y = _y + dy1;
                            double c2x = c1x + _stack[i + 1], c2y = c1y + _stack[i + 2];
                            CurveTo(c1x, c1y, c2x, c2y, c2x + _stack[i + 3], c2y);
                            dy1 = 0;
                        }

                        _sp = 0;
                        break;
                    }

                    case 30: // vhcurveto
                    case 31: // hvcurveto
                    {
                        bool horizontal = b0 == 31;
                        int i = 0;
                        while (i + 3 < _sp)
                        {
                            bool last = _sp - i == 5;
                            double c1x, c1y, c2x, c2y, ex, ey;
                            if (horizontal)
                            {
                                (c1x, c1y) = (_x + _stack[i], _y);
                                (c2x, c2y) = (c1x + _stack[i + 1], c1y + _stack[i + 2]);
                                (ex, ey) = (c2x + (last ? _stack[i + 4] : 0), c2y + _stack[i + 3]);
                            }
                            else
                            {
                                (c1x, c1y) = (_x, _y + _stack[i]);
                                (c2x, c2y) = (c1x + _stack[i + 1], c1y + _stack[i + 2]);
                                (ex, ey) = (c2x + _stack[i + 3], c2y + (last ? _stack[i + 4] : 0));
                            }

                            CurveTo(c1x, c1y, c2x, c2y, ex, ey);
                            i += last ? 5 : 4;
                            horizontal = !horizontal;
                        }

                        _sp = 0;
                        break;
                    }

                    case 10: // callsubr
                    case 29: // callgsubr
                    {
                        CffIndex subrs = b0 == 10 ? locals : font._globalSubrs;
                        int index = (int)Pop() + Bias(subrs.Count);
                        if (index < 0 || index >= subrs.Count)
                        {
                            throw FontData.Malformed($"a charstring calls subroutine {index} of {subrs.Count}");
                        }

                        Execute(subrs.Item(index), depth + 1);
                        break;
                    }

                    case 11: // return
                        return;
                    case 14: // endchar
                        Width(_sp == 1 || _sp == 5);
                        if (_sp >= 4)
                        {
                            throw new FontUnsupportedException("a glyph composes an accent with endchar's seac, which isn't read");
                        }

                        CloseContour();
                        _ended = true;
                        _sp = 0;
                        return;
                    case 12:
                        Escape(code.U8(at++));
                        break;
                    default:
                        throw FontData.Malformed($"a charstring has the reserved operator {b0}");
                }
            }
        }

        private void Escape(int op)
        {
            switch (op)
            {
                case 34: // hflex
                {
                    Need(7);
                    double y0 = _y;
                    double c1x = _x + _stack[0], c1y = _y;
                    double c2x = c1x + _stack[1], c2y = c1y + _stack[2];
                    double p3x = c2x + _stack[3], p3y = c2y;
                    CurveTo(c1x, c1y, c2x, c2y, p3x, p3y);
                    double c4x = p3x + _stack[4], c4y = p3y;
                    double c5x = c4x + _stack[5], c5y = y0;
                    CurveTo(c4x, c4y, c5x, c5y, c5x + _stack[6], y0);
                    break;
                }

                case 35: // flex
                    Need(13);
                    Curve(0);
                    Curve(6);
                    break;
                case 36: // hflex1
                {
                    Need(9);
                    double y0 = _y;
                    double c1x = _x + _stack[0], c1y = _y + _stack[1];
                    double c2x = c1x + _stack[2], c2y = c1y + _stack[3];
                    double p3x = c2x + _stack[4], p3y = c2y;
                    CurveTo(c1x, c1y, c2x, c2y, p3x, p3y);
                    double c4x = p3x + _stack[5], c4y = p3y;
                    double c5x = c4x + _stack[6], c5y = c4y + _stack[7];
                    CurveTo(c4x, c4y, c5x, c5y, c5x + _stack[8], y0);
                    break;
                }

                case 37: // flex1
                {
                    Need(11);
                    double x0 = _x, y0 = _y;
                    double c1x = _x + _stack[0], c1y = _y + _stack[1];
                    double c2x = c1x + _stack[2], c2y = c1y + _stack[3];
                    double p3x = c2x + _stack[4], p3y = c2y + _stack[5];
                    CurveTo(c1x, c1y, c2x, c2y, p3x, p3y);
                    double c4x = p3x + _stack[6], c4y = p3y + _stack[7];
                    double c5x = c4x + _stack[8], c5y = c4y + _stack[9];
                    double dx = c5x - x0, dy = c5y - y0;
                    if (Math.Abs(dx) > Math.Abs(dy))
                    {
                        CurveTo(c4x, c4y, c5x, c5y, c5x + _stack[10], y0);
                    }
                    else
                    {
                        CurveTo(c4x, c4y, c5x, c5y, x0, c5y + _stack[10]);
                    }

                    break;
                }

                case 3: Binary((a, b) => a != 0 && b != 0 ? 1 : 0); return; // and
                case 4: Binary((a, b) => a != 0 || b != 0 ? 1 : 0); return; // or
                case 5: Push(Pop() == 0 ? 1 : 0); return; // not
                case 9: Push(Math.Abs(Pop())); return; // abs
                case 10: Binary((a, b) => a + b); return; // add
                case 11: Binary((a, b) => a - b); return; // sub
                case 12: Binary((a, b) => b == 0 ? 0 : a / b); return; // div
                case 14: Push(-Pop()); return; // neg
                case 15: Binary((a, b) => a == b ? 1 : 0); return; // eq
                case 18: Pop(); return; // drop
                case 20: // put
                {
                    int i = (int)Pop();
                    double v = Pop();
                    if (i >= 0 && i < TransientSize)
                    {
                        _transient[i] = v;
                    }

                    return;
                }

                case 21: // get
                {
                    int i = (int)Pop();
                    Push(i >= 0 && i < TransientSize ? _transient[i] : 0);
                    return;
                }

                case 22: // ifelse
                {
                    double v2 = Pop(), v1 = Pop(), s2 = Pop(), s1 = Pop();
                    Push(v1 <= v2 ? s1 : s2);
                    return;
                }

                case 23: Push(0.5); return; // random: a fixed value, so a glyph is the same each time
                case 24: Binary((a, b) => a * b); return; // mul
                case 26: Push(Math.Sqrt(Math.Max(0, Pop()))); return; // sqrt
                case 27: { double v = Pop(); Push(v); Push(v); return; } // dup
                case 28: { double b = Pop(), a = Pop(); Push(b); Push(a); return; } // exch
                case 29: // index
                {
                    int i = (int)Pop();
                    if (i < 0)
                    {
                        i = 0;
                    }

                    if (i >= _sp)
                    {
                        throw FontData.Malformed("a charstring's index reaches past its stack");
                    }

                    Push(_stack[_sp - 1 - i]);
                    return;
                }

                case 30: // roll
                {
                    int j = (int)Pop(), n = (int)Pop();
                    if (n <= 0 || n > _sp)
                    {
                        throw FontData.Malformed("a charstring's roll reaches past its stack");
                    }

                    j = ((j % n) + n) % n;
                    _stack.AsSpan(_sp - n, n).Reverse();
                    _stack.AsSpan(_sp - n, j).Reverse();
                    _stack.AsSpan(_sp - n + j, n - j).Reverse();
                    return;
                }

                default:
                    throw FontData.Malformed($"a charstring has the reserved operator 12 {op}");
            }

            _sp = 0;
        }

        private void Need(int count)
        {
            if (_sp < count)
            {
                throw FontData.Malformed($"a charstring's flex has {_sp} arguments of {count}");
            }
        }

        private void Binary(Func<double, double, double> op)
        {
            double b = Pop(), a = Pop();
            Push(op(a, b));
        }

        private void Push(double value)
        {
            if (_sp >= MaxStack)
            {
                throw FontData.Malformed($"a charstring's stack holds more than {MaxStack} numbers");
            }

            _stack[_sp++] = value;
        }

        private double Pop() => _sp > 0 ? _stack[--_sp] : throw FontData.Malformed("a charstring's stack is empty where it needs a number");

        // The first stack-clearing operator may carry the glyph's width first, which isn't needed (hmtx gives it)
        private void Width(bool has)
        {
            if (!_widthDone && has)
            {
                Array.Copy(_stack, 1, _stack, 0, _sp - 1);
                _sp--;
            }

            _widthDone = true;
        }

        private double Arg(int i) => i < _sp ? _stack[i] : throw FontData.Malformed("a charstring's operator has too few arguments");

        private void MoveTo(double x, double y)
        {
            CloseContour();
            (_x, _y) = (x, y);
            _sink.MoveTo(new Point2D(x, y));
            _open = true;
            _sp = 0;
        }

        private void Open()
        {
            if (!_open)
            {
                _sink.MoveTo(new Point2D(_x, _y));
                _open = true;
            }
        }

        private void LineTo(double x, double y)
        {
            Open();
            (_x, _y) = (x, y);
            _sink.LineTo(new Point2D(x, y));
        }

        private void Curve(int i)
        {
            double c1x = _x + _stack[i], c1y = _y + _stack[i + 1];
            double c2x = c1x + _stack[i + 2], c2y = c1y + _stack[i + 3];
            CurveTo(c1x, c1y, c2x, c2y, c2x + _stack[i + 4], c2y + _stack[i + 5]);
        }

        private void CurveTo(double c1x, double c1y, double c2x, double c2y, double x, double y)
        {
            Open();
            (_x, _y) = (x, y);
            _sink.CubicTo(new Point2D(c1x, c1y), new Point2D(c2x, c2y), new Point2D(x, y));
        }

        private void CloseContour()
        {
            if (!_open)
            {
                return;
            }

            bool line = _sink.Close();
            ImmutableArray<OutlineSegment> segments = _sink.Drain();
            if (segments.Length > 0)
            {
                Contours.Add(new Contour(segments, line));
            }

            _open = false;
        }
    }
}

/// <summary>A CFF INDEX: a count of items and their offsets, checked to rise within the data when read.</summary>
internal sealed class CffIndex
{
    public static readonly CffIndex Empty = new(default, [1], 0);

    private readonly FontData _data;     // the items' data, from the byte before the first (offsets count from 1)
    private readonly int[] _offsets;

    private CffIndex(FontData data, int[] offsets, int end)
    {
        _data = data;
        _offsets = offsets;
        End = end;
    }

    public int Count => _offsets.Length - 1;

    /// <summary>Where the INDEX ends in the table it was read from.</summary>
    public int End { get; }

    public FontData Item(int i) => _data.Slice(_offsets[i], _offsets[i + 1] - _offsets[i], "a CFF INDEX item");

    public static CffIndex Read(FontData table, long at, string what)
    {
        if (at < 0 || at > table.Length)
        {
            throw FontData.Malformed($"{what} starts past the CFF table");
        }

        int start = (int)at;
        int count = table.U16(start);
        if (count == 0)
        {
            return new CffIndex(default, [1], start + 2);
        }

        int offSize = table.U8(start + 2);
        if (offSize is < 1 or > 4)
        {
            throw FontData.Malformed($"{what} has offsets of {offSize} bytes");
        }

        table.CheckArray(start + 3, count + 1L, offSize, what);
        var offsets = new int[count + 1];
        int previous = 1;
        for (int i = 0; i <= count; i++)
        {
            int p = start + 3 + i * offSize;
            long value = offSize switch
            {
                1 => table.U8(p),
                2 => table.U16(p),
                3 => table.U24(p),
                _ => table.U32(p),
            };
            if (value < previous || (i == 0 && value != 1) || value > int.MaxValue)
            {
                throw FontData.Malformed($"{what}'s offsets don't rise from 1");
            }

            offsets[i] = (int)value;
            previous = (int)value;
        }

        int dataStart = start + 3 + (count + 1) * offSize - 1; // offsets count from 1
        FontData data = table.Slice(dataStart, (long)offsets[count], what);
        return new CffIndex(data, offsets, dataStart + offsets[count]);
    }
}

/// <summary>A CFF DICT: each operator's operands.</summary>
internal sealed class CffDict
{
    private readonly Dictionary<int, double[]> _entries;

    private CffDict(Dictionary<int, double[]> entries) => _entries = entries;

    public static int Escape(int op) => 1200 + op;

    public bool TryGet(int op, out double[] operands) => _entries.TryGetValue(op, out operands!);

    public double Number(int op, double fallback) => _entries.TryGetValue(op, out double[]? v) && v.Length > 0 ? v[0] : fallback;

    public int Offset(int op, string what) =>
        _entries.TryGetValue(op, out double[]? v) ? ToOffset(v, what) : throw FontData.Malformed($"{what} is missing");

    public static int ToOffset(double[] operands, string what)
    {
        if (operands.Length < 1 || operands[^1] is < 0 or > int.MaxValue || operands[^1] != Math.Floor(operands[^1]))
        {
            throw FontData.Malformed($"{what} has no offset that is a whole number");
        }

        return (int)operands[^1];
    }

    public static CffDict Read(FontData data, string what)
    {
        var entries = new Dictionary<int, double[]>();
        var operands = new List<double>();
        int at = 0;
        while (at < data.Length)
        {
            int b0 = data.U8(at++);
            if (b0 <= 21)
            {
                int op = b0 == 12 ? Escape(data.U8(at++)) : b0;
                entries[op] = [.. operands];
                operands.Clear();
                continue;
            }

            if (operands.Count >= MaxOperands)
            {
                throw FontData.Malformed($"{what} has more than {MaxOperands} operands for one operator");
            }

            switch (b0)
            {
                case 28:
                    operands.Add(data.I16(at));
                    at += 2;
                    break;
                case 29:
                    operands.Add(data.I32(at));
                    at += 4;
                    break;
                case 30:
                    operands.Add(Real(data, ref at));
                    break;
                case >= 32 and <= 246:
                    operands.Add(b0 - 139);
                    break;
                case >= 247 and <= 250:
                    operands.Add((b0 - 247) * 256 + data.U8(at++) + 108);
                    break;
                case >= 251 and <= 254:
                    operands.Add(-(b0 - 251) * 256 - data.U8(at++) - 108);
                    break;
                default:
                    throw FontData.Malformed($"{what} has the reserved byte {b0}");
            }
        }

        return new CffDict(entries);
    }

    private const int MaxOperands = 48;

    private static double Real(FontData data, ref int at)
    {
        var text = new System.Text.StringBuilder();
        while (true)
        {
            int b = data.U8(at++);
            for (int half = 0; half < 2; half++)
            {
                int nibble = half == 0 ? b >> 4 : b & 0xF;
                switch (nibble)
                {
                    case <= 9:
                        text.Append((char)('0' + nibble));
                        break;
                    case 0xA:
                        text.Append('.');
                        break;
                    case 0xB:
                        text.Append('E');
                        break;
                    case 0xC:
                        text.Append("E-");
                        break;
                    case 0xE:
                        text.Append('-');
                        break;
                    case 0xF:
                        return double.TryParse(text.ToString(), System.Globalization.NumberStyles.Float,
                            System.Globalization.CultureInfo.InvariantCulture, out double value) && double.IsFinite(value) ? value : 0;
                    default:
                        throw FontData.Malformed("a CFF DICT has a real number with a reserved nibble");
                }

                if (text.Length > 64)
                {
                    throw FontData.Malformed("a CFF DICT has a real number longer than any");
                }
            }
        }
    }
}
