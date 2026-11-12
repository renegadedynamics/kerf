// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// Tells a piece of a curve that lies beside a piece of a surface from one that may touch it, for the edge/face
/// intersector's localisation (Kerf's own: unit 15Y2, decision 32801).
/// </summary>
/// <remarks>
/// <para>
/// OCCT's localisation keeps a pair of samples while their boxes overlap, and the boxes lie along the axes. An edge that
/// runs beside a narrow face, askew to the axes, overlaps it box for box until the samples are as small as the gap, and
/// each pair kept costs an extremum (decision 2183). Here the two pieces are bounded along their own directions instead:
/// along a direction, a piece of the curve lies between its ends and a piece of the surface between its four corners,
/// each to within a bound on what the piece can bulge that way, and the two are apart if some direction separates them by
/// more than the clearance asked.
/// </para>
/// <para>
/// The bulge is proved, not estimated. Along a direction a, a curve over a range of h is within h*h/8 times the most of
/// |a.C''| of the line between its ends, and a surface within (hu*hu max|a.Suu| + hv*hv max|a.Svv|)/8 of the bilinear
/// patch on its corners, which lies between them. The second derivatives are bounded span by span: on a span a polynomial
/// curve's or surface's second derivative is a polynomial, so along a direction it's no more than the Lebesgue constant
/// of the Chebyshev nodes times its greatest value at them (under (2/pi) ln(n) + 1 for n nodes; Rivlin). The nodes are
/// each span's, kept for the run; where that bound is what stops a small piece of the surface being told apart (the span's
/// second derivative may be far above the piece's: a patch with a side that's a point bulges at its base, not by its
/// apex), the piece's own nodes are taken, which bound it as a span's bound the span. So this only
/// answers for lines, circles, ellipses and polynomial (not rational) Bezier and B-spline curves, on polynomial Bezier
/// and B-spline surfaces, and only for a piece with no knot inside it where the first derivative breaks; for anything
/// else (a periodic B-spline among them) it says "not apart", and the localisation goes on as OCCT's does. The spans are
/// the knots' own: an adaptor's intervals of continuity leave out the knots a spline of a high degree is smooth at.
/// </para>
/// <para>A working object for one run of the intersector (plan section 6): it holds what it has bounded so far.</para>
/// </remarks>
internal sealed class SampleSeparation
{
    // Pieces over more spans than this aren't bounded: they're the first, coarse samples, which nothing separates anyway
    private const int MostCurveSpans = 8;
    private const int MostSurfaceCells = 16;

    private readonly TransformedCurve _curve;
    private readonly TransformedSurface _surface;
    private readonly bool _usable;

    // The curve: one bound of |C''| for all of it (a line, a circle, an ellipse), or C'' at each span's nodes, found when
    // first asked for
    private readonly double _curveBound = -1.0;
    private readonly int _curveNodes;
    private readonly double[] _curveBreaks = [];
    private readonly double[] _curveKinks = [];
    private readonly double[]?[] _spans = [];

    // The surface: Suu and Svv at each cell's nodes
    private readonly int _uNodes;
    private readonly int _vNodes;
    private readonly double[] _uBreaks = [];
    private readonly double[] _vBreaks = [];
    private readonly double[] _uKinks = [];
    private readonly double[] _vKinks = [];
    private readonly double[]?[] _cells = [];

    // The piece being asked about
    private int _spanFrom;
    private int _spanTo;
    private int _uFrom;
    private int _uTo;
    private int _vFrom;
    private int _vTo;
    private double _curveFactor;
    private double _uFactor;
    private double _vFactor;
    private double _u0;
    private double _u1;
    private double _v0;
    private double _v1;
    private double[]? _own;

    /// <summary>For the curve and the surface of a run.</summary>
    public SampleSeparation(TransformedCurve curve, TransformedSurface surface)
    {
        _curve = curve;
        _surface = surface;

        bool curveOk;
        switch (curve.GetType())
        {
            case CurveKind.Line:
                _curveBound = 0.0;
                curveOk = true;
                break;
            case CurveKind.Circle:
                _curveBound = curve.Circle().Radius;
                curveOk = true;
                break;
            case CurveKind.Ellipse:
                _curveBound = curve.Ellipse().MajorRadius;
                curveOk = true;
                break;
            case CurveKind.BezierCurve when curve.Is3DCurve():
                _curveNodes = curve.Degree() + 1;
                _curveBreaks = [curve.FirstParameter(), curve.LastParameter()];
                curveOk = !curve.IsRational;
                break;
            case CurveKind.BSplineCurve when curve.GeomCurve() is BSplineCurve3D { IsPeriodic: false } spline:
                _curveNodes = spline.Degree + 1;
                Knots(spline.InternalKnots(), spline.InternalMultiplicities(), spline.Degree, out _curveBreaks, out _curveKinks);
                curveOk = !spline.IsRational;
                break;
            default:
                curveOk = false;
                break;
        }

        bool surfaceOk = false;
        if (curveOk && !surface.IsURational && !surface.IsVRational)
        {
            if (surface.GetType() == SurfaceKind.BezierSurface)
            {
                _uBreaks = [surface.FirstUParameter(), surface.LastUParameter()];
                _vBreaks = [surface.FirstVParameter(), surface.LastVParameter()];
                surfaceOk = true;
            }
            else if (surface.GetType() == SurfaceKind.BSplineSurface
                     && surface.GeomSurfaceOriginal() is BSplineSurface { IsUPeriodic: false, IsVPeriodic: false } spline)
            {
                Knots(spline.InternalUKnots(), spline.InternalUMultiplicities(), spline.UDegree, out _uBreaks, out _uKinks);
                Knots(spline.InternalVKnots(), spline.InternalVMultiplicities(), spline.VDegree, out _vBreaks, out _vKinks);
                surfaceOk = true;
            }
        }

        if (surfaceOk)
        {
            _uNodes = surface.UDegree() + 1;
            _vNodes = surface.VDegree() + 1;
        }

        _usable = curveOk && surfaceOk && _curveNodes <= 26 && _uNodes * _vNodes <= 400;
        if (_usable)
        {
            _spans = new double[]?[Math.Max(_curveBreaks.Length - 1, 0)];
            _cells = new double[]?[(_uBreaks.Length - 1) * (_vBreaks.Length - 1)];
        }
    }

    /// <summary>
    /// True if the curve between <paramref name="t0"/> and <paramref name="t1"/> is proved further than
    /// <paramref name="clearance"/> from the surface over the given ranges. False says nothing.
    /// </summary>
    public bool Apart(double t0, double t1, double u0, double u1, double v0, double v1, double clearance)
    {
        if (!_usable)
        {
            return false;
        }

        double h = t1 - t0, hu = u1 - u0, hv = v1 - v0;
        if (_curveBound < 0.0)
        {
            Spans(_curveBreaks, t0, t1, out _spanFrom, out _spanTo);
            if (_spanTo - _spanFrom > MostCurveSpans || HasKink(_curveKinks, t0, t1))
            {
                return false;
            }

            _curveFactor = Lebesgue(_curveNodes) * h * h / 8.0;
        }
        else
        {
            _curveFactor = _curveBound * h * h / 8.0;
        }

        Spans(_uBreaks, u0, u1, out _uFrom, out _uTo);
        Spans(_vBreaks, v0, v1, out _vFrom, out _vTo);
        if ((_uTo - _uFrom) * (_vTo - _vFrom) > MostSurfaceCells || HasKink(_uKinks, u0, u1) || HasKink(_vKinks, v0, v1))
        {
            return false;
        }

        double lebesgue = Lebesgue(_uNodes) * Lebesgue(_vNodes);
        _uFactor = lebesgue * hu * hu / 8.0;
        _vFactor = lebesgue * hv * hv / 8.0;

        _u0 = u0;
        _u1 = u1;
        _v0 = v0;
        _v1 = v1;
        _own = null;

        Point3D a = _curve.EvalD0(t0), b = _curve.EvalD0(t1);
        Point3D p00 = _surface.EvalD0(u0, v0), p10 = _surface.EvalD0(u1, v0);
        Point3D p01 = _surface.EvalD0(u0, v1), p11 = _surface.EvalD0(u1, v1);
        Span<double> q = [p00.X, p00.Y, p00.Z, p10.X, p10.Y, p10.Z, p01.X, p01.Y, p01.Z, p11.X, p11.Y, p11.Z];
        Span<double> s = [a.X, a.Y, a.Z, b.X, b.Y, b.Z];

        // Any direction may be tried: one that separates proves it. The patch's normal, the chord, and the directions
        // square to both, to the patch's sides in its own plane and to the chord and each side
        Span<double> d = [b.X - a.X, b.Y - a.Y, b.Z - a.Z];
        Span<double> n = stackalloc double[3];
        Cross([p11.X - p00.X, p11.Y - p00.Y, p11.Z - p00.Z], [p10.X - p01.X, p10.Y - p01.Y, p10.Z - p01.Z], n);
        Span<double> axis = stackalloc double[3];
        if (Separates(n, q, s, clearance) || Separates(d, q, s, clearance))
        {
            return true;
        }

        Cross(d, n, axis);
        if (Separates(axis, q, s, clearance))
        {
            return true;
        }

        Span<double> sides =
        [
            p10.X - p00.X, p10.Y - p00.Y, p10.Z - p00.Z, p01.X - p00.X, p01.Y - p00.Y, p01.Z - p00.Z,
            p11.X - p01.X, p11.Y - p01.Y, p11.Z - p01.Z, p11.X - p10.X, p11.Y - p10.Y, p11.Z - p10.Z,
        ];
        for (int k = 0; k < 4; k++)
        {
            Span<double> side = sides.Slice(3 * k, 3);
            Cross(side, n, axis);
            if (Separates(axis, q, s, clearance))
            {
                return true;
            }

            Cross(d, side, axis);
            if (Separates(axis, q, s, clearance))
            {
                return true;
            }
        }

        return false;
    }

    private static void Cross(ReadOnlySpan<double> a, ReadOnlySpan<double> b, Span<double> c)
    {
        c[0] = (a[1] * b[2]) - (a[2] * b[1]);
        c[1] = (a[2] * b[0]) - (a[0] * b[2]);
        c[2] = (a[0] * b[1]) - (a[1] * b[0]);
    }

    // True if the piece of the surface (its corners q) and of the curve (its ends s) are further apart along the axis than
    // the clearance and what the two can bulge along it
    private bool Separates(ReadOnlySpan<double> axis, ReadOnlySpan<double> q, ReadOnlySpan<double> s, double clearance)
    {
        double square = (axis[0] * axis[0]) + (axis[1] * axis[1]) + (axis[2] * axis[2]);
        if (!(square > 0.0))
        {
            return false;
        }

        double qMin = double.MaxValue, qMax = double.MinValue;
        for (int k = 0; k < 12; k += 3)
        {
            double at = (axis[0] * q[k]) + (axis[1] * q[k + 1]) + (axis[2] * q[k + 2]);
            qMin = Math.Min(qMin, at);
            qMax = Math.Max(qMax, at);
        }

        double s0 = (axis[0] * s[0]) + (axis[1] * s[1]) + (axis[2] * s[2]);
        double s1 = (axis[0] * s[3]) + (axis[1] * s[4]) + (axis[2] * s[5]);
        double gap = Math.Max(qMin - Math.Max(s0, s1), Math.Min(s0, s1) - qMax);

        // All along the axis as it is, not a unit one: the clearance by its length
        double length = Math.Sqrt(square);
        double reach = clearance * length;
        if (!(gap > reach))
        {
            return false;
        }

        reach += _curveFactor * (_curveBound < 0.0 ? MostAlong(axis, _spans, _spanFrom, _spanTo, 0, 0, 1, 0) : length);
        if (!(gap > reach))
        {
            return false;
        }

        int uCount = _uBreaks.Length - 1;
        double spans = (_uFactor * MostAlong(axis, _cells, _uFrom, _uTo, uCount, _vFrom, _vTo, 0))
                       + (_vFactor * MostAlong(axis, _cells, _uFrom, _uTo, uCount, _vFrom, _vTo, 3));
        if (gap > reach + spans)
        {
            return true;
        }

        // The piece's own nodes, where it lies in one span each way: a bound nearer the piece's own second derivative
        if (_uTo - _uFrom != 1 || _vTo - _vFrom != 1)
        {
            return false;
        }

        _own ??= Nodes(_u0, _u1, _v0, _v1);
        return gap > reach + (_uFactor * MostAlong(axis, _own, 0)) + (_vFactor * MostAlong(axis, _own, 3));
    }

    private static double MostAlong(ReadOnlySpan<double> axis, double[] nodes, int offset)
    {
        double most = 0.0;
        for (int k = offset; k < nodes.Length; k += 6)
        {
            most = Math.Max(most, Math.Abs((axis[0] * nodes[k]) + (axis[1] * nodes[k + 1]) + (axis[2] * nodes[k + 2])));
        }

        return most;
    }

    // The greatest of |axis . w| over the second derivatives w kept at the nodes of the spans or cells given (a cell keeps
    // Suu at an offset of 0 and Svv at 3 of each node's six numbers; a span keeps C'' alone)
    private double MostAlong(ReadOnlySpan<double> axis, double[]?[] kept, int from, int to, int stride, int rowFrom, int rowTo,
                             int offset)
    {
        bool surface = ReferenceEquals(kept, _cells);
        int width = surface ? 6 : 3;
        double most = 0.0;
        for (int row = rowFrom; row < rowTo; row++)
        {
            for (int i = from; i < to; i++)
            {
                double[] nodes = kept[(row * stride) + i] ??= surface ? CellNodes(i, row) : SpanNodes(i);
                for (int k = offset; k < nodes.Length; k += width)
                {
                    most = Math.Max(most, Math.Abs((axis[0] * nodes[k]) + (axis[1] * nodes[k + 1]) + (axis[2] * nodes[k + 2])));
                }
            }
        }

        return most;
    }

    private double[] SpanNodes(int i)
    {
        double[] nodes = new double[3 * _curveNodes];
        for (int k = 0; k < _curveNodes; k++)
        {
            Vector3D d2 = _curve.EvalD2(Node(_curveBreaks[i], _curveBreaks[i + 1], k, _curveNodes)).D2;
            nodes[3 * k] = d2.X;
            nodes[(3 * k) + 1] = d2.Y;
            nodes[(3 * k) + 2] = d2.Z;
        }

        return nodes;
    }

    private double[] CellNodes(int i, int j) => Nodes(_uBreaks[i], _uBreaks[i + 1], _vBreaks[j], _vBreaks[j + 1]);

    // Suu and Svv at the Chebyshev nodes of a range that lies in one span each way
    private double[] Nodes(double uFirst, double uLast, double vFirst, double vLast)
    {
        double[] nodes = new double[6 * _uNodes * _vNodes];
        int at = 0;
        for (int ku = 0; ku < _uNodes; ku++)
        {
            double u = Node(uFirst, uLast, ku, _uNodes);
            for (int kv = 0; kv < _vNodes; kv++)
            {
                Surface.ResD2 d2 = _surface.EvalD2(u, Node(vFirst, vLast, kv, _vNodes));
                nodes[at++] = d2.D2U.X;
                nodes[at++] = d2.D2U.Y;
                nodes[at++] = d2.D2U.Z;
                nodes[at++] = d2.D2V.X;
                nodes[at++] = d2.D2V.Y;
                nodes[at++] = d2.D2V.Z;
            }
        }

        return nodes;
    }

    // The Lebesgue constant of n Chebyshev nodes is under this (Rivlin, The Chebyshev Polynomials, 1974)
    private static double Lebesgue(int nodes) => ((2.0 / Math.PI) * Math.Log(nodes)) + 1.0;

    private static double Node(double first, double last, int k, int nodes) =>
        (0.5 * (first + last)) + (0.5 * (last - first) * Math.Cos(((2 * k) + 1) * Math.PI / (2 * nodes)));

    // The spans [from, to) of the breaks that the range first..last lies over
    private static void Spans(double[] breaks, double first, double last, out int from, out int to)
    {
        int last1 = breaks.Length - 2;
        from = 0;
        while (from < last1 && breaks[from + 1] <= first)
        {
            from++;
        }

        to = from + 1;
        while (to <= last1 && breaks[to] < last)
        {
            to++;
        }
    }

    // True if a knot where the first derivative breaks lies inside the range: within a rounding of an end it's no more
    // inside than the end is
    private static bool HasKink(double[] kinks, double first, double last)
    {
        foreach (double kink in kinks)
        {
            double slack = 1e-10 * (1.0 + Math.Abs(kink));
            if (kink > first + slack && kink < last - slack)
            {
                return true;
            }
        }

        return false;
    }

    // A spline's spans, and the knots between its ends where its first derivative breaks
    private static void Knots(NCollection_Array1<double> knots, NCollection_Array1<int> multiplicities, int degree,
                              out double[] breaks, out double[] kinks)
    {
        breaks = new double[knots.Length()];
        List<double> broken = [];
        for (int i = 0; i < breaks.Length; i++)
        {
            breaks[i] = knots.Value(knots.Lower() + i);
            if (i > 0 && i < breaks.Length - 1 && multiplicities.Value(multiplicities.Lower() + i) >= degree)
            {
                broken.Add(breaks[i]);
            }
        }

        kinks = [.. broken];
    }
}
