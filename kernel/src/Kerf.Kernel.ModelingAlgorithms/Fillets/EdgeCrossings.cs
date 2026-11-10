// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Bounds.Computation;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Fillets;

/// <summary>
/// The check behind "a fillet whose edges cross fails" (decision 2123; OCCT #1570): a result that passes the validity check
/// with two of its edges crossing, which only the Booleans' self-interference check would find. This asks the same
/// question of fewer pairs: only an edge of a face the operation made or trimmed can have been made to cross anything, so
/// those edges are checked against every edge of the result, boxes first, and the pairs of edges the operation left alone
/// aren't asked at all; it stops at the first crossing.
/// </summary>
/// <remarks>
/// A pair is asked as <c>BOPAlgo_CheckerSI</c>'s edge level asks it (<c>BOPAlgo_PaveFiller::PerformEE</c>): each edge on
/// its shrunk range (the part outside its vertices' tolerance spheres, as <c>IntTools_ShrunkRange</c> finds it), through
/// <c>IntTools_EdgeEdge</c>. A point the two have in common counts when its common range lies clear of both edges' ends:
/// one that reaches an end is the edges leaving a vertex together, within tolerance, or a vertex of one on the other,
/// which the validity check allows and the filler reports under other kinds. Two edges between the same vertices that
/// coincide along their length count too. An edge whose shrunk range is empty (a micro edge) is passed over, as the filler
/// passes it over.
/// </remarks>
internal static class EdgeCrossings
{
    /// <summary>
    /// The error if two edges of <paramref name="result"/> cross where the operation on <paramref name="input"/> could have
    /// made them (the edges of faces that aren't the input's), naming the edges, or null.
    /// </summary>
    /// <param name="input">The shape the operation was made on.</param>
    /// <param name="result">What it made.</param>
    /// <param name="what">What to call the result (<c>The fillet's result</c>).</param>
    /// <param name="code">The error's code.</param>
    public static Diagnostic? Problem(Shape input, Shape result, string what, DiagnosticCode code)
    {
        if (result.IsNull || input.IsNull)
        {
            return null;
        }

        Crossing? found;
        try
        {
            found = Find(input, result, null);
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            return Diagnostic.Error(DiagnosticCode.InvalidResult, $"{what} couldn't be checked for edges that cross: {FailureDiagnostics.PublicText(e)}");
        }

        if (found is not { } crossing)
        {
            return null;
        }

        Point3D p = crossing.Point;
        string where = crossing.Along
            ? "two of its edges run along each other"
            : FormattableString.Invariant($"two of its edges meet at ({p.X:G6}, {p.Y:G6}, {p.Z:G6})");
        return Diagnostic.Error(code, $"{what} isn't reliable: {where}.").WithShapes(crossing.Edge1, crossing.Edge2);
    }

    /// <summary>Two edges that cross, where, and at what parameter of each (<paramref name="Parameter1"/>,
    /// <paramref name="Parameter2"/>), or run along each other (<paramref name="Along"/>, with no parameters).</summary>
    internal readonly record struct Crossing(Edge Edge1, Edge Edge2, Point3D Point, bool Along, double Parameter1 = double.NaN, double Parameter2 = double.NaN);

    /// <summary>Every crossing <see cref="Problem"/> would find, for tests and probes; the check itself stops at the first.</summary>
    internal static IReadOnlyList<Crossing> All(Shape input, Shape result)
    {
        var all = new List<Crossing>();
        Find(input, result, all);
        return all;
    }

    /// <summary>Counted for tests (<c>EdgeCrossingsTests</c>), as the Booleans' point checks are: checks made, pairs whose boxes met, pairs settled by their tangents and boxes, and pairs asked the intersector.</summary>
    internal static int s_checks;

    internal static int s_boxPairs;

    internal static int s_settled;

    internal static int s_intersections;

    // An edge of the result with what the check needs of it, found when first asked
    private sealed class Checked(Edge edge, bool isNew)
    {
        public Edge Edge { get; } = edge;

        /// <summary>An edge of a face the operation made or trimmed.</summary>
        public bool IsNew { get; init; } = isNew;

        /// <summary>The edge's box, with its tolerance (null until asked).</summary>
        public Box? Box { get; set; }

        /// <summary>True once the shrunk range has been asked for; <see cref="Skipped"/> if it couldn't be found.</summary>
        public bool HasShrunk { get; set; }

        public bool Skipped { get; set; }

        /// <summary>The edge's range and its shrunk range.</summary>
        public double First { get; set; }

        public double Last { get; set; }

        public double ShrunkFirst { get; set; }

        public double ShrunkLast { get; set; }

        /// <summary>The box of the curve on the shrunk range, with the edge's tolerance.</summary>
        public Box ShrunkBox { get; set; }

        public Vertex FirstVertex { get; set; }

        public Vertex LastVertex { get; set; }

        /// <summary>The curve, once the shrunk range has been found.</summary>
        public EdgeCurveAdaptor? Curve { get; set; }

        /// <summary>The edge's own tolerance, and its vertices' as the shrunk range took them.</summary>
        public double Tolerance { get; set; }

        public double FirstTolerance { get; set; }

        public double LastTolerance { get; set; }

        /// <summary>What the curve does at each end of its shrunk range, once asked.</summary>
        public End? FromFirst { get; set; }

        public End? FromLast { get; set; }

        /// <summary>The edge's curve as stored, with its placement and range, once asked.</summary>
        public bool HasGeometry { get; set; }

        public Curve3D? Geometry { get; set; }

        public Transform3D Placement { get; set; }

        public double GeometryFirst { get; set; }

        public double GeometryLast { get; set; }
    }

    // The first crossing, or every one into all when it's given
    private static Crossing? Find(Shape input, Shape result, List<Crossing>? all)
    {
        s_checks++;

        // The faces the operation left alone are the input's own TShapes; every other face it made or trimmed
        var inputFaces = new HashSet<TShape>(ReferenceEqualityComparer.Instance);
        for (var faces = new Explorer(input, ShapeType.Face); faces.More(); faces.Next())
        {
            if (faces.Current().TShape() is { } tShape)
            {
                inputFaces.Add(tShape);
            }
        }

        var edges = new List<Checked>();
        var seen = new Dictionary<TShape, int>(ReferenceEqualityComparer.Instance);
        int newCount = 0;
        for (var faces = new Explorer(result, ShapeType.Face); faces.More(); faces.Next())
        {
            Shape face = faces.Current();
            bool isNew = face.TShape() is not { } tShape || !inputFaces.Contains(tShape);
            for (var exp = new Explorer(face, ShapeType.Edge); exp.More(); exp.Next())
            {
                Edge edge = ShapeCasts.Edge(exp.Current());
                if (edge.TShape() is not { } tEdge)
                {
                    continue;
                }
                if (seen.TryGetValue(tEdge, out int at))
                {
                    if (isNew && !edges[at].IsNew)
                    {
                        edges[at] = new Checked(edges[at].Edge, true);
                        newCount++;
                    }
                    continue;
                }
                seen.Add(tEdge, edges.Count);
                edges.Add(new Checked(edge, isNew));
                if (isNew)
                {
                    newCount++;
                }
            }
        }
        if (newCount == 0)
        {
            return null;
        }

        // The new edges' boxes together: an edge outside all of them meets none
        var reach = new Box();
        foreach (Checked c in edges)
        {
            if (c.IsNew && BoxOf(c) is { } box)
            {
                reach = reach.Added(box);
            }
        }
        if (reach.IsVoid)
        {
            return null;
        }

        for (int i = 0; i < edges.Count; i++)
        {
            Checked first = edges[i];
            if (!first.IsNew || BoxOf(first) is not { } box1)
            {
                continue;
            }
            for (int j = 0; j < edges.Count; j++)
            {
                Checked second = edges[j];
                // A pair of new edges is asked once; a pair the operation left alone was in the input as it is
                if (j == i || (second.IsNew && j < i))
                {
                    continue;
                }
                if (BoxOf(second) is not { } box2 || box1.IsOut(box2) || (!second.IsNew && reach.IsOut(box2)))
                {
                    continue;
                }
                s_boxPairs++;
                if (!Shrunk(first) || !Shrunk(second) || first.ShrunkBox.IsOut(second.ShrunkBox))
                {
                    continue;
                }
                if (OnlyAtSharedVertex(first, second))
                {
                    s_settled++;
                    continue;
                }
                if (Cross(first, second) is { } crossing)
                {
                    if (all is null)
                    {
                        return crossing;
                    }
                    all.Add(crossing);
                }
            }
        }
        return null;
    }

    // The edge's box with its tolerance, as the filler's data structure boxes an edge; null for an edge the filler passes over
    private static Box? BoxOf(Checked c)
    {
        if (c.Skipped)
        {
            return null;
        }
        if (c.Box is { } known)
        {
            return known;
        }
        if (Tool.Degenerated(c.Edge))
        {
            c.Skipped = true;
            return null;
        }
        var box = new Box();
        Bounds.Add(c.Edge, ref box, false);
        c.Box = box;
        return box;
    }

    // The edge's shrunk range, the part outside its vertices' tolerance spheres, and its box: false for an edge too short to have one
    private static bool Shrunk(Checked c)
    {
        if (c.Skipped)
        {
            return false;
        }
        if (c.HasShrunk)
        {
            return true;
        }
        c.HasShrunk = true;
        Tool.Range(c.Edge, out double t1, out double t2);
        Exploration.Vertices(c.Edge, out Vertex v1, out Vertex v2);
        if (v1.IsNull || v2.IsNull || t2 - t1 < Precision.PConfusion())
        {
            c.Skipped = true;
            return false;
        }

        // As IntTools_ShrunkRange::Perform, without the length it finds for the filler's splitting
        double tolE = Tool.Tolerance(c.Edge);
        double tolV1 = Math.Max(Tool.Tolerance(v1), tolE) + Precision.Confusion();
        double tolV2 = Math.Max(Tool.Tolerance(v2), tolE) + Precision.Confusion();
        var curve = new EdgeCurveAdaptor(c.Edge);
        double ts1 = t1, ts2 = t2;
        if (!BrepLibrary.FindValidRange(curve, tolE, t1, Tool.Pnt(v1), tolV1, t2, Tool.Pnt(v2), tolV2, ref ts1, ref ts2) || ts2 - ts1 < Precision.PConfusion())
        {
            c.Skipped = true;
            return false;
        }
        var box = new Box();
        Add3dCurve.Add(curve, ts1, ts2, tolE + Precision.Confusion(), ref box);
        c.First = t1;
        c.Last = t2;
        c.ShrunkFirst = ts1;
        c.ShrunkLast = ts2;
        c.ShrunkBox = box;
        c.FirstVertex = v1;
        c.LastVertex = v2;
        c.Curve = curve;
        c.Tolerance = tolE;
        c.FirstTolerance = tolV1;
        c.LastTolerance = tolV2;
        return true;
    }

    // Two edges that share a vertex lie within tolerance of each other beside it, which is no crossing (Cross passes such a
    // common part over, as it reaches the edges' ends), and that stretch is short: where their tangents there part at an
    // angle, the curves are apart past the tolerance over the sine of half the angle along each, to first order; where
    // they're tangent, past the square root of the tolerance over half the difference of their curvatures, to second order.
    // Each is cut back by twice that, where the model says they're four tolerances apart, and the boxes of what's left, in
    // the frame of the direction they part in, halved down as Apart does, settle the rest: boxes apart means nothing left
    // to find, and the pair is settled with no intersection. The model has to hold over the cut: each curve's curvature
    // (or, for tangents, the rate its curvature changes) times the cut is held to a fraction of the parting rate, and a
    // pair that bends more than that, or parts too slowly to tell, is boxed as it is and, if the boxes meet, asked in full.
    private static bool OnlyAtSharedVertex(Checked a, Checked b)
    {
        double a1 = a.ShrunkFirst, a2 = a.ShrunkLast, b1 = b.ShrunkFirst, b2 = b.ShrunkLast;
        double tau = a.Tolerance + b.Tolerance + Precision.Confusion();
        CoordinateSystem? frame = null;
        if (!Cut(a.FirstVertex, true) || !Cut(a.LastVertex, false))
        {
            return false;
        }

        // A pair with no vertex in common is looked at as it is, in the world's frame; halved down to quarters, or to
        // thirty-seconds where both boxes come cheap (a line's from its ends, a spline's from its poles)
        int depth = CheapToBox(a) && CheapToBox(b) ? 5 : 2;
        return Apart(a, a1, a2, b, b1, b2, frame ?? new CoordinateSystem(), 0.5 * tau, depth);

        // Cuts both edges back at a vertex they share, and settles the frame they part in there; false if the pair can't
        // be settled at this vertex
        bool Cut(Vertex v, bool aAtFirst)
        {
            bool bAtFirst;
            if (v.IsSame(b.FirstVertex))
            {
                bAtFirst = true;
            }
            else if (v.IsSame(b.LastVertex))
            {
                bAtFirst = false;
            }
            else
            {
                return true;
            }
            End ea = EndOf(a, aAtFirst), eb = EndOf(b, bAtFirst);
            if (ea.Speed < Kerf.Engine.Geometry.Constants.Resolution() || eb.Speed < Kerf.Engine.Geometry.Constants.Resolution())
            {
                return false;
            }
            Vector3D ta = ea.Heading.Divided(ea.Speed), tb = eb.Heading.Divided(eb.Speed);
            Vector3D parting = ta.Subtracted(tb);
            double halfSine = 0.5 * parting.Magnitude;
            double cut;
            Direction3D x, y;
            if (halfSine >= 0.005)
            {
                // Apart at the rate of twice the half-angle's sine along each: four tolerances at the cut
                cut = 2.0 * tau / halfSine;
                if (ea.Curvature * cut > 0.2 * halfSine || eb.Curvature * cut > 0.2 * halfSine)
                {
                    return false;
                }
                x = new Direction3D(parting);
                Vector3D mean = ta.Added(tb);
                Vector3D xv = new(x);
                y = mean.Magnitude > 1.0e-6 ? new Direction3D(mean.Subtracted(xv.Multiplied(mean.Dot(xv)))) : Perpendicular(x);
            }
            else
            {
                // Tangent: apart as half the difference of the curvatures times the square of the length along each
                Vector3D bend = ea.Bend.Subtracted(eb.Bend);
                double rate = bend.Magnitude;
                if (rate < 1.0e-12)
                {
                    return false;
                }
                cut = 2.0 * Math.Sqrt(2.0 * tau / rate);
                if (ea.Twist * cut > 0.2 * rate || eb.Twist * cut > 0.2 * rate)
                {
                    return false;
                }
                x = new Direction3D(bend);
                Vector3D xv = new(x);
                y = new Direction3D(ta.Subtracted(xv.Multiplied(ta.Dot(xv))));
            }
            double du = cut / ea.Speed, dv = cut / eb.Speed;
            if (aAtFirst)
            {
                a1 += du;
            }
            else
            {
                a2 -= du;
            }
            if (bAtFirst)
            {
                b1 += dv;
            }
            else
            {
                b2 -= dv;
            }
            if (a2 - a1 < Precision.PConfusion() || b2 - b1 < Precision.PConfusion())
            {
                return false;
            }
            frame ??= new CoordinateSystem(Tool.Pnt(v), x.Crossed(y), x);
            return true;
        }
    }

    private static bool CheapToBox(Checked c)
    {
        if (c.Curve!.GetType() == CurveKind.Line)
        {
            return true;
        }
        Geometry(c);
        return (c.Geometry as BSplineCurve3D ?? (c.Geometry as TrimmedCurve3D)?.BasisCurve as BSplineCurve3D) is { IsPeriodic: false };
    }

    // The edge's curve as stored, with its placement and range, once
    private static void Geometry(Checked c)
    {
        if (!c.HasGeometry)
        {
            c.HasGeometry = true;
            c.Geometry = Tool.Curve(c.Edge, out Placement placement, out double first, out double last);
            c.Placement = placement.Transformation;
            c.GeometryFirst = first;
            c.GeometryLast = last;
        }
    }

    // True if the boxes of the two curves' ranges, in the frame, are apart, or those of their halves, and so on down
    private static bool Apart(Checked a, double a1, double a2, Checked b, double b1, double b2, in CoordinateSystem frame, double tol, int depth)
    {
        if (BoxIn(a, a1, a2, frame, tol) is not { } boxA || BoxIn(b, b1, b2, frame, tol) is not { } boxB)
        {
            return false;
        }
        if (boxA.IsOut(boxB))
        {
            return true;
        }
        if (depth == 0)
        {
            return false;
        }
        double am = 0.5 * (a1 + a2), bm = 0.5 * (b1 + b2);
        return Apart(a, a1, am, b, b1, bm, frame, tol, depth - 1) && Apart(a, a1, am, b, bm, b2, frame, tol, depth - 1)
               && Apart(a, am, a2, b, b1, bm, frame, tol, depth - 1) && Apart(a, am, a2, b, bm, b2, frame, tol, depth - 1);
    }

    // A spline's box from its poles, with no copy of it: each span of the curve lies in the hull of its degree + 1 poles (its
    // weights are positive), so the poles of the spans that meet [u1, u2], in the frame, bound the curve there. Null for a
    // periodic spline, whose poles wrap, which is boxed as any other curve.
    private static Box? PoleBox(BSplineCurve3D spline, double u1, double u2, in Transform3D into, double tol)
    {
        if (spline.IsPeriodic)
        {
            return null;
        }
        NCollection_Array1<double> flat = spline.InternalKnotSequence();
        NCollection_Array1<Point3D> poles = spline.InternalPoles();
        int degree = spline.Degree;
        int count = poles.Upper() - poles.Lower() + 1;
        if (flat.Upper() - flat.Lower() + 1 != count + degree + 1)
        {
            return null;
        }
        if (count == degree + 1 && degree <= 32)
        {
            return BezierBox(spline, flat[flat.Lower()], flat[flat.Upper()], u1, u2, into, tol);
        }

        // Span k (1-based in the flat knots) runs from the kth flat knot to the next, for k from degree + 1 to the number
        // of poles, and is shaped by the poles k - degree to k
        int lowest = degree + 1, highest = count;
        int k1 = lowest, k2 = lowest;
        while (k1 < highest && flat[flat.Lower() + k1] <= u1)
        {
            k1++;
        }
        while (k2 < highest && flat[flat.Lower() + k2] < u2)
        {
            k2++;
        }
        var box = new Box();
        for (int i = k1 - degree; i <= k2; i++)
        {
            box = box.Added(poles[poles.Lower() + i - 1].Transformed(into));
        }
        return box.Enlarged(tol);
    }

    // A spline of one span is a Bezier curve, and the control points of its part [u1, u2] come from de Casteljau's
    // subdivision, in homogeneous coordinates where it's rational: their hull holds that part, and nothing more
    private static Box BezierBox(BSplineCurve3D spline, double t0, double t1, double u1, double u2, in Transform3D into, double tol)
    {
        NCollection_Array1<Point3D> poles = spline.InternalPoles();
        NCollection_Array1<double>? weights = spline.IsRational ? spline.InternalWeightsArray() : null;
        int n = spline.Degree;
        Span<double> x = stackalloc double[n + 1], y = stackalloc double[n + 1], z = stackalloc double[n + 1], w = stackalloc double[n + 1];
        for (int i = 0; i <= n; i++)
        {
            Point3D pole = poles[poles.Lower() + i];
            double weight = weights is null ? 1.0 : weights[weights.Lower() + i];
            x[i] = pole.X * weight;
            y[i] = pole.Y * weight;
            z[i] = pole.Z * weight;
            w[i] = weight;
        }
        double span = t1 - t0;
        double s2 = Math.Clamp((u2 - t0) / span, 0.0, 1.0), s1 = Math.Clamp((u1 - t0) / span, 0.0, s2);
        // The left part up to s2: the first point of each column
        if (s2 < 1.0)
        {
            for (int r = 1; r <= n; r++)
            {
                for (int i = n; i >= r; i--)
                {
                    x[i] = (1 - s2) * x[i - 1] + s2 * x[i];
                    y[i] = (1 - s2) * y[i - 1] + s2 * y[i];
                    z[i] = (1 - s2) * z[i - 1] + s2 * z[i];
                    w[i] = (1 - s2) * w[i - 1] + s2 * w[i];
                }
            }
        }
        // Then the right part of that from s1: the last point of each column
        double s = s2 > 0.0 ? s1 / s2 : 0.0;
        if (s > 0.0)
        {
            for (int r = 1; r <= n; r++)
            {
                for (int i = 0; i <= n - r; i++)
                {
                    x[i] = (1 - s) * x[i] + s * x[i + 1];
                    y[i] = (1 - s) * y[i] + s * y[i + 1];
                    z[i] = (1 - s) * z[i] + s * z[i + 1];
                    w[i] = (1 - s) * w[i] + s * w[i + 1];
                }
            }
        }
        var box = new Box();
        for (int i = 0; i <= n; i++)
        {
            box = box.Added(new Point3D(x[i] / w[i], y[i] / w[i], z[i] / w[i]).Transformed(into));
        }
        return box.Enlarged(tol);
    }

    private static Direction3D Perpendicular(in Direction3D d)
    {
        Vector3D v = new(d);
        Vector3D other = Math.Abs(d.X) < 0.5 ? new Vector3D(1, 0, 0) : new Vector3D(0, 1, 0);
        return new Direction3D(v.Crossed(other));
    }

    /// <summary>
    /// What the curve does at one end of its shrunk range: its first derivative into the edge and its speed, its curvature
    /// as a number and as a vector (<paramref name="Bend"/>), and how fast its second derivative changes along it, by
    /// length (<paramref name="Twist"/>), from its third.
    /// </summary>
    private readonly record struct End(Vector3D Heading, double Speed, double Curvature, Vector3D Bend, double Twist);

    private static End EndOf(Checked c, bool atFirst)
    {
        End? kept = atFirst ? c.FromFirst : c.FromLast;
        if (kept is { } known)
        {
            return known;
        }
        c.Curve!.D3(atFirst ? c.ShrunkFirst : c.ShrunkLast, out _, out Vector3D d1, out Vector3D d2, out Vector3D d3);
        double speed = d1.Magnitude;
        End end;
        if (speed < Kerf.Engine.Geometry.Constants.Resolution())
        {
            end = new End(d1, speed, double.PositiveInfinity, new Vector3D(0, 0, 0), double.PositiveInfinity);
        }
        else
        {
            // The curvature vector: the second derivative's part across the tangent, over the speed squared
            Vector3D t = d1.Divided(speed);
            Vector3D bend = d2.Subtracted(t.Multiplied(d2.Dot(t))).Divided(speed * speed);
            end = new End(atFirst ? d1 : d1.Reversed(), speed, bend.Magnitude, bend, d3.Magnitude / (speed * speed * speed));
        }
        if (atFirst)
        {
            c.FromFirst = end;
        }
        else
        {
            c.FromLast = end;
        }
        return end;
    }

    // The box of the curve on [u1, u2] in a frame, enlarged by tol; null for an edge without a curve
    private static Box? BoxIn(Checked c, double u1, double u2, in CoordinateSystem frame, double tol)
    {
        Geometry(c);
        if (c.Geometry is not { } geometry)
        {
            return null;
        }
        Transform3D into = Transform3D.Transformation(frame).Multiplied(c.Placement);
        if (c.Curve!.GetType() == CurveKind.Line)
        {
            // A line's box is its ends'
            Point3D p1 = geometry.Value(u1).Transformed(into), p2 = geometry.Value(u2).Transformed(into);
            var line = new Box();
            line = line.Added(p1).Added(p2);
            return line.Enlarged(tol);
        }
        BSplineCurve3D? spline = geometry as BSplineCurve3D ?? (geometry as TrimmedCurve3D)?.BasisCurve as BSplineCurve3D;
        if (spline is not null && PoleBox(spline, u1, u2, into, tol) is { } hull)
        {
            return hull;
        }
        var adaptor = new TransformedCurve(geometry, c.GeometryFirst, c.GeometryLast, into);
        Box box = new CurveBounds(adaptor).Box(u1, u2, tol);
        return box.IsVoid ? null : box;
    }

    // Where the two edges, on their shrunk ranges, have a point in common clear of their ends, or a length in common
    private static Crossing? Cross(Checked first, Checked second)
    {
        s_intersections++;
        Edge e1 = first.Edge, e2 = second.Edge;
        bool sameVertices = (first.FirstVertex.IsSame(second.FirstVertex) && first.LastVertex.IsSame(second.LastVertex))
                            || (first.FirstVertex.IsSame(second.LastVertex) && first.LastVertex.IsSame(second.FirstVertex));
        var edgeEdge = new EdgeEdge(e1, first.ShrunkFirst, first.ShrunkLast, e2, second.ShrunkFirst, second.ShrunkLast);
        edgeEdge.UseQuickCoincidenceCheck(sameVertices);
        edgeEdge.Perform();
        if (!edgeEdge.IsDone)
        {
            return null;
        }

        NCollection_Sequence<CommonPrt> parts = edgeEdge.CommonParts();
        int count = parts.Length();
        for (int i = 1; i <= count; i++)
        {
            CommonPrt part = parts[i];
            switch (part.Type())
            {
                case ShapeType.Vertex:
                {
                    // The filler's paves: the bands between each edge's range and its shrunk range, and the tolerance beyond
                    ParameterRange r1 = part.Range1(), r2 = part.Ranges2().Value(1);
                    if (OnPave(first, r1) || OnPave(second, r2))
                    {
                        continue;
                    }
                    IntersectionTools.VertexParameters(part, out double t1, out double t2);
                    AlgoTools.MakeNewVertex(e1, t1, e2, t2, out Vertex made);
                    return new Crossing(e1, e2, Tool.Pnt(made), false, t1, t2);
                }
                case ShapeType.Edge:
                {
                    // The filler takes a common length only of edges between the same vertices, found alone
                    if (count == 1 && sameVertices)
                    {
                        return new Crossing(e1, e2, Tool.Pnt(first.FirstVertex), true);
                    }
                    break;
                }
                default:
                    break;
            }
        }
        return null;
    }

    // True if the common range reaches either end of the edge's shrunk range, as the filler's IsOnPave1 reads it
    private static bool OnPave(Checked c, in ParameterRange common)
    {
        double tol = Precision.Confusion();
        var start = new ParameterRange(c.First, c.ShrunkFirst);
        var end = new ParameterRange(c.ShrunkLast, c.Last);
        return IntersectionTools.IsOnPave1(common.First(), start, tol) || IntersectionTools.IsOnPave1(common.Last(), start, tol)
               || IntersectionTools.IsOnPave1(common.First(), end, tol) || IntersectionTools.IsOnPave1(common.Last(), end, tol);
    }
}
