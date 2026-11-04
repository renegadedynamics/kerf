using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Constraints;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Occt;

namespace Kerf.Modeling;

/// <summary>
/// A target for a constrained arc or line: an edge (or axis) to be tangent to, with an optional
/// <see cref="Kerf.Modeling.Tangency"/> qualifier, or a point to pass through.
/// </summary>
public readonly record struct Tangent
{
    private Tangent(Edge? edge, Axis? axis, Vector? point, Tangency tangency) => (Edge, Axis, Point, Tangency) = (edge, axis, point, tangency);

    /// <summary>The edge to be tangent to.</summary>
    public Edge? Edge { get; }

    /// <summary>The axis to be tangent to (or, for an oriented line, to measure from).</summary>
    public Axis? Axis { get; }

    /// <summary>The point to pass through.</summary>
    public Vector? Point { get; }

    /// <summary>Which side of an edge the solution lies on.</summary>
    public Tangency Tangency { get; }

#pragma warning disable CS1591 // Conversions from each kind of target
    public static implicit operator Tangent(Edge edge) => new(edge, null, null, Tangency.Unqualified);
    public static implicit operator Tangent(Axis axis) => new(null, axis, null, Tangency.Unqualified);
    public static implicit operator Tangent(Vector point) => new(null, null, point, Tangency.Unqualified);
    public static implicit operator Tangent(Vertex vertex) => new(null, null, vertex.ToVector(), Tangency.Unqualified);
    public static implicit operator Tangent((double X, double Y) point) => new(null, null, (Vector)point, Tangency.Unqualified);
    public static implicit operator Tangent((double X, double Y, double Z) point) => new(null, null, (Vector)point, Tangency.Unqualified);
    public static implicit operator Tangent((Edge Edge, Tangency Tangency) t) => new(t.Edge, null, null, t.Tangency);
    public static implicit operator Tangent((Axis Axis, Tangency Tangency) t) => new(null, t.Axis, null, t.Tangency);
#pragma warning restore CS1591

    /// <summary>The target with an axis made into an edge.</summary>
    internal Tangent Resolved() => Axis is { } a ? new Tangent(Kerf.Modeling.Edge.FromAxis(a), null, null, Tangency) : this;

    internal bool IsPoint => Point is not null;
}

public partial class Edge
{
    /// <summary>
    /// The planar circular arcs on XY of <paramref name="radius"/> tangent to two curves or passing through points: of each
    /// circle, the short arc, the long one or both between the two tangency points.
    /// </summary>
    /// <exception cref="ArgumentException">The radius isn't positive.</exception>
    /// <exception cref="InvalidOperationException">No tangent arc exists.</exception>
    public static ShapeList<Edge> MakeConstrainedArcs(Tangent tangencyOne, Tangent tangencyTwo, double radius, Sagitta sagitta = Sagitta.Short)
    {
        if (radius <= 0)
            throw new ArgumentException("radius must be > 0.0");
        return ConstrainedSolvers.TwoTanRadArcs(Sorted(tangencyOne, tangencyTwo), radius, sagitta);
    }

    /// <summary>
    /// The planar circular arcs on XY tangent to two objects whose centre lies on a curve (or axis): build123d's
    /// <c>make_constrained_arcs</c> with <c>center_on</c>.
    /// </summary>
    public static ShapeList<Edge> MakeConstrainedArcsCenteredOn(Tangent tangencyOne, Tangent tangencyTwo, Tangent centerOn,
                                                                Sagitta sagitta = Sagitta.Short) =>
        ConstrainedSolvers.TwoTanOnArcs(Sorted(tangencyOne, tangencyTwo), centerOn.Resolved(), sagitta);

    /// <summary>The planar circular arcs on XY tangent to three curves, between the tangency points of the first two.</summary>
    public static ShapeList<Edge> MakeConstrainedArcs(Tangent tangencyOne, Tangent tangencyTwo, Tangent tangencyThree, Sagitta sagitta = Sagitta.Short) =>
        ConstrainedSolvers.ThreeTanArcs(Sorted(tangencyOne, tangencyTwo, tangencyThree), sagitta);

    /// <summary>The planar circles on XY with a fixed centre tangent to a curve or passing through a point.</summary>
    public static ShapeList<Edge> MakeConstrainedArcs(Tangent tangencyOne, Vector center) =>
        ConstrainedSolvers.TanCenArcs(tangencyOne.Resolved(), center);

    /// <summary>The planar circles on XY of a radius, tangent to a curve or through a point, whose centre lies on a curve.</summary>
    /// <exception cref="ArgumentException">The radius isn't positive.</exception>
    public static ShapeList<Edge> MakeConstrainedArcs(Tangent tangencyOne, double radius, Tangent centerOn)
    {
        if (radius <= 0)
            throw new ArgumentException("radius must be > 0.0");
        return ConstrainedSolvers.TanOnRadArcs(tangencyOne.Resolved(), centerOn.Resolved(), radius);
    }

    /// <summary>The planar lines on XY tangent to two curves, or to a curve and through a point.</summary>
    public static ShapeList<Edge> MakeConstrainedLines(Tangent tangencyOne, Tangent tangencyTwo)
    {
        var sorted = Sorted(tangencyOne, tangencyTwo);
        if (sorted[0].IsPoint)
            throw new ArgumentException("Internal error - 1st tangency can't be an Axis | Vector");
        return ConstrainedSolvers.TwoTanLines(sorted[0], sorted[1]);
    }

    /// <summary>The planar lines on XY tangent to a curve at <paramref name="angle"/> degrees from a reference axis.</summary>
    public static ShapeList<Edge> MakeConstrainedLines(Tangent tangencyOne, Axis reference, double angle) =>
        ConstrainedSolvers.TanOrientedLines(tangencyOne.Resolved(), reference, angle * Constants.DegToRad);

    /// <summary>The planar lines on XY tangent to a curve in a direction (its X and Y), measured from a reference axis.</summary>
    public static ShapeList<Edge> MakeConstrainedLines(Tangent tangencyOne, Axis reference, Vector direction) =>
        ConstrainedSolvers.TanOrientedLines(tangencyOne.Resolved(), reference, Math.Atan2(direction.Y, direction.X));

    /// <summary>The targets with axes made into edges, points last (a stable sort, as build123d's).</summary>
    private static List<Tangent> Sorted(params Tangent[] tangents) => [.. tangents.Select(t => t.Resolved()).OrderBy(t => t.IsPoint)];
}

/// <summary>The constrained arc and line solvers of build123d's topology/constrained_lines.py, over OCCT's Geom2dGcc.</summary>
internal static class ConstrainedSolvers
{
    // The XY plane for 3D to 2D projection and for building edges from 2D curves
    private static readonly Plane3D PlnXY = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
    private static readonly PlaneSurface SurfXY = new(PlnXY);

    /// <summary>A target as a GCC argument: a qualified curve (with its 2D curve and range), or a point.</summary>
    private sealed record GccArg(QualifiedCurve? Curve, CartesianPoint2? Point, Curve2D? H2d, double First, double Last)
    {
        public bool IsEdge => Curve is not null;
    }

    private static double NormOnPeriod(double u, double first, double period) =>
        Math.Abs(u - first) < Constants.Tolerance ? first : PyMath.Mod(u - first, period) + first;

    private static double ForwardDelta(double u1, double u2, double first, double period)
    {
        var delta = NormOnPeriod(u2, first, period) - NormOnPeriod(u1, first, period);
        return delta < 0.0 ? delta + period : delta;
    }

    internal static (QualifiedCurve Q, Curve2D H2d, double First, double Last, CurveAdaptor2D Adaptor) EdgeToQualified2D(
        global::Kerf.Topology.Edge edge, Tangency tangency)
    {
        double f = 0, l = 0;
        var curve3d = Tool.Curve(edge, out f, out l);
        double first = 0, last = 0;
        Tool.Range(edge, out first, out last);
        var h2d = Projections.To2d(curve3d!, PlnXY)!;
        var adaptor = new CurveAdaptor2D(h2d, first, last);
        var q = new QualifiedCurve(adaptor, (Qualifier)(int)tangency);
        return (q, h2d, first, last, adaptor);
    }

    private static GccArg AsGccArg(Tangent t)
    {
        if (t.Edge is { } edge)
        {
            if (edge.IsEmpty)
                throw new ArgumentException("Can't create a qualified curve from empty edge");
            var (q, h2d, first, last, _) = EdgeToQualified2D(edge.TopoEdge, t.Tangency);
            return new GccArg(q, null, h2d, first, last);
        }
        var p = t.Point!.Value;
        var pnt = new Point2D(p.X, p.Y);
        return new GccArg(null, new CartesianPoint2(pnt), null, 0, 0);
    }

    private static global::Kerf.Topology.Edge EdgeFromCircle(CircleCurve2D circle, double u1, double u2)
    {
        var arc = new TrimmedCurve2D(circle, u1, u2, true);
        var maker = new MakeEdge(arc, SurfXY);
        return ShapeCasts.Edge(BrepLibrary.Curves3dBuilt(maker.Edge(), out _));
    }

    private static bool ParamInTrim(double u, double first, double last, Curve2D h2d)
    {
        if (h2d.IsPeriodic)
            u = NormOnPeriod(u, first, h2d.Period);
        return first - Constants.Tolerance <= u && u <= last + Constants.Tolerance;
    }

    /// <summary>The forward (minor) and complementary (major) arcs of a circle between two parameters.</summary>
    internal static List<global::Kerf.Topology.Edge> TwoArcEdgesFromParams(Circle2D circ, double u1, double u2)
    {
        var circle = new CircleCurve2D(circ);
        var period = circle.Period;
        var d = ForwardDelta(u1, u2, 0.0, period);
        var u1n = NormOnPeriod(u1, 0.0, period);
        var u2n = NormOnPeriod(u2, 0.0, period);
        if (d <= Constants.Tolerance || Math.Abs(period - d) <= Constants.Tolerance)
            return [];
        return [EdgeFromCircle(circle, u1n, u1n + d), EdgeFromCircle(circle, u2n, u2n + (period - d))];
    }

    private static double ArcLength(global::Kerf.Topology.Edge e)
    {
        var adaptor = new EdgeCurveAdaptor(e);
        return AbscissaPoint.Length(adaptor);
    }

    /// <summary>The arcs to keep by sagitta: the short, the long, or both.</summary>
    private static IEnumerable<global::Kerf.Topology.Edge> BySagitta(List<global::Kerf.Topology.Edge> arcs, Sagitta sagitta)
    {
        if (sagitta == Sagitta.Both)
            return arcs;
        var sorted = arcs.OrderBy(ArcLength).ToList();
        return [sagitta == Sagitta.Short ? sorted[0] : sorted[^1]];
    }

    internal static global::Kerf.Topology.Edge EdgeFromLine(Point2D p1, Point2D p2)
    {
        var a = new Point3D(p1.X, p1.Y, 0);
        var b = new Point3D(p2.X, p2.Y, 0);
        var va = new MakeVertex(a);
        var vb = new MakeVertex(b);
        var maker = new MakeEdge(va.Vertex(), vb.Vertex());
        if (!maker.IsDone)
            throw new InvalidOperationException("Failed to build edge from line contacts");
        return maker.Edge();
    }

    /// <summary>
    /// Adds π to the circle parameters of a solution enclosed by a circular target, when at least one target isn't circular
    /// (GccAna_Circ2d3Tan, used for all-circle inputs, already gets the enclosed direction right).
    /// </summary>
    private static double[] EnclosedCircParamOffset(IReadOnlyList<Tangent> tangents, Circle2D circ, double[] parameters)
    {
        var isCirc = tangents.Select(t => t.Edge is { } e && e.GeomType == GeomType.Circle).ToList();
        if (isCirc.All(c => c))
            return parameters;
        var center = circ.Location;
        var centerV = new Vector(center.X, center.Y, 0);
        var result = parameters.ToArray();
        for (int i = 0; i < parameters.Length && i < tangents.Count; i++)
            if (tangents[i].Edge is { } e && isCirc[i] && (centerV - e.ArcCenter).Length < e.Radius)
                result[i] = parameters[i] + Math.PI;
        return result;
    }

    // ---- Arcs ----

    internal static ShapeList<Edge> TwoTanRadArcs(List<Tangent> tangents, double radius, Sagitta sagitta)
    {
        var args = tangents.Select(AsGccArg).ToList();
        Circ2d2TanRad gcc = (args[0].IsEdge, args[1].IsEdge) switch
        {
            (true, true) => new(args[0].Curve!.Value, args[1].Curve!.Value, radius, Constants.Tolerance),
            (true, false) => new(args[0].Curve!.Value, args[1].Point!, radius, Constants.Tolerance),
            _ => new(args[0].Point!, args[1].Point!, radius, Constants.Tolerance),
        };
        if (!gcc.IsDone || gcc.SolutionCount == 0)
            throw new InvalidOperationException("Unable to find a tangent arc");

        var solutions = new List<global::Kerf.Topology.Edge>();
        for (int i = 1; i <= gcc.SolutionCount; i++)
        {
            var circ = gcc.ThisSolution(i);
            var center = circ.Location;
            var solRadius = circ.Radius;
            // The solver's parameters are unstable on transformed curves (ellipses especially): verify, then correct
            var pTmp = new Point2D();
            double uc1 = 0, ua1 = 0, uc2 = 0, ua2 = 0;
            gcc.Tangency1(i, out uc1, out ua1, out pTmp);
            gcc.Tangency2(i, out uc2, out ua2, out pTmp);
            (double UCirc, double UArg)[] candidates = [(uc1, ua1), (uc2, ua2)];
            var corrected = new List<double>();
            bool valid = true;
            for (int j = 0; j < 2 && valid; j++)
            {
                var (uCirc, uArg) = candidates[j];
                var arg = args[j];
                if (!arg.IsEdge)
                {
                    // A point target: the tangency point is the point itself
                    var p = new Point2D(arg.Point!.X(), arg.Point.Y());
                    if (Math.Abs(p.DistanceTo(center) - solRadius) < Constants.Tolerance)
                        corrected.Add(Math.Atan2(p.Y - center.Y, p.X - center.X));
                    else
                        valid = false;
                    continue;
                }
                var ps = arg.H2d!.Value(uArg);
                if (Math.Abs(ps.DistanceTo(center) - solRadius) < Constants.Tolerance && ParamInTrim(uArg, arg.First, arg.Last, arg.H2d))
                {
                    corrected.Add(uCirc);
                    continue;
                }
                // Project the centre onto the curve to find the real tangency point
                var c2 = new Point2D(center.X, center.Y);
                var proj = new ProjectPointOnCurve2D(c2, arg.H2d);
                if (proj.NbPoints() == 0)
                {
                    valid = false;
                    continue;
                }
                var uAct = proj.LowerDistanceParameter();
                var pAct = arg.H2d.Value(uAct);
                if (Math.Abs(pAct.DistanceTo(center) - solRadius) > Constants.Tolerance || !ParamInTrim(uAct, arg.First, arg.Last, arg.H2d))
                {
                    valid = false;
                    continue;
                }
                corrected.Add(Math.Atan2(pAct.Y - center.Y, pAct.X - center.X));
            }
            if (!valid)
                continue;
            solutions.AddRange(BySagitta(TwoArcEdgesFromParams(circ, corrected[0], corrected[1]), sagitta));
        }
        return new ShapeList<Edge>(solutions.Select(e => new Edge(e)));
    }

    internal static ShapeList<Edge> TwoTanOnArcs(List<Tangent> tangents, Tangent centerOn, Sagitta sagitta)
    {
        List<Tangent> all = [.. tangents, centerOn];
        var args = all.Select(AsGccArg).ToList();
        var adaptOn = new CurveAdaptor2D(args[2].H2d!, args[2].First, args[2].Last);
        // Initial guesses in the middle of each edge
        var guesses = args.Where(a => a.IsEdge).Select(a => (a.Last - a.First) / 2 + a.First).ToList();
        Circ2d2TanOn gcc = guesses.Count switch
        {
            3 => new(args[0].Curve!.Value, args[1].Curve!.Value, adaptOn, Constants.Tolerance, guesses[0], guesses[1], guesses[2]),
            2 => new(args[0].Curve!.Value, args[1].Point!, adaptOn, Constants.Tolerance, guesses[0], guesses[1]),
            _ => new(args[0].Point!, args[1].Point!, adaptOn, Constants.Tolerance),
        };
        if (!gcc.IsDone || gcc.SolutionCount == 0)
            throw new InvalidOperationException("Unable to find a tangent arc with center_on constraint");
        bool Ok(int i, double u) => !args[i].IsEdge || ParamInTrim(u, args[i].First, args[i].Last, args[i].H2d!);

        var solutions = new List<global::Kerf.Topology.Edge>();
        for (int i = 1; i <= gcc.SolutionCount; i++)
        {
            var circ = gcc.ThisSolution(i);
            var p = new Point2D();
            double uc1 = 0, ua1 = 0, uc2 = 0, ua2 = 0;
            gcc.Tangency1(i, out uc1, out ua1, out p);
            if (!Ok(0, ua1))
                continue;
            gcc.Tangency2(i, out uc2, out ua2, out p);
            if (!Ok(1, ua2))
                continue;
            var u = EnclosedCircParamOffset(all, circ, [uc1, uc2]);
            solutions.AddRange(BySagitta(TwoArcEdgesFromParams(circ, u[0], u[1]), sagitta));
        }
        return new ShapeList<Edge>(solutions.Select(e => new Edge(e)));
    }

    internal static ShapeList<Edge> ThreeTanArcs(List<Tangent> tangents, Sagitta sagitta)
    {
        if (tangents.Any(t => t.IsPoint))
            throw new ArgumentException("Three tangency targets must all be curves");
        var args = tangents.Select(AsGccArg).ToList();
        var guesses = args.Select(a => (a.Last - a.First) / 2 + a.First).ToList();
        const string message = "Unable to find a circle tangent to all three objects";
        Circ2d3Tan gcc;
        try
        {
            gcc = new Circ2d3Tan(args[0].Curve!.Value, args[1].Curve!.Value, args[2].Curve!.Value, Constants.Tolerance, guesses[0], guesses[1], guesses[2]);
        }
        catch (FailureException e)
        {
            throw new InvalidOperationException(message, e);
        }
        {
            if (!gcc.IsDone || gcc.SolutionCount == 0)
                throw new InvalidOperationException(message);
            bool Ok(int i, double u) => ParamInTrim(u, args[i].First, args[i].Last, args[i].H2d!);
            var solutions = new List<global::Kerf.Topology.Edge>();
            for (int i = 1; i <= gcc.SolutionCount; i++)
            {
                var circ = gcc.ThisSolution(i);
                var p = new Point2D();
                double uc1 = 0, ua1 = 0, uc2 = 0, ua2 = 0, uc3 = 0, ua3 = 0;
                gcc.Tangency1(i, out uc1, out ua1, out p);
                if (!Ok(0, ua1))
                    continue;
                gcc.Tangency2(i, out uc2, out ua2, out p);
                if (!Ok(1, ua2))
                    continue;
                // The third tangency validates the circle; it doesn't bound the arc
                gcc.Tangency3(i, out uc3, out ua3, out p);
                if (!Ok(2, ua3))
                    continue;
                solutions.AddRange(BySagitta(TwoArcEdgesFromParams(circ, uc1, uc2), sagitta));
            }
            return new ShapeList<Edge>(solutions.Select(e => new Edge(e)));
        }
    }

    internal static ShapeList<Edge> TanCenArcs(Tangent tangent, Vector center)
    {
        var c2d = new Point2D(center.X, center.Y);
        var arg = AsGccArg(tangent);
        var solutions = new List<global::Kerf.Topology.Edge>();
        if (!arg.IsEdge)
        {
            // Through a point: the circle about the centre through it
            var p = arg.Point!.Pnt2d();
            double dx = p.X - c2d.X, dy = p.Y - c2d.Y;
            var r = Math.Sqrt(dx * dx + dy * dy);
            if (r <= Constants.Tolerance)
                return [];   // the centre is the point: no circle
            var dir = new Direction2D(1.0, 0.0);
            var ax = new Axis2D(c2d, dir);
            var circ = new Circle2D(ax, r);
            var h2d = new CircleCurve2D(circ);
            solutions.Add(EdgeFromCircle(h2d, 0.0, h2d.Period));
        }
        else
        {
            var centerPoint = new CartesianPoint2(c2d);
            var gcc = new Circ2dTanCen(arg.Curve!.Value, centerPoint, Constants.Tolerance);
            if (!gcc.IsDone || gcc.SolutionCount == 0)
                throw new InvalidOperationException("Unexpected: GCC failed to return a tangent circle");
            for (int i = 1; i <= gcc.SolutionCount; i++)
            {
                var circ = gcc.ThisSolution(i);
                var p = new Point2D();
                double uc = 0, ua = 0;
                gcc.Tangency1(i, out uc, out ua, out p);
                if (!ParamInTrim(ua, arg.First, arg.Last, arg.H2d!))
                    continue;
                var h2d = new CircleCurve2D(circ);
                solutions.Add(EdgeFromCircle(h2d, 0.0, h2d.Period));
            }
        }
        return new ShapeList<Edge>(solutions.Select(e => new Edge(e)));
    }

    internal static ShapeList<Edge> TanOnRadArcs(Tangent tangent, Tangent centerOn, double radius)
    {
        var arg = AsGccArg(tangent);
        if (centerOn.Edge is not { } onEdge)
            throw new ArgumentException("center_on must be an Edge (line/circle/curve) for TanOnRad.");
        var (_, hOn2d, onFirst, onLast, adaptOn) = EdgeToQualified2D(onEdge.TopoEdge, Tangency.Unqualified);
        Circ2dTanOnRad gcc = arg.IsEdge
            ? new(arg.Curve!.Value, adaptOn, radius, Constants.Tolerance)
            : new(arg.Point!, adaptOn, radius, Constants.Tolerance);
        if (!gcc.IsDone || gcc.SolutionCount == 0)
            throw new InvalidOperationException("Unable to find circle(s) for TanOnRad constraints");
        var solutions = new List<global::Kerf.Topology.Edge>();
        for (int i = 1; i <= gcc.SolutionCount; i++)
        {
            var circ = gcc.ThisSolution(i);
            var p = new Point2D();
            double uc = 0, ua = 0;
            gcc.Tangency1(i, out uc, out ua, out p);
            if (arg.IsEdge && !ParamInTrim(ua, arg.First, arg.Last, arg.H2d!))
                continue;
            // The centre must lie on the trimmed locus
            var center2d = circ.Location;
            var proj = new ProjectPointOnCurve2D(center2d, hOn2d);
            if (proj.NbPoints() == 0 || !ParamInTrim(proj.Parameter(1), onFirst, onLast, hOn2d))
                continue;
            var h2d = new CircleCurve2D(circ);
            solutions.Add(EdgeFromCircle(h2d, 0.0, h2d.Period));
        }
        return new ShapeList<Edge>(solutions.Select(e => new Edge(e)));
    }

    // ---- Lines ----

    internal static ShapeList<Edge> TwoTanLines(Tangent one, Tangent two)
    {
        var a1 = AsGccArg(one);
        Lin2d2Tan gcc;
        GccArg? a2 = null;
        if (two.Point is { } point)
        {
            var p = new Point2D(point.X, point.Y);
            gcc = new Lin2d2Tan(a1.Curve!.Value, p, Constants.Tolerance);
        }
        else
        {
            a2 = AsGccArg(two);
            gcc = new Lin2d2Tan(a1.Curve!.Value, a2.Curve!.Value, Constants.Tolerance);
        }
        {
            if (!gcc.IsDone || gcc.SolutionCount == 0)
                throw new InvalidOperationException("Unable to find common tangent line(s)");
            var edges = new List<global::Kerf.Topology.Edge>();
            for (int i = 1; i <= gcc.SolutionCount; i++)
            {
                var solution = gcc.ThisSolution(i);
                var lin2d = new LineCurve2D(solution);
                // The two tangency points: Tangency1/2 can index the same line differently, so intersect instead
                var inter1 = new CurveCurveIntersection2D(lin2d, a1.H2d!);
                var pt1 = inter1.Point(1);   // there is always one tangent intersection
                Point2D pt2;
                if (two.Point is { } pp)
                    pt2 = new Point2D(pp.X, pp.Y);
                else
                {
                    var inter2 = new CurveCurveIntersection2D(lin2d, a2!.H2d!);
                    pt2 = inter2.Point(1);
                }
                {
                    var separation = pt1.DistanceTo(pt2);
                    if (double.IsNaN(separation) || separation < Constants.Tolerance)
                        continue;   // degenerate
                    edges.Add(EdgeFromLine(pt1, pt2));
                }
            }
            return new ShapeList<Edge>(edges.Select(e => new Edge(e)));
        }
    }

    internal static ShapeList<Edge> TanOrientedLines(Tangent tangent, Axis reference, double angle)
    {
        if (Math.Abs(Math.Abs(reference.Direction.Z) - 1) < Constants.Tolerance)
            throw new ArgumentException("reference Axis can't be perpendicular to Plane.XY");
        var arg = AsGccArg(tangent);
        var thetaAbs = Math.Atan2(reference.Direction.Y, reference.Direction.X) + angle;
        var dir2d = new Direction2D(Math.Cos(thetaAbs), Math.Sin(thetaAbs));
        var refPnt = new Point2D(reference.Position.X, reference.Position.Y);
        var refDir = new Direction2D(reference.Direction.X, reference.Direction.Y);
        var refAx = new Axis2D(refPnt, refDir);
        var refLin = new Line2D(refAx);
        var gcc = new Lin2dTanObl(arg.Curve!.Value, refLin, Constants.Tolerance, angle);
        var edges = new List<global::Kerf.Topology.Edge>();
        for (int i = 1; i <= gcc.SolutionCount; i++)
        {
            var pTan = new Point2D();
            double us = 0, ua = 0;
            gcc.Tangency1(i, out us, out ua, out pTan);
            var tanLine = new Line2D(pTan, dir2d);
            // Intersection2 isn't reliable: intersect with the reference line instead
            var inter = new AnalyticIntersection(tanLine, refLin);
            if (!inter.IsDone || inter.PointCount == 0)
                continue;
            var ip = inter.Point(1);
            var pIsect = ip.Value();
            var separation = pTan.DistanceTo(pIsect);
            if (double.IsNaN(separation) || separation < Constants.Tolerance)
                continue;
            edges.Add(EdgeFromLine(pTan, pIsect));
        }
        return new ShapeList<Edge>(edges.Select(e => new Edge(e)));
    }
}
