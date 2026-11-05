using System.Collections.Immutable;
using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Topology;

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
#pragma warning restore CS1591

    /// <summary>An edge as a target, the solution on the side <paramref name="tangency"/> says.</summary>
    public Tangent(Edge edge, Tangency tangency) : this(edge, null, null, tangency) { }

    /// <summary>An axis as a target, the solution on the side <paramref name="tangency"/> says.</summary>
    public Tangent(Axis axis, Tangency tangency) : this(null, axis, null, tangency) { }

    /// <summary>The target with an axis made into an edge.</summary>
    internal Tangent Resolved() => Axis is { } a ? new Tangent(Edge.FromAxis(a), null, null, Tangency) : this;

    internal bool IsPoint => Point is not null;
}

public static partial class EdgeExtensions
{
    extension(Edge)
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
            return ConstrainedSolvers.TwoTanRadArcs(EdgeExtensions.Sorted(tangencyOne, tangencyTwo), radius, sagitta);
        }

        /// <summary>
        /// The planar circular arcs on XY tangent to two objects whose centre lies on a curve (or axis): build123d's
        /// <c>make_constrained_arcs</c> with <c>center_on</c>.
        /// </summary>
        public static ShapeList<Edge> MakeConstrainedArcsCenteredOn(Tangent tangencyOne, Tangent tangencyTwo, Tangent centerOn,
                                                                    Sagitta sagitta = Sagitta.Short) =>
            ConstrainedSolvers.TwoTanOnArcs(EdgeExtensions.Sorted(tangencyOne, tangencyTwo), centerOn.Resolved(), sagitta);

        /// <summary>The planar circular arcs on XY tangent to three curves, between the tangency points of the first two.</summary>
        public static ShapeList<Edge> MakeConstrainedArcs(Tangent tangencyOne, Tangent tangencyTwo, Tangent tangencyThree, Sagitta sagitta = Sagitta.Short) =>
            ConstrainedSolvers.ThreeTanArcs(EdgeExtensions.Sorted(tangencyOne, tangencyTwo, tangencyThree), sagitta);

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
            var sorted = EdgeExtensions.Sorted(tangencyOne, tangencyTwo);
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
    }


    /// <summary>The targets with axes made into edges, points last (a stable sort, as build123d's).</summary>
    private static List<Tangent> Sorted(params Tangent[] tangents) => [.. tangents.Select(t => t.Resolved()).OrderBy(t => t.IsPoint)];
}

/// <summary>The constrained arc and line solvers of build123d's topology/constrained_lines.py, over the kernel's
/// <see cref="Tangents2D"/> (OCCT's Geom2dGcc).</summary>
internal static class ConstrainedSolvers
{
    // The XY plane for 3D to 2D projection and for building edges from 2D curves
    private static readonly Plane3D PlnXY = new(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
    private static readonly PlaneSurface SurfXY = new(PlnXY);

    // A target as the solvers take it, with the 2D curve and range of an edge (null for a point)
    internal sealed record GccArg(TangentTarget2D Target, Curve2D? H2d, double First, double Last)
    {
        public bool IsEdge => H2d is not null;
    }

    private static double NormOnPeriod(double u, double first, double period) =>
        Math.Abs(u - first) < Constants.Tolerance ? first : Numerics.FloorMod(u - first, period) + first;

    private static double ForwardDelta(double u1, double u2, double first, double period)
    {
        var delta = NormOnPeriod(u2, first, period) - NormOnPeriod(u1, first, period);
        return delta < 0.0 ? delta + period : delta;
    }

    // The edge's curve in Plane.XY's coordinates, bounded by the edge's range
    internal static GccArg EdgeToQualified2D(Edge edge, Tangency tangency)
    {
        if (!edge.TryGetCurve(out var curve3d, out var range))
            throw new ArgumentException("Can't create a qualified curve from an edge without a 3D curve");
        var h2d = Failures.Get(curve3d.ToCurve2D(PlnXY), () => "express the edge in Plane.XY");
        // The values match GccEnt_Position, as Kerf.Geometry.Tangency's do
        return new GccArg(TangentTarget2D.Tangent(h2d, range, (global::Kerf.Geometry.Tangency)(int)tangency), h2d, range.Min, range.Max);
    }

    private static GccArg AsGccArg(Tangent t)
    {
        if (t.Edge is { } edge)
        {
            if (edge.IsEmpty)
                throw new ArgumentException("Can't create a qualified curve from empty edge");
            return EdgeToQualified2D(edge, t.Tangency);
        }
        var p = t.Point!.Value;
        return new GccArg(TangentTarget2D.Through(new Point2D(p.X, p.Y)), null, 0, 0);
    }

    // The solutions, or the solver's failure (or no solution) as build123d's error
    private static ImmutableArray<T> Solved<T>(Result<ImmutableArray<T>> result, string message) =>
        result.TryGetValue(out var solutions) && solutions.Length > 0 ? solutions : throw new InvalidOperationException(message);

    private static Edge EdgeFromCircle(CircleCurve2D circle, double u1, double u2) =>
        Failures.Get(Edge.FromCurve(new TrimmedCurve2D(circle, u1, u2, true), SurfXY), () => "make an arc");

    private static bool ParamInTrim(double u, double first, double last, Curve2D h2d)
    {
        if (h2d.IsPeriodic)
            u = NormOnPeriod(u, first, h2d.Period);
        return first - Constants.Tolerance <= u && u <= last + Constants.Tolerance;
    }

    /// <summary>The forward (minor) and complementary (major) arcs of a circle between two parameters.</summary>
    internal static List<Edge> TwoArcEdgesFromParams(Circle2D circ, double u1, double u2)
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

    private static double ArcLength(Edge e) => e.ToCurve().Length;

    /// <summary>The arcs to keep by sagitta: the short, the long, or both.</summary>
    private static IEnumerable<Edge> BySagitta(List<Edge> arcs, Sagitta sagitta)
    {
        if (sagitta == Sagitta.Both)
            return arcs;
        var sorted = arcs.OrderBy(ArcLength).ToList();
        return [sagitta == Sagitta.Short ? sorted[0] : sorted[^1]];
    }

    internal static Edge EdgeFromLine(Point2D p1, Point2D p2) =>
        Edge.Line(new Point3D(p1.X, p1.Y, 0), new Point3D(p2.X, p2.Y, 0)).TryGetValue(out var edge)
            ? edge
            : throw new InvalidOperationException("Failed to build edge from line contacts");

    // ---- Arcs ----

    internal static ShapeList<Edge> TwoTanRadArcs(List<Tangent> tangents, double radius, Sagitta sagitta)
    {
        var args = tangents.Select(AsGccArg).ToList();
        var circles = Solved(Tangents2D.Circles(args[0].Target, args[1].Target, radius, Constants.Tolerance), "Unable to find a tangent arc");
        var solutions = new List<Edge>();
        foreach (var solution in circles)
        {
            var circ = solution.Circle;
            var center = circ.Location;
            var solRadius = circ.Radius;
            // The solver's parameters are unstable on transformed curves (ellipses especially): verify, then correct
            var corrected = new List<double>();
            bool valid = true;
            for (int j = 0; j < 2 && valid; j++)
            {
                var contact = solution.Contacts[j];
                var (uCirc, uArg) = (contact.SolutionParameter, contact.TargetParameter);
                var arg = args[j];
                if (!arg.IsEdge)
                {
                    // A point target: the tangency point is the point itself
                    var p = arg.Target.Point;
                    if (Math.Abs(p.DistanceTo(center) - solRadius) < Constants.Tolerance)
                        corrected.Add(Math.Atan2(p.Y - center.Y, p.X - center.X));
                    else
                        valid = false;
                    continue;
                }
                if (contact.IsCoincident)
                {
                    // The solution is the target circle itself: no single point of contact
                    valid = false;
                    continue;
                }
                var ps = arg.H2d!.Evaluate(uArg);
                if (Math.Abs(ps.DistanceTo(center) - solRadius) < Constants.Tolerance && ParamInTrim(uArg, arg.First, arg.Last, arg.H2d))
                {
                    corrected.Add(uCirc);
                    continue;
                }
                // Project the centre onto the curve to find the real tangency point
                var projected = arg.H2d.Project(new Point2D(center.X, center.Y));
                if (projected.IsEmpty)
                {
                    valid = false;
                    continue;
                }
                var uAct = projected[0].Parameter;
                var pAct = arg.H2d.Evaluate(uAct);
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
        return new ShapeList<Edge>(solutions.Select(e => e));
    }

    internal static ShapeList<Edge> TwoTanOnArcs(List<Tangent> tangents, Tangent centerOn, Sagitta sagitta)
    {
        var args = tangents.Select(AsGccArg).ToList();
        var on = AsGccArg(centerOn);
        if (!on.IsEdge)
            throw new ArgumentException("The centre must lie on an edge (a line, circle or curve).");
        var circles = Solved(Tangents2D.CirclesCentredOn(args[0].Target, args[1].Target, on.H2d!, new Interval(on.First, on.Last),
                                                         Constants.Tolerance),
                             "Unable to find a tangent arc with center_on constraint");
        bool Ok(int i, in TangentContact2D c) =>
            !args[i].IsEdge || (!c.IsCoincident && ParamInTrim(c.TargetParameter, args[i].First, args[i].Last, args[i].H2d!));
        var solutions = new List<Edge>();
        foreach (var solution in circles)
        {
            if (!Ok(0, solution.Contacts[0]) || !Ok(1, solution.Contacts[1]))
                continue;
            solutions.AddRange(BySagitta(TwoArcEdgesFromParams(solution.Circle, solution.Contacts[0].SolutionParameter,
                                                               solution.Contacts[1].SolutionParameter), sagitta));
        }
        return new ShapeList<Edge>(solutions.Select(e => e));
    }

    internal static ShapeList<Edge> ThreeTanArcs(List<Tangent> tangents, Sagitta sagitta)
    {
        if (tangents.Any(t => t.IsPoint))
            throw new ArgumentException("Three tangency targets must all be curves");
        var args = tangents.Select(AsGccArg).ToList();
        var circles = Solved(Tangents2D.Circles(args[0].Target, args[1].Target, args[2].Target, Constants.Tolerance),
                             "Unable to find a circle tangent to all three objects");
        bool Ok(int i, in TangentContact2D c) => !c.IsCoincident && ParamInTrim(c.TargetParameter, args[i].First, args[i].Last, args[i].H2d!);
        var solutions = new List<Edge>();
        foreach (var solution in circles)
        {
            // The third tangency validates the circle; it doesn't bound the arc
            if (!Ok(0, solution.Contacts[0]) || !Ok(1, solution.Contacts[1]) || !Ok(2, solution.Contacts[2]))
                continue;
            solutions.AddRange(BySagitta(TwoArcEdgesFromParams(solution.Circle, solution.Contacts[0].SolutionParameter,
                                                               solution.Contacts[1].SolutionParameter), sagitta));
        }
        return new ShapeList<Edge>(solutions.Select(e => e));
    }

    internal static ShapeList<Edge> TanCenArcs(Tangent tangent, Vector center)
    {
        var c2d = new Point2D(center.X, center.Y);
        var arg = AsGccArg(tangent);
        var solutions = new List<Edge>();
        if (!arg.IsEdge)
        {
            // Through a point: the circle about the centre through it
            var p = arg.Target.Point;
            double dx = p.X - c2d.X, dy = p.Y - c2d.Y;
            var r = Math.Sqrt(dx * dx + dy * dy);
            if (r <= Constants.Tolerance)
                return [];   // the centre is the point: no circle
            var h2d = new CircleCurve2D(new Circle2D(new Axis2D(c2d, new Direction2D(1.0, 0.0)), r));
            solutions.Add(EdgeFromCircle(h2d, 0.0, h2d.Period));
        }
        else
        {
            var circles = Solved(Tangents2D.CirclesCentredAt(arg.Target, c2d, Constants.Tolerance),
                                 "Unexpected: GCC failed to return a tangent circle");
            foreach (var solution in circles)
            {
                var contact = solution.Contacts[0];
                if (contact.IsCoincident || !ParamInTrim(contact.TargetParameter, arg.First, arg.Last, arg.H2d!))
                    continue;
                var h2d = new CircleCurve2D(solution.Circle);
                solutions.Add(EdgeFromCircle(h2d, 0.0, h2d.Period));
            }
        }
        return new ShapeList<Edge>(solutions.Select(e => e));
    }

    internal static ShapeList<Edge> TanOnRadArcs(Tangent tangent, Tangent centerOn, double radius)
    {
        var arg = AsGccArg(tangent);
        if (centerOn.Edge is not { } onEdge)
            throw new ArgumentException("The centre must lie on an edge (a line, circle or curve).");
        var on = EdgeToQualified2D(onEdge, Tangency.Unqualified);
        var circles = Solved(Tangents2D.CirclesCentredOn(arg.Target, on.H2d!, radius, new Interval(on.First, on.Last), Constants.Tolerance),
                             "Unable to find circle(s) for TanOnRad constraints");
        var solutions = new List<Edge>();
        foreach (var solution in circles)
        {
            var contact = solution.Contacts[0];
            if (arg.IsEdge && (contact.IsCoincident || !ParamInTrim(contact.TargetParameter, arg.First, arg.Last, arg.H2d!)))
                continue;
            // The centre must lie on the trimmed locus (it lies on the locus, so its nearest projection is itself)
            var projected = on.H2d!.Project(solution.Circle.Location);
            if (projected.IsEmpty || !ParamInTrim(projected[0].Parameter, on.First, on.Last, on.H2d!))
                continue;
            var h2d = new CircleCurve2D(solution.Circle);
            solutions.Add(EdgeFromCircle(h2d, 0.0, h2d.Period));
        }
        return new ShapeList<Edge>(solutions.Select(e => e));
    }

    // ---- Lines ----

    internal static ShapeList<Edge> TwoTanLines(Tangent one, Tangent two)
    {
        var a1 = AsGccArg(one);
        var a2 = AsGccArg(two);
        var lines = Solved(Tangents2D.Lines(a1.Target, a2.Target, Constants.Tolerance), "Unable to find common tangent line(s)");
        var edges = new List<Edge>();
        foreach (var line in lines)
        {
            var (pt1, pt2) = (line.Contacts[0].Point, line.Contacts[1].Point);
            var separation = pt1.DistanceTo(pt2);
            if (double.IsNaN(separation) || separation < Constants.Tolerance)
                continue;   // degenerate
            edges.Add(EdgeFromLine(pt1, pt2));
        }
        return new ShapeList<Edge>(edges.Select(e => e));
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
        var refLin = new Line2D(new Axis2D(refPnt, refDir));
        var solved = Tangents2D.LinesAtAngle(arg.Target, refLin, angle, Constants.Tolerance);
        // Lines parallel to the reference never meet it: no edge
        if (Math.Abs(dir2d.Crossed(refDir)) <= Constants.Tolerance)
            return [];
        // A solver that fails finds no line
        if (!solved.TryGetValue(out var lines))
            return [];
        var edges = new List<Edge>();
        foreach (var line in lines)
        {
            var (pTan, pIsect) = (line.Contacts[0].Point, line.Contacts[1].Point);
            var separation = pTan.DistanceTo(pIsect);
            if (double.IsNaN(separation) || separation < Constants.Tolerance)
                continue;
            edges.Add(EdgeFromLine(pTan, pIsect));
        }
        return new ShapeList<Edge>(edges.Select(e => e));
    }
}
