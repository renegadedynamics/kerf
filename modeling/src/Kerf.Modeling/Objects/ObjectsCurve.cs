namespace Kerf.Modeling;

/// <summary>The base of the curve objects that make several edges (<see cref="ConstrainedArcs"/>, <see cref="ConstrainedLines"/>).</summary>
public abstract class BaseCurveObject : Curve
{
    /// <summary>A curve object of the given curve.</summary>
    protected BaseCurveObject(Curve curve) : base(curve.Shape) { }
}

/// <summary>The base of the curve objects that make a wire (<see cref="Polyline"/>, <see cref="FilletPolyline"/>, …).</summary>
public abstract class BaseLineObject : Curve
{
    /// <summary>A line object of the given wire.</summary>
    protected BaseLineObject(Wire curve) : base((Shape)curve) { }

    /// <summary>The wire this object holds.</summary>
    public Wire Wire => (Wire)Shape;

    /// <summary>The wire this object holds.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="line"/> is null.</exception>
    public static implicit operator Wire(BaseLineObject line)
    {
        ArgumentNullException.ThrowIfNull(line);
        return line.Wire;
    }
}

/// <summary>The base of the curve objects that make one edge (<see cref="Line"/>, <see cref="CenterArc"/>, …).</summary>
public abstract class BaseEdgeObject : Curve
{
    /// <summary>An edge object of the given edge.</summary>
    protected BaseEdgeObject(Edge curve) : base((Shape)curve) { }

    /// <summary>The edge this object holds.</summary>
    public Edge Edge => (Edge)Shape;

    /// <summary>The edge this object holds.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="line"/> is null.</exception>
    public static implicit operator Edge(BaseEdgeObject line)
    {
        ArgumentNullException.ThrowIfNull(line);
        return line.Edge;
    }
}

/// <summary>
/// What an arc or line is trimmed to instead of being given a size: a shape, axis, location, plane or point. The curve
/// ends at its first intersection with the limit.
/// </summary>
public readonly struct CurveLimit
{
    private CurveLimit(object target) => Target = target;

    /// <summary>The shape, axis, location, plane or point.</summary>
    public object Target { get; }

#pragma warning disable CS1591 // Conversions from each kind of limit
    public static implicit operator CurveLimit(Shape shape) => new(shape);
    public static implicit operator CurveLimit(Vertex shape) => new((Shape)shape);
    public static implicit operator CurveLimit(Edge shape) => new((Shape)shape);
    public static implicit operator CurveLimit(Wire shape) => new((Shape)shape);
    public static implicit operator CurveLimit(Face shape) => new((Shape)shape);
    public static implicit operator CurveLimit(Shell shape) => new((Shape)shape);
    public static implicit operator CurveLimit(Solid shape) => new((Shape)shape);
    public static implicit operator CurveLimit(Compound shape) => new((Shape)shape);
    public static implicit operator CurveLimit(ModelShape shape) => new(shape.Shape);
    public static implicit operator CurveLimit(Axis axis) => new(axis);
    public static implicit operator CurveLimit(Location location) => new(location);
    public static implicit operator CurveLimit(Plane plane) => new(plane);
    public static implicit operator CurveLimit(Vector point) => new(point);
#pragma warning restore CS1591

    /// <inheritdoc/>
    public override string ToString() => Target.ToString() ?? "";
}

/// <summary>Helpers shared by the curve objects.</summary>
internal static class CurveObjects
{
    /// <summary>The shorter of two arcs trimmed to a limit, or null if neither reaches it.</summary>
    internal static Edge? ShortestTrimmed(object limit, params Edge[] arcs)
    {
        var reaching = new ShapeList<Edge>(arcs.Select(a => a.TrimToOther(limit)).Where(a => !a.IsEmpty));
        return reaching.Count == 0 ? null : reaching.OrderBy(e => e.Length).First();
    }

    /// <summary>
    /// A parabola or hyperbola trimmed where it first meets a limit. Both curves are open, so the arc may leave the start
    /// either way and has no natural end: the curve is followed each way from the start as far as the limit extends,
    /// and the shorter of the arcs that reach the limit is returned, or null if neither does.
    /// </summary>
    /// <param name="makeArc">Builds the positioned curve between two angles.</param>
    /// <param name="angleRange">The angles between which the curve stays within a given distance of the origin.</param>
    /// <param name="origin">The parabola's vertex or the hyperbola's centre.</param>
    /// <param name="size">A length typical of the curve, used as a margin.</param>
    /// <param name="startAngle">Where the arc starts.</param>
    /// <param name="limit">What to trim to.</param>
    internal static Edge? OpenConicToLimit(Func<double, double, Edge> makeArc, Func<double, (double Low, double High)> angleRange,
                                           Vector origin, double size, double startAngle, object limit)
    {
        if (limit is Axis or Plane)
        {
            // Neither has an extent, so stand-ins of growing size are searched
            var anchor = limit is Axis a ? a.Position : ((Plane)limit).Origin;
            var baseExtent = 10 * ((anchor - origin).Length + size);
            foreach (var extent in new[] { baseExtent, 100 * baseExtent })
            {
                Shape standIn = limit is Axis axis
                    ? Edge.MakeLine(anchor - axis.Direction * extent, anchor + axis.Direction * extent)
                    : Face.MakeRect(2 * extent, 2 * extent, (Plane)limit);
                if (OpenConicToLimit(makeArc, angleRange, origin, size, startAngle, standIn) is { } trimmed)
                    return trimmed;
            }
            return null;
        }

        List<Vector> limitPoints;
        if (limit is Shape shape)
        {
            var box = shape.BoundingBox();
            limitPoints = [];
            foreach (var x in new[] { box.Min.X, box.Max.X })
                foreach (var y in new[] { box.Min.Y, box.Max.Y })
                    foreach (var z in new[] { box.Min.Z, box.Max.Z })
                        limitPoints.Add(new Vector(x, y, z));
        }
        else
            limitPoints = [limit is Location l ? l.Position : (Vector)limit];

        // No part of the limit is further than this from the origin of the curve
        var reach = limitPoints.Max(p => (p - origin).Length) + size;
        var (low, high) = angleRange(reach);
        var candidates = new List<Edge>();
        if (high > startAngle)
            candidates.Add(makeArc(startAngle, high));
        if (low < startAngle)
            candidates.Add(makeArc(startAngle, low).Reversed(reconstruct: true));   // built towards a smaller angle, it ends at the start
        return ShortestTrimmed(limit, [.. candidates]);
    }

    internal static AngularDirection DirectionOf(double arcSize) => arcSize >= 0 ? AngularDirection.CounterClockwise : AngularDirection.Clockwise;
}

/// <summary>
/// An airfoil described by a NACA 4-digit (or fractional) code such as "2412" or "2213.323": the first digit is the
/// maximum camber as a percentage of the chord, the second the position of the maximum camber in tenths of the chord, and
/// the last two the maximum thickness as a percentage of the chord.
/// </summary>
public class Airfoil : BaseLineObject
{
    /// <summary>An airfoil of <paramref name="pointCount"/> points per surface, with a finite trailing edge if <paramref name="finiteTE"/>.</summary>
    public Airfoil(string airfoilCode, int pointCount = 50, bool finiteTE = false)
        : this(airfoilCode, Make(airfoilCode, pointCount, finiteTE, out var camberPoints), camberPoints, finiteTE)
    {
    }

    private Airfoil(string code, Wire profile, List<Vector> camberPoints, bool finiteTE) : base(profile)
    {
        (MaxCamber, CamberPos, Thickness) = ParseNaca4(code);
        (Code, FiniteTE, _camberPoints) = (code, finiteTE, camberPoints);
    }

    private readonly List<Vector> _camberPoints;

    /// <summary>The NACA code.</summary>
    public string Code { get; }

    /// <summary>The maximum camber as a fraction of the chord.</summary>
    public double MaxCamber { get; }

    /// <summary>The position along the chord of the maximum camber (0 to 1).</summary>
    public double CamberPos { get; }

    /// <summary>The maximum thickness as a fraction of the chord.</summary>
    public double Thickness { get; }

    /// <summary>Whether the trailing edge is finite.</summary>
    public bool FiniteTE { get; }

    /// <summary>The airfoil's camber line.</summary>
    public Edge CamberLine => Edge.MakeSpline(_camberPoints);

    /// <summary>The maximum camber, its position and the thickness of a NACA 4-digit (or fractional) code.</summary>
    public static (double MaxCamber, double CamberPos, double Thickness) ParseNaca4(string value)
    {
        var s = value.Replace("NACA", "").Trim();
        var inv = System.Globalization.CultureInfo.InvariantCulture;
        if (s.Contains('.'))
        {
            var parts = s.Split('.', 2);
            var (intPart, fracPart) = (parts[0], parts[1]);
            return (int.Parse(intPart[..1]) / 100.0, int.Parse(intPart[1..2]) / 10.0,
                    double.Parse($"{int.Parse(intPart[2..]):00}.{fracPart}", inv) / 100);
        }
        return (int.Parse(s[..1]) / 100.0, int.Parse(s[1..2]) / 10.0, int.Parse(s[2..]) / 100.0);
    }

    private static Wire Make(string code, int n, bool finiteTE, out List<Vector> camberPoints)
    {
        // The thickness distribution: yₜ = 5t[0.2969√x - 0.1260x - 0.3516x² + 0.2843x³ - 0.1015x⁴] where x is the
        // distance along the chord (0 at the leading edge, 1 at the trailing edge), t the maximum thickness as a fraction
        // of the chord, and yₜ the half-thickness there
        var (m, p, t) = ParseNaca4(code);

        // Cosine-spaced x values (as numpy's linspace) for better nose resolution
        var step = Math.PI / (n - 1);
        var x = Enumerable.Range(0, n).Select(i => (1 - Math.Cos(i == n - 1 && n > 1 ? Math.PI : i * step)) / 2).ToArray();

        const double a0 = 0.2969, a1 = -0.1260, a2 = -0.3516, a3 = 0.2843;
        var a4 = finiteTE ? -0.1015 : -0.1036;
        var yt = x.Select(xi => 5 * t * (a0 * Math.Sqrt(xi) + a1 * xi + a2 * (xi * xi) + a3 * Math.Pow(xi, 3) + a4 * Math.Pow(xi, 4))).ToArray();

        // The camber line and its slope
        var yc = new double[n];
        var dycDx = new double[n];
        if (!(m == 0 || p == 0 || p == 1))
            for (int i = 0; i < n; i++)
            {
                var xi = x[i];
                if (xi < p)
                {
                    yc[i] = m / (p * p) * (2 * p * xi - xi * xi);
                    dycDx[i] = 2 * m / (p * p) * (p - xi);
                }
                else
                {
                    yc[i] = m / ((1 - p) * (1 - p)) * ((1 - 2 * p) + 2 * p * xi - xi * xi);
                    dycDx[i] = 2 * m / ((1 - p) * (1 - p)) * (p - xi);
                }
            }
        var theta = dycDx.Select(Math.Atan).ToArray();
        camberPoints = [.. x.Select((xi, i) => new Vector(xi, yc[i]))];

        // The upper and lower surfaces
        var upper = Enumerable.Range(0, n).Select(i => new Vector(x[i] - yt[i] * Math.Sin(theta[i]), yc[i] + yt[i] * Math.Cos(theta[i])));
        var lower = Enumerable.Range(0, n).Select(i => new Vector(x[i] + yt[i] * Math.Sin(theta[i]), yc[i] - yt[i] * Math.Cos(theta[i])));
        List<Vector> unique = [.. upper.Reverse().Concat(lower).Distinct()];
        var surface = Edge.MakeSpline(unique, periodic: !finiteTE);
        return finiteTE ? Wire.Make([surface, Edge.MakeLine(surface.PositionAt(0), surface.PositionAt(1))]) : Wire.Make([surface]);
    }
}

/// <summary>
/// A Bézier curve of control points: non-rational, or rational with a weight for each control point.
/// </summary>
public class Bezier : BaseEdgeObject
{
    /// <summary>A Bézier curve of the control points with optional weights.</summary>
    public Bezier(IEnumerable<Vector> controlPoints, IReadOnlyList<double>? weights = null)
        : base(Edge.MakeBezier([.. controlPoints], weights))
    {
    }

    /// <summary>A non-rational Bézier curve of the control points.</summary>
    public Bezier(params Vector[] controlPoints) : this((IEnumerable<Vector>)controlPoints) { }
}

/// <summary>
/// A smooth Bézier transition between two edges: a line (C0), or a cubic (C1) or quintic (C2) Bézier whose control
/// points match the position, first derivative and (for C2) second derivative of the edges at the joined ends.
/// </summary>
public class BlendCurve : BaseEdgeObject
{
    /// <summary>
    /// A blend from <paramref name="curve0"/> to <paramref name="curve1"/>, joining <paramref name="endPoints"/> (each the
    /// start or end of its curve; by default the closest pair of ends) with the first derivatives scaled by
    /// <paramref name="tangentScalars"/> (by default 1) to adjust the blend's tension.
    /// </summary>
    /// <exception cref="ArgumentException">An end point isn't at the start or end of its curve.</exception>
    public BlendCurve(Edge curve0, Edge curve1, ContinuityLevel continuity = ContinuityLevel.C2, (Vector, Vector)? endPoints = null,
                      (double Start, double End)? tangentScalars = null)
        : base(Make(curve0, curve1, continuity, endPoints, tangentScalars ?? (1.0, 1.0)))
    {
    }

    private static (Vector, Vector) Closest(Edge curve0, Edge curve1)
    {
        var (a, b) = curve0.SelectVertices().SelectMany(a => curve1.SelectVertices().Select(b => (a, b))).MinBy(pair => pair.a.DistanceTo(pair.b));
        return (a.ToVector(), b.ToVector());
    }

    private static Edge Make(Edge curve0, Edge curve1, ContinuityLevel continuity, (Vector, Vector)? endPoints, (double Start, double End) scalars)
    {
        // The vertices to join, the closest pair if not given
        var ends = endPoints ?? Closest(curve0, curve1);

        // The parameter of each curve at its end point
        Edge[] curves = [curve0, curve1];
        Vector[] given = [ends.Item1, ends.Item2];
        var endParams = new double[2];
        for (int i = 0; i < 2; i++)
        {
            if ((given[i] - curves[i].PositionAt(0)).Length < Constants.Tolerance)
                endParams[i] = 0;
            else if ((given[i] - curves[i].PositionAt(1)).Length < Constants.Tolerance)
                endParams[i] = 1;
            else
                throw new ArgumentException("Each end point must be at the start or end of a curve.");
        }

        // For a degree-n Bézier B'(0) = n(P₁ - P₀), B'(1) = n(Pₙ - Pₙ₋₁), B''(0) = n(n-1)(P₂ - 2P₁ + P₀) and
        // B''(1) = n(n-1)(Pₙ₋₂ - 2Pₙ₋₁ + Pₙ), so matching the derivatives D and curvature vectors K at the ends gives
        // P₁ = P₀ + D₀/n, P₂ = P₀ + 2D₀/n + K₀/(n(n-1)), Pₙ₋₁ = Pₙ - D₁/n and Pₙ₋₂ = Pₙ - 2D₁/n + K₁/(n(n-1)).
        // The derivatives aren't normalised, so without scalars the curves' natural "speed" is used.
        var startPos = curve0.PositionAt(endParams[0]);
        var endPos = curve1.PositionAt(endParams[1]);
        var startDeriv = curve0.DerivativeAt(endParams[0], 1) * scalars.Start;
        var endDeriv = curve1.DerivativeAt(endParams[1], 1) * scalars.End;

        switch (continuity)
        {
            case ContinuityLevel.C0:
                return new Line(startPos, endPos);
            case ContinuityLevel.C1:
                return new Bezier(startPos, startPos + startDeriv / 3, endPos - endDeriv / 3, endPos);
            default:
                var startCurv = curve0.DerivativeAt(endParams[0], 2);
                var endCurv = curve1.DerivativeAt(endParams[1], 2);
                return new Bezier(startPos, startPos + startDeriv / 5, startPos + (2 * startDeriv) / 5 + startCurv / 20,
                                  endPos - (2 * endDeriv) / 5 + endCurv / 20, endPos - endDeriv / 5, endPos);
        }
    }
}

/// <summary>
/// An exact B-spline from control points (which pull the curve but generally aren't on it), a knot sequence (repeated
/// knots give multiplicities) and optional weights. Unlike <see cref="Spline"/>, which interpolates points, it keeps the
/// spline definition given.
/// </summary>
public class BSpline : BaseEdgeObject
{
    /// <summary>A B-spline of the given degree.</summary>
    public BSpline(IEnumerable<Vector> controlPoints, IEnumerable<double> knots, int degree, IEnumerable<double>? weights = null,
                   bool periodic = false)
        : base(Edge.MakeBSpline(controlPoints, knots, degree, weights, periodic))
    {
    }
}

/// <summary>A circular arc of a centre and radius, from a start angle over an angular size or to a limit.</summary>
public class CenterArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="startAngle"/> degrees (from the X axis) over <paramref name="arcSize"/> degrees (negative for clockwise).</summary>
    public CenterArc(Vector center, double radius, double startAngle, double arcSize)
        : base(Make(center, radius, startAngle, arcSize))
    {
    }

    /// <summary>
    /// An arc from <paramref name="startAngle"/> degrees to its first intersection with <paramref name="arcLimit"/>,
    /// whichever way round is shorter: so less than 180°.
    /// </summary>
    /// <exception cref="ArgumentException">Neither way round meets the limit.</exception>
    public CenterArc(Vector center, double radius, double startAngle, CurveLimit arcLimit)
        : base(Make(center, radius, startAngle, arcLimit))
    {
    }

    private static Edge Make(Vector center, double radius, double startAngle, double arcSize)
    {
        var direction = arcSize > 0 ? AngularDirection.CounterClockwise : AngularDirection.Clockwise;
        var normalized = Numerics.FloorMod(arcSize + 360.0, 360.0);
        var endAngle = startAngle + normalized;
        if (normalized == 360.0)
            startAngle = endAngle;
        return Edge.MakeCircle(radius, new Plane(center), startAngle, endAngle, direction);
    }

    private static Edge Make(Vector center, double radius, double startAngle, CurveLimit limit)
    {
        var startRadius = new Vector(1, 0, 0).Rotate(Axis.Z, startAngle) * radius;
        var arc = Edge.MakeCircle(radius, new Plane(center, startRadius, new Vector(0, 0, 1)));
        return CurveObjects.ShortestTrimmed(limit.Target, arc, arc.Reversed(reconstruct: true))
               ?? throw new ArgumentException($"CenterArc doesn't intersect arc limit {limit}");
    }
}

/// <summary>
/// The planar circular arcs on XY constrained by other objects (see <c>MakeConstrainedArcs</c>),
/// as a curve of one or more edges, optionally narrowed by a selector.
/// </summary>
public class ConstrainedArcs : BaseCurveObject
{
    /// <summary>The arcs of a radius tangent to (or through) two objects.</summary>
    public ConstrainedArcs(Tangent tangencyOne, Tangent tangencyTwo, double radius, Sagitta sagitta = Sagitta.Short,
                           Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null)
        : base(Select(Edge.MakeConstrainedArcs(tangencyOne, tangencyTwo, radius, sagitta), selector))
    {
    }

    /// <summary>The arcs tangent to three objects.</summary>
    public ConstrainedArcs(Tangent tangencyOne, Tangent tangencyTwo, Tangent tangencyThree, Sagitta sagitta = Sagitta.Short,
                           Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null)
        : base(Select(Edge.MakeConstrainedArcs(tangencyOne, tangencyTwo, tangencyThree, sagitta), selector))
    {
    }

    /// <summary>The circles about a centre tangent to (or through) an object.</summary>
    public ConstrainedArcs(Tangent tangencyOne, Vector center, Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null)
        : base(Select(Edge.MakeConstrainedArcs(tangencyOne, center), selector))
    {
    }

    /// <summary>The circles of a radius tangent to (or through) an object, their centres on <paramref name="centerOn"/>.</summary>
    public ConstrainedArcs(Tangent tangencyOne, double radius, Tangent centerOn, Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null)
        : base(Select(Edge.MakeConstrainedArcs(tangencyOne, radius, centerOn), selector))
    {
    }

    private ConstrainedArcs(Curve arcs) : base(arcs) { }

    /// <summary>The arcs tangent to (or through) two objects, their centres on <paramref name="centerOn"/>.</summary>
    public static ConstrainedArcs CenteredOn(Tangent tangencyOne, Tangent tangencyTwo, Tangent centerOn, Sagitta sagitta = Sagitta.Short,
                                             Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        new(Select(Edge.MakeConstrainedArcsCenteredOn(tangencyOne, tangencyTwo, centerOn, sagitta), selector));

    internal static Curve Select(ShapeList<Edge> edges, Func<ShapeList<Edge>, IEnumerable<Edge>>? selector)
    {
        var selected = selector is null ? [.. edges] : selector(edges)?.ToList();
        if (selected is null || !selected.Any())
            throw new ArgumentException("The selector must return edges, not null.");
        return new Curve(selected);
    }
}

/// <summary>
/// The planar lines on XY constrained by other objects (see <c>MakeConstrainedLines</c>), as
/// a curve of one or more edges, optionally narrowed by a selector.
/// </summary>
public class ConstrainedLines : BaseCurveObject
{
    /// <summary>The lines tangent to two curves, or to a curve and through a point.</summary>
    public ConstrainedLines(Tangent tangencyOne, Tangent tangencyTwo, Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null)
        : base(ConstrainedArcs.Select(Edge.MakeConstrainedLines(tangencyOne, tangencyTwo), selector))
    {
    }

    /// <summary>The lines tangent to a curve at <paramref name="angle"/> degrees from a reference axis.</summary>
    public ConstrainedLines(Tangent tangencyOne, Axis reference, double angle, Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null)
        : base(ConstrainedArcs.Select(Edge.MakeConstrainedLines(tangencyOne, reference, angle), selector))
    {
    }

    /// <summary>The lines tangent to a curve in a direction (its X and Y) measured from a reference axis.</summary>
    public ConstrainedLines(Tangent tangencyOne, Axis reference, Vector direction, Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null)
        : base(ConstrainedArcs.Select(Edge.MakeConstrainedLines(tangencyOne, reference, direction), selector))
    {
    }
}

/// <summary>
/// A circular arc from a point and tangent to a tangent point on another curve; which arc <see cref="Keep.Top"/> or
/// <see cref="Keep.Bottom"/> picks depends on the geometry. Contains a solver.
/// </summary>
public class DoubleTangentArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="point"/> along <paramref name="tangent"/>, tangent to <paramref name="other"/>.</summary>
    /// <exception cref="ArgumentException">The keep isn't Top or Bottom, or the objects aren't in one plane.</exception>
    /// <exception cref="InvalidOperationException">No arc was found.</exception>
    public DoubleTangentArc(Vector point, Vector tangent, IShape1D other, Keep keep = Keep.Top)
        : base(Make(point, tangent, other, keep))
    {
    }

    private static Edge Make(Vector arcPoint, Vector tangent, IShape1D other, Keep keep)
    {
        if (keep is not (Keep.Top or Keep.Bottom))
            throw new ArgumentException($"Only the TOP or BOTTOM options are supported not {keep}");
        var arcTangent = tangent.Normalized();
        var coplane = Edge.MakeLine(arcPoint, arcPoint + arcTangent).CommonPlane(other.SelectEdges().Select(e => (IShape1D?)e))
                      ?? throw new ArgumentException("DoubleTangentArc only works on a single plane");
        var workplane = -coplane;   // flipped to help with TOP/BOTTOM
        var rotationAxis = new Axis(new Vector(0, 0, 0), workplane.ZDir);
        // Protect against massive circles that are effectively straight lines
        var maxSize = 10 * other.BoundingBox().Add(arcPoint).Diagonal;

        var centers = new List<Vector>();
        foreach (var angle in new[] { 90.0, -90.0 })
        {
            var bisector = arcTangent.Rotate(rotationAxis, angle);
            var radius = Numerics.MinimizeNelderMead(r => Math.Abs(other.DistanceTo(arcPoint + bisector * r) - r), 0.0, 0.0, maxSize,
                                                   Constants.Tolerance);
            var center = arcPoint + bisector * radius;

            // Check for matching tangents
            var circle = Edge.MakeCircle(radius, new Plane(center, zDir: rotationAxis.Direction));
            var (distance, p1, p2) = other.DistanceToWithClosestPoints(circle);
            if (distance > Constants.Tolerance)
                continue;   // they aren't touching
            var otherAxis = new Axis(p1, other.TangentAt(p1));
            var circleAxis = new Axis(p2, circle.TangentAt(p2));
            if (otherAxis.IsParallel(circleAxis, 0.05))
                centers.Add(center);
        }
        if (centers.Count == 0)
            throw new InvalidOperationException("No double tangent arcs found");

        // If there are several solutions, the one wanted
        var chosen = keep == Keep.Top ? centers[0] : centers[^1];
        var (_, tangentPoint, _) = other.DistanceToWithClosestPoints(chosen);
        return new TangentArc(arcPoint, tangentPoint, arcTangent);
    }
}

/// <summary>An elliptical arc of a centre and x and y radii, over an angular size or to a limit, optionally rotated.</summary>
public class EllipticalCenterArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="startAngle"/> over <paramref name="arcSize"/> degrees (negative for clockwise), rotated by <paramref name="rotation"/>.</summary>
    public EllipticalCenterArc(Vector center, double xRadius, double yRadius, double startAngle = 0.0, double arcSize = 90.0,
                               double rotation = 0.0)
        : base(Edge.MakeEllipse(xRadius, yRadius, startAngle: startAngle, endAngle: startAngle + arcSize,
                                angularDirection: CurveObjects.DirectionOf(arcSize)).Rotate(Axis.Z, rotation).Translate(center))
    {
    }

    /// <summary>An arc from <paramref name="startAngle"/> to its first intersection with <paramref name="arcLimit"/>, whichever way round is shorter.</summary>
    /// <exception cref="ArgumentException">Neither way round meets the limit.</exception>
    public EllipticalCenterArc(Vector center, double xRadius, double yRadius, double startAngle, CurveLimit arcLimit, double rotation = 0.0)
        : base(Make(center, xRadius, yRadius, startAngle, arcLimit, rotation))
    {
    }

    private static Edge Make(Vector center, double xRadius, double yRadius, double startAngle, CurveLimit limit, double rotation)
    {
        // The whole ellipse, beginning and ending at the start of the arc
        var curve = Edge.MakeEllipse(xRadius, yRadius, startAngle: startAngle, endAngle: startAngle + 360.0)
            .Rotate(Axis.Z, rotation).Translate(center);
        return CurveObjects.ShortestTrimmed(limit.Target, curve, curve.Reversed(reconstruct: true))
               ?? throw new ArgumentException($"EllipticalCenterArc doesn't intersect arc limit {limit}");
    }
}

/// <summary>
/// An elliptical arc from a start point and tangent, of x and y radii and an angular size, placed by the angular position
/// of the start or by the direction of the major axis.
/// </summary>
public class EllipticalStartArc : BaseEdgeObject
{
    /// <summary>
    /// An arc from <paramref name="startPoint"/> along <paramref name="startTangent"/> over <paramref name="arcSize"/>
    /// degrees (negative for clockwise); one of <paramref name="startAngle"/> (the start's angular position) or
    /// <paramref name="majorAxisDir"/> (the ellipse's x direction) must be given.
    /// </summary>
    /// <exception cref="ArgumentException">Neither the start angle nor the major axis direction is given.</exception>
    public EllipticalStartArc(Vector startPoint, Vector startTangent, double xRadius, double yRadius, double arcSize, double? startAngle = null,
                              Vector? majorAxisDir = null)
        : base(Make(startPoint, startTangent, xRadius, yRadius, arcSize, startAngle, majorAxisDir))
    {
    }

    private static Edge Make(Vector startPoint, Vector startTangent, double xRadius, double yRadius, double arcSize, double? startAngle,
                             Vector? majorAxisDir)
    {
        static Vector ProjectToPlane(Vector v, Vector n)
        {
            n = n.Normalized();
            return v - n * v.Dot(n);
        }

        // On the XY plane (the builder's workplane in builder mode)
        var tangent = startTangent.Normalized();
        var normal = new Vector(0, 0, 1);
        Vector xDir, yDir, origin;
        double angle;
        if (startAngle is { } given)
        {
            var rad = given * Constants.DegToRad;
            var planeTangent = ProjectToPlane(tangent, normal).Normalized();
            var a = -xRadius * Math.Sin(rad);
            var b = yRadius * Math.Cos(rad);
            xDir = ((planeTangent * a - normal.Cross(planeTangent) * b) / (a * a + b * b)).Normalized();
            yDir = normal.Cross(xDir);
            origin = startPoint - xDir * (xRadius * Math.Cos(rad)) - yDir * (yRadius * Math.Sin(rad));
            angle = given;
        }
        else if (majorAxisDir is { } major)
        {
            xDir = ProjectToPlane(major, normal).Normalized();
            yDir = normal.Cross(xDir);
            var planeTangent = ProjectToPlane(tangent, normal);
            var rad = Math.Atan2(-(planeTangent.Dot(xDir) / xRadius), planeTangent.Dot(yDir) / yRadius);
            origin = startPoint - xDir * (xRadius * Math.Cos(rad)) - yDir * (yRadius * Math.Sin(rad));
            angle = rad * Constants.RadToDeg;
        }
        else
            throw new ArgumentException("Give a start angle or a major axis direction.");

        var plane = new Plane(origin, xDir, normal);
        return Edge.MakeEllipse(xRadius, yRadius, plane, angle, angle + arcSize, CurveObjects.DirectionOf(arcSize));
    }
}

/// <summary>A parabolic arc of a vertex and focal length, over an angular size or to a limit, optionally rotated.</summary>
public class ParabolicCenterArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="startAngle"/> over <paramref name="arcSize"/> degrees (negative for clockwise), rotated by <paramref name="rotation"/>.</summary>
    public ParabolicCenterArc(Vector vertex, double focalLength, double startAngle = 0.0, double arcSize = 90.0, double rotation = 0.0)
        : base(Edge.MakeParabola(focalLength, startAngle: startAngle, endAngle: startAngle + arcSize,
                                 angularDirection: CurveObjects.DirectionOf(arcSize)).Rotate(Axis.Z, rotation).Translate(vertex))
    {
    }

    /// <summary>An arc from <paramref name="startAngle"/> to where it first meets <paramref name="arcLimit"/>, whichever way is shorter.</summary>
    /// <exception cref="ArgumentException">Neither way meets the limit.</exception>
    public ParabolicCenterArc(Vector vertex, double focalLength, double startAngle, CurveLimit arcLimit, double rotation = 0.0)
        : base(CurveObjects.OpenConicToLimit(
                   (from, to) => Edge.MakeParabola(focalLength, startAngle: from, endAngle: to,
                                                   angularDirection: to >= from ? AngularDirection.CounterClockwise : AngularDirection.Clockwise)
                       .Rotate(Axis.Z, rotation).Translate(vertex),
                   // The angle is the distance along the directrix, so a point of the parabola is at least that far from the vertex
                   reach => (-reach * Constants.RadToDeg, reach * Constants.RadToDeg),
                   vertex, focalLength, startAngle, arcLimit.Target)
               ?? throw new ArgumentException($"ParabolicCenterArc doesn't intersect arc limit {arcLimit}"))
    {
    }
}

/// <summary>A hyperbolic arc of a centre and x and y radii, over an angular size or to a limit, optionally rotated.</summary>
public class HyperbolicCenterArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="startAngle"/> over <paramref name="arcSize"/> degrees (negative for clockwise), rotated by <paramref name="rotation"/>.</summary>
    public HyperbolicCenterArc(Vector center, double xRadius, double yRadius, double startAngle = 0.0, double arcSize = 90.0,
                               double rotation = 0.0)
        : base(Edge.MakeHyperbola(xRadius, yRadius, startAngle: startAngle, endAngle: startAngle + arcSize,
                                  angularDirection: CurveObjects.DirectionOf(arcSize)).Rotate(Axis.Z, rotation).Translate(center))
    {
    }

    /// <summary>An arc from <paramref name="startAngle"/> to where it first meets <paramref name="arcLimit"/>, whichever way is shorter.</summary>
    /// <exception cref="ArgumentException">Neither way meets the limit.</exception>
    public HyperbolicCenterArc(Vector center, double xRadius, double yRadius, double startAngle, CurveLimit arcLimit, double rotation = 0.0)
        : base(Make(center, xRadius, yRadius, startAngle, arcLimit, rotation))
    {
    }

    private static Edge Make(Vector center, double xRadius, double yRadius, double startAngle, CurveLimit limit, double rotation)
    {
        Edge Hyperbola(double from, double to) =>
            Edge.MakeHyperbola(xRadius, yRadius, startAngle: from, endAngle: to,
                               angularDirection: to >= from ? AngularDirection.CounterClockwise : AngularDirection.Clockwise)
                .Rotate(Axis.Z, rotation).Translate(center);

        // A point of the hyperbola is minor radius × sinh(angle) from its major axis, measured from where the curve crosses it
        var minorRadius = Math.Min(xRadius, yRadius);
        var axisAngle = yRadius > xRadius ? 90.0 : 0.0;
        (double, double) Within(double reach)
        {
            var halfRange = Math.Asinh(reach / minorRadius) * Constants.RadToDeg;
            return (axisAngle - halfRange, axisAngle + halfRange);
        }

        return CurveObjects.OpenConicToLimit(Hyperbola, Within, center, xRadius + yRadius, startAngle, limit.Target)
               ?? throw new ArgumentException($"HyperbolicCenterArc doesn't intersect arc limit {limit}");
    }
}

/// <summary>
/// A helix of a pitch, height and radius, optionally tapered by a cone angle (positive to widen, negative to narrow, the
/// radius being that at the start).
/// </summary>
public class Helix : BaseEdgeObject
{
    /// <summary>A helix about <paramref name="direction"/> through <paramref name="center"/>.</summary>
    public Helix(double pitch, double height, double radius, Vector? center = null, Vector? direction = null, double coneAngle = 0,
                 bool lefthand = false)
        : base(Edge.MakeHelix(pitch, height, radius, center ?? new Vector(0, 0, 0), direction ?? new Vector(0, 0, 1), coneAngle, lefthand))
    {
    }
}

/// <summary>
/// Straight lines through successive points with their corners filleted to a radius (zero for a sharp corner), optionally
/// closed with an extra line.
/// </summary>
public class FilletPolyline : BaseLineObject
{
    /// <summary>A polyline with every corner filleted to <paramref name="radius"/>.</summary>
    /// <exception cref="ArgumentException">Fewer than two points, or a negative radius.</exception>
    public FilletPolyline(IEnumerable<Vector> points, double radius, bool close = false)
        : base(Make([.. points], null, radius, close))
    {
    }

    /// <summary>A polyline with each corner filleted to its own radius (one per corner, in order).</summary>
    /// <exception cref="ArgumentException">Fewer than two points, a radius count that doesn't match the corners, or a negative radius.</exception>
    public FilletPolyline(IEnumerable<Vector> points, IEnumerable<double> radii, bool close = false)
        : base(Make([.. points], [.. radii], 0, close))
    {
    }

    private static Wire Make(List<Vector> points, List<double>? radii, double radius, bool close)
    {
        // Polylines closed by the user
        if ((points[0] - points[^1]).Length < Constants.Tolerance)
        {
            close = true;
            points.RemoveAt(points.Count - 1);
        }
        if (points.Count < 2)
            throw new ArgumentException("FilletPolyline requires two or more pts");

        var openEnds = close ? 0 : 1;
        if (radii is null)
            radii = [.. Enumerable.Repeat(radius, points.Count)];
        else if (radii.Count != points.Count - openEnds * 2)
            throw new ArgumentException($"radius list length ({radii.Count}) must match angle count ({points.Count - openEnds * 2})");
        foreach (var r in radii)
            if (r < 0)
                throw new ArgumentException($"radius {r} must be non-negative");

        // The polyline
        var edges = Enumerable.Range(0, points.Count - 1).Select(i => Edge.MakeLine(points[i], points[i + 1])).ToList();
        if (close && (edges[0].PositionAt(0) - edges[^1].PositionAt(1)).Length > 1e-5)
            edges.Add(Edge.MakeLine(edges[^1].PositionAt(1), edges[0].PositionAt(0)));
        var wireOfLines = Wire.Make(edges);

        // The wire's vertices in the order of the points, so the fillets are in order
        var wireVertices = wireOfLines.SelectVertices();
        var orderedVertices = points.Select(p => wireVertices.MinBy(v => (p - v.ToVector()).Length)!).ToList();

        // A fillet edge for each corner, or null for a sharp one (keyed as build123d's dict, so a repeated vertex counts once)
        var wireEdges = wireOfLines.SelectEdges();
        var vertexToEdges = new List<(Vertex Vertex, List<Edge> Edges)>();
        foreach (var v in orderedVertices)
            if (!vertexToEdges.Any(entry => entry.Vertex == v))
                vertexToEdges.Add((v, [.. wireEdges.Where(e => e.SelectVertices().Contains(v))]));
        var fillets = new List<Edge?>();
        for (int i = 0; i < vertexToEdges.Count; i++)
        {
            var (vertex, corner) = vertexToEdges[i];
            if (corner.Count != 2)
                continue;
            var currentRadius = radii[i - openEnds];
            if (currentRadius == 0)
            {
                fillets.Add(null);
                continue;
            }
            var others = corner.SelectMany(e => e.SelectVertices()).Where(ve => ve != vertex).Distinct().ToList();
            var thirdEdge = Edge.MakeLine(others[0].ToVector(), others[1].ToVector());
            var filletFace = Face.Make(Wire.Make([.. corner, thirdEdge])).Fillet2D(currentRadius, [vertex]);
            fillets.Add(filletFace.SelectEdges().Where(GeomType.Circle)[0]);
        }

        // The lines joining the fillets
        var interior = new List<Edge>();
        var endEdges = new List<Edge>();
        if (close)
        {
            for (int i = 0; i < fillets.Count; i++)
            {
                var previous = fillets[(i - 1 + fillets.Count) % fillets.Count];
                var current = fillets[i];
                var start = previous is not { } before ? orderedVertices[(i - 1 + orderedVertices.Count) % orderedVertices.Count].ToVector() : before.PositionAt(1);
                var end = current is not { } arc ? orderedVertices[i].ToVector() : arc.PositionAt(0);
                interior.Add(Edge.MakeLine(start, end));
            }
        }
        else
        {
            for (int i = 0; i < fillets.Count - 1; i++)
            {
                var current = fillets[i];
                var next = fillets[i + 1];
                // + 1 as the first vertex has no fillet
                var start = current is not { } arc ? orderedVertices[i + 1].ToVector() : arc.PositionAt(1);
                var end = next is not { } after ? orderedVertices[i + 2].ToVector() : after.PositionAt(0);
                interior.Add(Edge.MakeLine(start, end));
            }
            var startEdge = fillets[0] is { } first
                ? Edge.MakeLine(wireOfLines.PositionAt(0), first.PositionAt(0))
                : Edge.MakeLine(wireOfLines.PositionAt(0), orderedVertices[1].ToVector());
            var endEdge = fillets[^1] is { } last
                ? Edge.MakeLine(last.PositionAt(1), wireOfLines.PositionAt(1))
                : Edge.MakeLine(orderedVertices[^2].ToVector(), wireOfLines.PositionAt(1));
            endEdges = [startEdge, endEdge];
        }
        return Wire.Make([.. endEdges, .. interior, .. fillets.OfType<Edge>()]);
    }
}

/// <summary>A circular arc from a start point and tangent, of a radius, over an angular size or to a limit.</summary>
public class JernArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="start"/> along <paramref name="tangent"/> over <paramref name="arcSize"/> degrees (negative to turn the other way).</summary>
    public JernArc(Vector start, Vector tangent, double radius, double arcSize)
        : this(start, Make(start, tangent, radius, arcSize, null))
    {
    }

    /// <summary>An arc from <paramref name="start"/> along <paramref name="tangent"/> to its first intersection with <paramref name="arcLimit"/>, whichever way is shorter.</summary>
    /// <exception cref="ArgumentException">Neither way meets the limit.</exception>
    public JernArc(Vector start, Vector tangent, double radius, CurveLimit arcLimit)
        : this(start, Make(start, tangent, radius, 360, arcLimit))
    {
    }

    private JernArc(Vector start, Edge arc) : base(arc)
    {
        Start = start;
        CenterPoint = arc.ArcCenter;
        EndOfArc = arc.PositionAt(1);
    }

    /// <summary>The start point.</summary>
    public Vector Start { get; }

    /// <summary>The end point of the arc.</summary>
    public Vector EndOfArc { get; }

    /// <summary>The centre of the arc.</summary>
    public Vector CenterPoint { get; }

    private static Edge Make(Vector start, Vector tangent, double radius, double arcSize, CurveLimit? limit)
    {
        var startTangent = tangent.Normalized();
        var arcNormal = new Vector(0, 0, 1);
        if (Math.Abs(startTangent.Dot(arcNormal)) > Constants.Tolerance)
        {
            arcNormal = startTangent.Cross(arcNormal);
            if (arcNormal.Length <= Constants.Tolerance)
                arcNormal = startTangent.Cross(new Vector(1, 0, 0));
            arcNormal = arcNormal.Normalized();
        }

        var arcDirection = limit is null ? Math.CopySign(1.0, arcSize) : 1;
        var normalAxis = new Axis(start, arcNormal);
        var centerPoint = start + startTangent.Rotate(normalAxis, arcDirection * 90) * Math.Abs(radius);
        var endOfArc = centerPoint + (start - centerPoint).Rotate(normalAxis, arcSize);
        if (Math.Abs(arcSize) < 360)
            return Edge.MakeTangentArc(start, startTangent, endOfArc);

        var arc = Edge.MakeCircle(radius, new Plane(centerPoint, start - centerPoint, arcNormal));
        if (limit is not { } l)
            return arc;
        var centerPoint2 = start + startTangent.Rotate(normalAxis, -arcDirection * 90) * Math.Abs(radius);
        var arc2 = Edge.MakeCircle(radius, new Plane(centerPoint2, start - centerPoint2, arcNormal));
        if (arc2.TangentAt(0).Dot(startTangent) < 0)
            arc2 = arc2.Reversed(reconstruct: true);
        return CurveObjects.ShortestTrimmed(l.Target, arc, arc2) ?? throw new ArgumentException($"JernArc doesn't intersect arc limit {l}");
    }
}

/// <summary>A straight line between two points.</summary>
public class Line : BaseEdgeObject
{
    /// <summary>A line from <paramref name="start"/> to <paramref name="end"/>.</summary>
    public Line(Vector start, Vector end) : base(Edge.MakeLine(start, end)) { }

    /// <summary>A line between two points.</summary>
    /// <exception cref="ArgumentException">Not exactly two points.</exception>
    public Line(IEnumerable<Vector> points) : this(TwoPoints(points)) { }

    private Line((Vector Start, Vector End) points) : this(points.Start, points.End) { }

    private static (Vector, Vector) TwoPoints(IEnumerable<Vector> points)
    {
        var list = points.ToList();
        return list.Count == 2 ? (list[0], list[1]) : throw new ArgumentException("Line requires two pts");
    }
}

/// <summary>A straight line from a point in a direction to where it first meets another curve.</summary>
public class IntersectingLine : BaseEdgeObject
{
    /// <summary>A line from <paramref name="start"/> along <paramref name="direction"/> to the nearest intersection with <paramref name="other"/>.</summary>
    /// <exception cref="ArgumentException">The line doesn't meet the other curve.</exception>
    public IntersectingLine(Vector start, Vector direction, Shape other)
        : base(Make(start, direction, other))
    {
    }

    private static Edge Make(Vector start, Vector direction, Shape other)
    {
        direction = direction.Normalized();
        var axis = new Axis(start, direction);
        var points = other.SelectEdges().SelectMany(e => e.FindIntersectionPoints(axis)).ToList();
        if (points.Count == 0)
            throw new ArgumentException("No intersections found");
        var length = points.Min(p => (start - p.ToVector()).Length);
        return Edge.MakeLine(start, start + direction * length);
    }
}

/// <summary>
/// A straight line from a start point at an angle (or in a direction), of a length that is the line's own, or its
/// horizontal or vertical component; or to where it meets a limit.
/// </summary>
public class PolarLine : BaseEdgeObject
{
    /// <summary>A line of <paramref name="length"/> (measured as <paramref name="lengthMode"/>) at <paramref name="angle"/> degrees from the X axis or along <paramref name="direction"/>.</summary>
    /// <exception cref="ArgumentException">Neither the angle nor the direction is given.</exception>
    public PolarLine(Vector start, double length, double? angle = null, Vector? direction = null, LengthMode lengthMode = LengthMode.Diagonal)
        : base(Make(start, length, angle, direction, lengthMode))
    {
    }

    /// <summary>A line at <paramref name="angle"/> degrees from the X axis or along <paramref name="direction"/> to where it meets <paramref name="lengthLimit"/>.</summary>
    /// <exception cref="ArgumentException">Neither the angle nor the direction is given, or the line doesn't meet the limit.</exception>
    public PolarLine(Vector start, CurveLimit lengthLimit, double? angle = null, Vector? direction = null)
        : base(Make(start, lengthLimit, angle, direction))
    {
    }

    private static (Vector Direction, double Angle) Direction(double? angle, Vector? direction)
    {
        if (direction is { } d)
        {
            var normalized = d.Normalized();
            return (normalized, new Vector(1, 0, 0).GetAngle(normalized));
        }
        if (angle is { } a)
            return (new Vector(1, 0, 0).Rotate(Axis.Z, a), a);
        throw new ArgumentException("Give an angle or a direction.");
    }

    private static Edge Make(Vector start, double length, double? angle, Vector? direction, LengthMode lengthMode)
    {
        var (dir, a) = Direction(angle, direction);
        var lengthVector = lengthMode switch
        {
            LengthMode.Diagonal => dir * length,
            LengthMode.Horizontal => dir * Math.Abs(length / Math.Cos(a * Constants.DegToRad)),
            _ => dir * Math.Abs(length / Math.Sin(a * Constants.DegToRad)),
        };
        return Edge.MakeLine(start, start + lengthVector);
    }

    private static Edge Make(Vector start, CurveLimit limit, double? angle, Vector? direction)
    {
        var (dir, _) = Direction(angle, direction);
        if (limit.Target is Shape shape)
        {
            var maxLength = shape.BoundingBox().Add(start).Diagonal;
            var trimmed = Edge.MakeLine(start, start + dir * maxLength).TrimToOther(shape);
            return trimmed.IsEmpty ? throw new ArgumentException($"Polar line doesn't intersect length limit {limit}") : trimmed;
        }
        var polarAxis = new Axis(start, dir);
        IGeometry? contact = limit.Target switch
        {
            Axis axis => polarAxis.Intersect(axis),
            Plane plane => polarAxis.Intersect(plane),
            Location location => polarAxis.Intersect(location),
            _ => polarAxis.Intersect((Vector)limit.Target),
        };
        // A contact point that isn't behind the start
        if (contact is Vector point && (point - start).Dot(dir) > Constants.Tolerance)
            return Edge.MakeLine(start, point);
        throw new ArgumentException($"Polar line doesn't intersect length limit {limit}");
    }
}

/// <summary>Straight lines through successive points, optionally closed with an extra line.</summary>
public class Polyline : BaseLineObject
{
    /// <summary>A polyline through the points, closed if <paramref name="close"/>.</summary>
    /// <exception cref="ArgumentException">Fewer than two points.</exception>
    public Polyline(IEnumerable<Vector> points, bool close = false) : base(Make([.. points], close)) { }

    /// <summary>An open polyline through the points.</summary>
    public Polyline(params Vector[] points) : this((IEnumerable<Vector>)points) { }

    private static Wire Make(List<Vector> points, bool close)
    {
        if (points.Count < 2)
            throw new ArgumentException("Polyline requires two or more pts");
        var edges = Enumerable.Range(0, points.Count - 1).Select(i => Edge.MakeLine(points[i], points[i + 1])).ToList();
        if (close && (edges[0].PositionAt(0) - edges[^1].PositionAt(1)).Length > 1e-5)
            edges.Add(Edge.MakeLine(edges[^1].PositionAt(1), edges[0].PositionAt(0)));
        return Wire.Combine(edges)[0];
    }
}

/// <summary>
/// A circular arc between two points of a radius: the short way round (the short sagitta) or the long way. A negative
/// radius bends the arc the other way.
/// </summary>
public class RadiusArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="startPoint"/> to <paramref name="endPoint"/> of <paramref name="radius"/>.</summary>
    /// <exception cref="ArgumentException">The radius is too small to reach the end point.</exception>
    public RadiusArc(Vector startPoint, Vector endPoint, double radius, bool shortSagitta = true)
        : base(Make(startPoint, endPoint, radius, shortSagitta))
    {
    }

    private static Edge Make(Vector start, Vector end, double radius, bool shortSagitta)
    {
        var length = (end - start).Length / 2.0;
        var root = radius * radius - length * length;
        if (root < 0)
            throw new ArgumentException("Arc radius is not large enough to reach the end point.");
        var sagitta = shortSagitta ? Math.Abs(radius) - Math.Sqrt(root) : -Math.Abs(radius) - Math.Sqrt(root);
        return new SagittaArc(start, end, radius > 0 ? sagitta : -sagitta);
    }
}

/// <summary>A circular arc between two points of a sagitta: the height of the arc from the chord (negative to bend the other way).</summary>
public class SagittaArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="startPoint"/> to <paramref name="endPoint"/> of <paramref name="sagitta"/>.</summary>
    public SagittaArc(Vector startPoint, Vector endPoint, double sagitta)
        : base(Make(startPoint, endPoint, sagitta))
    {
    }

    private static Edge Make(Vector start, Vector end, double sagitta)
    {
        var midPoint = (end + start) * 0.5;
        var sagittaVector = ((end - start).Normalized() * Math.Abs(sagitta)).Rotate(Axis.Z, sagitta > 0 ? 90 : -90);
        return new ThreePointArc(start, midPoint + sagittaVector, end);
    }
}

/// <summary>
/// A spline through points, optionally constrained by tangents (at the ends only, or at every point) with optional
/// scalars for their magnitude.
/// </summary>
public class Spline : BaseEdgeObject
{
    /// <summary>A spline through the points, periodic (closed) if <paramref name="periodic"/>.</summary>
    public Spline(IEnumerable<Vector> points, IEnumerable<Vector>? tangents = null, IEnumerable<double>? tangentScalars = null,
                  bool periodic = false)
        : base(Make([.. points], tangents?.ToList(), tangentScalars?.ToList(), periodic))
    {
    }

    /// <summary>A spline through the points.</summary>
    public Spline(params Vector[] points) : this((IEnumerable<Vector>)points) { }

    private static Edge Make(List<Vector> points, List<Vector>? tangents, List<double>? scalars, bool periodic)
    {
        var unitTangents = tangents is { Count: > 0 } ? tangents.Select(t => t.Normalized()).ToList() : null;
        var tangentScalars = tangents is not null && scalars is null ? [.. Enumerable.Repeat(1.0, tangents.Count)] : scalars ?? [];
        List<Vector?>? scaled = unitTangents is null ? null : [.. unitTangents.Zip(tangentScalars, (t, s) => (Vector?)(t * s))];
        return Edge.MakeSpline(points, scaled, periodic, scale: scalars is null);
    }
}

/// <summary>A circular arc between two points with a tangent at the first (or, flipping the arc, at the last).</summary>
public class TangentArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="start"/> to <paramref name="end"/> along <paramref name="tangent"/> at the first point, or at the last.</summary>
    public TangentArc(Vector start, Vector end, Vector tangent, bool tangentFromFirst = true)
        : base(tangentFromFirst ? Edge.MakeTangentArc(start, tangent.Normalized(), end) : Edge.MakeTangentArc(end, tangent.Normalized(), start))
    {
    }

    /// <summary>An arc between two points along <paramref name="tangent"/> at the first point, or at the last.</summary>
    /// <exception cref="ArgumentException">Not exactly two points.</exception>
    public TangentArc(IEnumerable<Vector> points, Vector tangent, bool tangentFromFirst = true)
        : this(Points(points, 2, "tangent_arc requires two points"), tangent, tangentFromFirst)
    {
    }

    private TangentArc(List<Vector> points, Vector tangent, bool tangentFromFirst)
        : this(points[0], points[1], tangent, tangentFromFirst)
    {
    }

    internal static List<Vector> Points(IEnumerable<Vector> points, int count, string message)
    {
        var list = points.ToList();
        return list.Count == count ? list : throw new ArgumentException(message);
    }
}

/// <summary>A circular arc through three points.</summary>
public class ThreePointArc : BaseEdgeObject
{
    /// <summary>An arc from <paramref name="point1"/> through <paramref name="point2"/> to <paramref name="point3"/>.</summary>
    public ThreePointArc(Vector point1, Vector point2, Vector point3)
        : base(Edge.MakeThreePointArc(point1, point2, point3))
    {
    }

    /// <summary>An arc through three points.</summary>
    /// <exception cref="ArgumentException">Not exactly three points.</exception>
    public ThreePointArc(IEnumerable<Vector> points)
        : this(TangentArc.Points(points, 3, "ThreePointArc requires three points"))
    {
    }

    private ThreePointArc(List<Vector> points) : this(points[0], points[1], points[2]) { }
}
