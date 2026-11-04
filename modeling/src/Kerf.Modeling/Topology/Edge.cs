using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>
/// A one-dimensional piece of a curve (a line, an arc, a spline, …): the building block of wires, faces and the
/// operations on them such as fillets and chamfers.
/// </summary>
public partial class Edge : Shape1D
{
    /// <summary>The empty Edge.</summary>
    public Edge() : base(null) { }

    /// <summary>An edge wrapping an OCCT edge.</summary>
    public Edge(global::Kerf.Topology.Edge edge, string label = "", Color? color = null) : base(edge, label, color) { }

    /// <summary>An infinite line along an axis.</summary>
    public static Edge FromAxis(Axis axis)
    {
        var pnt = axis.Position.ToGpPnt();
        var dir = axis.Direction.ToGpDir();
        var line = new LineCurve3D(pnt, dir);
        return FromCurve(line);
    }

    internal global::Kerf.Topology.Edge TopoEdge => (global::Kerf.Topology.Edge)TopoShape;

    internal override double Order => 1.0;

    internal override Shape MakeZero() => new Edge();

    /// <summary>An edge along the whole of a curve.</summary>
    internal static Edge FromCurve(Curve3D curve) => new(Failures.Get(global::Kerf.Topology.Edge.FromCurve(curve), () => "edge"));

    /// <summary>The centre of the circle or ellipse the edge lies on.</summary>
    /// <exception cref="InvalidOperationException">The edge is neither.</exception>
    public Vector ArcCenter
    {
        get
        {
            var type = GeomType;
            var geom = GeomAdaptor();
            switch (type)
            {
                case GeomType.Circle:
                {
                    geom.TryGetCircle(out var c);
                    var pos = c.Position;
                    var loc = pos.Location;
                    return new Vector(loc);
                }
                case GeomType.Ellipse:
                {
                    geom.TryGetEllipse(out var c);
                    var pos = c.Position;
                    var loc = pos.Location;
                    return new Vector(loc);
                }
                default:
                    throw new InvalidOperationException($"{type} has no arc center");
            }
        }
    }

    /// <summary>Whether the edge is a line longer than 1e100: an axis made into an edge.</summary>
    public bool IsInfinite => GeomType == GeomType.Line && Length > 1e100;

    // ---- Factories ----

    /// <summary>A vertex extruded into an edge.</summary>
    public static Edge Extrude(Vertex obj, Vector direction)
    {
        if (obj.IsEmpty)
            throw new ArgumentException("Can't extrude empty vertex");
        return new Edge((global::Kerf.Topology.Edge)TopoUtil.Extrude(obj.TopoShape, direction));
    }

    /// <summary>
    /// A Bézier curve, rational if <paramref name="weights"/> (one per control point) are given. The first and last
    /// control points are the ends of the curve.
    /// </summary>
    public static Edge MakeBezier(IReadOnlyList<Vector> controlPoints, IReadOnlyList<double>? weights = null)
    {
        if (controlPoints.Count < 2)
            throw new ArgumentException("At least two control points must be provided (start, end)");
        if (controlPoints.Count > 25)
            throw new ArgumentException("The maximum number of control points is 25");
        if (weights is { Count: > 0 } && weights.Count != controlPoints.Count)
            throw new ArgumentException("A weight must be provided for each control point");
        var poles = PointArray(controlPoints);
        BezierCurve3D curve;
        if (weights is { Count: > 0 })
            curve = new BezierCurve3D(poles, [.. weights]);
        else
            curve = new BezierCurve3D(poles);
        return FromCurve(curve);
    }

    /// <inheritdoc cref="MakeBezier(IReadOnlyList{Vector}, IReadOnlyList{double}?)"/>
    public static Edge MakeBezier(params Vector[] controlPoints) => MakeBezier((IReadOnlyList<Vector>)controlPoints);

    /// <summary>A circle, or an arc of one, centred on the plane's origin (default XY).</summary>
    public static Edge MakeCircle(double radius, Plane? plane = null, double startAngle = 360.0, double endAngle = 360.0,
                                  AngularDirection angularDirection = AngularDirection.CounterClockwise)
    {
        plane ??= Plane.XY;
        var ax2 = plane.ToGpAx2();
        var circle = new Circle3D(ax2, radius);
        if (startAngle == endAngle)   // a full circle
            return new Edge(Failures.Get(global::Kerf.Topology.Edge.Circle(circle), () => "circle"));
        bool ccw = angularDirection == AngularDirection.CounterClockwise;
        var (start, end) = ccw ? (startAngle, endAngle) : (endAngle, startAngle);
        var arc = Failures.Get(TrimmedCurve3D.Arc(circle, start * Constants.DegToRad, end * Constants.DegToRad, ccw), () => "arc of circle");
        return FromCurve(arc);
    }

    /// <summary>An ellipse, or an arc of one, centred on the plane's origin (default XY).</summary>
    public static Edge MakeEllipse(double xRadius, double yRadius, Plane? plane = null, double startAngle = 360.0, double endAngle = 360.0,
                                   AngularDirection angularDirection = AngularDirection.CounterClockwise)
    {
        plane ??= Plane.XY;
        var origin = plane.Origin.ToGpPnt();
        var zDir = plane.ZDir.ToGpDir();
        var ax1 = new Axis3D(origin, zDir);
        var ax2 = plane.ToGpAx2();
        double correction;
        Ellipse3D ellipse;
        if (yRadius > xRadius)
        {
            // OCCT wants the major radius first: swap the radii and turn the ellipse a quarter turn
            correction = 90.0 * Constants.DegToRad;
            var swapped = new Ellipse3D(ax2, yRadius, xRadius);
            ellipse = swapped.Rotated(ax1, correction);
        }
        else
        {
            correction = 0.0;
            ellipse = new Ellipse3D(ax2, xRadius, yRadius);
        }
        {
            if (startAngle == endAngle)   // a full ellipse
                return FromCurve(new EllipseCurve3D(ellipse));
            double a1 = startAngle * Constants.DegToRad - correction;
            double a2 = endAngle * Constants.DegToRad - correction;
            double alpha1, alpha2;
            bool sense;
            if (angularDirection == AngularDirection.CounterClockwise)
            {
                if (a2 <= a1)
                    a2 += 2 * Math.PI;
                (sense, alpha1, alpha2) = (true, a1, a2);
            }
            else
            {
                if (a2 >= a1)
                    a2 -= 2 * Math.PI;
                (sense, alpha1, alpha2) = (false, a2, a1);
            }
            var arc = Failures.Get(TrimmedCurve3D.Arc(ellipse, alpha1, alpha2, sense), () => "arc of ellipse");
            return FromCurve(arc);
        }
    }

    /// <summary>An arc of a parabola with its vertex at the plane's origin, opening along the plane's x axis.</summary>
    public static Edge MakeParabola(double focalLength, Plane? plane = null, double startAngle = 0.0, double endAngle = 90.0,
                                    AngularDirection angularDirection = AngularDirection.CounterClockwise)
    {
        plane ??= Plane.XY;
        var ax2 = plane.ToGpAx2();
        var parabola = new Parabola3D(ax2, focalLength);
        var arc = Failures.Get(TrimmedCurve3D.Arc(parabola, startAngle * Constants.DegToRad, endAngle * Constants.DegToRad,
                                                  angularDirection == AngularDirection.CounterClockwise), () => "arc of parabola");
        return FromCurve(arc);
    }

    /// <summary>A hyperbola, or an arc of one, centred on the plane's origin (default XY).</summary>
    public static Edge MakeHyperbola(double xRadius, double yRadius, Plane? plane = null, double startAngle = 360.0, double endAngle = 360.0,
                                     AngularDirection angularDirection = AngularDirection.CounterClockwise)
    {
        plane ??= Plane.XY;
        var origin = plane.Origin.ToGpPnt();
        var zDir = plane.ZDir.ToGpDir();
        var ax1 = new Axis3D(origin, zDir);
        var ax2 = plane.ToGpAx2();
        double correction;
        Hyperbola3D hyperbola;
        if (yRadius > xRadius)
        {
            correction = 90.0 * Constants.DegToRad;
            var swapped = new Hyperbola3D(ax2, yRadius, xRadius);
            hyperbola = swapped.Rotated(ax1, correction);
        }
        else
        {
            correction = 0.0;
            hyperbola = new Hyperbola3D(ax2, xRadius, yRadius);
        }
        {
            if (startAngle == endAngle)
                return FromCurve(new HyperbolaCurve3D(hyperbola));
            var arc = Failures.Get(TrimmedCurve3D.Arc(hyperbola, startAngle * Constants.DegToRad - correction,
                                                      endAngle * Constants.DegToRad - correction,
                                                      angularDirection == AngularDirection.CounterClockwise), () => "arc of hyperbola");
            return FromCurve(arc);
        }
    }

    /// <summary>
    /// A helix of a given pitch, height and radius, on a cylinder or, with a cone <paramref name="angle"/> (degrees), on a cone.
    /// </summary>
    public static Edge MakeHelix(double pitch, double height, double radius, Vector? center = null, Vector? normal = null,
                                 double angle = 0.0, bool lefthand = false)
    {
        var c = (center ?? new Vector(0, 0, 0)).ToGpPnt();
        var n = (normal ?? new Vector(0, 0, 1)).ToGpDir();
        var ax3 = new CoordinateSystem(c, n);
        // The surface the helix is wrapped around
        Surface surface = angle == 0.0
            ? new CylindricalSurface(ax3, radius)
            : new ConicalSurface(ax3, angle * Constants.DegToRad, radius);
        // A segment in the surface's (u, v) domain
        var lineDir = new Vector((lefthand ? -1 : 1) * 2 * Math.PI, pitch).Normalized();
        var lineLength = height / lineDir.Y / Math.Cos(angle * Constants.DegToRad);
        var origin2d = new Point2D(0, 0);
        var dir2d = new Direction2D(lineDir.X, lineDir.Y);
        var line = new LineCurve2D(origin2d, dir2d);
        var curve = new TrimmedCurve2D(line, 0, lineLength, true, true);
        // Wrap the segment around the surface, then give the edge its 3D curve
        var onSurface = Failures.Get(global::Kerf.Topology.Edge.FromCurve(curve, surface), () => "helix");
        // Its 3D curve, as build123d builds it (whether or not that fully succeeds)
        var edge = Failures.Get(onSurface.WithCurves3D(1e-9, Continuity.C1, 14, 2000), () => "helix").Shape;
        return new Edge((global::Kerf.Topology.Edge)edge);
    }

    /// <summary>A line between two points.</summary>
    public static Edge MakeLine(Vector point1, Vector point2)
    {
        var p1 = point1.ToGpPnt();
        var p2 = point2.ToGpPnt();
        var line = global::Kerf.Topology.Edge.Line(p1, p2);
        if (!line.TryGetValue(out var edge))
            throw new ArgumentException($"Can't make a line from {point1} to {point2}: {line.Error!.Message}");
        return new Edge(edge);
    }

    /// <summary>
    /// A line between two edges, at the fraction <paramref name="middle"/> of the way from the first to the second.
    /// Edges running opposite ways are paired end to end so the line isn't truncated.
    /// </summary>
    public static Edge MakeMidWay(Edge first, Edge second, double middle = 0.5)
    {
        // The ends are paired by the way each edge runs, which PositionAt follows
        var firstRun = first.PositionAt(1) - first.PositionAt(0);
        var secondRun = second.PositionAt(1) - second.PositionAt(0);
        bool flip = firstRun.Dot(secondRun) < 0;
        var points = new[] { 0, 1 }
            .Select(i => MakeLine(first.PositionAt(i), second.PositionAt(flip ? 1 - i : i)).PositionAt(middle)).ToArray();
        return MakeLine(points[0], points[1]);
    }

    /// <summary>A spline interpolated through points.</summary>
    /// <param name="points">The points the spline passes through.</param>
    /// <param name="tangents">The start and end tangents, or one for each point (null where it isn't given).</param>
    /// <param name="periodic">Make a periodic curve.</param>
    /// <param name="parameters">The parameter at each point (one more if periodic).</param>
    /// <param name="scale">Scale the tangents to the derivative of the Lagrange interpolation, using only their direction.</param>
    /// <param name="tolerance">How close points and how short tangents may be.</param>
    public static Edge MakeSpline(IReadOnlyList<Vector> points, IReadOnlyList<Vector?>? tangents = null, bool periodic = false,
                                  IReadOnlyList<double>? parameters = null, bool scale = true, double tolerance = 1e-6)
    {
        var pnts = points.Select(p => p.ToGpPnt()).ToArray();
        if (parameters is not null && parameters.Count != points.Count + (periodic ? 1 : 0))
            throw new ArgumentException(
                "There must be one parameter for each interpolation point (plus one if periodic), or none specified. "
                + $"Parameter count: {parameters.Count}, point count: {points.Count}");
        System.Collections.Immutable.ImmutableArray<Vector3D> tangentArray = [];
        if (tangents is { Count: > 0 })
        {
            if (tangents.Count == 2 && points.Count != 2)
                // The start and end tangents only
                tangentArray = [(tangents[0] ?? new Vector(0, 0, 0)).ToGpVec(), (tangents[1] ?? new Vector(0, 0, 0)).ToGpVec()];
            else
            {
                if (tangents.Count != points.Count)
                    throw new ArgumentException(
                        "There must be one tangent for each interpolation point, or just two end point tangents. "
                        + $"Tangent count: {tangents.Count}, point count: {points.Count}");
                // A missing tangent is a zero one, which the kernel leaves free
                tangentArray = [.. tangents.Select(t => (t ?? new Vector(0, 0, 0)).ToGpVec())];
            }
        }
        var options = new InterpolationOptions
        {
            Periodic = periodic,
            Tangents = tangentArray,
            ScaleTangents = scale,
            Parameters = parameters is null ? [] : [.. parameters],
            Tolerance = tolerance,
        };
        var interpolated = BSplineCurve3D.Interpolate(pnts, options);
        if (!interpolated.TryGetValue(out var curve))
            throw new InvalidOperationException("B-spline interpolation failed");
        return FromCurve(curve);
    }

    /// <summary>
    /// An exact B-spline from control points and knots. Repeated knots become one knot with a multiplicity; optional
    /// weights (one per control point) make it rational.
    /// </summary>
    public static Edge MakeBSpline(IEnumerable<Vector> controlPoints, IEnumerable<double> knots, int degree,
                                   IEnumerable<double>? weights = null, bool periodic = false)
    {
        var knotList = knots.ToList();
        if (knotList.Count == 0)
            throw new ArgumentException("B-spline requires at least one knot");
        var uniqueKnots = new List<double> { knotList[0] };
        var multiplicities = new List<int> { 1 };
        foreach (var knot in knotList.Skip(1))
        {
            if (Math.Abs(knot - uniqueKnots[^1]) <= Constants.Tolerance)
                multiplicities[^1]++;
            else
            {
                uniqueKnots.Add(knot);
                multiplicities.Add(1);
            }
        }
        var poles = PointArray(controlPoints.ToList());
        var weightList = weights?.ToList() ?? [];
        var curve = new BSplineCurve3D(poles, [.. uniqueKnots], [.. multiplicities], degree, periodic,
                                       weightList.ToArray());
        return FromCurve(curve);
    }

    /// <summary>
    /// A spline approximating points within <paramref name="tolerance"/>, optionally with variational
    /// <paramref name="smoothing"/> weights (which raise the maximum degree to at least 6).
    /// </summary>
    public static Edge MakeSplineApprox(IReadOnlyList<Vector> points, double tolerance = 1e-3,
                                        (double, double, double)? smoothing = null, int minDeg = 1, int maxDeg = 6)
    {
        var array = points.Select(p => p.ToGpPnt()).ToArray();
        var options = smoothing is var (w1, w2, w3)
            // The smoothing asks for C2, whose Jacobi basis needs a degree of at least 6 (OCCT 8)
            ? new ApproximationOptions { Smoothing = (w1, w2, w3), MaxDegree = Math.Max(maxDeg, 6), Continuity = Continuity.C2, Tolerance = tolerance }
            : new ApproximationOptions { MinDegree = minDeg, MaxDegree = maxDeg, Continuity = Continuity.C2, Tolerance = tolerance };
        var approximated = BSplineCurve3D.Approximate(array, options);
        if (!approximated.TryGetValue(out var curve))
            throw new InvalidOperationException("B-spline approximation failed");
        return FromCurve(curve);
    }

    /// <summary>A circular arc from <paramref name="start"/>, leaving in the direction of <paramref name="tangent"/>, to <paramref name="end"/>.</summary>
    public static Edge MakeTangentArc(Vector start, Vector tangent, Vector end)
    {
        var p1 = start.ToGpPnt();
        var t = tangent.ToGpVec();
        var p2 = end.ToGpPnt();
        return FromCurve(Failures.Get(TrimmedCurve3D.ArcTangent(p1, t, p2), () => "tangent arc"));
    }

    /// <summary>A circular arc through three points.</summary>
    public static Edge MakeThreePointArc(Vector point1, Vector point2, Vector point3)
    {
        var p1 = point1.ToGpPnt();
        var p2 = point2.ToGpPnt();
        var p3 = point3.ToGpPnt();
        return FromCurve(Failures.Get(TrimmedCurve3D.ArcThrough(p1, p2, p3), () => "three-point arc"));
    }

    internal static Point3D[] PointArray(IReadOnlyList<Vector> points) => [.. points.Select(p => p.ToGpPnt())];

    // ---- Curve parameters ----

    /// <summary>The kernel's curve of the edge (OCCT's <c>BRepAdaptor_Curve</c>), for evaluation and lengths.</summary>
    public EdgeCurve GeomAdaptor()
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't find adaptor for empty edge");
        return TopoEdge.ToCurve();
    }

    internal override ShapeCurve Curve() => new(GeomAdaptor());

    internal override (EdgeCurve Curve, double Param, bool Forward) OcctParamAt(double position, PositionMode mode = PositionMode.Parameter)
    {
        var curve = GeomAdaptor();
        var length = curve.Length;
        bool forward = IsForward;
        double value;
        if (mode == PositionMode.Parameter)
            value = forward ? position : 1 - position;
        else
        {
            var own = Length;
            value = (forward ? position : own - position) / own;
        }
        return (curve, curve.ParameterAtLength(length * value, curve.Range.Min), forward);
    }

    /// <summary>
    /// The OCCT curve parameter at a normalised arc-length position, 0 at the start of the edge and 1 at its end as the
    /// edge is oriented (on a reversed edge, <c>ParamAt(0)</c> is larger than <c>ParamAt(1)</c>). A closed edge may give a
    /// value outside its nominal range.
    /// </summary>
    public override double ParamAt(double position)
    {
        var (curve, param, _) = OcctParamAt(position);
        return param;
    }

    /// <inheritdoc/>
    /// <remarks>
    /// A point at one of the edge's vertices uses that vertex's parameter. Otherwise the point is projected onto the curve,
    /// and if that fails to validate, the edge is searched numerically.
    /// </remarks>
    /// <exception cref="ArgumentException">The point isn't on the edge.</exception>
    public override double ParamAtPoint(Vector point, bool normalize = true)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't find param on empty edge");
        var (paramMin, paramMax) = TopoEdge.Range;

        // Method 1: the point is a vertex of the edge (on a closed edge, either end)
        var nearest = Vertices().MinBy(v => (v.ToVector() - point).Length);
        if (nearest is not null && !nearest.IsEmpty && (nearest.ToVector() - point).Length <= Constants.Tolerance)
        {
            var param = TopoEdge.ParameterOf(nearest.TopoVertex);
            var adaptor = TopoEdge.ToCurve();
            var u = adaptor.LengthBetween(paramMin, param) / adaptor.Length;
            // The length runs along the curve while PositionAt measures a reversed edge from its other end
            var normalized = IsForward ? u : 1 - u;
            return normalize ? normalized : param;
        }

        var separation = DistanceTo(point);
        if (!TopologyUtils.IsCloseB(separation, 0, absTol: Constants.Tolerance))
            throw new ArgumentException($"point ({point}) is {separation} from edge");

        // Method 2: project the point onto the curve. Some curves give a value outside the range.
        double projected;
        {
            TopoEdge.TryGetCurve(out var curve, out _);
            var pnt = point.ToGpPnt();
            if (curve is null || !curve.TryProjectNearest(pnt, out var nearestPoint))
                throw new InvalidOperationException("The point can't be projected onto the edge's curve");
            projected = nearestPoint.Parameter;
        }
        {
            var adaptor = TopoEdge.ToCurve();
            if (adaptor.IsPeriodic)
                projected = paramMin + PyMath.Mod(projected - paramMin, adaptor.Period);
            var u = adaptor.LengthBetween(paramMin, projected) / adaptor.Length;
            if (!IsForward)
                u = 1 - u;
            if ((PositionAt(u) - point).Length < Constants.Tolerance)
                return normalize ? u : projected;
        }

        // Method 3: search the edge for the point, minimising the distance over ever finer subdivisions of [0, 1]
        // (a B-spline can have several local minima). The first minimum below the curve's resolution wins.
        for (int division = 0; division < 10; division++)
        {
            int intervals = 1 << division;
            double step = 1.0 / intervals;
            for (int i = 0; i < intervals; i++)
            {
                var (x, fun) = PyMath.MinimizeBounded(u => (PositionAt(u) - point).Length, i * step, (i + 1) * step,
                                                       Constants.Tolerance / 2);
                if (fun < (PositionAt(x + Constants.Tolerance) - PositionAt(x - Constants.Tolerance)).Length)
                {
                    var normalized = Math.Round(x, Constants.ToleranceDigits);
                    return normalize ? normalized : ParamAt(normalized);
                }
            }
        }
        throw new InvalidOperationException("Unable to find parameter, Edge is too complex");
    }

    // ---- Edits ----

    /// <summary>
    /// A copy with the opposite orientation. With <paramref name="reconstruct"/> the curve itself is rebuilt reversed,
    /// rather than the orientation flag being set.
    /// </summary>
    public Edge Reversed(bool reconstruct = false)
    {
        if (IsEmpty)
            throw new InvalidOperationException("An empty edge can't be reversed");
        if (!reconstruct)
            return (Edge)Rewrapped(TopoShape.Reversed());
        TopoEdge.TryGetCurve(out var curve, out var range);
        var rFirst = curve!.ReversedParameter(range.Min);
        var rLast = curve.ReversedParameter(range.Max);
        var reversedCurve = curve.Reversed();
        return (Edge)Rewrapped(Failures.Get(global::Kerf.Topology.Edge.FromCurve(reversedCurve, new Interval(rLast, rFirst)), () => "reverse"));
    }

    /// <summary>A new edge of the section between two positions (normalised parameters or points on the edge).</summary>
    public Edge Trim(CurvePosition start, CurvePosition end)
    {
        var startU = start.ToParam(this);
        var endU = end.ToParam(this);
        var startPoint = PositionAt(startU);
        var (startCurve, paramStart, _) = OcctParamAt(startU);
        var (endCurve, paramEnd, _) = OcctParamAt(endU);
        Edge trimmed;
        {
            TopoEdge.TryGetCurve(out var curve, out _);
            var range = new Interval(Math.Min(paramStart, paramEnd), Math.Max(paramStart, paramEnd));
            trimmed = new Edge(Failures.Get(global::Kerf.Topology.Edge.FromCurve(curve!, range), () => "trim"));
        }
        // Reverse it if necessary
        bool sameStart = (trimmed.PositionAt(0) - startPoint).Length < Constants.Tolerance;
        bool sameDirection = TangentAt(startU).Dot(trimmed.TangentAt(0)) > 1 - Constants.Tolerance;
        return sameStart && sameDirection ? trimmed : trimmed.Reversed(reconstruct: true);
    }

    /// <summary>A new edge of a given length starting at a position (a normalised parameter or a point on the edge).</summary>
    public Edge TrimToLength(CurvePosition start, double length)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't trim empty edge");
        var startU = start.ToParam(this);
        TopoEdge.TryGetCurve(out var curve, out _);
        // Walk along the curve the way the edge is oriented
        var paramStart = ParamAt(startU);
        var paramEnd = curve!.ParameterAtLength(IsForward ? length : -length, paramStart);
        var trimmedCurve = new TrimmedCurve3D(curve, Math.Min(paramStart, paramEnd), Math.Max(paramStart, paramEnd));
        var edge = FromCurve(trimmedCurve);
        return IsForward ? edge : edge.Reversed(reconstruct: true);
    }

    /// <summary>The shortest piece of this edge from its start up to where it meets <paramref name="other"/>, or the empty Edge if they don't meet.</summary>
    public Edge TrimToOther(object other)
    {
        var intersections = Intersect(other);
        if (intersections.Count == 0)
            return new Edge();
        var trims = new ShapeList<Edge>(intersections.Vertices().Select(p => Trim(0, p.ToVector())));
        return trims.SortBy(e => e.Length)[0];
    }

    /// <summary>An infinite line trimmed to 2 × <paramref name="halfLength"/> about its centre; any other edge unchanged.</summary>
    public Edge TrimInfinite(double halfLength)
    {
        if (!IsInfinite)
            return this;
        var origin = Center();
        var direction = (EndPoint() - StartPoint()).Normalized();
        return MakeLine(origin - direction * halfLength, origin + direction * halfLength);
    }

    /// <summary>This edge if closed, else a wire closing it with a line.</summary>
    public Shape1D Close() => IsClosed ? this : new Wire([this]).Close();

    // ---- Queries ----

    /// <summary>The points where this planar edge crosses itself, or another edge or an axis in its plane.</summary>
    public ShapeList<Vertex> FindIntersectionPoints(object? other = null, double tolerance = Constants.Tolerance) =>
        new(FindIntersections(other, tolerance).Select(p => new Vertex(p)));

    /// <inheritdoc cref="FindIntersectionPoints"/>
    public IReadOnlyList<Vector> FindIntersections(object? other = null, double tolerance = Constants.Tolerance)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't find intersections of empty edge");
        Edge? otherEdge = other switch
        {
            null => null,
            Edge e => e,
            // An axis becomes an edge at least as large as this edge and the axis's start
            Axis axis => AxisEdge(axis),
            _ => throw new ArgumentException($"Can't intersect with a {other.GetType().Name}"),
        };
        var plane = CommonPlane(otherEdge) ?? throw new ArgumentException("All objects must be on the same plane");
        var pln = plane.ToGpPln();
        var selfCurve = CurveOnPlane(this, pln);
        var intersected = otherEdge is { IsEmpty: false }
            ? selfCurve.Intersect(CurveOnPlane(otherEdge, pln), tolerance)
            : selfCurve.SelfIntersections(tolerance);
        var crosses = new List<Vector>();
        foreach (var hit in Failures.Get(intersected, () => "intersect"))
            crosses.Add(plane.FromLocalCoords(new Vector(hit.Point.X, hit.Point.Y)));
        // The crossings may lie beyond the ends of the edges
        var valid = new List<Vector>();
        foreach (var pnt in crosses)
        {
            try
            {
                if (DistanceTo(pnt) <= Constants.Tolerance && (otherEdge is null || otherEdge.DistanceTo(pnt) <= Constants.Tolerance))
                    valid.Add(pnt);
            }
            catch (InvalidOperationException) { }
        }
        return valid;

        Edge AxisEdge(Axis axis)
        {
            var diagonal = BoundingBox().Add(new Vertex(axis.Position).BoundingBox()).Diagonal;
            return MakeLine(axis.Position - axis.Direction * diagonal, axis.Position + axis.Direction * diagonal);
        }
    }

    private static Curve2D CurveOnPlane(Edge edge, Plane3D plane)
    {
        edge.TopoEdge.TryGetCurveOnPlane(plane, out var curve, out _);
        return curve!;
    }

    /// <summary>The parameters (0 to 1) where the tangent of this planar edge is at <paramref name="angle"/> degrees.</summary>
    public IReadOnlyList<double> FindTangent(double angle)
    {
        angle = PyMath.Mod(angle, 360);
        if (GeomType == GeomType.Line)
            return TangentAngleAt(0) == angle ? [0] : [];
        // A curve of the tangent angle along the edge, intercepted by horizontal lines
        int periodic = IsClosed ? 1 : 0;   // a closed edge's end is its start
        var points = new List<Vector>();
        double? previous = null;
        double discontinuities = 0.0;
        for (int i = 0; i < 101 - periodic; i++)
        {
            var tangent = TangentAngleAt(i / 100.0) + discontinuities * 360;
            // An angle wrapping from 360 to 0 is a discontinuity: add 360 and intercept another line
            if (previous is { } p && Math.Abs(p - tangent) > 300)
            {
                discontinuities = Math.CopySign(1.0, p - tangent);
                tangent += 360 * discontinuities;
            }
            previous = tangent;
            points.Add(new Vector(i / 100.0, tangent));
        }
        var tanCurve = MakeSpline(points);
        var box = tanCurve.BoundingBox();
        int minRange = 360 * (int)Math.Floor(box.Min.Y / 360);
        int maxRange = 360 * (int)Math.Ceiling(box.Max.Y / 360);
        var intercepts = new List<Vector>();
        for (int i = minRange; i <= maxRange; i += 360)
        {
            var line = MakeLine(new Vector(0, angle + i, 0), new Vector(100, angle + i, 0));
            intercepts.AddRange(tanCurve.FindIntersections(line));
        }
        return [.. intercepts.Select(p => p.X)];
    }

    /// <summary>
    /// Whether two edges have the same geometry within <paramref name="tolerance"/>, whatever their topology: two edges
    /// made independently along the same curve are equal.
    /// </summary>
    public bool GeomEqual(Edge other, double tolerance = 1e-6, int numInterpolationPoints = 5)
    {
        if (GeomType != other.GeomType)
            return false;
        if (PositionAt(0) != other.PositionAt(0) || PositionAt(1) != other.PositionAt(1))
            return false;
        var ga1 = GeomAdaptor();
        var ga2 = other.GeomAdaptor();
        static Vector V(Point3D p) => new(p);
        static Vector D(Axis3D a) => new(a.Direction);
        switch (GeomType)
        {
            case GeomType.Line:
                return true;   // the ends define it
            case GeomType.Circle:
            {
                ga1.TryGetCircle(out var c1);
                ga2.TryGetCircle(out var c2);
                return Math.Abs(c1.Radius - c2.Radius) < tolerance && V(c1.Location) == V(c2.Location)
                       && D(c1.Axis) == D(c2.Axis);
            }
            case GeomType.Ellipse:
            {
                ga1.TryGetEllipse(out var e1);
                ga2.TryGetEllipse(out var e2);
                return Math.Abs(e1.MajorRadius - e2.MajorRadius) < tolerance
                       && Math.Abs(e1.MinorRadius - e2.MinorRadius) < tolerance
                       && V(e1.Location) == V(e2.Location) && D(e1.Axis) == D(e2.Axis);
            }
            case GeomType.Hyperbola:
            {
                ga1.TryGetHyperbola(out var h1);
                ga2.TryGetHyperbola(out var h2);
                return Math.Abs(h1.MajorRadius - h2.MajorRadius) < tolerance
                       && Math.Abs(h1.MinorRadius - h2.MinorRadius) < tolerance
                       && V(h1.Location) == V(h2.Location) && D(h1.Axis) == D(h2.Axis);
            }
            case GeomType.Parabola:
            {
                ga1.TryGetParabola(out var p1);
                ga2.TryGetParabola(out var p2);
                return Math.Abs(p1.Focal - p2.Focal) < tolerance && V(p1.Location) == V(p2.Location)
                       && D(p1.Axis) == D(p2.Axis);
            }
            case GeomType.Bezier:
            {
                ga1.TryGetBezier(out var b1);
                ga2.TryGetBezier(out var b2);
                if (b1!.Degree != b2!.Degree || b1.PoleCount != b2.PoleCount)
                    return false;
                for (int i = 0; i < b1.PoleCount; i++)
                {
                    if (V(b1.Poles[i]) != V(b2.Poles[i]))
                        return false;
                    if (b1.IsRational && Math.Abs(b1.Weights[i] - b2.Weights[i]) >= tolerance)
                        return false;
                }
                return true;
            }
            case GeomType.BSpline:
            {
                ga1.TryGetBSpline(out var s1);
                ga2.TryGetBSpline(out var s2);
                if (s1!.Degree != s2!.Degree || s1.IsPeriodic != s2.IsPeriodic
                    || s1.PoleCount != s2.PoleCount || s1.KnotCount != s2.KnotCount)
                    return false;
                for (int i = 0; i < s1.PoleCount; i++)
                {
                    if (V(s1.Poles[i]) != V(s2.Poles[i]))
                        return false;
                    if (s1.IsRational && Math.Abs(s1.Weights[i] - s2.Weights[i]) >= tolerance)
                        return false;
                }
                for (int i = 0; i < s1.KnotCount; i++)
                    if (Math.Abs(s1.Knots[i] - s2.Knots[i]) >= tolerance || s1.Multiplicities[i] != s2.Multiplicities[i])
                        return false;
                return true;
            }
            case GeomType.Offset:
            {
                ga1.TryGetOffsetCurve(out var o1);
                ga2.TryGetOffsetCurve(out var o2);
                if (Math.Abs(o1!.Offset - o2!.Offset) >= tolerance)
                    return false;
                var d1 = o1.Direction;
                var d2 = o2.Direction;
                if (new Vector(d1) != new Vector(d2))
                    return false;
                var basis1 = o1.BasisCurve;
                var basis2 = o2.BasisCurve;
                return FromCurve(basis1!).GeomEqual(FromCurve(basis2!), tolerance);
            }
            default:
                for (int i = 1; i <= numInterpolationPoints; i++)
                {
                    double t = i / (double)(numInterpolationPoints + 1);
                    if (PositionAt(t) != other.PositionAt(t))
                        return false;
                }
                return true;
        }
    }

    /// <summary>
    /// The edge projected onto a shape along <paramref name="direction"/>, or from a <paramref name="center"/>; one of the
    /// two must be given. The projections run the same way as this edge.
    /// </summary>
    public ShapeList<Edge> ProjectToShape(Shape targetObject, Vector? direction = null, Vector? center = null) =>
        new Wire([this]).ProjectToShape(targetObject, direction, center).Edges();
}
