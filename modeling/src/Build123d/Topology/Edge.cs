using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Build123d;

/// <summary>
/// A one-dimensional piece of a curve (a line, an arc, a spline, …): the building block of wires, faces and the
/// operations on them such as fillets and chamfers.
/// </summary>
public partial class Edge : Shape1D
{
    /// <summary>The empty Edge.</summary>
    public Edge() : base(null) { }

    /// <summary>An edge wrapping an OCCT edge.</summary>
    public Edge(TopoDS_Edge edge, string label = "", Color? color = null) : base(edge, label, color) { }

    /// <summary>An infinite line along an axis.</summary>
    public static Edge FromAxis(Axis axis)
    {
        var pnt = axis.Position.ToGpPnt();
        var dir = axis.Direction.ToGpDir();
        var line = new Geom_Line(pnt, dir);
        return FromCurve(line);
    }

    internal TopoDS_Edge TopoEdge => TopoDS.Edge(TopoShape);

    internal override double Order => 1.0;

    internal override Shape MakeZero() => new Edge();

    /// <summary>An edge along the whole of a curve.</summary>
    internal static Edge FromCurve(Geom_Curve curve)
    {
        var maker = new BRepBuilderAPI_MakeEdge(curve);
        return new Edge(maker.Edge());
    }

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
                    var c = geom.Circle();
                    var pos = c.Position();
                    var loc = pos.Location();
                    return new Vector(loc);
                }
                case GeomType.Ellipse:
                {
                    var c = geom.Ellipse();
                    var pos = c.Position();
                    var loc = pos.Location();
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
        return new Edge(TopoDS.Edge(TopoUtil.Extrude(obj.TopoShape, direction)));
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
        Geom_BezierCurve curve;
        if (weights is { Count: > 0 })
        {
            var w = DoubleArray(weights);
            curve = new Geom_BezierCurve(poles, w);
        }
        else
            curve = new Geom_BezierCurve(poles);
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
        {
            var maker = new BRepBuilderAPI_MakeEdge(circle);
            return new Edge(maker.Edge());
        }
        bool ccw = angularDirection == AngularDirection.CounterClockwise;
        var (start, end) = ccw ? (startAngle, endAngle) : (endAngle, startAngle);
        var arc = new GC_MakeArcOfCircle(circle, start * Constants.DegToRad, end * Constants.DegToRad, ccw);
        var geom = arc.Value();
        return FromCurve(geom!);
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
            {
                var maker = new BRepBuilderAPI_MakeEdge(ellipse);
                return new Edge(maker.Edge());
            }
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
            var arc = new GC_MakeArcOfEllipse(ellipse, alpha1, alpha2, sense);
            var geom = arc.Value();
            return FromCurve(geom!);
        }
    }

    /// <summary>An arc of a parabola with its vertex at the plane's origin, opening along the plane's x axis.</summary>
    public static Edge MakeParabola(double focalLength, Plane? plane = null, double startAngle = 0.0, double endAngle = 90.0,
                                    AngularDirection angularDirection = AngularDirection.CounterClockwise)
    {
        plane ??= Plane.XY;
        var ax2 = plane.ToGpAx2();
        var parabola = new Parabola3D(ax2, focalLength);
        var arc = new GC_MakeArcOfParabola(parabola, startAngle * Constants.DegToRad, endAngle * Constants.DegToRad,
                                                 angularDirection == AngularDirection.CounterClockwise);
        var geom = arc.Value();
        return FromCurve(geom!);
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
            {
                var maker = new BRepBuilderAPI_MakeEdge(hyperbola);
                return new Edge(maker.Edge());
            }
            var arc = new GC_MakeArcOfHyperbola(hyperbola, startAngle * Constants.DegToRad - correction,
                                                      endAngle * Constants.DegToRad - correction,
                                                      angularDirection == AngularDirection.CounterClockwise);
            var geom = arc.Value();
            return FromCurve(geom!);
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
        Geom_Surface surface = angle == 0.0
            ? new Geom_CylindricalSurface(ax3, radius)
            : new Geom_ConicalSurface(ax3, angle * Constants.DegToRad, radius);
        // A segment in the surface's (u, v) domain
        var lineDir = new Vector((lefthand ? -1 : 1) * 2 * Math.PI, pitch).Normalized();
        var lineLength = height / lineDir.Y / Math.Cos(angle * Constants.DegToRad);
        var origin2d = new Point2D(0, 0);
        var dir2d = new Direction2D(lineDir.X, lineDir.Y);
        var line = new Geom2d_Line(origin2d, dir2d);
        var curve = new Geom2d_TrimmedCurve(line, 0, lineLength, true, true);
        // Wrap the segment around the surface, then give the edge its 3D curve
        var builder = new BRepBuilderAPI_MakeEdge(curve, surface);
        var edge = BRepLib.Curves3dBuilt(builder.Edge(), 1e-9, out _, Continuity.C1, 14, 2000);
        return new Edge(TopoDS.Edge(edge));
    }

    /// <summary>A line between two points.</summary>
    public static Edge MakeLine(Vector point1, Vector point2)
    {
        var p1 = point1.ToGpPnt();
        var p2 = point2.ToGpPnt();
        var maker = new BRepBuilderAPI_MakeEdge(p1, p2);
        if (!maker.IsDone())
            throw new ArgumentException($"Can't make a line from {point1} to {point2}: {maker.Error()}");
        return new Edge(maker.Edge());
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
        var pnts = new NCollection_HArray1<Point3D>(1, points.Count);
        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i].ToGpPnt();
            pnts.SetValue(i + 1, p);
        }
        GeomAPI_Interpolate builder;
        if (parameters is null)
            builder = new GeomAPI_Interpolate(pnts, periodic, tolerance);
        else
        {
            if (parameters.Count != points.Count + (periodic ? 1 : 0))
                throw new ArgumentException(
                    "There must be one parameter for each interpolation point (plus one if periodic), or none specified. "
                    + $"Parameter count: {parameters.Count}, point count: {points.Count}");
            var parameterArray = new NCollection_HArray1<double>(1, parameters.Count);
            for (int i = 0; i < parameters.Count; i++)
                parameterArray.SetValue(i + 1, parameters[i]);
            builder = new GeomAPI_Interpolate(pnts, parameterArray, periodic, tolerance);
        }
        {
            if (tangents is { Count: > 0 })
            {
                if (tangents.Count == 2 && points.Count != 2)
                {
                    // The start and end tangents only
                    var t0 = (tangents[0] ?? new Vector(0, 0, 0)).ToGpVec();
                    var t1 = (tangents[1] ?? new Vector(0, 0, 0)).ToGpVec();
                    builder.Load(t0, t1, scale);
                }
                else
                {
                    if (tangents.Count != points.Count)
                        throw new ArgumentException(
                            "There must be one tangent for each interpolation point, or just two end point tangents. "
                            + $"Tangent count: {tangents.Count}, point count: {points.Count}");
                    var tangentArray = new NCollection_Array1<Vector3D>(1, tangents.Count);
                    var enabled = new NCollection_HArray1<bool>(1, tangents.Count);
                    for (int i = 0; i < tangents.Count; i++)
                    {
                        enabled.SetValue(i + 1, tangents[i] is not null);
                        var t = (tangents[i] ?? new Vector(0, 0, 0)).ToGpVec();
                        tangentArray.SetValue(i + 1, t);
                    }
                    builder.Load(tangentArray, enabled, scale);
                }
            }
            builder.Perform();
            if (!builder.IsDone())
                throw new InvalidOperationException("B-spline interpolation failed");
            var curve = builder.Curve();
            return FromCurve(curve!);
        }
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
        var knotArray = DoubleArray(uniqueKnots);
        var multArray = new NCollection_Array1<int>(1, multiplicities.Count);
        for (int i = 0; i < multiplicities.Count; i++)
            multArray.SetValue(i + 1, multiplicities[i]);
        var weightList = weights?.ToList() ?? [];
        Geom_BSplineCurve curve;
        if (weightList.Count > 0)
        {
            var w = DoubleArray(weightList);
            curve = new Geom_BSplineCurve(poles, w, knotArray, multArray, degree, periodic);
        }
        else
            curve = new Geom_BSplineCurve(poles, knotArray, multArray, degree, periodic);
        return FromCurve(curve);
    }

    /// <summary>
    /// A spline approximating points within <paramref name="tolerance"/>, optionally with variational
    /// <paramref name="smoothing"/> weights (which raise the maximum degree to at least 6).
    /// </summary>
    public static Edge MakeSplineApprox(IReadOnlyList<Vector> points, double tolerance = 1e-3,
                                        (double, double, double)? smoothing = null, int minDeg = 1, int maxDeg = 6)
    {
        var pnts = new NCollection_HArray1<Point3D>(1, points.Count);
        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i].ToGpPnt();
            pnts.SetValue(i + 1, p);
        }
        // GeomAPI_PointsToBSpline takes the plain array the handle holds
        var array = new NCollection_Array1<Point3D>(1, points.Count);
        for (int i = 1; i <= points.Count; i++)
        {
            var p = pnts.Value(i);
            array.SetValue(i, p);
        }
        GeomAPI_PointsToBSpline builder;
        if (smoothing is var (w1, w2, w3))
            // The smoothing asks for C2, whose Jacobi basis needs a degree of at least 6 (OCCT 8)
            builder = new GeomAPI_PointsToBSpline(array, w1, w2, w3, Math.Max(maxDeg, 6), Continuity.C2, tolerance);
        else
            builder = new GeomAPI_PointsToBSpline(array, minDeg, maxDeg, Continuity.C2, tolerance);
        {
            if (!builder.IsDone())
                throw new InvalidOperationException("B-spline approximation failed");
            var curve = builder.Curve();
            return FromCurve(curve!);
        }
    }

    /// <summary>A circular arc from <paramref name="start"/>, leaving in the direction of <paramref name="tangent"/>, to <paramref name="end"/>.</summary>
    public static Edge MakeTangentArc(Vector start, Vector tangent, Vector end)
    {
        var p1 = start.ToGpPnt();
        var t = tangent.ToGpVec();
        var p2 = end.ToGpPnt();
        var arc = new GC_MakeArcOfCircle(p1, t, p2);
        var geom = arc.Value();
        return FromCurve(geom!);
    }

    /// <summary>A circular arc through three points.</summary>
    public static Edge MakeThreePointArc(Vector point1, Vector point2, Vector point3)
    {
        var p1 = point1.ToGpPnt();
        var p2 = point2.ToGpPnt();
        var p3 = point3.ToGpPnt();
        var arc = new GC_MakeArcOfCircle(p1, p2, p3);
        var geom = arc.Value();
        return FromCurve(geom!);
    }

    internal static NCollection_Array1<Point3D> PointArray(IReadOnlyList<Vector> points)
    {
        var array = new NCollection_Array1<Point3D>(1, points.Count);
        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i].ToGpPnt();
            array.SetValue(i + 1, p);
        }
        return array;
    }

    internal static NCollection_Array1<double> DoubleArray(IReadOnlyList<double> values)
    {
        var array = new NCollection_Array1<double>(1, values.Count);
        for (int i = 0; i < values.Count; i++)
            array.SetValue(i + 1, values[i]);
        return array;
    }

    // ---- Curve parameters ----

    /// <inheritdoc/>
    public override BRepAdaptor_Curve GeomAdaptor()
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't find adaptor for empty edge");
        return new BRepAdaptor_Curve(TopoEdge);
    }

    internal override (BRepAdaptor_Curve Curve, double Param, bool Forward) OcctParamAt(double position, PositionMode mode = PositionMode.Parameter)
    {
        var curve = GeomAdaptor();
        var length = GCPnts_AbscissaPoint.Length(curve);
        bool forward = IsForward;
        double value;
        if (mode == PositionMode.Parameter)
            value = forward ? position : 1 - position;
        else
        {
            var own = Length;
            value = (forward ? position : own - position) / own;
        }
        var abscissa = new GCPnts_AbscissaPoint(curve, length * value, curve.FirstParameter());
        return (curve, abscissa.Parameter(), forward);
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
        double paramMin = 0, paramMax = 0;
        BRep_Tool.Range(TopoEdge, out paramMin, out paramMax);

        // Method 1: the point is a vertex of the edge (on a closed edge, either end)
        var nearest = Vertices().MinBy(v => (v.ToVector() - point).Length);
        if (nearest is not null && !nearest.IsEmpty && (nearest.ToVector() - point).Length <= Constants.Tolerance)
        {
            var param = BRep_Tool.Parameter(nearest.TopoVertex, TopoEdge);
            var adaptor = new BRepAdaptor_Curve(TopoEdge);
            var u = GCPnts_AbscissaPoint.Length(adaptor, paramMin, param) / GCPnts_AbscissaPoint.Length(adaptor);
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
            double first = 0, last = 0;
            var curve = BRep_Tool.Curve(TopoEdge, out first, out last);
            var pnt = point.ToGpPnt();
            var projector = new GeomAPI_ProjectPointOnCurve(pnt, curve!);
            projected = projector.LowerDistanceParameter();
        }
        {
            var adaptor = new BRepAdaptor_Curve(TopoEdge);
            if (adaptor.IsPeriodic())
                projected = paramMin + PyMath.Mod(projected - paramMin, adaptor.Period());
            var u = GCPnts_AbscissaPoint.Length(adaptor, paramMin, projected) / GCPnts_AbscissaPoint.Length(adaptor);
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
        double first = 0, last = 0;
        BRep_Tool.Range(TopoEdge, out first, out last);
        var curve = BRep_Tool.Curve(TopoEdge, out first, out last)!;
        var rFirst = curve.ReversedParameter(first);
        var rLast = curve.ReversedParameter(last);
        var reversedCurve = curve.Reversed();
        var maker = new BRepBuilderAPI_MakeEdge(reversedCurve, rLast, rFirst);
        return (Edge)Rewrapped(maker.Edge());
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
            double first = startCurve.FirstParameter(), last = startCurve.LastParameter();
            var curve = BRep_Tool.Curve(TopoEdge, out first, out last);
            var maker = new BRepBuilderAPI_MakeEdge(curve!, Math.Min(paramStart, paramEnd), Math.Max(paramStart, paramEnd));
            trimmed = new Edge(maker.Edge());
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
        double first = 0, last = 0;
        BRep_Tool.Range(TopoEdge, out first, out last);
        var curve = BRep_Tool.Curve(TopoEdge, out first, out last)!;
        var adaptor = new GeomAdaptor_Curve(curve);
        // Walk along the curve the way the edge is oriented
        var paramStart = ParamAt(startU);
        var abscissa = new GCPnts_AbscissaPoint(adaptor, IsForward ? length : -length, paramStart);
        var paramEnd = abscissa.Parameter();
        var trimmedCurve = new Geom_TrimmedCurve(curve, Math.Min(paramStart, paramEnd), Math.Max(paramStart, paramEnd));
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
        var planeFace = MakePlaneFace(plane);
        var surface = BRep_Tool.Surface(planeFace);
        var selfCurve = CurveOnPlane(this, surface!);
        Geom2dAPI_InterCurveCurve intersector = otherEdge is { IsEmpty: false }
            ? new Geom2dAPI_InterCurveCurve(selfCurve, CurveOnPlane(otherEdge, surface!), tolerance)
            : new Geom2dAPI_InterCurveCurve(selfCurve, tolerance);
        var crosses = new List<Vector>();
        for (int i = 1; i <= intersector.NbPoints(); i++)
        {
            var p = intersector.Point(i);
            crosses.Add(plane.FromLocalCoords(new Vector(p.X(), p.Y())));
        }
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

    private static Geom2d_Curve CurveOnPlane(Edge edge, Geom_Surface surface)
    {
        double first = 0, last = 0;
        BRep_Tool.Range(edge.TopoEdge, out first, out last);
        var plane = (surface as Geom_Plane)!;
        var loc = new Placement();
        return BRep_Tool.CurveOnPlane(edge.TopoEdge, plane, loc, out first, out last)!;
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
        static Vector D(Axis3D a) => new(a.Direction());
        switch (GeomType)
        {
            case GeomType.Line:
                return true;   // the ends define it
            case GeomType.Circle:
            {
                var c1 = ga1.Circle();
                var c2 = ga2.Circle();
                return Math.Abs(c1.Radius() - c2.Radius()) < tolerance && V(c1.Location()) == V(c2.Location())
                       && D(c1.Axis()) == D(c2.Axis());
            }
            case GeomType.Ellipse:
            {
                var e1 = ga1.Ellipse();
                var e2 = ga2.Ellipse();
                return Math.Abs(e1.MajorRadius() - e2.MajorRadius()) < tolerance
                       && Math.Abs(e1.MinorRadius() - e2.MinorRadius()) < tolerance
                       && V(e1.Location()) == V(e2.Location()) && D(e1.Axis()) == D(e2.Axis());
            }
            case GeomType.Hyperbola:
            {
                var h1 = ga1.Hyperbola();
                var h2 = ga2.Hyperbola();
                return Math.Abs(h1.MajorRadius() - h2.MajorRadius()) < tolerance
                       && Math.Abs(h1.MinorRadius() - h2.MinorRadius()) < tolerance
                       && V(h1.Location()) == V(h2.Location()) && D(h1.Axis()) == D(h2.Axis());
            }
            case GeomType.Parabola:
            {
                var p1 = ga1.Parabola();
                var p2 = ga2.Parabola();
                return Math.Abs(p1.Focal() - p2.Focal()) < tolerance && V(p1.Location()) == V(p2.Location())
                       && D(p1.Axis()) == D(p2.Axis());
            }
            case GeomType.Bezier:
            {
                var b1 = ga1.Bezier()!;
                var b2 = ga2.Bezier()!;
                if (b1.Degree() != b2.Degree() || b1.NbPoles() != b2.NbPoles())
                    return false;
                for (int i = 1; i <= b1.NbPoles(); i++)
                {
                    if (V(b1.Pole(i)) != V(b2.Pole(i)))
                        return false;
                    if (b1.IsRational() && Math.Abs(b1.Weight(i) - b2.Weight(i)) >= tolerance)
                        return false;
                }
                return true;
            }
            case GeomType.BSpline:
            {
                var s1 = ga1.BSpline()!;
                var s2 = ga2.BSpline()!;
                if (s1.Degree() != s2.Degree() || s1.IsPeriodic() != s2.IsPeriodic()
                    || s1.NbPoles() != s2.NbPoles() || s1.NbKnots() != s2.NbKnots())
                    return false;
                for (int i = 1; i <= s1.NbPoles(); i++)
                {
                    if (V(s1.Pole(i)) != V(s2.Pole(i)))
                        return false;
                    if (s1.IsRational() && Math.Abs(s1.Weight(i) - s2.Weight(i)) >= tolerance)
                        return false;
                }
                for (int i = 1; i <= s1.NbKnots(); i++)
                    if (Math.Abs(s1.Knot(i) - s2.Knot(i)) >= tolerance || s1.Multiplicity(i) != s2.Multiplicity(i))
                        return false;
                return true;
            }
            case GeomType.Offset:
            {
                var o1 = ga1.OffsetCurve()!;
                var o2 = ga2.OffsetCurve()!;
                if (Math.Abs(o1.Offset() - o2.Offset()) >= tolerance)
                    return false;
                var d1 = o1.Direction();
                var d2 = o2.Direction();
                if (new Vector(d1) != new Vector(d2))
                    return false;
                var basis1 = o1.BasisCurve();
                var basis2 = o2.BasisCurve();
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
