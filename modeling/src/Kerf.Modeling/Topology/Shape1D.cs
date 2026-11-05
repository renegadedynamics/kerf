using Kerf.Construction;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>
/// A place along an edge or wire: a normalised parameter (or length, by <see cref="PositionMode"/>), or a point on it.
/// </summary>
public readonly record struct CurvePosition
{
    /// <summary>The parameter or length, if the position is given as a number.</summary>
    public double? Value { get; }

    /// <summary>The point, if the position is given as a point on the curve.</summary>
    public Vector? Point { get; }

    private CurvePosition(double? value, Vector? point) => (Value, Point) = (value, point);

    /// <summary>A position given as a parameter or length.</summary>
    public static implicit operator CurvePosition(double value) => new(value, null);

    /// <summary>A position given as a point on the curve.</summary>
    public static implicit operator CurvePosition(Vector point) => new(null, point);

    /// <summary>A position given as a vertex on the curve.</summary>
    public static implicit operator CurvePosition(Vertex vertex) => new(null, vertex.ToVector());

    /// <summary>The normalised parameter on <paramref name="curve"/>: the number itself, or that of the point.</summary>
    internal double ToParam(Shape1D curve) => Value ?? curve.ParamAtPoint(Point!.Value);
}

/// <summary>
/// The methods shared by edges and wires (build123d's <c>Mixin1D</c>).
/// </summary>
public abstract partial class Shape1D : Shape
{
    private protected Shape1D(global::Kerf.Topology.Shape? obj, string label = "", Color? color = null) : base(obj, label, color) { }

    internal override int? Dim => 1;

    /// <summary>The volume of an edge or wire, which is zero.</summary>
    public override double Volume => 0.0;

    /// <summary>Whether the start and end points are the same.</summary>
    public bool IsClosed
    {
        get
        {
            if (IsEmpty)
                throw new InvalidOperationException("Can't determine if empty Edge or Wire is closed");
            return TopoShape.CheckClosed();
        }
    }

    /// <summary>Whether the edge or wire runs forward along its curve (rather than reversed).</summary>
    public bool IsForward
    {
        get
        {
            if (IsEmpty)
                throw new InvalidOperationException("Can't determine direction of empty Edge or Wire");
            return TopoShape.Orientation == global::Kerf.Topology.Orientation.Forward;
        }
    }

    /// <summary>The length.</summary>
    public double Length
    {
        get
        {
            return TopoShape.Length();
        }
    }

    /// <summary>The radius of the circle the edge lies on; for a wire, that of its first edge.</summary>
    /// <exception cref="InvalidOperationException">The curve isn't a circle.</exception>
    public double Radius
    {
        get
        {
            if (this is Wire)
                // A wire: its comp curve reports OtherCurve, so ask its first edge
                return Edges()[0].Radius;
            if (!((Edge)this).GeomAdaptor().TryGetCircle(out var circle))
                throw new InvalidOperationException("Shape could not be reduced to a circle");
            return circle.Radius;
        }
    }

    /// <summary>
    /// How the shape this edge was selected from sits around it: Convex on the outer edge of a box, Concave at the inner
    /// corner of a pocket, Smooth where a fillet meets the face it blends into, and Saddle for an edge that turns one way
    /// at one end and the other way at the other. A seam edge is Smooth.
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// The edge has no <see cref="Shape.TopoParent"/>, lies on one face only, or more than two faces meet at it.
    /// </exception>
    public override Convexity Convexity
    {
        get
        {
            var faces = OneD.ConnectedFaces(this);
            if (faces.Count == 1)
            {
                if (OneD.IsSeamOf((global::Kerf.Topology.Edge)TopoShape, faces[0]))
                    return Convexity.Smooth;
                throw new InvalidOperationException(
                    "this edge lies on one face only, so there is no crease to classify - convexity needs an edge shared by two faces");
            }
            if (faces.Count != 2)
                throw new InvalidOperationException($"{faces.Count} faces meet at this edge - convexity needs exactly two");
            return OneD.DihedralConvexity((global::Kerf.Topology.Edge)TopoShape, faces[0], faces[1]);
        }
    }

    /// <summary>Whether this is a concave edge of the shape it was selected from (<c>Convexity == Concave</c>).</summary>
    public bool IsInterior => Convexity == Convexity.Concave;

    // ---- Curve evaluation ----

    /// <summary>The kernel's curve: an edge's <see cref="EdgeCurve"/>, or a wire's <see cref="WireCurve"/>.</summary>
    internal abstract ShapeCurve Curve();

    /// <summary>
    /// The edge (as its curve) and the OCCT curve parameter at a position, and whether that edge runs forward.
    /// </summary>
    internal abstract (EdgeCurve Curve, double Param, bool Forward) OcctParamAt(double position, PositionMode mode = PositionMode.Parameter);

    /// <summary>The OCCT curve parameter at a normalised arc-length position (0 at the start, 1 at the end).</summary>
    public abstract double ParamAt(double position);

    /// <summary>
    /// The parameter of the point on this curve closest to <paramref name="point"/>: the arc-length fraction
    /// <see cref="PositionAt"/> uses, or with <paramref name="normalize"/> false the native OCCT parameter.
    /// </summary>
    public abstract double ParamAtPoint(Vector point, bool normalize = true);

    /// <summary>The point at a position along the curve.</summary>
    /// <remarks>
    /// Positions are normally within the shape: [0, 1] as a parameter or [0, length] as a length. Beyond that an edge
    /// extrapolates or wraps where its curve allows, while a wire clamps to its first or last point.
    /// </remarks>
    public Vector PositionAt(double position, PositionMode mode = PositionMode.Parameter)
    {
        var (curve, param, _) = OcctParamAt(position, mode);
        var p = curve.Evaluate(param);
        return new Vector(p);
    }

    /// <summary>The points at several positions along the curve.</summary>
    public IReadOnlyList<Vector> Positions(IEnumerable<double> distances, PositionMode mode = PositionMode.Parameter) =>
        [.. distances.Select(d => PositionAt(d, mode))];

    /// <summary>Points along the curve such that the polygon through them stays within <paramref name="deflection"/> of it.</summary>
    public IReadOnlyList<Vector> Positions(double deflection)
    {
        var (curve, parameters) = DeflectionParameters(deflection);
        return [.. parameters.Select(p => { var pnt = curve.Evaluate(p); return new Vector(pnt); })];
    }

    /// <summary>The curve and the parameters along it that keep a polygon through them within <paramref name="deflection"/>.</summary>
    private (ShapeCurve, IReadOnlyList<double>) DeflectionParameters(double deflection)
    {
        var curve = Curve();
        // GCPnts_UniformDeflection where the curve is C2, else GCPnts_QuasiUniformDeflection (the kernel's Discretize)
        var parameters = curve.Discretize(deflection);
        if (parameters.IsEmpty)
            throw new InvalidOperationException("Deflection calculation failed");
        return (curve, parameters);
    }

    /// <summary>The derivative of the curve of the given <paramref name="order"/> at a position or point.</summary>
    public Vector DerivativeAt(CurvePosition position, int order = 2, PositionMode mode = PositionMode.Parameter)
    {
        EdgeCurve curve;
        double param;
        bool closestForward;
        Shape1D closest = this;
        if (position.Value is { } value)
            (curve, param, closestForward) = OcctParamAt(value, mode);
        else
        {
            var point = position.Point!.Value;
            if (this is Wire)
                closest = Edges().MinBy(e => e.DistanceTo(point))!;
            var u = closest.ParamAtPoint(point);
            (curve, param, closestForward) = closest.OcctParamAt(u);
        }
        {
            var d = curve.DerivativeN(param, order);
            if (d.Magnitude == 0)
                return new Vector(0, 0, 0);
            var derivative = new Vector(d);
            // An odd derivative runs the way the shape does
            if (order % 2 == 1)
            {
                bool forward = this is Wire ? closestForward == IsForward : IsForward;
                if (!forward)
                    derivative = -derivative;
            }
            return derivative;
        }
    }

    /// <summary>The unit tangent at a position or point.</summary>
    /// <remarks>Beyond the shape an edge uses its extrapolated or periodic tangent, a wire its first or last.</remarks>
    public Vector TangentAt(CurvePosition position, PositionMode mode = PositionMode.Parameter) =>
        DerivativeAt(position, 1, mode).Normalized();

    /// <summary>The unit tangent at the middle of the curve.</summary>
    public Vector TangentAt() => TangentAt(0.5);

    /// <summary>The angle in degrees (0 to 360) of the tangent at a position, measured in <paramref name="plane"/> (default XY).</summary>
    public double TangentAngleAt(double locationParam = 0.5, PositionMode mode = PositionMode.Parameter, Plane? plane = null)
    {
        plane ??= Plane.XY;
        var tangent = TangentAt(locationParam, mode);
        return Numerics.FloorMod(plane.XDir.GetSignedAngle(tangent, plane.ZDir) + 360, 360.0);
    }

    /// <summary>The start point. A circle may start and end at the same point.</summary>
    public Vector StartPoint()
    {
        var curve = Curve();
        var p = curve.Evaluate(IsForward ? curve.Range.Min : curve.Range.Max);
        return new Vector(p);
    }

    /// <summary>The end point. A circle may start and end at the same point.</summary>
    public Vector EndPoint()
    {
        var curve = Curve();
        var p = curve.Evaluate(IsForward ? curve.Range.Max : curve.Range.Min);
        return new Vector(p);
    }

    /// <summary>The centre: the point half way along (by default), the centre of mass, or of the bounding box.</summary>
    public override Vector Center() => Center(CenterOf.Geometry);

    /// <inheritdoc cref="Center()"/>
    public override Vector Center(CenterOf centerOf)
    {
        NeedsGeometry("center");
        switch (centerOf)
        {
            case CenterOf.Geometry:
                return PositionAt(0.5);
            case CenterOf.Mass:
                return new Vector(TopoShape.MassProperties(Dimension.Length).CentreOfMass);
            default:
                return BoundingBox().Center();
        }
    }

    // ---- Locations ----

    /// <summary>The local coordinate system at a position along the curve: x along the normal, z along the tangent.</summary>
    /// <param name="distance">Normalised parameter, or length.</param>
    /// <param name="mode">How <paramref name="distance"/> is measured.</param>
    /// <param name="frameMethod">
    /// The moving frame: Frenet can twist or flip near flat spots; Corrected behaves like a sweep profile, smoother and
    /// more stable.
    /// </param>
    /// <param name="xDir">An x direction to use instead, which must be perpendicular to the tangent.</param>
    public Location LocationAt(double distance, PositionMode mode = PositionMode.Parameter,
                               FrameMethod frameMethod = FrameMethod.Frenet, Vector? xDir = null)
    {
        ShapeCurve curve;
        double param;
        // ParamAt and OcctParamAt measure a reversed shape from its other end
        if (this is Wire)
        {
            if (frameMethod == FrameMethod.Corrected)
            {
                // A comp-curve parameter
                curve = Curve();
                param = ParamAt(mode == PositionMode.Parameter ? distance : distance / Length);
            }
            else
            {
                // An edge-curve parameter
                var (c, p, _) = OcctParamAt(mode == PositionMode.Parameter ? distance * Length : distance, PositionMode.Length);
                (curve, param) = (new ShapeCurve(c), p);
            }
        }
        else
        {
            param = ParamAt(mode == PositionMode.Parameter ? distance : distance / Length);
            curve = Curve();
        }
        return FrameLocation(curve.Frames([param], Law(frameMethod))[0], xDir);
    }

    /// <summary>The local coordinate systems at several positions along the curve.</summary>
    public IReadOnlyList<Location> Locations(IEnumerable<double> distances, PositionMode mode = PositionMode.Parameter,
                                             FrameMethod frameMethod = FrameMethod.Frenet, Vector? xDir = null) =>
        [.. distances.Select(d => LocationAt(d, mode, frameMethod, xDir))];

    /// <summary>
    /// Local coordinate systems wherever the curve needs one for the polygon through them to stay within
    /// <paramref name="deflection"/> of it, found in one pass over the whole curve.
    /// </summary>
    public IReadOnlyList<Location> Locations(double deflection, FrameMethod frameMethod = FrameMethod.Frenet, Vector? xDir = null)
    {
        var (curve, parameters) = DeflectionParameters(deflection);
        return [.. curve.Frames([.. parameters], Law(frameMethod)).Select(f => FrameLocation(f, xDir))];
    }

    private static FrameLaw Law(FrameMethod method) => method == FrameMethod.Frenet ? FrameLaw.Frenet : FrameLaw.CorrectedFrenet;

    /// <summary>The Location of a frame on the curve: at its point, its Z along the tangent.</summary>
    private Location FrameLocation(CurveFrame frame, Vector? xDir)
    {
        var (pnt, tangent, normal, _) = frame;
        var trsf = new Transform3D();
        var zDir = new Direction3D(tangent);
        var target = new CoordinateSystem();
        if (xDir is { } x)
        {
            try
            {
                var xd = x.ToDirection3D();
                var ax3 = new CoordinateSystem(pnt, zDir, xd);
                trsf = trsf.WithTransformation(ax3, target);
            }
            catch (Exception e) when (Diagnostic.IsKernelFailure(e) && Diagnostic.FromException(e).Code.Value == "engine.construction-error")
            {
                throw new ArgumentException(
                    $"Unable to create location with given x_dir {x}. x_dir must be perpendicular to shape's tangent {new Vector(tangent)}.", e);
            }
        }
        else
        {
            var nd = new Direction3D(normal);
            var ax3 = new CoordinateSystem(pnt, zDir, nd);
            trsf = trsf.WithTransformation(ax3, target);
        }
        var loc = new Location(trsf);
        return IsForward ? loc : -loc;
    }

    /// <summary>Locations evenly spread along the curve, from <paramref name="start"/> to <paramref name="stop"/>.</summary>
    /// <param name="count">How many (two or more).</param>
    /// <param name="start">Where to start, as a parameter.</param>
    /// <param name="stop">Where to stop, as a parameter.</param>
    /// <param name="positionsOnly">Leave out the orientation.</param>
    public IReadOnlyList<Location> DistributeLocations(int count, double start = 0.0, double stop = 1.0, bool positionsOnly = false)
    {
        if (count < 2)
            throw new ArgumentException("count must be two or greater", nameof(count));
        var locations = Locations(Enumerable.Range(0, count).Select(i => start + i * (stop - start) / (count - 1)));
        return positionsOnly ? [.. locations.Select(l => l.WithOrientation(new Vector(0, 0, 0)))] : locations;
    }

    // ---- Geometry ----

    /// <summary>
    /// The plane containing this and the other edges or wires, or null if there is none. Coaxial lines lie in infinitely
    /// many planes, of which one is chosen.
    /// </summary>
    public Plane? CommonPlane(IEnumerable<Shape1D?> lines, double tolerance = Constants.Tolerance)
    {
        var allLines = lines.OfType<Shape1D>().Prepend(this).ToList();
        if (allLines.Any(l => l.IsEmpty))
            return null;   // nothing lies in no plane
        Plane? result = null;

        // All co-axial: pick one of the infinite planes
        var allEdges = allLines.SelectMany(l => l.Edges()).ToList();
        if (allEdges.All(e => e.GeomType == GeomType.Line))
        {
            var asAxis = allEdges.Select(e => new Axis(e.PositionAt(0), e.TangentAt(0))).ToList();
            bool coaxial = true;
            for (int i = 0; i < asAxis.Count && coaxial; i++)
                for (int j = i + 1; j < asAxis.Count && coaxial; j++)
                    coaxial = asAxis[i].IsCoaxial(asAxis[j]);
            if (coaxial)
                result = new Plane(asAxis[0].Position, zDir: new Plane(asAxis[0]).XDir);
        }

        if (result is null)
        {
            // Shorten infinite lines (from axes)
            var normalLines = allLines.Where(l => l.Length <= 1e50);
            var shortened = allLines.Where(l => l.Length > 1e50).Select(l => (Shape1D)((Edge)l).TrimToLength(0.5, 10));
            allLines = [.. normalLines, .. shortened];
            var points = new List<Vector>();
            foreach (var line in allLines)
            {
                int n = line.GeomType == GeomType.Line ? 2 : 8;
                points.AddRange(Enumerable.Range(0, n).Select(i => line.PositionAt(i / (double)(n - 1))));
            }
            points = PointOrder.Distinct(points);   // the distinct points, in build123d's order
            // The points making the largest triangle make the most accurate plane; on ties the last such found wins
            double bestArea = double.NegativeInfinity;
            (Vector, Vector, Vector) extremes = default;
            for (int i = 0; i < points.Count; i++)
                for (int j = i + 1; j < points.Count; j++)
                    for (int k = j + 1; k < points.Count; k++)
                    {
                        var area = 0.5 * (points[j] - points[i]).Cross(points[k] - points[i]).Length;
                        if (area >= bestArea)
                            (bestArea, extremes) = (area, (points[i], points[j], points[k]));
                    }
            var (e0, e1, e2) = extremes;
            var zDir = (e2 - e0).Cross((e1 - e0).Normalized());
            Plane? candidate;
            try
            {
                candidate = new Plane((e0 + e1 + e2) / 3, zDir: zDir);
            }
            catch (ArgumentException e) when (!Diagnostic.IsKernelFailure(e))
            {
                candidate = null;   // no valid common plane
            }
            result = candidate is not null && points.All(p => candidate.Contains(p, tolerance)) ? candidate : null;
        }
        if (result is null)
            return null;

        // Centre the plane on the lines
        var globalCenter = allLines.Aggregate(new Vector(0, 0, 0), (sum, l) => sum + l.PositionAt(0.5)) / allLines.Count;
        var origin = (Vector)result.Intersect(new Axis(globalCenter, result.ZDir))!;
        return result.ShiftOrigin(origin);
    }

    /// <inheritdoc cref="CommonPlane(IEnumerable{Shape1D?}, double)"/>
    public Plane? CommonPlane(params Shape1D?[] lines) => CommonPlane((IEnumerable<Shape1D?>)lines);

    /// <summary>The normal of a planar curve.</summary>
    /// <exception cref="InvalidOperationException">The curve isn't planar.</exception>
    public Vector Normal()
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't find normal of empty edge/wire");
        switch (GeomType)
        {
            case GeomType.Circle:
            {
                ((Edge)this).GeomAdaptor().TryGetCircle(out var circ);
                var axis = circ.Axis;
                var dir = axis.Direction;
                return new Vector(dir);
            }
            case GeomType.Ellipse:
            {
                ((Edge)this).GeomAdaptor().TryGetEllipse(out var ell);
                var axis = ell.Axis;
                var dir = axis.Direction;
                return new Vector(dir);
            }
            default:
            {
                TopoShape.TryFindSurface(-1, out var surface, onlyPlane: true);
                var plane = (surface as PlaneSurface) ?? throw new InvalidOperationException("Normal not defined");
                var pln = plane.Plane;
                var axis = pln.Axis;
                var dir = axis.Direction;
                return new Vector(dir);
            }
        }
    }

    /// <summary>A line on <paramref name="plane"/> (default XY) of a given length, perpendicular to and centred on the curve at a position.</summary>
    public Edge PerpendicularLine(double length, double uValue, Plane? plane = null)
    {
        plane ??= Plane.XY;
        var start = PositionAt(uValue);
        var local = new Plane(start, TangentAt(uValue), plane.ZDir);
        return Edge.MakeLine(start + local.YDir * length / 2, start - local.YDir * length / 2);
    }

    /// <summary>
    /// The curvature comb of a curve in the XY plane: short lines erected perpendicular to the curve, of a length
    /// proportional to the signed curvature, for inspecting fairness and continuity.
    /// </summary>
    /// <param name="count">How many samples along the curve.</param>
    /// <param name="maxToothSize">The tallest tooth; by default a tenth of the curve's largest dimension.</param>
    public ShapeList<Edge> CurvatureComb(int count = 100, double? maxToothSize = null)
    {
        if (IsEmpty)
            throw new InvalidOperationException("An empty curve has no curvature comb.");
        var pln = CommonPlane();
        if (pln is null || !TopologyUtils.IsCloseB(Math.Abs(pln.ZDir.Z), 1.0, absTol: Constants.Tolerance))
            throw new InvalidOperationException("A curvature comb needs a curve on Plane.XY.");
        // A closed curve's first and last tooth would be the same, so the last is left out
        bool endpoint = !IsClosed;
        var uValues = Enumerable.Range(0, count)
            .Select(i => count == 1 ? 0.0 : i / (double)(endpoint ? count - 1 : count)).ToList();
        var kappas = new List<double>();
        var tangents = new List<Vector>();
        foreach (var u in uValues)
        {
            var tangent = DerivativeAt(u, 1);
            var curvature = DerivativeAt(u, 2);
            tangents.Add(tangent);
            var cross = tangent.Cross(curvature);
            var kappa = cross.Length / (Math.Pow(tangent.Length, 3) + Constants.Tolerance);
            kappas.Add(cross.Z >= 0 ? kappa : -kappa);
        }
        var maxKappa = Math.Max(Constants.Tolerance, kappas.Max(Math.Abs));
        var size = BoundingBox().Size;
        var tooth = maxToothSize ?? Math.Max(size.X, Math.Max(size.Y, size.Z)) / 10;
        var scale = tooth / maxKappa;
        var comb = new List<Edge>();
        for (int i = 0; i < uValues.Count; i++)
        {
            var length = scale * kappas[i];
            if (Math.Abs(length) < Constants.Tolerance)
                continue;   // avoid tiny teeth
            var onCurve = PositionAt(uValues[i]);
            var kappaDir = tangents[i].Normalized().Cross(new Vector(0, 0, 1));
            comb.Add(Edge.MakeLine(onCurve, onCurve + kappaDir * length));
        }
        return new ShapeList<Edge>(comb);
    }

    /// <summary>The curve projected onto a face along a direction: the projection closest to the curve, or all of them.</summary>
    public ShapeList<Shape1D> Project(Face face, Vector direction, bool closest = true)
    {
        if (IsEmpty || face.IsEmpty)
            throw new InvalidOperationException("Can't project an empty Edge or Wire onto empty Face");
        var dir = direction.ToDirection3D();
        var projected = TopoShape.IsEdge()
            ? ShapeProjections.Project((global::Kerf.Topology.Edge)TopoShape, face.TopoShape, dir)
            : ShapeProjections.Project((global::Kerf.Topology.Wire)TopoShape, face.TopoShape, dir);
        // A projection that meets nothing is no pieces
        var pieces = projected.TryGetValue(out var wires) ? wires.Select(w => w.AsShape()).ToList() : [];
        if (!closest)
            return new ShapeList<Shape1D>(pieces.Select(p => (Shape1D)Cast(p)));
        Shape1D? best = null;
        double minDist = double.PositiveInfinity;
        foreach (var piece in pieces)
        {
            var d = Failures.Get(TopoShape.Distance(piece), () => "distance").Value;
            if (d < minDist)
                (minDist, best) = (d, (Shape1D)Cast(piece));
        }
        return best is null ? [] : [best];
    }

    // ---- Combining ----

    /// <summary>
    /// This edge or wire joined with other edges and wires (build123d's <c>+</c>): connected pieces make a wire,
    /// others a fused curve; a single edge is returned as an Edge.
    /// </summary>
    internal override Shape AddCore(IEnumerable<Shape?> other) => OneD.Add1D(this, other);

    internal override ShapeList<Shape>? IntersectCore(object other, double tolerance, bool includeTouched)
    {
        other = other switch
        {
            Vector v => new Vertex(v),
            Location l => new Vertex(l.Position),
            Axis a => Edge.FromAxis(a),
            _ => other,
        };
        var results = new List<Shape>();
        // Trim infinite edges before OCCT operations
        if (other is Edge { IsInfinite: true } infinite)
        {
            var box = BoundingBox(optimal: false);
            other = infinite.TrimInfinite(box.Diagonal + (infinite.Center() - box.Center()).Length);
        }
        switch (other)
        {
            case Plane plane:
            {
                // 1D + plane: a section with the plane's face
                var face = Face.FromTopoFace(MakePlaneFace(plane));
                results.AddRange(BoolOpList([this], [face], BooleanKind.Section).Expand());
                break;
            }
            case Shape1D line:
            {
                // 1D + 1D: the common part (collinear overlap) and the section (crossings)
                results.AddRange(BoolOpList([this], [line], BooleanKind.Common).Expand());
                results.AddRange(BoolOpList([this], [line], BooleanKind.Section).OfType<Vertex>().Where(v => !v.IsEmpty));
                break;
            }
            case Vertex vertex:
                if (vertex.DistanceTo(this) <= tolerance)
                    results.Add(vertex);
                break;
            case Shape shape:
                // Delegate to higher-order shapes
                if (shape.IntersectCore(this, tolerance, includeTouched) is { } found)
                    results.AddRange(found);
                break;
        }
        return results.Count > 0 ? new ShapeList<Shape>(results) : null;
    }

    // ---- Operators ----


}

/// <summary>The kernel's curve of an edge or a wire, for the code the two share (OCCT's adaptors, once).</summary>
internal sealed class ShapeCurve
{
    private readonly EdgeCurve? _edge;
    private readonly WireCurve? _wire;

    internal ShapeCurve(EdgeCurve edge) => _edge = edge;

    internal ShapeCurve(WireCurve wire) => _wire = wire;

    public Interval Range => _edge?.Range ?? _wire!.Range;

    public Continuity Continuity => _edge?.Continuity ?? _wire!.Continuity;

    public Point3D Evaluate(double u) => _edge?.Evaluate(u) ?? _wire!.Evaluate(u);

    public Vector3D DerivativeN(double u, int n) => _edge?.DerivativeN(u, n) ?? _wire!.DerivativeN(u, n);

    public double Length => _edge?.Length ?? _wire!.Length;

    public double LengthBetween(double u1, double u2) => _edge?.LengthBetween(u1, u2) ?? _wire!.LengthBetween(u1, u2);

    public double ParameterAtLength(double length, double from) =>
        _edge?.ParameterAtLength(length, from) ?? _wire!.ParameterAtLength(length, from);

    public System.Collections.Immutable.ImmutableArray<double> Discretize(double deflection) =>
        _edge?.Discretize(deflection) ?? _wire!.Discretize(deflection);

    public System.Collections.Immutable.ImmutableArray<CurveFrame> Frames(ReadOnlySpan<double> parameters, FrameLaw law) =>
        _edge is not null ? _edge.Frames(parameters, law) : _wire!.Frames(parameters, law);
}
