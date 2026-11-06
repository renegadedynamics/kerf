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
    public Point3D? Point { get; }

    private CurvePosition(double? value, Point3D? point) => (Value, Point) = (value, point);

    /// <summary>A position given as a parameter or length.</summary>
    public static implicit operator CurvePosition(double value) => new(value, null);

    /// <summary>A position given as a point on the curve.</summary>
    public static implicit operator CurvePosition(Point3D point) => new(null, point);

    /// <summary>A position given as a vertex on the curve.</summary>
    public static implicit operator CurvePosition(Vertex vertex) => new(null, vertex.Point);

    /// <summary>The normalised parameter on <paramref name="curve"/>: the number itself, or that of the point.</summary>
    internal double ToParam(IShape1D curve) => Value ?? curve.ParamAtPoint(Point!.Value);
}

/// <summary>
/// The methods shared by edges and wires (build123d's <c>Mixin1D</c>).
/// </summary>
public static partial class Shape1DExtensions
{
    /// <summary>How <paramref name="parent"/> sits around an edge of it (see <c>ConvexityOf</c>).</summary>
    internal static Convexity ConvexityIn(IShape1D edge, Shape parent)
    {
        var faces = OneD.ConnectedFaces(edge, parent);
        if (faces.Count == 1)
        {
            if (OneD.IsSeamOf((Edge)edge.AsShape(), faces[0]))
                return Convexity.Smooth;
            throw new InvalidOperationException(
                "this edge lies on one face only, so there is no crease to classify - convexity needs an edge shared by two faces");
        }
        if (faces.Count != 2)
            throw new InvalidOperationException($"{faces.Count} faces meet at this edge - convexity needs exactly two");
        return OneD.DihedralConvexity((Edge)edge.AsShape(), faces[0], faces[1]);
    }

    extension<TSelf>(TSelf self) where TSelf : IShape1D
    {

        /// <summary>Whether the edge or wire runs forward along its curve (rather than reversed).</summary>
        public bool IsForward
        {
            get
            {
                if (self.IsEmpty)
                    throw new InvalidOperationException("Can't determine direction of empty Edge or Wire");
                return self.AsShape().Orientation == Orientation.Forward;
            }
        }

        /// <summary>The length.</summary>
        public double Length => self.IsEmpty ? 0.0 : self.AsShape().IntegrateLength();

        /// <summary>The radius of the circle the edge lies on; for a wire, that of its first edge.</summary>
        /// <exception cref="InvalidOperationException">The curve isn't a circle.</exception>
        public double Radius
        {
            get
            {
                if (self.AsShape().TryAs<Wire>(out _))
                    // A wire: its comp curve reports OtherCurve, so ask its first edge
                    return self.Edges()[0].Radius;
                if (!((Edge)self.AsShape()).AsCurve().TryGetCircle(out var circle))
                    throw new InvalidOperationException("Shape could not be reduced to a circle");
                return circle.Radius;
            }
        }

        /// <summary>The point at a position along the curve.</summary>
        /// <remarks>
        /// Positions are normally within the shape: [0, 1] as a parameter or [0, length] as a length. Beyond that an edge
        /// extrapolates or wraps where its curve allows, while a wire clamps to its first or last point.
        /// </remarks>
        public Point3D PositionAt(double position, PositionMode mode = PositionMode.Parameter)
        {
            var (curve, param, _) = self.OcctParamAt(position, mode);
            var p = curve.Evaluate(param);
            return p;
        }

        /// <summary>The points at several positions along the curve.</summary>
        public IReadOnlyList<Point3D> Positions(IEnumerable<double> distances, PositionMode mode = PositionMode.Parameter) =>
            [.. distances.Select(d => self.PositionAt(d, mode))];

        /// <summary>Points along the curve such that the polygon through them stays within <paramref name="deflection"/> of it.</summary>
        public IReadOnlyList<Point3D> Positions(double deflection)
        {
            var (curve, parameters) = self.DeflectionParameters(deflection);
            return [.. parameters.Select(p => { var pnt = curve.Evaluate(p); return pnt; })];
        }

        /// <summary>The curve and the parameters along it that keep a polygon through them within <paramref name="deflection"/>.</summary>
        private (ShapeCurve, IReadOnlyList<double>) DeflectionParameters(double deflection)
        {
            var curve = self.Curve();
            // GCPnts_UniformDeflection where the curve is C2, else GCPnts_QuasiUniformDeflection (the kernel's Discretize)
            var parameters = curve.Discretize(deflection);
            if (parameters.IsEmpty)
                throw new InvalidOperationException("Deflection calculation failed");
            return (curve, parameters);
        }

        /// <summary>The derivative of the curve of the given <paramref name="order"/> at a position or point.</summary>
        public Vector3D DerivativeAt(CurvePosition position, int order = 2, PositionMode mode = PositionMode.Parameter)
        {
            EdgeCurve curve;
            double param;
            bool closestForward;
            IShape1D closest = self;
            if (position.Value is { } value)
                (curve, param, closestForward) = self.OcctParamAt(value, mode);
            else
            {
                var point = position.Point!.Value;
                if (self.AsShape().TryAs<Wire>(out _))
                    closest = self.Edges().MinBy(e => e.DistanceTo(point))!;
                var u = closest.ParamAtPoint(point);
                (curve, param, closestForward) = closest.OcctParamAt(u);
            }
            {
                var d = curve.DerivativeN(param, order);
                if (d.Magnitude == 0)
                    return new Vector3D(0, 0, 0);
                var derivative = d;
                // An odd derivative runs the way the shape does
                if (order % 2 == 1)
                {
                    bool forward = self.AsShape().TryAs<Wire>(out _) ? closestForward == self.IsForward : self.IsForward;
                    if (!forward)
                        derivative = -derivative;
                }
                return derivative;
            }
        }

        /// <summary>The unit tangent at a position or point.</summary>
        /// <remarks>Beyond the shape an edge uses its extrapolated or periodic tangent, a wire its first or last.</remarks>
        public Vector3D TangentAt(CurvePosition position, PositionMode mode = PositionMode.Parameter) =>
            self.DerivativeAt(position, 1, mode).Normalized();

        /// <summary>The unit tangent at the middle of the curve.</summary>
        public Vector3D TangentAt() => self.TangentAt(0.5);

        /// <summary>The angle in degrees (0 to 360) of the tangent at a position, measured in <paramref name="plane"/> (default XY).</summary>
        public double TangentAngleAt(double locationParam = 0.5, PositionMode mode = PositionMode.Parameter, Plane? plane = null)
        {
            plane ??= Plane.XY;
            var tangent = self.TangentAt(locationParam, mode);
            return Numerics.FloorMod(plane.XDir.GetSignedAngle(tangent, plane.ZDir) + 360, 360.0);
        }

        /// <summary>The start point. A circle may start and end at the same point.</summary>
        public Point3D StartPoint()
        {
            var curve = self.Curve();
            var p = curve.Evaluate(self.IsForward ? curve.Range.Min : curve.Range.Max);
            return p;
        }

        /// <summary>The end point. A circle may start and end at the same point.</summary>
        public Point3D EndPoint()
        {
            var curve = self.Curve();
            var p = curve.Evaluate(self.IsForward ? curve.Range.Max : curve.Range.Min);
            return p;
        }

        /// <summary>The centre: the point half way along (by default), the centre of mass, or of the bounding box.</summary>
        internal Point3D CenterOfShape1D() => self.Center(CenterOf.Geometry);

        /// <summary>As the other forms of <c>Center</c>.</summary>
        internal Point3D CenterOfShape1D(CenterOf centerOf)
        {
            self.NeedsGeometry("center");
            switch (centerOf)
            {
                case CenterOf.Geometry:
                    return self.PositionAt(0.5);
                case CenterOf.Mass:
                    return self.AsShape().MassProperties(Dimension.Length).CentreOfMass;
                default:
                    return self.Bounds().Center();
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
                                   FrameMethod frameMethod = FrameMethod.Frenet, Vector3D? xDir = null)
        {
            ShapeCurve curve;
            double param;
            // ParamAt and OcctParamAt measure a reversed shape from its other end
            if (self.AsShape().TryAs<Wire>(out _))
            {
                if (frameMethod == FrameMethod.Corrected)
                {
                    // A comp-curve parameter
                    curve = self.Curve();
                    param = self.ParamAt(mode == PositionMode.Parameter ? distance : distance / self.Length);
                }
                else
                {
                    // An edge-curve parameter
                    var (c, p, _) = self.OcctParamAt(mode == PositionMode.Parameter ? distance * self.Length : distance, PositionMode.Length);
                    (curve, param) = (new ShapeCurve(c), p);
                }
            }
            else
            {
                param = self.ParamAt(mode == PositionMode.Parameter ? distance : distance / self.Length);
                curve = self.Curve();
            }
            return self.FrameLocation(curve.Frames([param], Shape1DExtensions.Law(frameMethod))[0], xDir);
        }

        /// <summary>The local coordinate systems at several positions along the curve.</summary>
        public IReadOnlyList<Location> Locations(IEnumerable<double> distances, PositionMode mode = PositionMode.Parameter,
                                                 FrameMethod frameMethod = FrameMethod.Frenet, Vector3D? xDir = null) =>
            [.. distances.Select(d => self.LocationAt(d, mode, frameMethod, xDir))];

        /// <summary>
        /// Local coordinate systems wherever the curve needs one for the polygon through them to stay within
        /// <paramref name="deflection"/> of it, found in one pass over the whole curve.
        /// </summary>
        public IReadOnlyList<Location> Locations(double deflection, FrameMethod frameMethod = FrameMethod.Frenet, Vector3D? xDir = null)
        {
            var (curve, parameters) = self.DeflectionParameters(deflection);
            return [.. curve.Frames([.. parameters], Shape1DExtensions.Law(frameMethod)).Select(f => self.FrameLocation(f, xDir))];
        }

        /// <summary>The Location of a frame on the curve: at its point, its Z along the tangent.</summary>
        private Location FrameLocation(CurveFrame frame, Vector3D? xDir)
        {
            var (pnt, tangent, normal, _) = frame;
            var trsf = new Transform3D();
            var zDir = new Direction3D(tangent);
            var target = new CoordinateSystem();
            if (xDir is { } x)
            {
                try
                {
                    var xd = x.ToDirection();
                    var ax3 = new CoordinateSystem(pnt, zDir, xd);
                    trsf = trsf.WithTransformation(ax3, target);
                }
                catch (Exception e) when (Diagnostic.IsKernelFailure(e) && Diagnostic.FromException(e).Code.Value == "engine.construction-error")
                {
                    throw new ArgumentException(
                        $"Unable to create location with given x_dir {x.Describe()}. x_dir must be perpendicular to shape's tangent {tangent.Describe()}.", e);
                }
            }
            else
            {
                var nd = new Direction3D(normal);
                var ax3 = new CoordinateSystem(pnt, zDir, nd);
                trsf = trsf.WithTransformation(ax3, target);
            }
            var loc = new Location(trsf);
            return self.IsForward ? loc : -loc;
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
            var locations = self.Locations(Enumerable.Range(0, count).Select(i => start + i * (stop - start) / (count - 1)));
            return positionsOnly ? [.. locations.Select(l => l.WithOrientation(new Vector3D(0, 0, 0)))] : locations;
        }

        // ---- Geometry ----

        /// <summary>
        /// The plane containing this and the other edges or wires, or null if there is none. Coaxial lines lie in infinitely
        /// many planes, of which one is chosen.
        /// </summary>
        public Plane? CommonPlane<T>(IEnumerable<T> lines, double tolerance = Constants.Tolerance) where T : IShape1D =>
            self.CommonPlane(lines.Select(l => (IShape1D?)l), tolerance);

        /// <summary>
        /// The plane containing this and the other edges or wires, or null if there is none. Coaxial lines lie in infinitely
        /// many planes, of which one is chosen.
        /// </summary>
        public Plane? CommonPlane(IEnumerable<IShape1D?> lines, double tolerance = Constants.Tolerance)
        {
            var allLines = lines.OfType<IShape1D>().Prepend(self).ToList();
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
                var shortened = allLines.Where(l => l.Length > 1e50).Select(l => (IShape1D)((Edge)l.AsShape()).TrimToLength(0.5, 10));
                allLines = [.. normalLines, .. shortened];
                var points = new List<Point3D>();
                foreach (var line in allLines)
                {
                    int n = line.GeomType == GeomType.Line ? 2 : 8;
                    points.AddRange(Enumerable.Range(0, n).Select(i => line.PositionAt(i / (double)(n - 1))));
                }
                points = PointOrder.Distinct(points);   // the distinct points, in build123d's order
                // The points making the largest triangle make the most accurate plane; on ties the last such found wins
                double bestArea = double.NegativeInfinity;
                (Point3D, Point3D, Point3D) extremes = default;
                for (int i = 0; i < points.Count; i++)
                    for (int j = i + 1; j < points.Count; j++)
                        for (int k = j + 1; k < points.Count; k++)
                        {
                            var area = 0.5 * (points[j] - points[i]).Cross(points[k] - points[i]).Magnitude;
                            if (area >= bestArea)
                                (bestArea, extremes) = (area, (points[i], points[j], points[k]));
                        }
                var (e0, e1, e2) = extremes;
                var zDir = (e2 - e0).Cross((e1 - e0).Normalized());
                Plane? candidate;
                try
                {
                    candidate = new Plane(((e0.ToVector() + e1.ToVector() + e2.ToVector()) * (1.0 / 3)).ToPoint(), zDir: zDir);
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
            var globalCenter = (allLines.Aggregate(Vector3D.Zero, (sum, l) => sum + l.PositionAt(0.5).ToVector()) * (1.0 / allLines.Count)).ToPoint();
            var origin = (Point3D)result.Intersect(new Axis(globalCenter, result.ZDir))!;
            return result.ShiftOrigin(origin);
        }

        /// <summary>As the other forms of <c>CommonPlane</c>.</summary>
        public Plane? CommonPlane(params IShape1D?[] lines) => self.CommonPlane((IEnumerable<IShape1D?>)lines);

        /// <summary>The normal of a planar curve.</summary>
        /// <exception cref="InvalidOperationException">The curve isn't planar.</exception>
        public Vector3D Normal()
        {
            if (self.IsEmpty)
                throw new InvalidOperationException("Can't find normal of empty edge/wire");
            switch (self.GeomType)
            {
                case GeomType.Circle:
                {
                    ((Edge)self.AsShape()).AsCurve().TryGetCircle(out var circ);
                    var axis = circ.Axis;
                    var dir = axis.Direction;
                    return dir;
                }
                case GeomType.Ellipse:
                {
                    ((Edge)self.AsShape()).AsCurve().TryGetEllipse(out var ell);
                    var axis = ell.Axis;
                    var dir = axis.Direction;
                    return dir;
                }
                default:
                {
                    self.AsShape().TryFindSurface(-1, out var surface, onlyPlane: true);
                    var plane = (surface as PlaneSurface) ?? throw new InvalidOperationException("Normal not defined");
                    var pln = plane.Plane;
                    var axis = pln.Axis;
                    var dir = axis.Direction;
                    return dir;
                }
            }
        }

        /// <summary>A line on <paramref name="plane"/> (default XY) of a given length, perpendicular to and centred on the curve at a position.</summary>
        public Edge PerpendicularLine(double length, double uValue, Plane? plane = null)
        {
            plane ??= Plane.XY;
            var start = self.PositionAt(uValue);
            var local = new Plane(start, self.TangentAt(uValue), plane.ZDir);
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
            if (self.IsEmpty)
                throw new InvalidOperationException("An empty curve has no curvature comb.");
            var pln = self.CommonPlane();
            if (pln is null || !TopologyUtils.IsCloseB(Math.Abs(pln.ZDir.Z), 1.0, absTol: Constants.Tolerance))
                throw new InvalidOperationException("A curvature comb needs a curve on Plane.XY.");
            // A closed curve's first and last tooth would be the same, so the last is left out
            bool endpoint = !self.IsClosed;
            var uValues = Enumerable.Range(0, count)
                .Select(i => count == 1 ? 0.0 : i / (double)(endpoint ? count - 1 : count)).ToList();
            var kappas = new List<double>();
            var tangents = new List<Vector3D>();
            foreach (var u in uValues)
            {
                var tangent = self.DerivativeAt(u, 1);
                var curvature = self.DerivativeAt(u, 2);
                tangents.Add(tangent);
                var cross = tangent.Cross(curvature);
                var kappa = cross.Magnitude / (Math.Pow(tangent.Magnitude, 3) + Constants.Tolerance);
                kappas.Add(cross.Z >= 0 ? kappa : -kappa);
            }
            var maxKappa = Math.Max(Constants.Tolerance, kappas.Max(Math.Abs));
            var size = self.Bounds().Size;
            var tooth = maxToothSize ?? Math.Max(size.X, Math.Max(size.Y, size.Z)) / 10;
            var scale = tooth / maxKappa;
            var comb = new List<Edge>();
            for (int i = 0; i < uValues.Count; i++)
            {
                var length = scale * kappas[i];
                if (Math.Abs(length) < Constants.Tolerance)
                    continue;   // avoid tiny teeth
                var onCurve = self.PositionAt(uValues[i]);
                var kappaDir = tangents[i].Normalized().Cross(new Vector3D(0, 0, 1));
                comb.Add(Edge.MakeLine(onCurve, onCurve + kappaDir * length));
            }
            return new ShapeList<Edge>(comb);
        }

        /// <summary>The curve projected onto a face along a direction: the projection closest to the curve, or all of them.</summary>
        public ShapeList<IShape1D> Project(Face face, Vector3D direction, bool closest = true)
        {
            if (self.IsEmpty || face.IsEmpty)
                throw new InvalidOperationException("Can't project an empty Edge or Wire onto empty Face");
            var dir = direction.ToDirection();
            var projected = self.AsShape().IsEdge()
                ? ShapeProjections.Project((Edge)self.AsShape(), face, dir)
                : ShapeProjections.Project((Wire)self.AsShape(), face, dir);
            // A projection that meets nothing is no pieces
            var pieces = projected.TryGetValue(out var wires) ? wires.Select(w => w.AsShape()).ToList() : [];
            if (!closest)
                return new ShapeList<IShape1D>(pieces.Select(p => TopoUtil.As1D(p)));
            IShape1D? best = null;
            double minDist = double.PositiveInfinity;
            foreach (var piece in pieces)
            {
                var d = Failures.Get(MeasureOperations.Distance(self.AsShape(), piece), () => "distance").Value;
                if (d < minDist)
                    (minDist, best) = (d, TopoUtil.As1D(piece));
            }
            return best is null ? [] : [best];
        }

        // ---- Combining ----

        internal ShapeList<Shape>? IntersectCoreOfShape1D(object other, double tolerance, bool includeTouched)
        {
            other = other switch
            {
                Point3D v => Vertex.Make(v),
                Location l => Vertex.Make(l.Position),
                Axis a => Edge.FromAxis(a),
                _ => other,
            };
            var results = new List<Shape>();
            // Trim infinite edges before OCCT operations
            if (other is Edge { IsInfinite: true } infinite)
            {
                var box = self.Bounds(optimal: false);
                other = infinite.TrimInfinite(box.Diagonal + (infinite.Center() - box.Center()).Magnitude);
            }
            switch (other)
            {
                case Plane plane:
                {
                    // 1D + plane: a section with the plane's face
                    var face = FaceExtensions.FromTopoFace(ShapeExtensions.MakePlaneFace(plane));
                    results.AddRange(ShapeExtensions.BoolOpList([self.AsShape()], [face], BooleanKind.Section).Expand());
                    break;
                }
                case IShape1D line:
                {
                    // 1D + 1D: the common part (collinear overlap) and the section (crossings)
                    results.AddRange(ShapeExtensions.BoolOpList([self.AsShape()], [line.AsShape()], BooleanKind.Common).Expand());
                    results.AddRange(ShapeExtensions.BoolOpList([self.AsShape()], [line.AsShape()], BooleanKind.Section).OfType<Vertex>().Where(v => !v.IsEmpty).Upcast());
                    break;
                }
                case Vertex vertex:
                    if (vertex.DistanceTo(self.AsShape()) <= tolerance)
                        results.Add(vertex);
                    break;
                case IShape shape:
                    // Delegate to higher-order shapes
                    if (ShapeExtensions.IntersectOne(shape.AsShape(), self.AsShape(), tolerance, includeTouched) is { } found)
                        results.AddRange(found);
                    break;
            }
            return results.Count > 0 ? new ShapeList<Shape>(results) : null;
        }
    }



    private static FrameLaw Law(FrameMethod method) => method == FrameMethod.Frenet ? FrameLaw.Frenet : FrameLaw.CorrectedFrenet;
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