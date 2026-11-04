using Kerf.Construction;
using Kerf.Fillets;
using Kerf.Measurement;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Modeling;

/// <summary>
/// A bounded piece of a surface: a two-dimensional element of a model, bounded by an outer wire and holes.
/// </summary>
public partial class Face : Shape2D
{
    /// <summary>The empty Face.</summary>
    public Face() : base(null) { }

    /// <summary>A face wrapping an OCCT face.</summary>
    public Face(global::Kerf.Topology.Face face, string label = "", Color? color = null) : base(face, label, color) { }

    /// <summary>
    /// A face bounded by a closed wire, with optional holes. It lies on <paramref name="surface"/>'s surface (used
    /// untrimmed, where that face places it), or on the plane the wires lie in when no surface is given. The wires must
    /// lie on the surface; they aren't projected onto it.
    /// </summary>
    /// <exception cref="ArgumentException">A wire isn't closed, the wires aren't planar (without a surface), or the surface is empty.</exception>
    /// <exception cref="InvalidOperationException">The wires don't lie on the surface.</exception>
    public Face(Wire outerWire, IEnumerable<Wire>? innerWires = null, string label = "", Color? color = null, Face? surface = null)
        : base(MakeFromWires(outerWire, innerWires, surface), label, color) { }

    private static global::Kerf.Topology.Face MakeFromWires(Wire outerWire, IEnumerable<Wire>? innerWires, Face? surface)
    {
        var inner = innerWires?.Select(w => w.TopoWire).ToList() ?? [];
        if (!outerWire.TopoShape.CheckClosed() || inner.Any(w => !w.CheckClosed()))
            throw new ArgumentException("Face can only be created with closed wires");
        if (surface is null)
            return TopoUtil.MakeFaceFromWires(outerWire.TopoWire, inner);
        if (surface.IsEmpty)
            throw new ArgumentException("Can't build a face on an empty surface");
        return TopoUtil.MakeFaceOnSurface(surface.TopoFace, outerWire.TopoWire, inner);
    }

    internal static Face FromTopoFace(global::Kerf.Topology.Face face) => new(face);

    /// <summary>A face on the plane, as large as OCCT's defaults allow (an infinite plane).</summary>
    public static Face FromPlane(Plane plane) => new(MakePlaneFace(plane));

    internal global::Kerf.Topology.Face TopoFace => (global::Kerf.Topology.Face)TopoShape;

    internal override double Order => 2.0;

    internal override Shape MakeZero() => new Face();

    /// <summary>The plane the face was made on, if it records one, for a later extrusion.</summary>
    public Plane? CreatedOn { get; private set; }

    /// <summary>A copy recording the plane the face was made on.</summary>
    public Face WithCreatedOn(Plane? plane)
    {
        var copy = (Face)Clone();
        copy.CreatedOn = plane;
        return copy;
    }

    private protected override void CopySubclassAttributesTo(Shape target)
    {
        if (target is Face face)
            face.CreatedOn ??= CreatedOn;
    }

    // ---- Properties ----

    /// <summary>The volume of a face, which is zero.</summary>
    public override double Volume => 0.0;

    /// <summary>The area of the face as if its holes were filled in.</summary>
    public double AreaWithoutHoles => IsEmpty ? 0.0 : WithoutHoles().Area;

    /// <summary>The axis of rotation of a cone, cylinder, sphere, torus or surface of revolution; else null.</summary>
    public Axis? AxisOfRotation
    {
        get
        {
            if (IsEmpty)
                return null;
            var surface = GeomAdaptor();
            // Unwrap trimmed and offset surfaces to get at the basis surface
            while (true)
            {
                Surface? basis = null;
                if ((surface as TrimmedSurface) is { } trimmed)
                    basis = trimmed.BasisSurface;
                else if ((surface as OffsetSurface) is { } offset)
                    basis = offset.BasisSurface;
                if (basis is null)
                    break;
                surface = basis;
            }
            switch (surface)
            {
                case ConicalSurface cone:
                    return new Axis(cone.Cone.Axis);
                case CylindricalSurface cylinder:
                    return new Axis(cylinder.Cylinder.Axis);
                case SphericalSurface sphere:
                {
                    var ax3 = sphere.Sphere.Position;
                    var loc = ax3.Location;
                    var dir = ax3.Direction;
                    var ax1 = new Axis3D(loc, dir);
                    return new Axis(ax1);
                }
                case ToroidalSurface torus:
                    return new Axis(torus.Torus.Axis);
                case RevolutionSurface revolution:
                    return new Axis(revolution.Axis);
                default:
                    return null;
            }
        }
    }

    /// <summary>
    /// The axes of symmetry of a planar face, each through its centre. Candidate directions (through the holes' centres,
    /// or towards points around the outline) are tested by splitting the face along each and mirroring one half onto the
    /// other.
    /// </summary>
    /// <exception cref="InvalidOperationException">The face is empty or isn't planar.</exception>
    public IReadOnlyList<Axis> AxesOfSymmetry
    {
        get
        {
            if (IsEmpty)
                throw new InvalidOperationException("Can't determine axes_of_symmetry of empty face");
            if (IsPlanar is null)
                throw new InvalidOperationException("axes_of_symmetry only supports for planar faces");
            var cog = Center();
            var normal = NormalAt();
            var innerWires = InnerWires();
            List<Vector> crossDirs;
            if (innerWires.Count > 0)
            {
                var holeFaces = innerWires.Select(w => new Face(w)).ToList();
                var holesCentroid = CombinedCenter(holeFaces);
                // If the holes aren't centred on the centre of gravity, the axis must pass through both
                crossDirs = (holesCentroid - cog).Length > Constants.Tolerance
                    ? [(holesCentroid - cog).Normalized()]
                    : [.. holeFaces.Select(f => (f.Center() - cog).Normalized())];
            }
            else
            {
                var outer = OuterWire();
                var shapeEdges = outer.Edges();
                IEnumerable<Edge> edges = shapeEdges;
                if (shapeEdges.Any(e => e.GeomType != GeomType.Line))
                {
                    var corners = OrientedBoundingBox().Corners;
                    edges = edges.Concat(Enumerable.Range(0, 4).Select(i => Edge.MakeLine(corners[i], corners[(i + 1) % 4])));
                }
                crossDirs = [.. edges.SelectMany(e => new[] { 0.0, 0.5, 1.0 }.Select(p => (e.PositionAt(p) - cog).Normalized()))];
            }

            var symmetryDirs = new List<Vector>();
            foreach (var crossDir in crossDirs)
            {
                // Split the face along the candidate axis and flip the top over
                var splitPlane = Plane.Create(cog, crossDir, crossDir.Cross(normal));
                var (top, bottom) = SplitBoth(splitPlane);
                if (top.Count == 0 || bottom.Count == 0 || top.Count != bottom.Count)
                    continue;
                var axis = new Axis(cog, crossDir);
                var bottomList = bottom.OrderBy(axis);
                var flipped = new ShapeList<Shape>(top.Select(f => f.Mirror(splitPlane))).OrderBy(axis);
                var bottomArea = bottomList.Sum(f => f.Area);
                double intersectArea = 0;
                foreach (var (f, b) in flipped.Zip(bottomList))
                {
                    var intersection = f.Intersect(b);
                    if (intersection.Count == 0)
                    {
                        intersectArea = -1.0;
                        break;
                    }
                    intersectArea = intersection.Faces().Sum(x => x.Area);
                }
                if (intersectArea == -1.0)
                    continue;
                // The halves match: an axis, unless its opposite is already one
                if (Math.Abs(intersectArea - bottomArea) < Constants.Tolerance
                    && !symmetryDirs.Any(d => d.Dot(crossDir) < -1 + Constants.Tolerance) && !symmetryDirs.Contains(crossDir))
                    symmetryDirs.Add(crossDir);
            }
            return [.. symmetryDirs.Select(d => new Axis(cog, d))];
        }
    }

    /// <summary>The location at the centre of the face, normal to it.</summary>
    public Location CenterLocation
    {
        get
        {
            NeedsGeometry("center");
            var origin = PositionAt(0.5, 0.5);
            return new Plane(origin, zDir: NormalAt(origin)).Location;
        }
    }

    /// <summary>The kind of planar face: "POLYGON", "RECTANGLE" or "SQUARE"; null for others.</summary>
    public string? Geometry
    {
        get
        {
            if (IsEmpty || IsPlanar is null)
                return null;
            var flat = new Plane(this).ToLocalCoords(this);
            var flatEdges = flat.Edges();
            if (!flatEdges.All(e => e.GeomType == GeomType.Line))
                return null;
            var result = "POLYGON";
            if (flatEdges.Count == 4)
            {
                var pairs = flat.Vertices().Select(v => flatEdges.Where(e => e.Vertices().Contains(v)).ToList()).ToList();
                if (pairs.All(p => p[0].TangentAt(0).GetAngle(p[1].TangentAt(0)) == 90))
                {
                    result = "RECTANGLE";
                    if (flatEdges.GroupBy(ShapeKey.Length).Count == 1)
                        result = "SQUARE";
                }
            }
            return result;
        }
    }

    /// <summary>
    /// How the material behind this face sits against it, read from the surface's principal curvatures against the outward
    /// normal over the face: Convex where the surface curves away from the material (a boss, a sphere's outside), Concave
    /// where it curves into it (a hole), Smooth where it doesn't curve (a plane), and Saddle where the curvatures disagree.
    /// </summary>
    /// <exception cref="InvalidOperationException">The surface is degenerate everywhere it was sampled.</exception>
    public override Convexity Convexity
    {
        get
        {
            if (IsEmpty)
                throw new InvalidOperationException("an empty face has no convexity");
            var surface = TopoFace.ToSurface();
            var (uMin, uMax, vMin, vMax) = UVBounds();
            double[] fractions = [0.15, 0.5, 0.85];
            var kinds = new HashSet<Convexity>();
            foreach (var uf in fractions)
                foreach (var vf in fractions)
                {
                    var u = uMin + (uMax - uMin) * uf;
                    var v = vMin + (vMax - vMin) * vf;
                    if (!surface.TryGetCurvature(u, v, Constants.Tolerance, out var props))
                        continue;
                    var normal = TopoUtil.FaceNormalVector(TopoFace, u, v);
                    if (normal.Length < 1e-12)
                        continue;
                    // Curvatures signed toward the outward normal: negative bends away
                    var propsNormal = props.Normal;
                    var normalDir = normal.ToGpDir();
                    var sign = propsNormal.Dot(normalDir) > 0 ? 1.0 : -1.0;
                    var a = sign * props.Min;
                    var b = sign * props.Max;
                    var (low, high) = (Math.Min(a, b), Math.Max(a, b));
                    bool away = low < -Constants.Tolerance;
                    bool into = high > Constants.Tolerance;
                    if (away && into)
                        return Convexity.Saddle;
                    kinds.Add(away ? Convexity.Convex : into ? Convexity.Concave : Convexity.Smooth);
                }
            if (kinds.Count == 0)
                throw new InvalidOperationException("the surface is degenerate everywhere it was sampled");
            kinds.Remove(Convexity.Smooth);
            return kinds.Count switch
            {
                0 => Convexity.Smooth,
                1 => kinds.Single(),
                _ => Convexity.Saddle,
            };
        }
    }

    /// <summary>Whether the face curves away from the material behind it (<c>Convexity == Convex</c>).</summary>
    public bool IsCircularConvex => Convexity == Convexity.Convex;

    /// <summary>Whether the face curves into the material behind it (<c>Convexity == Concave</c>).</summary>
    public bool IsCircularConcave => Convexity == Convexity.Concave;

    /// <summary>Whether the face's surface is planar (by <c>GeomLib_IsPlanarSurface</c>), whatever its geometry type.</summary>
    internal bool IsPlanarSurface()
    {
        TopoFace.TryGetSurface(out var surface);
        return surface!.TryGetPlane(Constants.Tolerance, out _);
    }

    /// <summary>The plane the face lies in if it is planar, even though its geometry type may not be Plane; else null.</summary>
    public Plane? IsPlanar
    {
        get
        {
            if (IsEmpty)
                return null;
            TopoFace.TryGetSurface(out var surface);
            if (!surface!.TryGetPlane(Constants.Tolerance, out var pln))
                return null;
            var position = pln.Position;
            if (!position.IsDirect)
            {
                // A left-handed plane was returned
                var ax2 = position.Frame;
                var ax3 = new CoordinateSystem(ax2);
                pln = new Plane3D(ax3);
            }
            return new Plane(pln);
        }
    }

    /// <summary>
    /// The length of a planar face (across it, along its plane's X) or a cylindrical one (along its axis), measured on
    /// the surface itself; null for other faces.
    /// </summary>
    public double? Length
    {
        get
        {
            if (IsEmpty)
                return null;
            if (IsPlanar is not null)
            {
                // Repositioned on Plane.XY
                var vertices = new Plane(this).ToLocalCoords(this).Vertices().OrderBy(Axis.X);
                return vertices[^1].X - vertices[0].X;
            }
            if (GeomType == GeomType.Cylinder)
                throw new NotImplementedException("Face.Length of a cylindrical face needs Face.UvFace, which comes with the sheet metal port");
            return null;
        }
    }

    /// <summary>
    /// The width of a planar face (across it, along its plane's Y) or a cylindrical one (the arc it wraps through); null
    /// for other faces. Length times width is the area of both.
    /// </summary>
    public double? Width
    {
        get
        {
            if (IsEmpty)
                return null;
            if (IsPlanar is not null)
            {
                var vertices = new Plane(this).ToLocalCoords(this).Vertices().OrderBy(Axis.Y);
                return vertices[^1].Y - vertices[0].Y;
            }
            if (GeomType == GeomType.Cylinder && Radius is not null)
                throw new NotImplementedException("Face.Width of a cylindrical face needs Face.UvFace, which comes with the sheet metal port");
            return null;
        }
    }

    /// <summary>The major and minor radii of a torus, else null.</summary>
    public (double Major, double Minor)? Radii
    {
        get
        {
            if (GeomType != GeomType.Torus)
                return null;
            TopoFace.ToSurface().TryGetTorus(out var torus);
            return (torus.MajorRadius, torus.MinorRadius);
        }
    }

    /// <summary>The radius of a cylindrical or spherical face, else null.</summary>
    public double? Radius
    {
        get
        {
            if (IsEmpty)
                return null;
            var adaptor = TopoFace.ToSurface();
            switch (GeomType)
            {
                case GeomType.Cylinder:
                {
                    adaptor.TryGetCylinder(out var c);
                    return c.Radius;
                }
                case GeomType.Sphere:
                {
                    adaptor.TryGetSphere(out var s);
                    return s.Radius;
                }
                default:
                    return null;
            }
        }
    }

    /// <summary>The seam edges of the face (where a periodic surface meets itself).</summary>
    public ShapeList<Edge> Seams
    {
        get
        {
            return Edges().Where(e => e.TopoEdge.IsSeam(TopoFace)).ToShapeList();
        }
    }

    /// <summary>The semi-angle in degrees of a cone, else null.</summary>
    public double? SemiAngle
    {
        get
        {
            if (GeomType != GeomType.Cone)
                return null;
            TopoFace.ToSurface().TryGetCone(out var cone);
            return cone.SemiAngle * Constants.RadToDeg;
        }
    }

    // ---- Factories ----

    /// <summary>An edge extruded into a face.</summary>
    public static Face Extrude(Edge obj, Vector direction)
    {
        if (obj.IsEmpty)
            throw new ArgumentException("Can't extrude empty object");
        return new Face((global::Kerf.Topology.Face)TopoUtil.Extrude(obj.TopoShape, direction));
    }

    /// <summary>A Bézier surface from a 2D array of control points (up to 25 each way), rational with weights.</summary>
    public static Face MakeBezierSurface(IReadOnlyList<IReadOnlyList<Vector>> points, IReadOnlyList<IReadOnlyList<double>>? weights = null)
    {
        if (points.Count < 2 || points[0].Count < 2)
            throw new ArgumentException("At least two control points must be provided (start, end)");
        if (points.Count > 25 || points[0].Count > 25)
            throw new ArgumentException("The maximum number of control points is 25");
        if (weights is { Count: > 0 } && (points.Count != weights.Count || points[0].Count != weights[0].Count))
            throw new ArgumentException("A weight must be provided for each control point");
        var poles = Grid(points, p => p.ToGpPnt());
        BezierSurface bezier = weights is { Count: > 0 } ? new BezierSurface(poles, Grid(weights, w => w)) : new BezierSurface(poles);
        return new Face(Failures.Get(global::Kerf.Topology.Face.FromSurface(bezier), () => "Bezier face"));
    }

    // Rows of values as a grid, each row as long as the first
    private static Grid<TOut> Grid<TIn, TOut>(IReadOnlyList<IReadOnlyList<TIn>> rows, Func<TIn, TOut> convert)
    {
        int columns = rows[0].Count;
        var values = new TOut[rows.Count * columns];
        for (int i = 0; i < rows.Count; i++)
            for (int j = 0; j < rows[i].Count; j++)
                values[i * columns + j] = convert(rows[i][j]);
        return new Grid<TOut>(values, rows.Count, columns);
    }

    /// <summary>A rectangle centred on the plane's origin (default XY).</summary>
    public static Face MakeRect(double width, double height, Plane? plane = null)
    {
        plane ??= Plane.XY;
        var pln = plane.ToGpPln();
        var bounds = (new Interval(-width * 0.5, width * 0.5), new Interval(-height * 0.5, height * 0.5));
        return new Face(Failures.Get(global::Kerf.Topology.Face.FromSurface(new PlaneSurface(pln), bounds), () => "rectangle"));
    }

    /// <summary>
    /// A possibly non-planar face bounded by a wire (or edges), optionally refined by points on the surface, with holes.
    /// </summary>
    /// <exception cref="InvalidOperationException">OCCT can't build the face, or it isn't valid.</exception>
    public static Face MakeSurface(IEnumerable<Edge> exterior, IEnumerable<Vector>? surfacePoints = null, IEnumerable<Wire>? interiorWires = null)
    {
        var edges = exterior.ToList();
        if (edges.Any(e => e.IsEmpty))
            throw new ArgumentException("exterior contains empty edges");
        // BRepOffsetAPI_MakeFilling(3, 15, 2, false, 1e-5, 1e-4, 0.01, 0.1, 8, 9): OCCT's defaults
        var boundaries = edges.Select(edge => new FillBoundary(edge.TopoEdge, Continuity.C0)).ToList();
        var face = BuildFilling(boundaries, FillOptions.Default, "Error building non-planar face with provided exterior");
        var points = surfacePoints?.ToList();
        if (points is { Count: > 0 })
            face = BuildFilling(boundaries, FillOptions.Default with { Points = [.. points.Select(p => p.ToGpPnt())] },
                                "Error building non-planar face with provided surface_points");
        if (interiorWires is not null)
        {
            var wires = interiorWires.ToList();
            if (wires.Count > 0)
                face = AddSurfaceHoles(face, wires);
        }
        face = face.Fix();
        if (!face.IsValid)
            throw new InvalidOperationException("non planar face is invalid");
        return face;
    }

    /// <inheritdoc cref="MakeSurface(IEnumerable{Edge}, IEnumerable{Vector}?, IEnumerable{Wire}?)"/>
    public static Face MakeSurface(Wire exterior, IEnumerable<Vector>? surfacePoints = null, IEnumerable<Wire>? interiorWires = null) =>
        MakeSurface(exterior.Edges(), surfacePoints, interiorWires);

    private static Face BuildFilling(IReadOnlyList<FillBoundary> boundaries, FillOptions options, string error)
    {
        // OCCT's filling of no boundary isn't done; the kernel's refuses the argument
        if (boundaries.Count == 0)
            throw new InvalidOperationException(error);
        var filled = Surfaces.Fill(boundaries, options);
        if (!filled.TryGetValue(out var face))
            throw new InvalidOperationException(error, new KerfException(filled.Diagnostics));
        return new Face(face.Shape);
    }

    private static Face AddSurfaceHoles(Face surfaceFace, IEnumerable<Wire> interiorWires)
    {
        var holes = new List<global::Kerf.Topology.Wire>();
        foreach (var wire in interiorWires)
        {
            if (wire.IsEmpty)
                throw new ArgumentException("interior_wires contain an empty wire");
            holes.Add(wire.TopoWire);
        }
        var made = surfaceFace.TopoFace.WithHoles(holes);
        if (!made.TryGetValue(out var face))
            throw new InvalidOperationException("Error adding interior hole in non-planar face with provided interior_wires",
                                                new KerfException(made.Diagnostics));
        return new Face(face);
    }

    /// <summary>
    /// A spline surface approximating a 2D array of points: the first index is v, the second u, in the face's parameters.
    /// <paramref name="smoothing"/> weights raise the maximum degree to at least 6.
    /// </summary>
    public static Face MakeSurfaceFromArrayOfPoints(IReadOnlyList<IReadOnlyList<Vector>> points, double tolerance = 1e-2,
                                                    (double, double, double)? smoothing = null, int minDeg = 1, int maxDeg = 3)
    {
        var array = Grid(points, p => p.ToGpPnt());
        var options = smoothing is var (w1, w2, w3)
            // The smoothing asks for C2, whose Jacobi basis needs a degree of at least 6 (OCCT 8)
            ? new ApproximationOptions { Smoothing = (w1, w2, w3), MaxDegree = Math.Max(maxDeg, 6), Continuity = Continuity.C2, Tolerance = tolerance }
            : new ApproximationOptions { MinDegree = minDeg, MaxDegree = maxDeg, Continuity = Continuity.C2, Tolerance = tolerance };
        if (!BSplineSurface.Approximate(array, options).TryGetValue(out var surface))
            throw new InvalidOperationException("B-spline approximation failed");
        return new Face(Failures.Get(global::Kerf.Topology.Face.FromSurface(surface), () => "B-spline face"));
    }

    /// <summary>A ruled surface between two edges.</summary>
    public static Face MakeSurfaceFromCurves(Edge edge1, Edge edge2) =>
        (Face)new Face().CastLike(Failures.Get(Sweeps.Ruled(edge1.TopoEdge, edge2.TopoEdge), () => "ruled surface").Shape);

    /// <summary>A ruled surface between two wires with the same number of edges: a face, or a shell of several.</summary>
    public static Shape MakeSurfaceFromCurves(Wire wire1, Wire wire2) =>
        new Face().CastLike(Failures.Get(Sweeps.Ruled(wire1.TopoWire, wire2.TopoWire), () => "ruled surface").Shape);

    /// <summary>
    /// A possibly non-planar patch bounded by edges, each optionally tangent (or curvature) continuous with a support face,
    /// optionally refined by points on the surface.
    /// </summary>
    public static Face MakeSurfacePatch(IEnumerable<(Edge Edge, Face Face, ContinuityLevel Continuity)>? edgeFaceConstraints = null,
                                        IEnumerable<Edge>? edgeConstraints = null, IEnumerable<Vector>? pointConstraints = null)
    {
        static Continuity Continuity(ContinuityLevel c) => c switch
        {
            ContinuityLevel.C0 => global::Kerf.Geometry.Continuity.C0,
            ContinuityLevel.C1 => global::Kerf.Geometry.Continuity.G1,
            _ => global::Kerf.Geometry.Continuity.G2,
        };
        var boundaries = new List<FillBoundary>();
        foreach (var (edge, face, continuity) in edgeFaceConstraints ?? [])
            boundaries.Add(new FillBoundary(edge.TopoEdge, Continuity(continuity)) { Support = face.TopoFace });
        foreach (var edge in edgeConstraints ?? [])
            boundaries.Add(new FillBoundary(edge.TopoEdge, global::Kerf.Geometry.Continuity.C0));
        var options = FillOptions.Default with { Points = [.. (pointConstraints ?? []).Select(p => p.ToGpPnt())] };
        var result = BuildFilling(boundaries, options, "Error building non-planar face with provided constraints").Fix();
        if (!result.IsValid || result.IsEmpty)
            throw new InvalidOperationException("Non planar face is invalid");
        return result;
    }

    /// <summary>An edge revolved through <paramref name="angle"/> degrees about an axis.</summary>
    public static Face Revolve(Edge profile, double angle, Axis axis)
    {
        var ax1 = axis.ToGpAx1();
        // On a copy of the profile, as build123d revolves (BRepPrimAPI_MakeRevol with copy)
        var revolved = Failures.Get(profile.TopoShape.Revolve(ax1, angle * Constants.DegToRad, copyProfile: true), () => "revolve");
        return new Face((global::Kerf.Topology.Face)revolved.Shape);
    }

    /// <summary>The faces sewn together, as groups of contiguous faces.</summary>
    public static IReadOnlyList<ShapeList<Face>> SewFaces(IEnumerable<Face> faces)
    {
        var sewn = TopoUtil.SewFaces(faces.Select(f => f.TopoShape));
        var groups = new List<ShapeList<Face>>();
        foreach (var top in TopoUtil.TopLevelShapes(sewn))
        {
            groups.Add(top.Type switch
            {
                global::Kerf.Topology.ShapeType.Face => [new Face((global::Kerf.Topology.Face)top)],
                global::Kerf.Topology.ShapeType.Shell => new Shell((global::Kerf.Topology.Shell)top).Faces(),
                global::Kerf.Topology.ShapeType.Solid => new ShapeList<Face>(TopoUtil.Entities(top, global::Kerf.Topology.ShapeType.Face).Select(f => new Face((global::Kerf.Topology.Face)f))),
                _ => throw new InvalidOperationException($"SewedShape returned a {TopoUtil.TypeName(top)} which was unexpected"),
            });
        }
        return groups;
    }

    /// <summary>A one-edge profile swept along a one-edge path (see <see cref="Shell.Sweep"/> for more edges).</summary>
    public static Face Sweep(Shape1D profile, Shape1D path, Transition transition = Transition.Transformed) => Sweep((Shape)profile, path, transition);

    /// <inheritdoc cref="Sweep(Shape1D, Shape1D, Transition)"/>
    public static Face Sweep(Shape profile, Shape path, Transition transition = Transition.Transformed)
    {
        if (profile.Edges().Count != 1 || path.Edges().Count != 1)
            throw new ArgumentException("Use Shell.sweep for multi Edge objects");
        var profileWire = new Wire([profile.SingleEdge()]);
        var pathWire = new Wire([path.SingleEdge()]);
        // BRepOffsetAPI_MakePipeShell's defaults (corrected Frenet), not closed into a solid
        var options = new SweepOptions { MakeSolid = false, Transition = TransitionMode(transition) };
        var built = Failures.Get(Sweeps.Sweep(profileWire.TopoShape, pathWire.TopoWire, options), () => "sweep");
        // The pipe builds a shell holding the one face
        var swept = TopoUtil.Downcast(built.Shape);
        var result = swept.IsFace() ? new Face((global::Kerf.Topology.Face)swept)
                                    : new Face((global::Kerf.Topology.Face)TopoUtil.Entities(swept, global::Kerf.Topology.ShapeType.Face).Single());
        return SkipClean.Clean ? result.Clean() : result;
    }

    internal static global::Kerf.Construction.TransitionMode TransitionMode(Transition transition) => transition switch
    {
        Transition.Transformed => global::Kerf.Construction.TransitionMode.Transformed,
        Transition.Round => global::Kerf.Construction.TransitionMode.RoundCorner,
        _ => global::Kerf.Construction.TransitionMode.RightCorner,
    };

    // ---- Evaluation ----

    /// <summary>The surface of the face.</summary>
    public Surface GeomAdaptor()
    {
        NeedsGeometry("surface");
        TopoFace.TryGetSurface(out var surface);
        return surface!;
    }

    internal (double UMin, double UMax, double VMin, double VMax) UVBounds()
    {
        NeedsGeometry("surface");
        var (u, v) = TopoFace.UVBounds;
        return (u.Min, u.Max, v.Min, v.Max);
    }

    /// <summary>
    /// The centre of the face: by default its geometric centre, which for a planar face is its centre of mass and
    /// otherwise the middle of its parameter range.
    /// </summary>
    public override Vector Center() => Center(CenterOf.Geometry);

    /// <inheritdoc cref="Center()"/>
    public override Vector Center(CenterOf centerOf)
    {
        NeedsGeometry("center");
        if (centerOf == CenterOf.Mass || (centerOf == CenterOf.Geometry && IsPlanar is not null))
            return TopoUtil.FaceCenter(TopoFace);
        if (centerOf == CenterOf.BoundingBox)
            return BoundingBox().Center();
        return TopoUtil.FacePosition(TopoFace, 0.5, 0.5);
    }

    /// <summary>The point at normalised surface parameters (u, v), each 0 to 1 across the face.</summary>
    public Vector PositionAt(double u, double v) => TopoUtil.FacePosition(TopoFace, u, v);

    /// <summary>The unit normal at normalised (u, v), with the face's orientation.</summary>
    public Vector NormalAt(double u, double v)
    {
        NeedsGeometry("normal");
        var (u0, u1, v0, v1) = UVBounds();
        return NormalAtParameters(u0 + u * (u1 - u0), v0 + v * (v1 - v0));
    }

    /// <summary>The unit normal at the point of the surface nearest <paramref name="surfacePoint"/> (default the middle of the face).</summary>
    public Vector NormalAt(Vector? surfacePoint = null)
    {
        NeedsGeometry("normal");
        if (surfacePoint is null)
            return NormalAt(0.5, 0.5);
        var (u, v) = NearestParameters(surfacePoint.Value);
        return NormalAtParameters(u, v);
    }

    /// <summary>The surface parameters of the point of the surface nearest <paramref name="point"/> (GeomAPI_ProjectPointOnSurf).</summary>
    private (double U, double V) NearestParameters(Vector point)
    {
        if (!GeomAdaptor().TryProjectNearest(point.ToGpPnt(), out var nearest))
            throw new InvalidOperationException("The point can't be projected onto the face's surface");
        return (nearest.U, nearest.V);
    }

    /// <summary>The unit normal at surface parameters (u, v), with the face's orientation.</summary>
    internal Vector NormalAtParameters(double u, double v) => TopoUtil.FaceNormalVector(TopoFace, u, v).Normalized();

    /// <summary>
    /// A partial derivative of the surface, <paramref name="uOrder"/> times in u and <paramref name="vOrder"/> times in v.
    /// Normalised, u and v are fractions of the face's parameter range and the derivative is with respect to those
    /// fractions; otherwise both are in OCCT's own parameters. These ignore the face's orientation.
    /// </summary>
    public Vector DerivativeAt(double u, double v, int uOrder, int vOrder, bool normalize = true)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't find derivative on empty face");
        if (uOrder < 0 || vOrder < 0 || uOrder + vOrder == 0)
            throw new ArgumentException("orders must not be negative and must not both be zero");
        double scale = 1.0;
        if (normalize)
        {
            var (uMin, uMax, vMin, vMax) = UVBounds();
            u = uMin + u * (uMax - uMin);
            v = vMin + v * (vMax - vMin);
            scale = Math.Pow(uMax - uMin, uOrder) * Math.Pow(vMax - vMin, vOrder);
        }
        var d = TopoFace.ToSurface().DerivativeN(u, v, uOrder, vOrder);
        return new Vector(d) * scale;
    }

    /// <summary>
    /// The surface parameters (u, v) of a point on the face's surface: fractions of the face's range (as
    /// <see cref="PositionAt"/> uses) or OCCT's own. On a periodic surface they are those nearest the middle of the face.
    /// </summary>
    /// <exception cref="ArgumentException">The point isn't on the surface.</exception>
    public (double U, double V) ParamAtPoint(Vector point, bool normalize = true)
    {
        if (IsEmpty)
            throw new InvalidOperationException("Can't find param on empty face");
        var surface = GeomAdaptor();
        var pnt = point.ToGpPnt();
        var (uv, separation) = TopoFace.ParametersOf(pnt, Constants.Tolerance);
        if (separation > Constants.Tolerance)
            throw new ArgumentException($"point ({point}) is {separation} from the face's surface");
        double u = uv.X, v = uv.Y;
        var (uMin, uMax, vMin, vMax) = UVBounds();
        if (surface.IsUPeriodic)
        {
            var period = surface.UPeriod;
            u += period * Math.Round(((uMin + uMax) / 2 - u) / period, MidpointRounding.ToEven);
        }
        if (surface.IsVPeriodic)
        {
            var period = surface.VPeriod;
            v += period * Math.Round(((vMin + vMax) / 2 - v) / period, MidpointRounding.ToEven);
        }
        if (normalize)
        {
            u = (u - uMin) / (uMax - uMin);
            v = (v - vMin) / (vMax - vMin);
        }
        return (u, v);
    }

    /// <summary>
    /// The location on the surface at normalised (u, v): its origin on the surface, z along the face's normal, and x
    /// along <paramref name="xDir"/> or else the u tangent.
    /// </summary>
    public Location LocationAt(double u, double v, Vector? xDir = null)
    {
        NeedsGeometry("location");
        var (uMin, uMax, vMin, vMax) = UVBounds();
        return LocationAtParameters(uMin + u * (uMax - uMin), vMin + v * (vMax - vMin), xDir);
    }

    /// <summary>The location on the surface at the point nearest <paramref name="surfacePoint"/> (default the middle of the face).</summary>
    public Location LocationAt(Vector? surfacePoint = null, Vector? xDir = null)
    {
        NeedsGeometry("location");
        if (surfacePoint is null)
            return LocationAt(0.5, 0.5, xDir);
        var (u, v) = NearestParameters(surfacePoint.Value);
        return LocationAtParameters(u, v, xDir);
    }

    private Location LocationAtParameters(double u, double v, Vector? xDir)
    {
        var d = GeomAdaptor().Derivatives(u, v, 1);
        var (pnt, du, dv) = (d.Point, d.DU, d.DV);
        var origin = new Vector(pnt);
        var zDir = new Vector(du).Cross(new Vector(dv)).Normalized();
        // The surface normal ignores the face's orientation, so flip it to match the face's own (see NormalAt)
        if (TopoShape.Orientation == global::Kerf.Topology.Orientation.Reversed)
            zDir = -zDir;
        var x = xDir is { } given ? given.Normalized() : new Vector(du).Normalized();
        return new Location(Plane.Create(origin, x, zDir));
    }

    // ---- Wires ----

    /// <summary>The perimeter wire.</summary>
    public Wire OuterWire()
    {
        NeedsGeometry("outer wire");
        return (Wire)new Wire(TopoFace.OuterWire).ExtractedFrom(this);
    }

    /// <summary>The hole wires.</summary>
    public ShapeList<Wire> InnerWires()
    {
        if (IsEmpty)
            return [];
        var outer = OuterWire();
        return new ShapeList<Wire>(Wires().Where(w => !w.IsSame(outer)));
    }

    /// <summary>Whether this planar face lies in <paramref name="plane"/>.</summary>
    public bool IsCoplanar(Plane plane)
    {
        var (u0, _, v0, _) = UVBounds();
        var pnt = TopoFace.ToSurface().Evaluate(u0, v0);
        var normal = TopoUtil.FaceNormalVector(TopoFace, u0, v0);
        return plane.Contains(new Vector(pnt)) && 1 - Math.Abs(plane.ZDir.Dot(normal)) < Constants.Tolerance;
    }

    /// <summary>Whether a point lies on the face within a tolerance, its boundary included.</summary>
    public bool IsInside(Vector point, double tolerance = 1.0e-6) => TopoUtil.PointOnFace(TopoFace, point, tolerance);

    /// <summary>
    /// This face with holes cut by interior wires, which must lie wholly inside it. More efficient than a boolean; OCCT may
    /// fail if a wire runs the wrong way.
    /// </summary>
    public Face MakeHoles(IEnumerable<Wire> interiorWires)
    {
        var made = TopoFace.WithHoles(interiorWires.Select(w => w.TopoWire));
        if (!made.TryGetValue(out var holed))
            throw new InvalidOperationException("Error adding interior hole in non-planar face with provided interior_wires",
                                                new KerfException(made.Diagnostics));
        return new Face(holed).Fix();
    }

    /// <summary>This face with its holes filled in.</summary>
    public Face WithoutHoles()
    {
        if (IsEmpty)
            throw new InvalidOperationException("Cannot remove holes from an empty face");
        var inner = InnerWires();
        if (inner.Count == 0)
            return this;
        // The face rebuilt without its holes (as BRepTools_ReShape removing them does)
        var kept = TopoShape.Children.Where(child => !inner.Any(hole => hole.TopoShape.IsSame(child)));
        return (Face)Rewrapped((global::Kerf.Topology.Face)TopoShape.WithChildren(kept));
    }

    /// <summary>
    /// This face with the given corners chamfered: <paramref name="distance"/> along the first edge at each corner
    /// (along <paramref name="edge"/> if given) and <paramref name="distance2"/> along the other.
    /// </summary>
    public Face Chamfer2D(double distance, double distance2, IEnumerable<Vertex> vertices, Edge? edge = null)
    {
        // The face's own MakeFillet2d, as build123d chamfered before it worked around OCCT's merging of a face's wires
        // (kernel decision 1474 keeps them apart)
        var list = vertices.Where(v => !v.IsEmpty).ToList();
        if (list.Count == 0)
            return this;
        var graph = ShapeGraph.Build(TopoFace);
        var corners = new List<(global::Kerf.Topology.Edge, global::Kerf.Topology.Edge)>();
        foreach (var v in list)
        {
            var edges = graph.Ancestors(v.TopoShape, global::Kerf.Topology.ShapeType.Edge);
            var (edge1, edge2) = Wire.OrderChamferEdges(edge, (new Edge((global::Kerf.Topology.Edge)edges.First()),
                                                               new Edge((global::Kerf.Topology.Edge)edges.Last())));
            if (!edge1.IsEmpty && !edge2.IsEmpty)
                corners.Add((edge1.TopoEdge, edge2.TopoEdge));
        }
        var built = Failures.Get(TopoFace.Chamfer2D(corners, distance, distance2), () => "2D chamfer");
        var face = new Face(built.Shape).Fix();
        if (NormalAt() != face.NormalAt())
            face = (Face)(-face);
        // The edges the chamfers trimmed are modified (OCCT's DescendantEdge); the builder answers nothing as generated
        var record = ShapeHistory.FromAlgorithm(s => built.History.Modified(s), _ => [], [TopoShape], face.TopoShape);
        return (Face)face.MadeBy(record.AddModified(TopoShape, face.TopoShape));
    }

    /// <summary>This face with the given corners filleted.</summary>
    public Face Fillet2D(double radius, IEnumerable<Vertex> vertices) =>
        EditWires(vertices, (wire, inWire) => wire.Fillet2D(radius, inWire));

    /// <summary>
    /// Fillets each wire on its own, with build123d's wire fillet, and rebuilds the face, recording the face as modified.
    /// </summary>
    private Face EditWires(IEnumerable<Vertex> vertices, Func<Wire, List<Vertex>, Wire> edit)
    {
        var list = vertices.Where(v => !v.IsEmpty).ToList();
        if (list.Count == 0)
            return this;
        var wires = new List<Wire>();
        var record = new ShapeHistory();
        foreach (var wire in new[] { OuterWire() }.Concat(InnerWires()))
        {
            var wireVertices = wire.Vertices();
            var inWire = list.Where(v => wireVertices.Any(wv => wv.TopoShape.IsSame(v.TopoShape))).ToList();
            if (inWire.Count > 0)
            {
                var edited = edit(wire, inWire);
                record = record.Merge(edited.History);
                wires.Add(edited);
            }
            else
                wires.Add(wire);
        }
        var face = new Face(wires[0], wires.Skip(1));
        if (NormalAt() != face.NormalAt())
            face = (Face)(-face);
        return (Face)face.MadeBy(record.AddModified(TopoShape, face.TopoShape));
    }

    /// <summary>
    /// This face projected onto a shape along <paramref name="direction"/>: the faces or shells of the projection, nearest
    /// first (those behind the projection aren't returned).
    /// </summary>
    public ShapeList<Shape2D> ProjectToShape(Shape targetObject, Vector direction)
    {
        if (targetObject is Vertex)
            throw new ArgumentException("projection to a vertex is not supported");
        var maxDimension = TopologyUtils.FindMaxDimension([this, targetObject]);
        var extruded = TopoUtil.Extrude(TopoShape, direction * maxDimension);
        var intersected = new List<Shape2D>();
        if (targetObject is Face face)
        {
            var shape = TopoUtil.BoolOp([extruded], [face.TopoShape], BooleanKind.Common);
            if (!shape.IsNull)
                intersected.AddRange(TopoUtil.TopLevelShapes(shape).Select(f => new Face((global::Kerf.Topology.Face)f)));
        }
        else
            foreach (var shell in targetObject.Shells())
            {
                var shape = TopoUtil.BoolOp([extruded], [shell.TopoShape], BooleanKind.Common);
                intersected.AddRange(TopoUtil.TopLevelShapes(shape).Select(s => new Shell((global::Kerf.Topology.Shell)s)));
            }
        var sorted = new ShapeList<Shape2D>(intersected).OrderBy(new Axis(Center(), direction));
        return new ShapeList<Shape2D>(sorted.Select(s => s.Faces().Count == 1 ? (Shape2D)s.SingleFace() : s));
    }
}
