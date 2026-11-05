using Kerf.Construction;
using Kerf.Features;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Offsets;
using Kerf.Geometry;

namespace Kerf.Modeling;

/// <summary>
/// A closed volume bounded by shells: the three-dimensional element of a model, and what booleans usually work on.
/// </summary>
public static partial class SolidExtensions
{
    extension(Solid self)
    {

        internal Solid TopoSolid => (Solid)self;

        /// <summary>The solid with a draft of <paramref name="angle"/> degrees applied to planar, cylindrical or conical faces.</summary>
        /// <exception cref="InvalidOperationException">
        /// The draft can't be added to a face, or built: the shorthand's failure (unit 10M3).
        /// <see cref="Operations.Draft(IEnumerable{Face}, Plane, double)"/> returns it as a result, with the code
        /// <see cref="ModelingDiagnostics.DraftFailed"/> and the face as the error's shape.
        /// </exception>
        public Solid Draft(IEnumerable<Face> faces, Plane neutralPlane, double angle)
        {
            var faceList = faces.ToList();
            foreach (var face in faceList)
                if (face.GeomType is not (GeomType.Plane or GeomType.Cylinder or GeomType.Cone))
                    throw new ArgumentException(
                        $"Face {face} has unsupported geometry type {face.GeomType.ToString().ToUpperInvariant()}. "
                        + "Only PLANAR, CYLINDRICAL, and CONICAL faces are supported.");
            var dir = neutralPlane.ZDir.ToDirection();
            var pln = neutralPlane.ToPlane3D();
            var drafted = self.Draft(faceList.Select(f => f), dir, angle * Constants.DegToRad, pln);
            if (!drafted.TryGetValue(out var built))
            {
                var error = drafted.Error!;
                Shape? problem = error.Shapes is [var shape, ..] ? shape : null;
                // A face that couldn't be drafted is the problematic shape itself; a build that failed has no face
                if (problem is { } p && p.IsFace() && faceList.FirstOrDefault(f => f.IsSame(p)) is { } face)
                    throw Failures.Expected(ModelingDiagnostics.DraftFailed, $"The draft can't be added to the face at {face.Center().Describe("G6")}.",
                                            drafted.Diagnostics, null, p);
                throw Failures.Expected(ModelingDiagnostics.DraftFailed, "The drafted solid couldn't be built.", drafted.Diagnostics, null,
                                        problem is { } q ? [q] : []);
            }
            var result = (Solid)built.Shape;
            return (Solid)result.MadeBy(ModelHistory.FromAlgorithm(s => built.History.Modified(s), s => built.History.Generated(s),
                                                                   [self], result));
        }

        // ---- Contacts ----

        /// <summary>
        /// Where this solid's boundary contacts another shape without their interiors overlapping: shared faces, edges and
        /// vertices with another solid, face or shell; edge ends and tangent points on the boundary for an edge or wire.
        /// </summary>
        internal ShapeList<Shape> TouchOfSolid(Shape other, double tolerance = 1e-6) => self.TouchCore(other, tolerance, null);

        internal ShapeList<Shape> TouchCore(Shape other, double tolerance, List<Shape>? foundSolids)
        {
            bool VertexOnEdges(Vertex v, IEnumerable<Edge> edges) => edges.Any(e => v.DistanceTo(e) <= tolerance);
            bool VertexOnFace(Vertex v, Face f) => v.DistanceTo(f) <= tolerance;
            bool VertexOnFaces(Vertex v, IEnumerable<Face> faces) => faces.Any(f => VertexOnFace(v, f));
            // Distance can't tell an edge on a face from one crossing it, so ask for the common part
            bool OnFaces(Shape s, IEnumerable<Face> faces) => faces.Any(f => ShapeExtensions.BoolOpList([s], [f], BooleanKind.Common).Count > 0);
            bool EdgeOnFaces(Edge e, IEnumerable<Face> faces) => OnFaces(e, faces);
            (Point3D, Vector3D) PointNormal(Face face, double u, double v)
            {
                var (u0, u1, v0, v1) = face.UVBounds();
                double pu = u0 + u * (u1 - u0), pv = v0 + v * (v1 - v0);
                return ((face.ToSurface().Evaluate(pu, pv)), TopoUtil.FaceNormalVector(face, pu, pv));
            }
            // Faces are equal geometrically, not as the same OCCT object: compare a grid of positions and normals
            bool FacesEqual(Face f1, Face f2, int gridSize = 4)
            {
                if (!f1.Bounds(optimal: false).IntersectsNear(f2.Bounds(optimal: false), tolerance))
                    return false;
                for (int i = 0; i < gridSize; i++)
                    for (int j = 0; j < gridSize; j++)
                    {
                        double u = i / (double)(gridSize - 1), v = j / (double)(gridSize - 1);
                        var (p1, n1) = PointNormal(f1, u, v);
                        var (p2, n2) = PointNormal(f2, u, v);
                        if ((p1 - p2).Magnitude > tolerance || Math.Abs(n1.Dot(n2)) < 0.99)
                            return false;
                    }
                return true;
            }
            bool IsDuplicate(Shape shape, IEnumerable<Shape> existing) => TopoUtil.Typed(shape) switch
            {
                Vertex v => existing.OfKind<Vertex>().Any(o => v.Point.IsNear(o.Point)),
                Edge e => existing.OfKind<Edge>().Any(o => e.GeomEqual(o, tolerance)),
                Face f => existing.OfKind<Face>().Any(o => FacesEqual(f, o)),
                _ => false,
            };

            var results = new List<Shape>();
            switch (TopoUtil.Typed(other))
            {
                case Solid or Face or Shell:
                {
                    // For two solids, leave out what bounds their common volume
                    var intersectFaces = new List<Face>();
                    if (other.TryAs<Solid>(out _))
                    {
                        foundSolids ??= [.. ShapeExtensions.IntersectOne(self.AsShape(), other, tolerance, includeTouched: false) ?? []];
                        intersectFaces = foundSolids.SelectMany(s => s.Faces()).ToList();
                    }
                    var selfFaces = self.Faces().Select(f => (Face: f, Box: f.Bounds(optimal: false))).ToList();
                    var otherFaces = other.Faces().Select(f => (Face: f, Box: f.Bounds(optimal: false))).ToList();
                    var raw = new List<Shape>();
                    foreach (var (sf, sfBox) in selfFaces)
                        foreach (var (of, ofBox) in otherFaces)
                        {
                            if (!sfBox.IntersectsNear(ofBox, tolerance))
                                continue;
                            // Touches first (cheap): tangent vertices
                            foreach (var r in sf.Touch(of, tolerance))
                                if (!IsDuplicate(r, raw) && !(r.TryAs(out Vertex rv) && VertexOnFaces(rv, intersectFaces)))
                                    raw.Add(r);
                            // Then the intersection (expensive): shared faces and edges
                            foreach (var r in sf.Intersect([of], tolerance))
                                if (!IsDuplicate(r, raw) && !OnFaces(r, intersectFaces))
                                    raw.Add(r);
                        }
                    // Keep only what no higher-dimensional result contains
                    var allFaces = raw.OfKind<Face>().ToList();
                    var allEdges = raw.OfKind<Edge>().ToList();
                    results.AddRange(raw.Where(r => TopoUtil.Typed(r) switch
                    {
                        Face => true,
                        Edge e => !EdgeOnFaces(e, allFaces),
                        Vertex v => !VertexOnFaces(v, allFaces) && !VertexOnEdges(v, allEdges),
                        _ => false,
                    }));
                    break;
                }
                case Edge or Wire:
                {
                    var selfFaces = self.Faces().Select(f => (Face: f, Box: f.Bounds(optimal: false))).ToList();
                    var otherBox = other.Bounds(optimal: false);
                    // The edge's ends on the boundary
                    foreach (var ov in other.Vertices())
                        if (selfFaces.Any(sf => VertexOnFace(ov, sf.Face)))
                            results.Add(ov);
                    // And where it is tangent to the surface
                    foreach (var (sf, sfBox) in selfFaces)
                    {
                        if (!sfBox.IntersectsNear(otherBox, tolerance))
                            continue;
                        var distance = MeasureOperations.Distance(sf, other);
                        if (!distance.TryGetValue(out var extrema) || extrema.Value > tolerance)
                            continue;
                        foreach (var solution in extrema.Solutions)
                        {
                            var p1 = solution.OnA;
                            var p2 = solution.OnB;
                            if (p1.DistanceTo(p2) > tolerance)
                                continue;
                            var vertex = Vertex.Make(p1.X, p1.Y, p1.Z);
                            if (!IsDuplicate(vertex, results))
                                results.Add(vertex);
                        }
                    }
                    break;
                }
                case Vertex vertex:
                    if (self.Faces().Any(sf => VertexOnFace(vertex, sf)))
                        results.Add(vertex);
                    break;
                default:
                    // A compound iterates; others have no touches
                    results.AddRange(other.Touch(self, tolerance));
                    break;
            }
            return new ShapeList<Shape>(results.Distinct(ShapeComparer.Same));
        }
    }

    extension(Solid)
    {

        // ---- Primitives ----

        /// <summary>A box from the plane's origin (default XY), extending along each of its positive axes.</summary>
        public static Solid MakeBox(double length, double width, double height, Plane? plane = null) =>
            Failures.Doing(() => $"box {Formatting.Number(length)} × {Formatting.Number(width)} × {Formatting.Number(height)}", () =>
            {
                var ax2 = (plane ?? Plane.XY).ToFrame3D();
                return Failures.Get(Solid.Box(length, width, height, ax2), () => "box").Shape;
            });

        /// <summary>A full or partial cone from the plane's origin (default XY).</summary>
        public static Solid MakeCone(double baseRadius, double topRadius, double height, Plane? plane = null, double angle = 360) =>
            Failures.Doing(() => $"cone of radii {Formatting.Number(baseRadius)} and {Formatting.Number(topRadius)}, height {Formatting.Number(height)}", () =>
            {
                var ax2 = (plane ?? Plane.XY).ToFrame3D();
                return Failures.Get(Solid.Cone(baseRadius, topRadius, height, ax2, angle * Constants.DegToRad),
                                              () => "cone").Shape;
            });

        /// <summary>A full or partial cylinder with its base centred on the plane's origin (default XY).</summary>
        public static Solid MakeCylinder(double radius, double height, Plane? plane = null, double angle = 360) =>
            Failures.Doing(() => $"cylinder of radius {Formatting.Number(radius)}, height {Formatting.Number(height)}", () =>
            {
                var ax2 = (plane ?? Plane.XY).ToFrame3D();
                return Failures.Get(Solid.Cylinder(radius, height, ax2, angle * Constants.DegToRad),
                                              () => "cylinder").Shape;
            });

        /// <summary>A full or partial sphere centred on the plane's origin (default XY).</summary>
        public static Solid MakeSphere(double radius, Plane? plane = null, double angle1 = -90, double angle2 = 90, double angle3 = 360) =>
            Failures.Doing(() => $"sphere of radius {Formatting.Number(radius)}", () =>
            {
                var ax2 = (plane ?? Plane.XY).ToFrame3D();
                var options = new SphereOptions
                {
                    MinLatitude = angle1 * Constants.DegToRad,
                    MaxLatitude = angle2 * Constants.DegToRad,
                    Angle = angle3 * Constants.DegToRad,
                };
                return Failures.Get(Solid.Sphere(radius, ax2, options), () => "sphere").Shape;
            });

        /// <summary>A full or partial torus centred on the plane's origin (default XY).</summary>
        public static Solid MakeTorus(double majorRadius, double minorRadius, Plane? plane = null, double startAngle = 0,
                                      double endAngle = 360, double majorAngle = 360) =>
            Failures.Doing(() => $"torus of radii {Formatting.Number(majorRadius)} and {Formatting.Number(minorRadius)}", () =>
            {
                var ax2 = (plane ?? Plane.XY).ToFrame3D();
                var options = new TorusOptions
                {
                    MinorStart = startAngle * Constants.DegToRad,
                    MinorEnd = endAngle * Constants.DegToRad,
                    Angle = majorAngle * Constants.DegToRad,
                };
                return Failures.Get(Solid.Torus(majorRadius, minorRadius, ax2, options), () => "torus").Shape;
            });

        /// <summary>A wedge on the plane (default XY).</summary>
        public static Solid MakeWedge(double deltaX, double deltaY, double deltaZ, double minX, double minZ, double maxX, double maxZ,
                                      Plane? plane = null) =>
            Failures.Doing(() => "wedge", () =>
            {
                var ax2 = (plane ?? Plane.XY).ToFrame3D();
                var options = new WedgeOptions { TopMin = (minX, minZ), TopMax = (maxX, maxZ) };
                return Failures.Get(Solid.Wedge(deltaX, deltaY, deltaZ, options, ax2), () => "wedge").Shape;
            });

        /// <summary>A loft through wires, optionally starting or ending with a vertex; ruled (stepped) or smooth.</summary>
        public static Solid MakeLoft<T>(IEnumerable<T> objs, bool ruled = false) where T : IShape => Solid.MakeLoft(objs.Select(o => o.AsShape()), ruled);

        /// <inheritdoc cref="MakeLoft{T}(IEnumerable{T}, bool)"/>
        public static Solid MakeLoft(IEnumerable<Shape> objs, bool ruled = false) =>
            Failures.Doing(() => "loft", () => ((Solid)TopoUtil.MakeLoft(objs, true, ruled)));

        /// <summary>A box the size of a bounding box, where it is.</summary>
        public static Solid FromBoundingBox(BoundingBox box) => Solid.MakeBox(box.Size.X, box.Size.Y, box.Size.Z).Located(new Location(box.Min));

        /// <summary>A box the size of an oriented bounding box, where it is.</summary>
        public static Solid FromBoundingBox(OrientedBoundBox box)
        {
            var moved = new Plane(new Location(-box.Size / 2)).Moved(box.Location);
            return Solid.MakeBox(box.Size.X, box.Size.Y, box.Size.Z, moved);
        }

        // ---- Extrusions, sweeps and revolutions ----

        /// <summary>A face extruded into a solid.</summary>
        public static Solid Extrude(Face obj, Vector3D direction) =>
            Failures.Doing(() => "extrude", () => ((Solid)TopoUtil.Extrude(obj, direction)));

        /// <summary>
        /// A "twisted prism": a section (a face, or a wire with optional holes) extruded along <paramref name="normal"/> while
        /// rotating through <paramref name="angle"/> degrees about it.
        /// </summary>
        public static Solid ExtrudeLinearWithRotation(IShape2D section, Point3D center, Vector3D normal, double angle) =>
            section.AsShape().TryAs(out Face face)
                ? Solid.ExtrudeLinearWithRotation(face.OuterWire(), center, normal, angle, face.InnerWires())
                : throw new ArgumentException("section must be a Face or a Wire");

        /// <inheritdoc cref="ExtrudeLinearWithRotation(IShape2D, Point3D, Vector3D, double)"/>
        public static Solid ExtrudeLinearWithRotation(Wire section, Point3D center, Vector3D normal, double angle, IEnumerable<Wire>? innerWires = null) =>
            Failures.Doing(() => "extrude with rotation", () => SolidExtensions.ExtrudeLinearWithRotationCore(section, center, normal, angle, innerWires));

        /// <summary>
        /// A face extruded with a taper (degrees; positive narrows away from the profile). Along the profile's normal with
        /// planar, cylindrical or conical sides the sides are drafted, which keeps them planes and cones; otherwise the profile
        /// is lofted to an offset copy. <paramref name="flipInner"/> tapers holes the opposite way, for mould release.
        /// </summary>
        /// <exception cref="InvalidOperationException">The taper can't be built, as when it collapses the profile.</exception>
        public static Solid ExtrudeTaper(Face profile, Vector3D direction, double taper, bool flipInner = true) =>
            Failures.Doing(() => "tapered extrude", () => SolidExtensions.ExtrudeTaperCore(profile, direction, taper, flipInner));

        /// <summary>
        /// A face extruded along <paramref name="direction"/> until it meets a surface of <paramref name="target"/>: the next
        /// surface (or the last, with <see cref="Until.Last"/>); Previous and First extrude the other way. The surface must
        /// fully cover the profile where they meet.
        /// </summary>
        /// <exception cref="ArgumentException">The extrusion doesn't reach the target, or the surface reached doesn't cut it.</exception>
        public static Solid ExtrudeUntil(Face profile, Shape target, Vector3D direction, Until until = Until.Next) =>
            Failures.Doing(() => "extrude until", () => SolidExtensions.ExtrudeUntilCore(profile, target, direction, until));

        /// <summary>A cross section (a face, or a wire with optional holes) revolved through <paramref name="angle"/> degrees about an axis.</summary>
        public static Solid Revolve(Face section, double angle, Axis axis) =>
            Failures.Doing(() => "revolve", () =>
            {
                var ax1 = axis.ToAxis3D();
                // On a copy of the profile, as build123d revolves (BRepPrimAPI_MakeRevol with copy)
                var revolved = Failures.Get(section.Revolve(ax1, angle * Constants.DegToRad, copyProfile: true), () => "revolve");
                return ((Solid)revolved.Shape);
            });

        /// <inheritdoc cref="Revolve(Face, double, Axis)"/>
        public static Solid Revolve(Wire section, double angle, Axis axis, IEnumerable<Wire>? innerWires = null) =>
            Solid.Revolve(Face.Make(section, innerWires), angle, axis);

        /// <summary>
        /// A cross section swept along a path into a solid (or a shell, without <paramref name="makeSolid"/>).
        /// </summary>
        /// <remarks>
        /// Without <paramref name="isFrenet"/> the profile's orientation is kept as constant as possible, which along a helix
        /// makes it creep round; with it, the orientation follows the path's curvature, which keeps a helical sweep steady but
        /// can twist oddly along other paths. <paramref name="mode"/> gives a fixed binormal direction or an auxiliary spine.
        /// </remarks>
        public static Shape Sweep(IShape2D section, IShape1D path, bool makeSolid = true, bool isFrenet = false, object? mode = null,
                                  Transition transition = Transition.Transformed) =>
            section.AsShape().TryAs(out Face face)
                ? SolidExtensions.SweepCore(face.OuterWire(), face.InnerWires(), path, makeSolid, isFrenet, mode, transition)
                : throw new ArgumentException("section must be a Face or a Wire");

        /// <inheritdoc cref="Sweep(IShape2D, IShape1D, bool, bool, object?, Transition)"/>
        public static Shape Sweep(Wire section, IShape1D path, IEnumerable<Wire>? innerWires = null, bool makeSolid = true, bool isFrenet = false,
                                  object? mode = null, Transition transition = Transition.Transformed) =>
            SolidExtensions.SweepCore(section, innerWires?.ToList() ?? [], path, makeSolid, isFrenet, mode, transition);

        /// <summary>A sweep through a sequence of profiles (wires or faces' outer wires) along a path.</summary>
        public static Solid SweepMulti(IEnumerable<Face> profiles, IShape1D path, bool makeSolid = true, bool isFrenet = false, object? binormal = null) =>
            Solid.SweepMulti(profiles.Select(p => p.OuterWire()), path, makeSolid, isFrenet, binormal);

        /// <inheritdoc cref="SweepMulti(IEnumerable{Face}, IShape1D, bool, bool, object?)"/>
        public static Solid SweepMulti(IEnumerable<Wire> profiles, IShape1D path, bool makeSolid = true, bool isFrenet = false, object? binormal = null) =>
            Failures.Doing(() => "multi-section sweep", () => SolidExtensions.SweepMultiCore(profiles, path, makeSolid, isFrenet, binormal));

        /// <summary>
        /// A face or shell thickened along its normals by <paramref name="depth"/> (positive or negative). For a face,
        /// <paramref name="normalOverride"/> says which way is up, flipping the face's normal if it points the other way.
        /// </summary>
        /// <exception cref="InvalidOperationException">OCCT can't thicken the surface.</exception>
        public static Solid Thicken(IShape2D surface, double depth, Vector3D? normalOverride = null) =>
            Failures.Doing(() => "thicken", () => SolidExtensions.ThickenCore(surface, depth, normalOverride));
    }



    extension(Solid)
    {
        /// <summary>The solid a closed shell bounds.</summary>
        public static Solid Make(Shell shell) => SolidExtensions.MakeSolid(shell);
    }


    private static Solid MakeSolid(Shell shell)
    {
        return Failures.Get(shell.MakeSolid(), () => "solid from shell").Shape;
    }

    private static Solid ExtrudeLinearWithRotationCore(Wire section, Point3D center, Vector3D normal, double angle, IEnumerable<Wire>? innerWires)
    {
        static Shape ExtrudeAuxSpine(Wire wire, Wire spine, Wire auxSpine)
        {
            // BRepOffsetAPI_MakePipeShell with an auxiliary spine, closed into a solid
            var options = new SweepOptions { Frame = SweepFrame.Auxiliary, AuxiliarySpine = auxSpine, MakeSolid = true };
            return Failures.Get(Sweeps.Sweep(wire, spine, options), () => "sweep").Shape;
        }
        // A straight spine and an auxiliary helical one that turns the section
        var straightSpine = Wire.Combine([Edge.MakeLine(center, center + normal)])[0];
        var pitch = 360.0 / angle * normal.Magnitude;
        var auxSpine = Wire.Make([Edge.MakeHelix(pitch, normal.Magnitude, 1, center, normal)]);
        var outer = ExtrudeAuxSpine(section, straightSpine, auxSpine);
        var inner = (innerWires ?? []).Select(w => ExtrudeAuxSpine(w, straightSpine, auxSpine)).ToList();
        var innerComp = TopoUtil.MakeCompound(inner);
        var difference = Failures.Get(TopoUtil.Boolean(BooleanKind.Cut, [outer], [innerComp]), () => "cut").Shape;
        // The result may be wrapped in a compound
        return difference.IsCompound()
            ? ((Solid)TopoUtil.UnwrapCompound((Compound)difference))
            : ((Solid)difference);
    }

    private static Solid ExtrudeTaperCore(Face profile, Vector3D direction, double taper, bool flipInner)
    {
        var pull = direction.Normalized();
        var profileWires = new List<Wire> { profile.OuterWire() };
        profileWires.AddRange(profile.InnerWires());

        // The prism shares the profile's face, which the draft no longer edits (the kernel's unit 10H5): no copy is made

        // Extrude the profile; its sides are the faces generated from its edges
        var directionVec = direction;
        var prismBuilt = Failures.Get(profile.Extrude(directionVec), () => "extrude");
        var prism = prismBuilt.Shape;
        var sideFaces = new List<(Face Face, double Angle)>();
        for (int i = 0; i < profileWires.Count; i++)
        {
            var angle = i > 0 && !flipInner ? -taper : taper;
            foreach (var edge in profileWires[i].Edges())
            {
                // The faces an edge generates: its side (the history goes on to its copy in the top cap)
                foreach (var side in prismBuilt.History.Generated(edge))
                    if (side.IsFace())
                        sideFaces.Add(((Face)side, angle));
            }
        }
        bool alongNormal = Math.Abs(pull.Dot(profile.NormalAt())) > 1 - Constants.Tolerance;
        Solid? result = null;
        if (alongNormal && sideFaces.All(s => s.Face.GeomType is GeomType.Plane or GeomType.Cylinder or GeomType.Cone))
        {
            // Draft the sides about the plane of the profile, which keeps its size
            var center = profile.Center();
            var pullDir = pull.ToDirection();
            var neutral = new Plane3D(center, pullDir);
            // The draft can't build every taper a loft can: a failure falls back to the loft
            var drafted = prism.Draft(sideFaces.Select(s => (s.Face, s.Angle * Constants.DegToRad)), pullDir, neutral);
            Built<Shape>? draft = null;
            if (drafted.TryGetValue(out var built) && built.Shape.IsSolid())
            {
                draft = built;
                result = (Solid)built.Shape;
            }
            // The draft can build sides that run into each other, as when a hole widens through a wall
            if (result is { IsValid: false })
                result = null;
            if (result is { } madeBy)
                result = madeBy.MadeBy(
                    ModelHistory.FromAlgorithm(s => prismBuilt.History.Modified(s), s => prismBuilt.History.Generated(s), [(Shape)profile], prism)
                    .Merge(ModelHistory.FromAlgorithm(s => draft!.History.Modified(s), s => draft!.History.Generated(s), [prism], result.Value)));
        }
        if (result is null)
        {
            // Loft to the profile offset by the amount that gives the taper
            var offset = -direction.Magnitude * Math.Tan(taper * Constants.DegToRad);
            List<Solid> solids;
            try
            {
                var plane = new Plane(profile);
                var taperWires = new List<Wire>();
                for (int i = 0; i < profileWires.Count; i++)
                {
                    var flip = i > 0 && flipInner ? -1 : 1;
                    var local = plane.ToLocalCoords(profileWires[i]);
                    var localTaper = local.Offset2D(flip * offset, kind: Kind.Intersection);
                    taperWires.Add(Wire.Make(plane.FromLocalCoords(localTaper).Edges()).Moved(new Location(direction)));
                }
                solids = [.. profileWires.Zip(taperWires).Select(p => Solid.MakeLoft([p.First, p.Second]))];
            }
            catch (Exception e) when (e is InvalidOperationException || Failures.IsKernelFailure(e))
            {
                throw new InvalidOperationException(
                    $"Tapered extrusion of {Formatting.Number(taper)} degrees over {Formatting.Number(direction.Magnitude)} could not be built. "
                    + "The taper may collapse the profile - reduce the taper or the extrusion distance", e);
            }
            result = solids.Count > 1 ? (Solid)solids[0].Cut(solids.Skip(1)) : solids[0];
        }
        return result.Value;
    }

    private static Solid ExtrudeUntilCore(Face profile, Shape target, Vector3D direction, Until until)
    {
        if (until is Until.Previous or Until.First)
        {
            direction = -direction;
            until = until == Until.Previous ? Until.Next : Until.Last;
        }
        // 1: an extrusion as long as the profile and target are far apart
        var maxDimension = TopologyUtils.FindMaxDimension([profile, target]);
        var extrusion = Solid.Extrude(profile, direction * maxDimension);

        // 2: the target's faces the extrusion modified
        // Non-destructive, so the target's tolerances are left alone
        var intersectOp = Failures.Get(TopoUtil.Boolean(BooleanKind.Common, [target], [extrusion]), () => "intersect");
        var intersection = intersectOp.Shape;
        if (!intersection.Explore<Face>().Any())
            throw new ArgumentException("No intersection: extrusion does not contact target");
        var history = intersectOp.History;
        var modifiedTargetFaces = new List<Shape>();
        foreach (var targetFace in target.Explore<Face>())
            modifiedTargetFaces.AddRange(history.Modified(targetFace).Select(m => (Shape)(Face)m));

        // 3: sewn into one shell per surface passed through, sorted by distance from the profile
        var sewn = TopoUtil.SewFaces(modifiedTargetFaces);
        var surfaces = new ShapeList<IShape2D>(TopoUtil.TopLevelShapes(sewn).Select(s =>
            s.IsFace() ? (IShape2D)(Face)s
            : s.IsShell() ? (Shell)s
            : throw new InvalidOperationException($"Invalid sewn shape {TopoUtil.TypeName(s)}"))).OrderBy(s => s.DistanceTo(profile)).ToShapeList();
        var limit = surfaces[until == Until.Next ? 0 : surfaces.Count - 1];

        // 4: split the extrusion by that surface
        var pieces = extrusion.Split(limit.AsShape(), Keep.All);
        if (pieces.Count < 2)
            throw new ArgumentException("the surface reached does not cut the extrusion - the profile lies in it, or only touches it");

        // 5: the profile is the extrusion's own base face and survives the split, so return the piece that holds it
        foreach (var piece in pieces)
            if (piece.Faces().Any(f => f.IsSame(profile)))
                return (Solid)piece;
        // A kernel that never writes its inputs (Kerf) gives the extrusion a copy of a profile it has to add pcurves
        // to, so then the piece holding the profile is found by place: the face at the profile's centre, of its area
        var center = profile.Center();
        foreach (var piece in pieces)
            if (piece.Faces().Any(f => Math.Abs(f.Area - profile.Area) <= Constants.Tolerance * Math.Max(1, profile.Area)
                                       && (f.Center() - center).Magnitude <= Constants.Tolerance))
                return (Solid)piece;
        throw new InvalidOperationException("The extrusion is empty.");
    }

    private static Shape SweepCore(Wire outerWire, IReadOnlyList<Wire> innerWires, IShape1D path, bool makeSolid, bool isFrenet,
                                   object? mode, Transition transition) =>
        Failures.Doing(() => "sweep", () => SolidExtensions.SweepShapes(outerWire, innerWires, path, makeSolid, isFrenet, mode, transition));

    private static Shape SweepShapes(Wire outerWire, IReadOnlyList<Wire> innerWires, IShape1D path, bool makeSolid, bool isFrenet,
                                     object? mode, Transition transition)
    {
        var shapes = new List<Shape>();
        var pathWire = TopoUtil.AsWire(path);
        foreach (var wire in innerWires.Prepend(outerWire))
        {
            var options = SolidExtensions.SweepMode(path, mode, isFrenet) with { Transition = FaceExtensions.TransitionMode(transition), MakeSolid = makeSolid };
            shapes.Add((Failures.Get(Sweeps.Sweep(wire, pathWire, options), () => "sweep").Shape));
        }
        var outer = shapes[0];
        if (shapes.Count > 1)
            outer = outer.Cut(shapes.Skip(1));
        // OCCT reports success for a sweep that folds over itself
        if (makeSolid && !outer.IsValid)
            Warnings.Warn("sweep created an invalid solid. A path with a sharp corner needs transition=Transition.RIGHT or "
                          + "Transition.ROUND, or a filleted path, and the section must be small enough to turn each corner.");
        return (outer);
    }

    /// <summary>
    /// The sweep's frame: fixed (a binormal vector, the profiles turned to the path), along an auxiliary spine (a wire or
    /// edge, by curvilinear equivalence), or Frenet or corrected Frenet.
    /// </summary>
    private static SweepOptions SweepMode(IShape1D path, object? binormal, bool isFrenet)
    {
        switch (binormal)
        {
            case null:
                return new SweepOptions { Frame = isFrenet ? SweepFrame.Frenet : SweepFrame.CorrectedFrenet };
            case Direction3D or Vector3D:
            {
                Vector3D v = binormal is Direction3D d ? d : (Vector3D)binormal;
                var system = new Frame3D();
                var start = path.StartPoint();
                var dir = v.ToDirection();
                system = system.WithLocation(start);
                system = system.WithDirection(dir);
                return new SweepOptions { Frame = SweepFrame.Fixed, FixedFrame = system, WithCorrection = true };
            }
            case Wire w:
                return new SweepOptions { Frame = SweepFrame.Auxiliary, AuxiliarySpine = w, CurvilinearEquivalence = true };
            case Edge e:
                return new SweepOptions { Frame = SweepFrame.Auxiliary, AuxiliarySpine = Wire.Make(e), CurvilinearEquivalence = true };
            default:
                throw new ArgumentException($"A sweep mode is a vector, direction, Wire or Edge, not a {binormal.GetType().Name}");
        }
    }

    private static Solid SweepMultiCore(IEnumerable<Wire> profiles, IShape1D path, bool makeSolid, bool isFrenet, object? binormal)
    {
        var pathWire = TopoUtil.AsWire(path);
        var options = SolidExtensions.SweepMode(path, binormal, isFrenet) with { MakeSolid = makeSolid };
        var built = Failures.Get(Sweeps.Sweep([.. profiles.Select(p => p)], pathWire, options), () => "sweep");
        var swept = ((Solid)built.Shape);
        if (makeSolid && !swept.IsValid)
            Warnings.Warn("sweep_multi created an invalid solid. The profiles may be too large for the curvature of the path "
                          + "or cross each other along it.");
        return swept;
    }

    private static Solid ThickenCore(IShape2D surface, double depth, Vector3D? normalOverride)
    {
        var adjusted = depth;
        if (surface.AsShape().TryAs(out Face face) && normalOverride is { } up)
        {
            var normal = face.NormalAt(face.Center()).Normalized();
            if (normal.Dot(up.Normalized()) < 0)
                adjusted = -depth;
        }
        var options = new OffsetOptions
        {
            Tolerance = 1.0e-5,
            Mode = OffsetMode.Skin,
            Intersection = true,
            SelfIntersection = false,
            Join = JoinKind.Intersection,
            RemoveInternalEdges = true,
        };
        var thickened = surface.AsShape().Thicken(adjusted, options);
        if (!thickened.TryGetValue(out var solid))
            throw Failures.Expected(ModelingDiagnostics.ThickenFailed, "The surface can't be thickened.", thickened.Diagnostics);
        return (solid.Shape);
    }
}

