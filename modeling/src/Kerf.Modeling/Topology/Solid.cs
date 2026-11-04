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
public partial class Solid : Shape3D
{
    /// <summary>The empty Solid.</summary>
    public Solid() : base(null) { }

    /// <summary>A solid wrapping an OCCT solid.</summary>
    public Solid(global::Kerf.Topology.Solid solid, string label = "", Color? color = null) : base(solid, label, color) { }

    /// <summary>The solid a closed shell bounds.</summary>
    public Solid(Shell shell, string label = "", Color? color = null) : base(MakeSolid(shell), label, color) { }

    private static global::Kerf.Topology.Solid MakeSolid(Shell shell)
    {
        return Failures.Get(shell.TopoShell.MakeSolid(), () => "solid from shell").Shape;
    }

    internal global::Kerf.Topology.Solid TopoSolid => (global::Kerf.Topology.Solid)TopoShape;

    internal override double Order => 3.0;

    internal override Shape MakeZero() => new Solid();

    /// <summary>The volume of the solid.</summary>
    public override double Volume => ComputeVolume();

    // ---- Primitives ----

    /// <summary>A box from the plane's origin (default XY), extending along each of its positive axes.</summary>
    public static Solid MakeBox(double length, double width, double height, Plane? plane = null) =>
        Failures.Doing(() => $"box {Formatting.Number(length)} × {Formatting.Number(width)} × {Formatting.Number(height)}", () =>
        {
            var ax2 = (plane ?? Plane.XY).ToGpAx2();
            return new Solid(Failures.Get(global::Kerf.Topology.Solid.Box(length, width, height, ax2), () => "box").Shape);
        });

    /// <summary>A full or partial cone from the plane's origin (default XY).</summary>
    public static Solid MakeCone(double baseRadius, double topRadius, double height, Plane? plane = null, double angle = 360) =>
        Failures.Doing(() => $"cone of radii {Formatting.Number(baseRadius)} and {Formatting.Number(topRadius)}, height {Formatting.Number(height)}", () =>
        {
            var ax2 = (plane ?? Plane.XY).ToGpAx2();
            return new Solid(Failures.Get(global::Kerf.Topology.Solid.Cone(baseRadius, topRadius, height, ax2, angle * Constants.DegToRad),
                                          () => "cone").Shape);
        });

    /// <summary>A full or partial cylinder with its base centred on the plane's origin (default XY).</summary>
    public static Solid MakeCylinder(double radius, double height, Plane? plane = null, double angle = 360) =>
        Failures.Doing(() => $"cylinder of radius {Formatting.Number(radius)}, height {Formatting.Number(height)}", () =>
        {
            var ax2 = (plane ?? Plane.XY).ToGpAx2();
            return new Solid(Failures.Get(global::Kerf.Topology.Solid.Cylinder(radius, height, ax2, angle * Constants.DegToRad),
                                          () => "cylinder").Shape);
        });

    /// <summary>A full or partial sphere centred on the plane's origin (default XY).</summary>
    public static Solid MakeSphere(double radius, Plane? plane = null, double angle1 = -90, double angle2 = 90, double angle3 = 360) =>
        Failures.Doing(() => $"sphere of radius {Formatting.Number(radius)}", () =>
        {
            var ax2 = (plane ?? Plane.XY).ToGpAx2();
            var options = new SphereOptions
            {
                MinLatitude = angle1 * Constants.DegToRad,
                MaxLatitude = angle2 * Constants.DegToRad,
                Angle = angle3 * Constants.DegToRad,
            };
            return new Solid(Failures.Get(global::Kerf.Topology.Solid.Sphere(radius, ax2, options), () => "sphere").Shape);
        });

    /// <summary>A full or partial torus centred on the plane's origin (default XY).</summary>
    public static Solid MakeTorus(double majorRadius, double minorRadius, Plane? plane = null, double startAngle = 0,
                                  double endAngle = 360, double majorAngle = 360) =>
        Failures.Doing(() => $"torus of radii {Formatting.Number(majorRadius)} and {Formatting.Number(minorRadius)}", () =>
        {
            var ax2 = (plane ?? Plane.XY).ToGpAx2();
            var options = new TorusOptions
            {
                MinorStart = startAngle * Constants.DegToRad,
                MinorEnd = endAngle * Constants.DegToRad,
                Angle = majorAngle * Constants.DegToRad,
            };
            return new Solid(Failures.Get(global::Kerf.Topology.Solid.Torus(majorRadius, minorRadius, ax2, options), () => "torus").Shape);
        });

    /// <summary>A wedge on the plane (default XY).</summary>
    public static Solid MakeWedge(double deltaX, double deltaY, double deltaZ, double minX, double minZ, double maxX, double maxZ,
                                  Plane? plane = null) =>
        Failures.Doing(() => "wedge", () =>
        {
            var ax2 = (plane ?? Plane.XY).ToGpAx2();
            var options = new WedgeOptions { TopMin = (minX, minZ), TopMax = (maxX, maxZ) };
            return new Solid(Failures.Get(global::Kerf.Topology.Solid.Wedge(deltaX, deltaY, deltaZ, options, ax2), () => "wedge").Shape);
        });

    /// <summary>A loft through wires, optionally starting or ending with a vertex; ruled (stepped) or smooth.</summary>
    public static Solid MakeLoft(IEnumerable<Shape> objs, bool ruled = false) =>
        Failures.Doing(() => "loft", () => Result(new Solid((global::Kerf.Topology.Solid)TopoUtil.MakeLoft(objs, true, ruled))));

    /// <summary>A box the size of a bounding box, where it is.</summary>
    public static Solid FromBoundingBox(BoundBox box) => MakeBox(box.Size.X, box.Size.Y, box.Size.Z).Located(new Location(box.Min));

    /// <summary>A box the size of an oriented bounding box, where it is.</summary>
    public static Solid FromBoundingBox(OrientedBoundBox box)
    {
        var moved = new Plane(new Location(-box.Size / 2)).Moved(box.Location);
        return MakeBox(box.Size.X, box.Size.Y, box.Size.Z, moved);
    }

    // ---- Extrusions, sweeps and revolutions ----

    /// <summary>A face extruded into a solid.</summary>
    public static Solid Extrude(Face obj, Vector direction) =>
        Failures.Doing(() => "extrude", () => Result(new Solid((global::Kerf.Topology.Solid)TopoUtil.Extrude(obj.TopoShape, direction))));

    /// <summary>
    /// A "twisted prism": a section (a face, or a wire with optional holes) extruded along <paramref name="normal"/> while
    /// rotating through <paramref name="angle"/> degrees about it.
    /// </summary>
    public static Solid ExtrudeLinearWithRotation(Shape2D section, Vector center, Vector normal, double angle) =>
        section is Face face
            ? ExtrudeLinearWithRotation(face.OuterWire(), center, normal, angle, face.InnerWires())
            : throw new ArgumentException("section must be a Face or a Wire");

    /// <inheritdoc cref="ExtrudeLinearWithRotation(Shape2D, Vector, Vector, double)"/>
    public static Solid ExtrudeLinearWithRotation(Wire section, Vector center, Vector normal, double angle, IEnumerable<Wire>? innerWires = null) =>
        Failures.Doing(() => "extrude with rotation", () => ExtrudeLinearWithRotationCore(section, center, normal, angle, innerWires));

    private static Solid ExtrudeLinearWithRotationCore(Wire section, Vector center, Vector normal, double angle, IEnumerable<Wire>? innerWires)
    {
        static global::Kerf.Topology.Shape ExtrudeAuxSpine(global::Kerf.Topology.Wire wire, global::Kerf.Topology.Wire spine, global::Kerf.Topology.Wire auxSpine)
        {
            // BRepOffsetAPI_MakePipeShell with an auxiliary spine, closed into a solid
            var options = new SweepOptions { Frame = SweepFrame.Auxiliary, AuxiliarySpine = auxSpine, MakeSolid = true };
            return Failures.Get(Sweeps.Sweep(wire, spine, options), () => "sweep").Shape;
        }
        // A straight spine and an auxiliary helical one that turns the section
        var straightSpine = Wire.Combine([Edge.MakeLine(center, center + normal)])[0].TopoWire;
        var pitch = 360.0 / angle * normal.Length;
        var auxSpine = new Wire([Edge.MakeHelix(pitch, normal.Length, 1, center, normal)]).TopoWire;
        var outer = ExtrudeAuxSpine(section.TopoWire, straightSpine, auxSpine);
        var inner = (innerWires ?? []).Select(w => ExtrudeAuxSpine(w.TopoWire, straightSpine, auxSpine)).ToList();
        var innerComp = TopoUtil.MakeCompound(inner);
        var difference = Failures.Get(TopoUtil.Boolean(BooleanKind.Cut, [outer], [innerComp]), () => "cut").Shape;
        // The result may be wrapped in a compound
        return difference.IsCompound()
            ? Result(new Solid((global::Kerf.Topology.Solid)TopoUtil.UnwrapCompound((global::Kerf.Topology.Compound)difference)))
            : Result(new Solid((global::Kerf.Topology.Solid)difference));
    }

    /// <summary>
    /// A face extruded with a taper (degrees; positive narrows away from the profile). Along the profile's normal with
    /// planar, cylindrical or conical sides the sides are drafted, which keeps them planes and cones; otherwise the profile
    /// is lofted to an offset copy. <paramref name="flipInner"/> tapers holes the opposite way, for mould release.
    /// </summary>
    /// <exception cref="InvalidOperationException">The taper can't be built, as when it collapses the profile.</exception>
    public static Solid ExtrudeTaper(Face profile, Vector direction, double taper, bool flipInner = true) =>
        Failures.Doing(() => "tapered extrude", () => ExtrudeTaperCore(profile, direction, taper, flipInner));

    private static Solid ExtrudeTaperCore(Face profile, Vector direction, double taper, bool flipInner)
    {
        var pull = direction.Normalized();
        var profileWires = new List<Wire> { profile.OuterWire() };
        profileWires.AddRange(profile.InnerWires());

        // The prism shares the profile's face, which the draft no longer edits (the kernel's unit 10H5): no copy is made

        // Extrude the profile; its sides are the faces generated from its edges
        var directionVec = direction.ToGpVec();
        var prismBuilt = Failures.Get(profile.TopoShape.Extrude(directionVec), () => "extrude");
        var prism = prismBuilt.Shape;
        var sideFaces = new List<(global::Kerf.Topology.Face Face, double Angle)>();
        for (int i = 0; i < profileWires.Count; i++)
        {
            var angle = i > 0 && !flipInner ? -taper : taper;
            foreach (var edge in profileWires[i].Edges())
            {
                // The faces an edge generates: its side (the history goes on to its copy in the top cap)
                foreach (var side in prismBuilt.History.Generated(edge.TopoShape))
                    if (side.IsFace())
                        sideFaces.Add(((global::Kerf.Topology.Face)side, angle));
            }
        }
        bool alongNormal = Math.Abs(pull.Dot(profile.NormalAt())) > 1 - Constants.Tolerance;
        Solid? result = null;
        if (alongNormal && sideFaces.All(s => new Face(s.Face).GeomType is GeomType.Plane or GeomType.Cylinder or GeomType.Cone))
        {
            // Draft the sides about the plane of the profile, which keeps its size
            var center = profile.Center().ToGpPnt();
            var pullDir = pull.ToGpDir();
            var neutral = new Plane3D(center, pullDir);
            // The draft can't build every taper a loft can: a failure falls back to the loft
            var drafted = prism.Draft(sideFaces.Select(s => (s.Face, s.Angle * Constants.DegToRad)), pullDir, neutral);
            Built<global::Kerf.Topology.Shape>? draft = null;
            if (drafted.TryGetValue(out var built) && built.Shape.IsSolid())
            {
                draft = built;
                result = new Solid((global::Kerf.Topology.Solid)built.Shape);
            }
            // The draft can build sides that run into each other, as when a hole widens through a wall
            if (result is not null && !result.IsValid)
                result = null;
            if (result is not null)
                result = (Solid)result.MadeBy(
                    ShapeHistory.FromAlgorithm(s => prismBuilt.History.Modified(s), s => prismBuilt.History.Generated(s), [profile.TopoShape], prism)
                    .Merge(ShapeHistory.FromAlgorithm(s => draft!.History.Modified(s), s => draft!.History.Generated(s), [prism], result.TopoShape)));
        }
        if (result is null)
        {
            // Loft to the profile offset by the amount that gives the taper
            var offset = -direction.Length * Math.Tan(taper * Constants.DegToRad);
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
                    taperWires.Add(new Wire(plane.FromLocalCoords(localTaper).Edges()).Moved(new Location(direction)));
                }
                solids = [.. profileWires.Zip(taperWires).Select(p => MakeLoft([p.First, p.Second]))];
            }
            catch (Exception e) when (e is InvalidOperationException || Failures.IsKernelFailure(e))
            {
                throw new InvalidOperationException(
                    $"Tapered extrusion of {Formatting.Number(taper)} degrees over {Formatting.Number(direction.Length)} could not be built. "
                    + "The taper may collapse the profile - reduce the taper or the extrusion distance", e);
            }
            result = solids.Count > 1 ? (Solid)solids[0].Cut(solids.Skip(1)) : solids[0];
        }
        return result;
    }

    /// <summary>
    /// A face extruded along <paramref name="direction"/> until it meets a surface of <paramref name="target"/>: the next
    /// surface (or the last, with <see cref="Until.Last"/>); Previous and First extrude the other way. The surface must
    /// fully cover the profile where they meet.
    /// </summary>
    /// <exception cref="ArgumentException">The extrusion doesn't reach the target, or the surface reached doesn't cut it.</exception>
    public static Solid ExtrudeUntil(Face profile, Shape target, Vector direction, Until until = Until.Next) =>
        Failures.Doing(() => "extrude until", () => ExtrudeUntilCore(profile, target, direction, until));

    private static Solid ExtrudeUntilCore(Face profile, Shape target, Vector direction, Until until)
    {
        if (until is Until.Previous or Until.First)
        {
            direction = -direction;
            until = until == Until.Previous ? Until.Next : Until.Last;
        }
        // 1: an extrusion as long as the profile and target are far apart
        var maxDimension = TopologyUtils.FindMaxDimension([profile, target]);
        var extrusion = Extrude(profile, direction * maxDimension);

        // 2: the target's faces the extrusion modified
        // Non-destructive, so the target's tolerances are left alone
        var intersectOp = Failures.Get(TopoUtil.Boolean(BooleanKind.Common, [target.TopoShape], [extrusion.TopoShape]), () => "intersect");
        var intersection = intersectOp.Shape;
        if (!intersection.Explore<global::Kerf.Topology.Face>().Any())
            throw new ArgumentException("No intersection: extrusion does not contact target");
        var history = intersectOp.History;
        var modifiedTargetFaces = new List<global::Kerf.Topology.Shape>();
        foreach (var targetFace in target.TopoShape.Explore<global::Kerf.Topology.Face>())
            modifiedTargetFaces.AddRange(history.Modified(targetFace).Select(m => (global::Kerf.Topology.Shape)(global::Kerf.Topology.Face)m));

        // 3: sewn into one shell per surface passed through, sorted by distance from the profile
        var sewn = TopoUtil.SewFaces(modifiedTargetFaces);
        var surfaces = new ShapeList<Shape2D>(TopoUtil.TopLevelShapes(sewn).Select(s =>
            s.IsFace() ? (Shape2D)new Face((global::Kerf.Topology.Face)s)
            : s.IsShell() ? new Shell((global::Kerf.Topology.Shell)s)
            : throw new InvalidOperationException($"Invalid sewn shape {TopoUtil.TypeName(s)}"))).SortBy(s => s.DistanceTo(profile));
        var limit = surfaces[until == Until.Next ? 0 : surfaces.Count - 1];

        // 4: split the extrusion by that surface
        var pieces = extrusion.Split(limit, Keep.All);
        if (pieces.Count < 2)
            throw new ArgumentException("the surface reached does not cut the extrusion - the profile lies in it, or only touches it");

        // 5: the profile is the extrusion's own base face and survives the split, so return the piece that holds it
        foreach (var piece in pieces)
            if (piece.Faces().Any(f => f.TopoShape.IsSame(profile.TopoShape)))
                return (Solid)piece;
        // A kernel that never writes its inputs (OCCT.NET) gives the extrusion a copy of a profile it has to add pcurves
        // to, so then the piece holding the profile is found by place: the face at the profile's centre, of its area
        var center = profile.Center();
        foreach (var piece in pieces)
            if (piece.Faces().Any(f => Math.Abs(f.Area - profile.Area) <= Constants.Tolerance * Math.Max(1, profile.Area)
                                       && (f.Center() - center).Length <= Constants.Tolerance))
                return (Solid)piece;
        throw new InvalidOperationException("Extrusion is None");
    }

    /// <summary>A cross section (a face, or a wire with optional holes) revolved through <paramref name="angle"/> degrees about an axis.</summary>
    public static Solid Revolve(Face section, double angle, Axis axis) =>
        Failures.Doing(() => "revolve", () =>
        {
            var ax1 = axis.ToGpAx1();
            // On a copy of the profile, as build123d revolves (BRepPrimAPI_MakeRevol with copy)
            var revolved = Failures.Get(section.TopoShape.Revolve(ax1, angle * Constants.DegToRad, copyProfile: true), () => "revolve");
            return Result(new Solid((global::Kerf.Topology.Solid)revolved.Shape));
        });

    /// <inheritdoc cref="Revolve(Face, double, Axis)"/>
    public static Solid Revolve(Wire section, double angle, Axis axis, IEnumerable<Wire>? innerWires = null) =>
        Revolve(new Face(section, innerWires), angle, axis);

    /// <summary>
    /// A cross section swept along a path into a solid (or a shell, without <paramref name="makeSolid"/>).
    /// </summary>
    /// <remarks>
    /// Without <paramref name="isFrenet"/> the profile's orientation is kept as constant as possible, which along a helix
    /// makes it creep round; with it, the orientation follows the path's curvature, which keeps a helical sweep steady but
    /// can twist oddly along other paths. <paramref name="mode"/> gives a fixed binormal direction or an auxiliary spine.
    /// </remarks>
    public static Shape Sweep(Shape2D section, Shape1D path, bool makeSolid = true, bool isFrenet = false, object? mode = null,
                              Transition transition = Transition.Transformed) =>
        section is Face face
            ? SweepCore(face.OuterWire(), face.InnerWires(), path, makeSolid, isFrenet, mode, transition)
            : throw new ArgumentException("section must be a Face or a Wire");

    /// <inheritdoc cref="Sweep(Shape2D, Shape1D, bool, bool, object?, Transition)"/>
    public static Shape Sweep(Wire section, Shape1D path, IEnumerable<Wire>? innerWires = null, bool makeSolid = true, bool isFrenet = false,
                              object? mode = null, Transition transition = Transition.Transformed) =>
        SweepCore(section, innerWires?.ToList() ?? [], path, makeSolid, isFrenet, mode, transition);

    private static Shape SweepCore(Wire outerWire, IReadOnlyList<Wire> innerWires, Shape1D path, bool makeSolid, bool isFrenet,
                                   object? mode, Transition transition) =>
        Failures.Doing(() => "sweep", () => SweepShapes(outerWire, innerWires, path, makeSolid, isFrenet, mode, transition));

    private static Shape SweepShapes(Wire outerWire, IReadOnlyList<Wire> innerWires, Shape1D path, bool makeSolid, bool isFrenet,
                                     object? mode, Transition transition)
    {
        var shapes = new List<Shape>();
        var pathWire = path as Wire ?? new Wire((Edge)path);
        foreach (var wire in innerWires.Prepend(outerWire))
        {
            var options = SweepMode(path, mode, isFrenet) with { Transition = Face.TransitionMode(transition), MakeSolid = makeSolid };
            shapes.Add(Cast(Failures.Get(Sweeps.Sweep(wire.TopoShape, pathWire.TopoWire, options), () => "sweep").Shape));
        }
        var outer = shapes[0];
        if (shapes.Count > 1)
            outer = outer.Cut(shapes.Skip(1));
        // OCCT reports success for a sweep that folds over itself
        if (makeSolid && !outer.IsValid)
            Warnings.Warn("sweep created an invalid solid. A path with a sharp corner needs transition=Transition.RIGHT or "
                          + "Transition.ROUND, or a filleted path, and the section must be small enough to turn each corner.");
        return Result(outer);
    }

    /// <summary>
    /// The sweep's frame: fixed (a binormal vector, the profiles turned to the path), along an auxiliary spine (a wire or
    /// edge, by curvilinear equivalence), or Frenet or corrected Frenet.
    /// </summary>
    private static SweepOptions SweepMode(Shape1D path, object? binormal, bool isFrenet)
    {
        switch (binormal)
        {
            case null:
                return new SweepOptions { Frame = isFrenet ? SweepFrame.Frenet : SweepFrame.CorrectedFrenet };
            case Vector v:
            {
                var system = new Frame3D();
                var start = path.StartPoint().ToGpPnt();
                var dir = v.ToGpDir();
                system = system.WithLocation(start);
                system = system.WithDirection(dir);
                return new SweepOptions { Frame = SweepFrame.Fixed, FixedFrame = system, WithCorrection = true };
            }
            case Wire w:
                return new SweepOptions { Frame = SweepFrame.Auxiliary, AuxiliarySpine = w.TopoWire, CurvilinearEquivalence = true };
            case Edge e:
                return new SweepOptions { Frame = SweepFrame.Auxiliary, AuxiliarySpine = new Wire(e).TopoWire, CurvilinearEquivalence = true };
            default:
                throw new ArgumentException($"A sweep mode is a Vector, Wire or Edge, not a {binormal.GetType().Name}");
        }
    }

    /// <summary>A sweep through a sequence of profiles (wires or faces' outer wires) along a path.</summary>
    public static Solid SweepMulti(IEnumerable<Shape2D> profiles, Shape1D path, bool makeSolid = true, bool isFrenet = false, object? binormal = null) =>
        SweepMulti(profiles.Select(p => ((Face)p).OuterWire()), path, makeSolid, isFrenet, binormal);

    /// <inheritdoc cref="SweepMulti(IEnumerable{Shape2D}, Shape1D, bool, bool, object?)"/>
    public static Solid SweepMulti(IEnumerable<Wire> profiles, Shape1D path, bool makeSolid = true, bool isFrenet = false, object? binormal = null) =>
        Failures.Doing(() => "multi-section sweep", () => SweepMultiCore(profiles, path, makeSolid, isFrenet, binormal));

    private static Solid SweepMultiCore(IEnumerable<Wire> profiles, Shape1D path, bool makeSolid, bool isFrenet, object? binormal)
    {
        var pathWire = path as Wire ?? new Wire((Edge)path);
        var options = SweepMode(path, binormal, isFrenet) with { MakeSolid = makeSolid };
        var built = Failures.Get(Sweeps.Sweep([.. profiles.Select(p => p.TopoShape)], pathWire.TopoWire, options), () => "sweep");
        var swept = Result(new Solid((global::Kerf.Topology.Solid)built.Shape));
        if (makeSolid && !swept.IsValid)
            Warnings.Warn("sweep_multi created an invalid solid. The profiles may be too large for the curvature of the path "
                          + "or cross each other along it.");
        return swept;
    }

    /// <summary>
    /// A face or shell thickened along its normals by <paramref name="depth"/> (positive or negative). For a face,
    /// <paramref name="normalOverride"/> says which way is up, flipping the face's normal if it points the other way.
    /// </summary>
    /// <exception cref="InvalidOperationException">OCCT can't thicken the surface.</exception>
    public static Solid Thicken(Shape2D surface, double depth, Vector? normalOverride = null) =>
        Failures.Doing(() => "thicken", () => ThickenCore(surface, depth, normalOverride));

    private static Solid ThickenCore(Shape2D surface, double depth, Vector? normalOverride)
    {
        var adjusted = depth;
        if (surface is Face face && normalOverride is { } up)
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
        var thickened = surface.TopoShape.Thicken(adjusted, options);
        if (!thickened.TryGetValue(out var solid))
            throw new InvalidOperationException("Error applying thicken to given surface", new KerfException(thickened.Diagnostics));
        return Result(new Solid(solid.Shape));
    }

    /// <summary>The solid with a draft of <paramref name="angle"/> degrees applied to planar, cylindrical or conical faces.</summary>
    /// <exception cref="DraftAngleError">The draft can't be added to a face, or built.</exception>
    public Solid Draft(IEnumerable<Face> faces, Plane neutralPlane, double angle)
    {
        var faceList = faces.ToList();
        foreach (var face in faceList)
            if (face.GeomType is not (GeomType.Plane or GeomType.Cylinder or GeomType.Cone))
                throw new ArgumentException(
                    $"Face {face} has unsupported geometry type {face.GeomType.ToString().ToUpperInvariant()}. "
                    + "Only PLANAR, CYLINDRICAL, and CONICAL faces are supported.");
        var dir = neutralPlane.ZDir.ToGpDir();
        var pln = neutralPlane.ToGpPln();
        var drafted = TopoShape.Draft(faceList.Select(f => f.TopoFace), dir, angle * Constants.DegToRad, pln);
        if (!drafted.TryGetValue(out var built))
        {
            var error = drafted.Error!;
            global::Kerf.Topology.Shape? problem = error.Shapes is [var shape, ..] ? shape : null;
            // A face that couldn't be drafted is the problematic shape itself; a build that failed has no face
            if (problem is { } p && p.IsFace() && faceList.FirstOrDefault(f => f.TopoShape.IsSame(p)) is { } face)
                throw new DraftAngleError("Draft could not be added to a face.", face, p);
            throw new DraftAngleError("Draft build failed on the given solid.", null, problem, new KerfException(drafted.Diagnostics));
        }
        var result = new Solid((global::Kerf.Topology.Solid)built.Shape);
        return (Solid)result.MadeBy(ShapeHistory.FromAlgorithm(s => built.History.Modified(s), s => built.History.Generated(s),
                                                               [TopoShape], result.TopoShape));
    }

    // ---- Contacts ----

    /// <summary>
    /// Where this solid's boundary contacts another shape without their interiors overlapping: shared faces, edges and
    /// vertices with another solid, face or shell; edge ends and tangent points on the boundary for an edge or wire.
    /// </summary>
    public override ShapeList<Shape> Touch(Shape other, double tolerance = 1e-6) => TouchCore(other, tolerance, null);

    internal ShapeList<Shape> TouchCore(Shape other, double tolerance, List<Shape>? foundSolids)
    {
        bool VertexOnEdges(Vertex v, IEnumerable<Edge> edges) => edges.Any(e => v.DistanceTo(e) <= tolerance);
        bool VertexOnFace(Vertex v, Face f) => v.DistanceTo(f) <= tolerance;
        bool VertexOnFaces(Vertex v, IEnumerable<Face> faces) => faces.Any(f => VertexOnFace(v, f));
        // Distance can't tell an edge on a face from one crossing it, so ask for the common part
        bool OnFaces(Shape s, IEnumerable<Face> faces) => faces.Any(f => BoolOpList([s], [f], BooleanKind.Common).Count > 0);
        bool EdgeOnFaces(Edge e, IEnumerable<Face> faces) => OnFaces(e, faces);
        (Vector, Vector) PointNormal(Face face, double u, double v)
        {
            var (u0, u1, v0, v1) = face.UVBounds();
            double pu = u0 + u * (u1 - u0), pv = v0 + v * (v1 - v0);
            return (new Vector(face.TopoFace.ToSurface().Evaluate(pu, pv)), TopoUtil.FaceNormalVector(face.TopoFace, pu, pv));
        }
        // Faces are equal geometrically, not as the same OCCT object: compare a grid of positions and normals
        bool FacesEqual(Face f1, Face f2, int gridSize = 4)
        {
            if (!f1.BoundingBox(optimal: false).Intersects(f2.BoundingBox(optimal: false), tolerance))
                return false;
            for (int i = 0; i < gridSize; i++)
                for (int j = 0; j < gridSize; j++)
                {
                    double u = i / (double)(gridSize - 1), v = j / (double)(gridSize - 1);
                    var (p1, n1) = PointNormal(f1, u, v);
                    var (p2, n2) = PointNormal(f2, u, v);
                    if ((p1 - p2).Length > tolerance || Math.Abs(n1.Dot(n2)) < 0.99)
                        return false;
                }
            return true;
        }
        bool IsDuplicate(Shape shape, IEnumerable<Shape> existing) => shape switch
        {
            Vertex v => existing.OfType<Vertex>().Any(o => v.ToVector() == o.ToVector()),
            Edge e => existing.OfType<Edge>().Any(o => e.GeomEqual(o, tolerance)),
            Face f => existing.OfType<Face>().Any(o => FacesEqual(f, o)),
            _ => false,
        };

        var results = new List<Shape>();
        switch (other)
        {
            case Solid or Face or Shell:
            {
                // For two solids, leave out what bounds their common volume
                var intersectFaces = new List<Face>();
                if (other is Solid)
                {
                    foundSolids ??= [.. IntersectCore(other, tolerance, includeTouched: false) ?? []];
                    intersectFaces = foundSolids.SelectMany(s => s.Faces()).ToList();
                }
                var selfFaces = Faces().Select(f => (Face: f, Box: f.BoundingBox(optimal: false))).ToList();
                var otherFaces = other.Faces().Select(f => (Face: f, Box: f.BoundingBox(optimal: false))).ToList();
                var raw = new List<Shape>();
                foreach (var (sf, sfBox) in selfFaces)
                    foreach (var (of, ofBox) in otherFaces)
                    {
                        if (!sfBox.Intersects(ofBox, tolerance))
                            continue;
                        // Touches first (cheap): tangent vertices
                        foreach (var r in sf.Touch(of, tolerance))
                            if (!IsDuplicate(r, raw) && !(r is Vertex rv && VertexOnFaces(rv, intersectFaces)))
                                raw.Add(r);
                        // Then the intersection (expensive): shared faces and edges
                        foreach (var r in sf.Intersect([of], tolerance))
                            if (!IsDuplicate(r, raw) && !OnFaces(r, intersectFaces))
                                raw.Add(r);
                    }
                // Keep only what no higher-dimensional result contains
                var allFaces = raw.OfType<Face>().ToList();
                var allEdges = raw.OfType<Edge>().ToList();
                results.AddRange(raw.Where(r => r switch
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
                var selfFaces = Faces().Select(f => (Face: f, Box: f.BoundingBox(optimal: false))).ToList();
                var otherBox = other.BoundingBox(optimal: false);
                // The edge's ends on the boundary
                foreach (var ov in other.Vertices())
                    if (selfFaces.Any(sf => VertexOnFace(ov, sf.Face)))
                        results.Add(ov);
                // And where it is tangent to the surface
                foreach (var (sf, sfBox) in selfFaces)
                {
                    if (!sfBox.Intersects(otherBox, tolerance))
                        continue;
                    var distance = sf.TopoShape.Distance(other.TopoShape);
                    if (!distance.TryGetValue(out var extrema) || extrema.Value > tolerance)
                        continue;
                    foreach (var solution in extrema.Solutions)
                    {
                        var p1 = solution.OnA;
                        var p2 = solution.OnB;
                        if (p1.DistanceTo(p2) > tolerance)
                            continue;
                        var vertex = new Vertex(p1.X, p1.Y, p1.Z);
                        if (!IsDuplicate(vertex, results))
                            results.Add(vertex);
                    }
                }
                break;
            }
            case Vertex vertex:
                if (Faces().Any(sf => VertexOnFace(vertex, sf)))
                    results.Add(vertex);
                break;
            default:
                // A compound iterates; others have no touches
                results.AddRange(other.Touch(this, tolerance));
                break;
        }
        return new ShapeList<Shape>(results.Distinct());
    }
}

/// <summary>The error <see cref="Solid.Draft"/> throws, with the face it failed on and OCCT's problematic shape.</summary>
public sealed class DraftAngleError : InvalidOperationException
{
    /// <summary>Creates the error.</summary>
    public DraftAngleError(string message, Face? face = null, global::Kerf.Topology.Shape? problematicShape = null, Exception? inner = null)
        : base(message, inner)
    {
        Face = face;
        ProblematicShape = problematicShape;
    }

    /// <summary>The face the draft couldn't be added to, if it was one.</summary>
    public Face? Face { get; }

    /// <summary>The shape OCCT reports as problematic.</summary>
    public global::Kerf.Topology.Shape? ProblematicShape { get; }
}
