using Kerf.Healing;

namespace Kerf.Modeling;

public static partial class Operations
{
    /// <summary>
    /// The convex hull of the points as a solid, one planar face for each flat side: <see cref="ConvexPolyhedron"/>, as
    /// a result rather than an exception (decision 1996).
    /// </summary>
    /// <param name="points">The points; those closer than <see cref="Constants.Tolerance"/> count as one.</param>
    /// <param name="rotation">Angles to rotate it about X, Y and Z.</param>
    /// <param name="align">How to align it; by default it isn't.</param>
    /// <returns>
    /// The polyhedron, or the failure <see cref="ModelingDiagnostics.HullFailed"/> saying why: the hull has no volume, a
    /// coordinate isn't finite, or the solid wasn't valid (then carrying it, through <c>TryGetInvalid</c>).
    /// </returns>
    /// <exception cref="ArgumentNullException"><paramref name="points"/> is null.</exception>
    public static Result<Part> MakeConvexPolyhedron(IEnumerable<Point3D> points, Rotation? rotation = null, Alignment? align = null)
    {
        ArgumentNullException.ThrowIfNull(points);
        return Failures.Try<Part>(() =>
        {
            try
            {
                return new ConvexPolyhedron(points, rotation, align);
            }
            catch (ArgumentException e)
            {
                throw Failures.Expected(ModelingDiagnostics.HullFailed, e.Message, inner: e);
            }
        });
    }

    // ---- draft ----

    /// <summary>
    /// The solid the faces were taken from, with a draft of <paramref name="angle"/> degrees applied to the faces about
    /// <paramref name="neutralPlane"/>.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// A face isn't a plane, cylinder or cone, or the faces aren't all taken from the same single solid.
    /// </exception>
    /// <returns>The drafted part, or the failure (<see cref="ModelingDiagnostics.DraftFailed"/>) if the draft can't be made.</returns>
    public static Result<Part> Draft(IEnumerable<Face> faces, Plane neutralPlane, double angle) =>
        Failures.Try(() => DraftOrThrow(TargetOf(faces, "draft"), faces, neutralPlane, angle));

    /// <summary>The faces of <paramref name="target"/> drafted (see <see cref="Draft(IEnumerable{Face}, Plane, double)"/>).</summary>
    public static Result<Part> Draft(Shape target, IEnumerable<Face> faces, Plane neutralPlane, double angle) =>
        Failures.Try(() => DraftOrThrow(target, faces, neutralPlane, angle));

    internal static Part DraftOrThrow(Shape target, IEnumerable<Face> faces, Plane neutralPlane, double angle)
    {
        using var log = HistoryLog.Open();
        var faceList = faces.ToList();

        GeomType[] validGeomTypes = [GeomType.Plane, GeomType.Cylinder, GeomType.Cone];
        var unsupported = faceList.Where(f => !validGeomTypes.Contains(f.GeomType)).ToList();
        if (unsupported.Count > 0)
            throw new ArgumentException("Draft not supported on face(s) with geometry: "
                                        + string.Join(", ", unsupported.Select(f => f.GeomType.ToString().ToUpperInvariant()).Distinct()));

        // Check that all the faces are of the one Solid
        var targetFaces = target.Faces();
        if (faceList.Any(f => !targetFaces.Contains(f, ShapeComparer<Face>.Same)))
            throw new ArgumentException("All faces must share the same topological parent (a Solid)");
        var parentSolids = target.Solids();
        if (parentSolids.Count != 1)
            throw new ArgumentException("Topological parent must be a single Solid");

        // Create the drafted solid
        // A face that can't be drafted is a failure whose first error holds the face
        var newSolid = parentSolids[0].Draft(faceList, neutralPlane, angle);

        // Phase 5: in builder mode the drafted solid replaces the builder's part
        return (Part)new Part([newSolid]).MadeBy(ModelHistory.Of([newSolid]));
    }

    /// <inheritdoc cref="Draft(IEnumerable{Face}, Plane, double)"/>
    public static Result<Part> Draft(Shape target, Face face, Plane neutralPlane, double angle) => Draft(target, [face], neutralPlane, angle);

    // ---- extrude ----

    /// <summary>
    /// The faces of a face or sketch extruded by <paramref name="amount"/>, its sign giving the direction. Each face is
    /// extruded along its normal, or along <see cref="ExtrudeOptions.Direction"/>, which non-planar faces need.
    /// </summary>
    /// <param name="toExtrude">The face or sketch to extrude.</param>
    /// <param name="amount">The distance to extrude; its sign controls the direction.</param>
    /// <param name="options">The direction, both ways, a taper and the clean; null for the defaults.</param>
    /// <exception cref="ArgumentNullException"><paramref name="toExtrude"/> is null.</exception>
    /// <exception cref="ArgumentException">A face is non-planar and no direction is given.</exception>
    /// <exception cref="InvalidOperationException">The tapered solid can't be built, as when the taper collapses the face.</exception>
    public static Result<Part> Extrude(Shape toExtrude, double amount, ExtrudeOptions? options = null) =>
        Failures.Try(() => ExtrudeOrThrow(toExtrude, amount, options));

    internal static Part ExtrudeOrThrow(Shape toExtrude, double amount, ExtrudeOptions? options)
    {
        // Phase 5: in builder mode the faces default to the pending faces (on their pending planes), and the result is
        // combined with mode
        return ExtrudeCore(toExtrude.Faces(), amount, null, null, options ?? ExtrudeOptions.Default);
    }

    /// <inheritdoc cref="Extrude(Shape, double, ExtrudeOptions?)"/>
    public static Result<Part> Extrude(IEnumerable<Face> toExtrude, double amount, ExtrudeOptions? options = null) =>
        Failures.Try(() => ExtrudeOrThrow(toExtrude, amount, options));

    internal static Part ExtrudeOrThrow(IEnumerable<Face> toExtrude, double amount, ExtrudeOptions? options)
    {
        ArgumentNullException.ThrowIfNull(toExtrude);
        return ExtrudeCore([.. toExtrude], amount, null, null, options ?? ExtrudeOptions.Default);
    }

    /// <summary>
    /// The faces of a face or sketch extruded up to a face of <paramref name="target"/>: the next or last one along each
    /// face's normal (or <see cref="ExtrudeOptions.Direction"/>), or with <see cref="Until.Previous"/> and
    /// <see cref="Until.First"/> the other way. <see cref="ExtrudeOptions.Taper"/> doesn't apply.
    /// </summary>
    /// <param name="toExtrude">The face or sketch to extrude.</param>
    /// <param name="until">Which face of the target to extrude up to.</param>
    /// <param name="target">The object to extrude up to.</param>
    /// <param name="options">The direction, both ways and the clean; null for the defaults.</param>
    /// <exception cref="ArgumentNullException"><paramref name="toExtrude"/> or <paramref name="target"/> is null.</exception>
    /// <exception cref="ArgumentException">
    /// A face is non-planar and no direction is given, the target is empty, or the extrusion doesn't reach it.
    /// </exception>
    public static Result<Part> Extrude(Shape toExtrude, Until until, Shape target, ExtrudeOptions? options = null) =>
        Failures.Try(() => ExtrudeOrThrow(toExtrude, until, target, options));

    internal static Part ExtrudeOrThrow(Shape toExtrude, Until until, Shape target, ExtrudeOptions? options)
    {
        return ExtrudeCore(toExtrude.Faces(), null, until, target, options ?? ExtrudeOptions.Default);
    }

    /// <inheritdoc cref="Extrude(Shape, Until, Shape, ExtrudeOptions?)"/>
    public static Result<Part> Extrude(IEnumerable<Face> toExtrude, Until until, Shape target, ExtrudeOptions? options = null) =>
        Failures.Try(() => ExtrudeOrThrow(toExtrude, until, target, options));

    internal static Part ExtrudeOrThrow(IEnumerable<Face> toExtrude, Until until, Shape target, ExtrudeOptions? options)
    {
        ArgumentNullException.ThrowIfNull(toExtrude);
        return ExtrudeCore([.. toExtrude], null, until, target, options ?? ExtrudeOptions.Default);
    }

    private static Part ExtrudeCore(IReadOnlyList<Face> faces, double? amount, Until? until, Shape? target, ExtrudeOptions options)
    {
        var (dir, both, taper, clean) = (options.Direction, options.Both, options.Taper, options.Clean);
        // Only the direction is read, so a face's centre of mass isn't worked out
        var normals = new List<Direction3D>();
        foreach (var face in faces)
        {
            try
            {
                normals.Add(Plane.NormalOf(face));
            }
            catch (ArgumentException e) when (!Diagnostic.IsKernelFailure(e))
            {
                // A non-planar face
            }
        }

        if (dir is { } direction)
            // Override in the provided direction
            normals = [.. faces.Select(f => new Plane(f.Center(), f.CenterLocation.XAxis.Direction, direction).ZDir)];
        if (normals.Count != faces.Count)
            throw new ArgumentException("A direction is needed to extrude a non-planar face.", nameof(options));

        var newSolids = new List<Shape>();
        for (int i = 0; i < faces.Count; i++)
        {
            var (face, normal) = (faces[i], normals[i]);
            foreach (var sign in both ? new[] { 1, -1 } : [1])
            {
                if (amount is { } a)
                {
                    newSolids.Add(taper == 0
                        ? Solid.Extrude(face, normal * a * sign)
                        : Solid.ExtrudeTaper(face, normal * a * sign, taper));
                }
                else
                {
                    if (target!.Value.IsEmpty)
                        throw new ArgumentException("The target to extrude up to is empty.", nameof(target));
                    newSolids.Add(Solid.ExtrudeUntil(face, target.Value, normal * sign, until!.Value));
                }
            }
        }

        if (both && newSolids.Count > 1)
        {
            // build123d fuses the last solid with all the others
            var last = newSolids[^1];
            newSolids = [last.Fuse(newSolids.Take(newSolids.Count - 1))];
        }
        if (clean)
        {
            // Plain prisms line up with their profiles, and one with nothing to merge needs no clean. Its record is the clean's: empty
            bool prisms = amount is not null && taper == 0 && !both;
            newSolids = [.. newSolids.Select((s, i) => prisms && TopoUtil.NothingToCleanAfterExtruding(faces[i], normals[i])
                ? s.MadeBy(HistoryLog.Current is null ? null : new ModelHistory())
                : s.Clean())];
        }

        return new Part(new ShapeList<Shape>(newSolids).Solids());
    }

    // ---- loft ----

    /// <summary>
    /// A solid lofted through the sections: faces, sketches and vertices, a vertex being allowed only first or last.
    /// </summary>
    /// <remarks>
    /// Faces may have holes, which are lofted and cut away from the outer loft. When the faces have more than one hole, the
    /// holes of each section are matched to those of the one before by the assignment with the smallest total distance
    /// between wire centres. That makes the result independent of the order of the inner wires, but it is only a
    /// geometric heuristic: holes that are close together, cross paths or move a lot may be matched wrongly. Every section
    /// must have the same number of holes.
    /// </remarks>
    /// <param name="sections">The slices to loft through.</param>
    /// <param name="ruled">Join the sections with ruled surfaces.</param>
    /// <param name="clean">Remove extraneous internal structure.</param>
    /// <exception cref="ArgumentException">
    /// There are no sections, a vertex isn't first or last, or the sections have different numbers of holes.
    /// </exception>
    /// <exception cref="InvalidOperationException">The loft is invalid and can't be recovered.</exception>
    public static Result<Part> Loft(IEnumerable<Shape> sections, bool ruled = false, bool clean = true) =>
        Failures.Try(() => LoftOrThrow(sections, ruled, clean));

    internal static Part LoftOrThrow(IEnumerable<Shape> sections, bool ruled, bool clean)
    {
        using var log = HistoryLog.Open();
        // Phase 5: in builder mode the sections default to the pending faces, and the result is combined with mode
        var sectionList = sections?.ToList() ?? [];
        if (sectionList.Count == 0)
            throw new ArgumentException("No sections provided");
        // An empty section is no section; a loft through nothing is nothing
        var inputSections = sectionList.Where(s => !s.IsEmpty).ToList();
        if (inputSections.Count == 0)
            return new Part();

        // Validate Vertex placement
        if (inputSections.Any(s => s.IsVertex()))
        {
            if (!inputSections[0].IsVertex() && !inputSections[^1].IsVertex())
                throw new ArgumentException("Vertices must be the first, last, or first and last elements");
            if (inputSections.Skip(1).SkipLast(1).Any(s => s.IsVertex()))
                throw new ArgumentException("Vertices must be the first, last, or first and last elements");
        }

        // Normalize all input into outer wires and collect the corresponding holes
        var loftSections = new List<Shape>();
        var faces = new List<Face>();
        foreach (var s in inputSections)
        {
            if (s.TryAs<Vertex>(out _))
                loftSections.Add(s);
            else
                foreach (var face in s.Faces())
                {
                    loftSections.Add(face.OuterWire());
                    faces.Add(face);
                }
        }

        var holeCounts = faces.Select(f => f.InnerWires().Count).ToList();
        if (holeCounts.Any(c => c != 0) && holeCounts.Distinct().Count() != 1)
            throw new ArgumentException("All sections must have the same number of inner wires");

        // Match holes between sections by minimizing the distance between their centers. This is needed because the
        // order of inner_wires is not guaranteed to be consistent between faces.
        var holeSections = new List<List<Wire>>();
        if (holeCounts.Count > 0 && holeCounts[0] != 0)
        {
            holeSections = [.. faces[0].InnerWires().Select(w => new List<Wire> { w })];
            foreach (var face in faces.Skip(1))
            {
                var innerWires = face.InnerWires();
                double Distance(int j, int k) => (holeSections[j][^1].Center() - innerWires[k].Center()).Magnitude;
                // The first assignment (in lexicographic order) of the least total distance, as build123d's sort finds
                int[]? best = null;
                double bestQuality = double.PositiveInfinity;
                foreach (var permutation in Permutations(holeSections.Count))
                {
                    double quality = 0;
                    for (int j = 0; j < permutation.Length; j++)
                        quality += Distance(j, permutation[j]);
                    if (quality < bestQuality)
                        (bestQuality, best) = (quality, permutation);
                }
                for (int j = 0; j < best!.Length; j++)
                    holeSections[j].Add(innerWires[best[j]]);
            }
        }

        // Perform lofts
        Shape newSolid = SolidExtensions.MakeLoft(loftSections, ruled);
        if (holeSections.Count > 0)
            newSolid = newSolid.Cut(holeSections.Select(holes => (Shape)SolidExtensions.MakeLoft(holes.Select(w => (Shape)w), ruled)));

        // Try to recover an invalid loft
        if (!newSolid.IsValid)
        {
            Shape unrecovered = newSolid;
            try
            {
                var recoveryFaces = newSolid.Faces().Concat(loftSections.Where(w => w.IsWire()).Select(w => Face.Make((Wire)w)));
                newSolid = Solid.Make(Shell.Make(recoveryFaces));
                if (clean)
                    newSolid = newSolid.Clean();
                if (!newSolid.IsValid)
                    throw new ArgumentException("Recovery failed");
            }
            catch (Exception e) when (e is not OperationCanceledException and not OutOfMemoryException)
            {
                // Kerf: running out of memory isn't a recovery that failed (decision 1943)
                throw Failures.Invalid(unrecovered, unrecovered.Validate(), "The loft's result", "It couldn't be recovered.");
            }
        }

        if (clean)
            newSolid = newSolid.Clean();

        return (Part)new Part([newSolid]).MadeBy(ModelHistory.Of([newSolid]));
    }

    /// <summary>The permutations of 0 … n - 1 in lexicographic order, as Python's <c>itertools.permutations</c>.</summary>
    private static IEnumerable<int[]> Permutations(int n)
    {
        var current = Enumerable.Range(0, n).ToArray();
        while (true)
        {
            yield return (int[])current.Clone();
            // The next permutation in lexicographic order
            int i = n - 2;
            while (i >= 0 && current[i] >= current[i + 1])
                i--;
            if (i < 0)
                yield break;
            int j = n - 1;
            while (current[j] <= current[i])
                j--;
            (current[i], current[j]) = (current[j], current[i]);
            Array.Reverse(current, i + 1, n - i - 1);
        }
    }

    // ---- project_workplane ----

    /// <summary>
    /// A workplane, for a sketch or line, with a known origin and X direction: <paramref name="distance"/> from
    /// <paramref name="origin"/> along <paramref name="projectionDir"/>, normal to it, its origin and X direction being the
    /// projections of <paramref name="origin"/> and <paramref name="xDir"/> onto it.
    /// </summary>
    /// <exception cref="ArgumentException"><paramref name="xDir"/> is perpendicular to <paramref name="projectionDir"/>.</exception>
    public static Plane ProjectWorkplane(Point3D origin, Vector3D xDir, Vector3D projectionDir, double distance)
    {
        // Phase 5: in builder mode this is only valid in a BuildPart
        var x = xDir.Normalized();
        var projection = projectionDir.Normalized();

        // Create a preliminary workplane without x direction set
        var workplaneOrigin = origin + projection * distance;
        var workplane = new Plane(workplaneOrigin, zDir: projection);

        // Project a point off the origin to find the projected x direction
        var screen = Face.MakeRect(1e9, 1e9, workplane);
        var xDirPointAxis = new Axis(origin + x, projection);
        var found = screen.FindIntersectionPoints(xDirPointAxis);
        if (found.Count == 0)
            throw new ArgumentException("The X direction is perpendicular to the projection direction.");

        // Set the workplane's x direction
        return workplane.WithXDir(found[0].Point - workplaneOrigin);
    }

    /// <inheritdoc cref="ProjectWorkplane(Point3D, Vector3D, Vector3D, double)"/>
    public static Plane ProjectWorkplane(Vertex origin, Vertex xDir, Vector3D projectionDir, double distance) =>
        ProjectWorkplane(origin.Point, xDir.Point.ToVector(), projectionDir, distance);

    // ---- revolve ----

    /// <summary>
    /// The faces of the profiles revolved through <paramref name="revolutionArc"/> degrees about an axis (by default
    /// Axis.Z). The axis is usually in the plane of the profile, but needn't be. A whole number of turns is a full one.
    /// </summary>
    /// <exception cref="ArgumentException">No profiles are given, or the arc is zero.</exception>
    public static Result<Part> Revolve(IEnumerable<Shape> profiles, Axis? axis = null, double revolutionArc = 360.0, bool clean = true) =>
        Failures.Try(() => RevolveOrThrow(profiles, axis, revolutionArc, clean));

    internal static Part RevolveOrThrow(IEnumerable<Shape> profiles, Axis? axis, double revolutionArc, bool clean)
    {
        // Phase 5: in builder mode the profiles default to the pending faces, and the result is combined with mode
        var profileList = profiles?.ToList() ?? [];

        // A zero arc makes nothing: a whole number of turns makes one full turn, but it isn't one
        if (revolutionArc == 0)
            throw new ArgumentException("A revolve of no arc makes nothing.", nameof(revolutionArc));
        // Make sure we account for users specifying angles larger than 360 degrees
        double sign = revolutionArc >= 0 ? 1 : -1;
        var angle = Numerics.FloorMod(revolutionArc, sign * 360.0);
        angle = angle == 0 ? sign * 360.0 : angle;

        if (profileList.Count == 0)
            throw new ArgumentException("No profiles provided");
        var profileFaces = new ShapeList<Shape>(profileList).Faces();

        var revolveAxis = axis ?? Axis.Z;
        var newSolids = profileFaces.Select(p => (Shape)Solid.Revolve(p, angle, revolveAxis)).ToList();
        if (clean)
            newSolids = [.. newSolids.Select(s => s.Clean())];

        return new Part(newSolids);
    }

    /// <inheritdoc cref="Revolve(IEnumerable{Shape}, Axis?, double, bool)"/>
    public static Result<Part> Revolve(Shape profile, Axis? axis = null, double revolutionArc = 360.0, bool clean = true) =>
        Revolve([profile], axis, revolutionArc, clean);

    // ---- section ----

    /// <summary>
    /// The cross sections of an object by planes (by default Plane.XZ), each moved <paramref name="height"/> along its
    /// normal.
    /// </summary>
    public static Sketch Section(Shape obj, IEnumerable<Plane> sectionBy, double height = 0.0, bool clean = true)
    {
        // Phase 5: in builder mode the object defaults to the builder's part and the planes to its workplanes

        var bbox = obj.Bounds(optimal: false);
        var maxSize = bbox.IsEmpty ? 0 : new[] { bbox.Min.X, bbox.Min.Y, bbox.Min.Z, bbox.Max.X, bbox.Max.Y, bbox.Max.Z }.Max(Math.Abs) + bbox.Diagonal;

        var planes = sectionBy.Select(plane =>
            Face.MakeRect(2 * maxSize, 2 * maxSize, new Plane(plane.Origin + plane.ZDir * height, zDir: plane.ZDir)));

        var newObjects = new List<Shape>();
        foreach (var plane in planes)
            newObjects.AddRange(obj.Intersect(plane));
        if (clean)
            newObjects = [.. newObjects.Select(r => r.Clean())];

        return new Sketch(newObjects);
    }

    /// <inheritdoc cref="Section(Shape, IEnumerable{Plane}, double, bool)"/>
    public static Sketch Section(Shape obj, Plane? sectionBy = null, double height = 0.0, bool clean = true) =>
        Section(obj, [sectionBy ?? Plane.XZ], height, clean);

    // ---- thicken ----

    /// <summary>
    /// Solids made from the faces of a face, sketch or shell, which may be non-planar, by thickening them along their
    /// normals by <paramref name="amount"/> (its sign giving the direction). <see cref="ThickenOptions.NormalOverride"/>
    /// says which way is up.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="toThicken"/> is null.</exception>
    public static Result<Part> Thicken(Shape toThicken, double amount, ThickenOptions? options = null) =>
        Failures.Try(() => ThickenOrThrow(toThicken, amount, options));

    internal static Part ThickenOrThrow(Shape toThicken, double amount, ThickenOptions? options)
    {
        // Phase 5: in builder mode the faces default to the pending faces, and the result is combined with mode.
        // Phase 7: a sheet Shell with SheetMetalParameters (and BuildSheet's pending sheets) is thickened as sheet material
        return ThickenCore(toThicken.Faces(), amount, options ?? ThickenOptions.Default);
    }

    /// <inheritdoc cref="Thicken(Shape, double, ThickenOptions?)"/>
    public static Result<Part> Thicken(IEnumerable<Face> toThicken, double amount, ThickenOptions? options = null) =>
        Failures.Try(() => ThickenOrThrow(toThicken, amount, options));

    internal static Part ThickenOrThrow(IEnumerable<Face> toThicken, double amount, ThickenOptions? options)
    {
        ArgumentNullException.ThrowIfNull(toThicken);
        return ThickenCore([.. toThicken], amount, options ?? ThickenOptions.Default);
    }

    private static Part ThickenCore(IReadOnlyList<Face> faces, double amount, ThickenOptions options)
    {
        var (normalOverride, both, clean) = (options.NormalOverride, options.Both, options.Clean);
        var newSolids = new List<Shape>();
        foreach (var face in faces)
        {
            var faceNormal = normalOverride ?? face.NormalAt();
            foreach (var sign in both ? new[] { 1, -1 } : [1])
                newSolids.Add(Solid.Thicken(face, amount, faceNormal * sign));
        }
        if (newSolids.Count > 1)
            newSolids = [newSolids[0].Fuse(newSolids.Skip(1))];
        if (clean)
            newSolids = [.. newSolids.Select(s => s.Clean())];
        return new Part(newSolids);
    }
}
