using Kerf.Engine.Foundation;
using Occt;

namespace Build123d;

/// <summary>
/// The operations of build123d (<c>extrude</c>, <c>fillet</c>, <c>mirror</c>, …) as static methods, for algebra mode:
/// bring them in with <c>using static Build123d.Operations;</c> and write <c>Extrude(face, 5)</c>.
/// </summary>
/// <remarks>
/// <para>
/// Every operation returns a new shape and leaves its inputs as they were. Where build123d returns a Part, Sketch or
/// Curve according to the dimension of its inputs, the result is typed <see cref="Compound"/> and is one of those
/// classes at run time.
/// </para>
/// <para>
/// Builder mode (phase 5) adds the builder's pending objects as defaults for the inputs and a <see cref="Mode"/> that
/// combines the result with the builder's object; in algebra mode the inputs are required and nothing is combined.
/// </para>
/// </remarks>
public static partial class Operations
{
    // ---- bounding_box ----

    /// <summary>
    /// The bounding boxes of the objects: as rectangular faces (a Sketch) when every object is 2D, otherwise as boxes (a
    /// Part), vertices being skipped.
    /// </summary>
    /// <exception cref="ArgumentException">No objects are given.</exception>
    public static Compound BoundingBox(IEnumerable<Shape> objects)
    {
        // Phase 5: in builder mode the objects default to the builder's object and the boxes are combined with mode
        if (objects is null)
            throw new ArgumentException("objects must be provided");
        var objectList = objects.ToList();

        if (objectList.Count > 0 && objectList.All(o => o.IsEmpty))
            return (Compound)Shape.MakeComposite([], objectList[0].Dim);

        if (objectList.All(o => o.Dim == 2))
        {
            var newFaces = new List<Shape>();
            foreach (var obj in objectList)
            {
                var bbox = obj.BoundingBox();
                Vector[] vertices =
                [
                    (bbox.Min.X, bbox.Min.Y), (bbox.Min.X, bbox.Max.Y), (bbox.Max.X, bbox.Max.Y), (bbox.Max.X, bbox.Min.Y),
                    (bbox.Min.X, bbox.Min.Y),
                ];
                newFaces.Add(new Face(Wire.MakePolygon(vertices)));
            }
            return new Sketch(newFaces);
        }

        var newObjects = new List<Shape>();
        foreach (var obj in objectList)
        {
            if (obj is Vertex)
                continue;
            var bbox = obj.BoundingBox();
            newObjects.Add(Solid.MakeBox(bbox.Size.X, bbox.Size.Y, bbox.Size.Z, new Plane(bbox.Min)));
        }
        return new Part(newObjects);
    }

    /// <inheritdoc cref="BoundingBox(IEnumerable{Shape})"/>
    public static Compound BoundingBox(Shape obj) => BoundingBox([obj]);

    // ---- chamfer ----

    /// <summary>
    /// The 3D object the edges were taken from, with the edges chamfered: <paramref name="length"/> on one side and
    /// <paramref name="length2"/> (or the length that makes <paramref name="angle"/> degrees) on the other, measured on
    /// <paramref name="reference"/> if given (which the edges must belong to).
    /// </summary>
    /// <exception cref="ArgumentException">
    /// No edges are given, they weren't taken from a 3D object, both <paramref name="length2"/> and <paramref name="angle"/>
    /// are given, or <paramref name="reference"/> is given without either.
    /// </exception>
    public static Part Chamfer(IEnumerable<Edge> edges, double length, double? length2 = null, double? angle = null, Face? reference = null) =>
        (Part)ChamferCore(edges, length, length2, angle, reference);

    /// <inheritdoc cref="Chamfer(IEnumerable{Edge}, double, double?, double?, Face?)"/>
    public static Part Chamfer(Edge edge, double length, double? length2 = null, double? angle = null, Face? reference = null) =>
        Chamfer([edge], length, length2, angle, reference);

    /// <summary>
    /// The 2D object (a Sketch), sheet (a Shell) or 1D object (a Wire) the vertices were taken from, with the corners at
    /// the vertices chamfered: <paramref name="length"/> along one edge and <paramref name="length2"/> (or the length that
    /// makes <paramref name="angle"/> degrees) along the other, <paramref name="length"/> being measured along
    /// <paramref name="reference"/> if given. The vertices at the ends of an open wire are skipped.
    /// </summary>
    /// <exception cref="ArgumentException">
    /// No vertices are given, they weren't taken from a 1D, 2D or sheet object, both <paramref name="length2"/> and
    /// <paramref name="angle"/> are given, or <paramref name="reference"/> is given without either.
    /// </exception>
    public static Shape Chamfer(IEnumerable<Vertex> vertices, double length, double? length2 = null, double? angle = null, Edge? reference = null) =>
        ChamferCore(vertices, length, length2, angle, reference);

    /// <inheritdoc cref="Chamfer(IEnumerable{Vertex}, double, double?, double?, Edge?)"/>
    public static Shape Chamfer(Vertex vertex, double length, double? length2 = null, double? angle = null, Edge? reference = null) =>
        Chamfer([vertex], length, length2, angle, reference);

    private static Shape ChamferCore(IEnumerable<Shape> objects, double length, double? length2, double? angle, Shape? reference)
    {
        // Python's truthiness: a zero length2 or angle counts as not given
        bool hasLength2 = length2 is { } l2 && l2 != 0;
        bool hasAngle = angle is { } a && a != 0;
        if (hasLength2 && hasAngle)
            throw new ArgumentException("Only one of length2 or angle should be provided");
        if (hasAngle)
        {
            length2 = length * Math.Tan(angle!.Value * Constants.DegToRad);
            hasLength2 = length2 != 0;
        }
        if (reference is not null && !(hasLength2 || hasAngle))
            throw new ArgumentException("reference can only be used in conjunction with length2 or angle");
        var distance2 = length2 ?? length;

        // Phase 5: in builder mode the target is the builder's object, and the result replaces it
        if (objects is null)
            throw new ArgumentException("No objects provided");
        var objectList = objects.ToList();
        var target = TargetOf(objectList, "chamfer");

        if (target is Shell)
        {
            var newFaces = EditSheetFaces(target, objectList, "chamfer",
                                          (face, inFace) => face.Chamfer2D(length, distance2, inFace, reference as Edge));
            return Shell.MakeSheet(newFaces);
        }

        switch (target.Dim)
        {
            case 3:
            {
                // A part object becomes a plain Part, so the result can be cast into one
                var solidTarget = target is BasePartObject ? new Part(target.TopoShape) : (Shape3D)target;
                if (!objectList.All(o => o is Edge))
                    throw new ArgumentException("3D chamfer operation takes only Edges");
                var newPart = solidTarget.Chamfer(length, distance2, objectList.Cast<Edge>(), reference as Face);
                return new Part([newPart]).MadeBy(ShapeHistory.Of([newPart]));
            }
            case 2:
            {
                if (!objectList.All(o => o is Vertex))
                    throw new ArgumentException("2D chamfer operation takes only Vertices");
                var newFaces = new List<Face>();
                foreach (var face in target.Faces())
                {
                    var inFace = face.Vertices().Where(objectList.Contains).ToList();
                    newFaces.Add(inFace.Count > 0 ? face.Chamfer2D(length, distance2, inFace, reference as Edge) : face);
                }
                return new Sketch(newFaces).MadeBy(ShapeHistory.Of(newFaces));
            }
            case 1:
            {
                var wire = WireTargetOf(target);
                if (!objectList.All(o => o is Vertex))
                    throw new ArgumentException("1D fillet operation takes only Vertices");
                return wire.Chamfer2D(length, distance2, InnerVertices(wire, objectList), reference as Edge);
            }
            default:
                throw new ArgumentException("Invalid object dimension");
        }
    }

    // ---- fillet ----

    /// <summary>The 3D object the edges were taken from, with the edges filleted.</summary>
    /// <remarks>
    /// 3D fillets propagate along chains of tangent edges (around a rounded slot, or the top of a box whose vertical edges
    /// were rounded earlier), which OpenCascade offers no way to stop: fillet the whole chain in one call, with the radius
    /// that suits it.
    /// </remarks>
    /// <exception cref="ArgumentException">No edges are given, or they weren't taken from a 3D object.</exception>
    public static Part Fillet(IEnumerable<Edge> edges, double radius) => (Part)FilletCore(edges, radius);

    /// <inheritdoc cref="Fillet(IEnumerable{Edge}, double)"/>
    public static Part Fillet(Edge edge, double radius) => Fillet([edge], radius);

    /// <summary>
    /// The 2D object (a Sketch), sheet (a Shell) or 1D object (a Wire) the vertices were taken from, with the corners at
    /// the vertices filleted. The vertices at the ends of an open wire are skipped.
    /// </summary>
    /// <exception cref="ArgumentException">No vertices are given, or they weren't taken from a 1D, 2D or sheet object.</exception>
    public static Shape Fillet(IEnumerable<Vertex> vertices, double radius) => FilletCore(vertices, radius);

    /// <inheritdoc cref="Fillet(IEnumerable{Vertex}, double)"/>
    public static Shape Fillet(Vertex vertex, double radius) => Fillet([vertex], radius);

    private static Shape FilletCore(IEnumerable<Shape> objects, double radius)
    {
        // Phase 5: in builder mode the target is the builder's object, and the result replaces it
        if (objects is null)
            throw new ArgumentException("No objects provided");
        var objectList = objects.ToList();
        var target = TargetOf(objectList, "fillet");

        if (target is Shell)
            return Shell.MakeSheet(EditSheetFaces(target, objectList, "fillet", (face, inFace) => face.Fillet2D(radius, inFace)));

        switch (target.Dim)
        {
            case 3:
            {
                var solidTarget = target is BasePartObject ? new Part(target.TopoShape) : (Shape3D)target;
                if (!objectList.All(o => o is Edge))
                    throw new ArgumentException("3D fillet operation takes only Edges");
                var newPart = solidTarget.Fillet(radius, objectList.Cast<Edge>());
                return new Part([newPart]).MadeBy(ShapeHistory.Of([newPart]));
            }
            case 2:
            {
                if (!objectList.All(o => o is Vertex))
                    throw new ArgumentException("2D fillet operation takes only Vertices");
                var newFaces = new List<Face>();
                foreach (var face in target.Faces())
                {
                    var inFace = face.Vertices().Where(objectList.Contains).ToList();
                    newFaces.Add(inFace.Count > 0 ? face.Fillet2D(radius, inFace) : face);
                }
                return new Sketch(newFaces).MadeBy(ShapeHistory.Of(newFaces));
            }
            case 1:
            {
                var wire = WireTargetOf(target);
                if (!objectList.All(o => o is Vertex))
                    throw new ArgumentException("1D fillet operation takes only Vertices");
                return wire.Fillet2D(radius, InnerVertices(wire, objectList));
            }
            default:
                throw new ArgumentException("Invalid object dimension");
        }
    }

    /// <summary>The shape the first of the objects was taken out of, which a chamfer or fillet rebuilds.</summary>
    private static Shape TargetOf(List<Shape> objectList, string operation)
    {
        var target = objectList.Count > 0 ? objectList[0].TopoParent : null;
        if (target is null || target.IsEmpty)
            throw new ArgumentException($"Nothing to {operation}");
        return target;
    }

    /// <summary>
    /// The faces of a sheet, those with the given vertices on their free boundary edited (chamfered or filleted in 2D).
    /// </summary>
    private static List<Face> EditSheetFaces(Shape target, List<Shape> objectList, string operation, Func<Face, List<Vertex>, Face> edit)
    {
        // build123d's messages name BuildSheet, which handles sheets in builder mode
        if (!objectList.All(o => o is Vertex))
            throw new ArgumentException($"BuildSheet {operation} takes only Vertices");
        var targetFaces = target.Faces();
        foreach (var vertex in objectList)
        {
            var supportingFaces = targetFaces.Where(f => f.Vertices().Any(v => v.Equals(vertex))).ToList();
            if (supportingFaces.Count != 1)
                throw new ArgumentException($"BuildSheet {operation} vertices must be on the free boundary of one face");
            if (supportingFaces[0].GeomType != GeomType.Plane)
                throw new ArgumentException($"BuildSheet can only {operation} planar faces");
        }
        var newFaces = new List<Face>();
        foreach (var face in targetFaces)
        {
            var inFace = face.Vertices().Where(objectList.Contains).ToList();
            newFaces.Add(inFace.Count > 0 ? edit(face, inFace) : face);
        }
        return newFaces;
    }

    /// <summary>The wire a 1D chamfer or fillet rebuilds: a line object as a plain wire, or the first wire of a curve.</summary>
    private static Wire WireTargetOf(Shape target) => target is Wire wire ? new Wire(wire) : target.Wires()[0];

    /// <summary>The vertices without those at the ends of an open wire, which can't be chamfered or filleted.</summary>
    private static List<Vertex> InnerVertices(Wire wire, List<Shape> vertices)
    {
        if (wire.IsClosed)
            return [.. vertices.Cast<Vertex>()];
        var start = wire.PositionAt(0);
        var end = wire.PositionAt(1);
        return [.. vertices.Cast<Vertex>().Where(v =>
            !(TopologyUtils.IsCloseB((v.ToVector() - start).Length, 0.0) || TopologyUtils.IsCloseB((v.ToVector() - end).Length, 0.0)))];
    }

    // ---- mirror ----

    /// <summary>The objects mirrored about a plane (by default Plane.XZ).</summary>
    /// <returns>A Part, Sketch or Curve when the objects are all 3D, 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given.</exception>
    public static Compound Mirror(IEnumerable<Shape> objects, Plane? about = null)
    {
        // Phase 5: in builder mode the objects default to the builder's object and the result is combined with mode;
        // BuildSheet mirrors faces into its sheet
        if (objects is null)
            throw new ArgumentException("objects must be provided");
        var objectList = objects.ToList();
        var plane = about ?? Plane.XZ;
        var mirrored = objectList.Select(o => o.Mirror(plane)).ToList();
        return OfInputDimension(objectList, mirrored);
    }

    /// <inheritdoc cref="Mirror(IEnumerable{Shape}, Plane?)"/>
    public static Compound Mirror(Shape obj, Plane? about = null) => Mirror([obj], about);

    /// <summary>
    /// A compound of the results, as a Part, Sketch or Curve when the inputs are all of that dimension (build123d's
    /// <c>all(obj._dim == n for obj in object_list)</c>, true of no inputs).
    /// </summary>
    private static Compound OfInputDimension(IReadOnlyList<Shape> inputs, IEnumerable<Shape> results)
    {
        var compound = new Compound(results);
        if (inputs.All(o => o.Dim == 3))
            return new Part(compound.TopoShape);
        if (inputs.All(o => o.Dim == 2))
            return new Sketch(compound.TopoShape);
        if (inputs.All(o => o.Dim == 1))
            return new Curve(compound.TopoShape);
        return compound;
    }

    // ---- offset ----

    /// <summary>
    /// The objects offset by <paramref name="amount"/>, outwards when positive: edges and wires (joined into one wire) are
    /// offset in their plane, faces have their outer wire offset out and their holes in, and solids are offset, with the
    /// faces among <paramref name="openings"/> left open (a hollow box with no lid). <paramref name="kind"/> shapes the
    /// corners.
    /// </summary>
    /// <param name="objects">Edges, wires, faces, solids or compounds of them.</param>
    /// <param name="amount">The offset distance: positive outwards, negative inwards.</param>
    /// <param name="openings">Faces of the solids to leave open.</param>
    /// <param name="kind">The transition at corners.</param>
    /// <param name="side">For edges, which side to offset to.</param>
    /// <param name="closed">For edges offset to one side, close the offset back to the edges.</param>
    /// <param name="minEdgeLength">Remove edges of the offset wires shorter than this, which an offset can degenerate into.</param>
    /// <returns>A Part, Sketch or Curve when the objects are all 3D, 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given, or one of them can't be offset.</exception>
    public static Compound Offset(IEnumerable<Shape> objects, double amount = 0, IEnumerable<Face>? openings = null, Kind kind = Kind.Arc,
                                  Side side = Side.Both, bool closed = true, double? minEdgeLength = null)
    {
        // Phase 5: in builder mode the objects default to the builder's object, and the result replaces it by default
        if (objects is null)
            throw new ArgumentException("objects must be provided");
        var objectList = objects.ToList();

        var edges = new List<Edge>();
        var faces = new List<Face>();
        var solids = new List<Solid>();
        foreach (var obj in objectList)
        {
            switch (obj)
            {
                case Compound compound:
                    edges.AddRange(compound.ElementsOfType<Edge>());
                    edges.AddRange(new ShapeList<Wire>(compound.ElementsOfType<Wire>()).Edges());
                    faces.AddRange(compound.ElementsOfType<Face>());
                    solids.AddRange(compound.ElementsOfType<Solid>());
                    break;
                case Solid solid:
                    solids.Add(solid);
                    break;
                case Face face:
                    faces.Add(face);
                    break;
                case Edge edge:
                    edges.Add(edge);
                    break;
                case Wire wire:
                    edges.AddRange(wire.Edges());
                    break;
                default:
                    throw new ArgumentException($"Unsupported type {obj.GetType().Name} for {obj}");
            }
        }

        var newFaces = new List<Shape>();
        foreach (var face in faces)
        {
            var outerWire = AsWire(face.OuterWire().Offset2D(amount, kind));
            if (minEdgeLength is { } outerMin)
                outerWire = outerWire.FixDegenerateEdges(outerMin);
            var innerWires = new List<Wire>();
            foreach (var innerWire in face.InnerWires())
            {
                try
                {
                    var offsetWire = AsWire(innerWire.Offset2D(-amount, kind));
                    innerWires.Add(minEdgeLength is { } innerMin ? offsetWire.FixDegenerateEdges(innerMin) : offsetWire);
                }
                catch (Exception e) when (e is InvalidOperationException or FailureException)
                {
                    // The hole closed up
                }
            }

            // Inner wires may go beyond the outer wire, so they're cut away as faces
            var newFace = new Face(outerWire);
            if ((newFace.NormalAt() - face.NormalAt()).Length > 0.001)
                newFace = (Face)(-newFace);
            if (innerWires.Count > 0)
            {
                var subtraction = newFace.Cut(innerWires.Select(w => new Face(w)));
                // build123d would fail here on a cut into several pieces (it asks the face, not the result, to unwrap
                // itself); the pieces are kept instead
                if (subtraction is Compound)
                    newFaces.AddRange(subtraction.Faces());
                else
                    newFaces.Add(subtraction);
            }
            else
                newFaces.Add(newFace);
        }

        var newWires = new List<Shape1D>();
        if (edges.Count > 0)
        {
            // The line itself is offset, not a copy made from its ends: its location says which plane it was drawn on
            var offsetLine = edges.Count == 1 && edges[0].GeomType == GeomType.Line
                ? edges[0].Offset2D(amount, kind, side, closed)
                : new Wire(edges).Offset2D(amount, kind, side, closed);
            newWires.Add(minEdgeLength is { } min ? AsWire(offsetLine).FixDegenerateEdges(min) : offsetLine);
        }

        var openingList = openings?.ToList() ?? [];
        var newSolids = new List<Shape>();
        foreach (var solid in solids)
        {
            var solidFaces = solid.Faces();
            var openingsInThisSolid = openingList.Where(solidFaces.Contains).ToList();
            newSolids.Add(solid.Offset3D(openingsInThisSolid, amount, kind: kind).Fix());
        }

        return OfInputDimension(objectList, [.. newWires, .. newFaces, .. newSolids]);
    }

    /// <inheritdoc cref="Offset(IEnumerable{Shape}, double, IEnumerable{Face}?, Kind, Side, bool, double?)"/>
    public static Compound Offset(Shape obj, double amount = 0, IEnumerable<Face>? openings = null, Kind kind = Kind.Arc,
                                  Side side = Side.Both, bool closed = true, double? minEdgeLength = null) =>
        Offset([obj], amount, openings, kind, side, closed, minEdgeLength);

    /// <summary>A 2D offset as a wire: a single-edge offset comes back as its edge.</summary>
    private static Wire AsWire(Shape1D offset) => offset as Wire ?? new Wire((Edge)offset);

    // ---- project ----

    /// <summary>
    /// Edges, wires and faces projected onto a workplane, along its normal, in the workplane's local coordinates. As in
    /// build123d's algebra mode, the faces of the projection are kept, so edges and wires project to an empty Curve.
    /// Points are projected with <see cref="ProjectPoints(IEnumerable{Vector}, Plane)"/>.
    /// </summary>
    /// <returns>A Sketch or Curve when the objects are all 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given, or one of them is a vertex.</exception>
    public static Compound Project(IEnumerable<Shape> objects, Plane workplane)
    {
        // Phase 5: in builder mode the workplane defaults to the builder's, BuildLine and BuildSketch keep the projection
        // in their local coordinates, and BuildPart projects its pending objects onto its part (a target)
        if (objects is null)
            throw new ArgumentException("No object to project");
        var objectList = objects.ToList();
        if (objectList.OfType<Vertex>().Any())
            throw new ArgumentException("Vertices are projected with ProjectPoints");
        if (objectList.Count > 0 && objectList.All(o => o.IsEmpty))
            // Nothing projects to nothing, of the same dimension
            return (Compound)Shape.MakeComposite([], objectList[0].Dim);

        // The size of the object determines the size of the target projection screen, as the screen is normal to the
        // direction of parallel projection
        var objectSize = new Compound(objectList).BoundingBox(optimal: false).Diagonal;
        var target = Face.MakeRect(3 * objectSize, 3 * objectSize, workplane);
        if (target.IsEmpty)
            throw new ArgumentException("A target object could not be determined");

        var projectedShapes = new List<Shape>();
        foreach (var obj in objectList.OfType<Face>().Concat<Shape>(objectList.Where(o => o is Edge or Wire)))
        {
            var objToScreen = (target.Center() - obj.Center()).Normalized();
            var direction = workplane.FromLocalCoords(objToScreen).Z < 0 ? -workplane.ZDir : workplane.ZDir;
            ShapeList<Face> projection = obj switch
            {
                Face face => face.ProjectToShape(target, direction).Faces(),
                Edge edge => edge.ProjectToShape(target, direction).Faces(),
                _ => ((Wire)obj).ProjectToShape(target, direction).Faces(),
            };
            projectedShapes.AddRange(projection);
        }

        var result = new Compound(projectedShapes);
        if (objectList.All(o => o.Dim == 2))
            return new Sketch(result.TopoShape);
        if (objectList.All(o => o.Dim == 1))
            return new Curve(result.TopoShape);
        return result;
    }

    /// <inheritdoc cref="Project(IEnumerable{Shape}, Plane)"/>
    public static Compound Project(Shape obj, Plane workplane) => Project([obj], workplane);

    /// <summary>
    /// Points projected onto a workplane along its normal, in the workplane's local coordinates (build123d's
    /// <c>project</c> of points, which returns a list of vectors).
    /// </summary>
    /// <exception cref="InvalidOperationException">The workplane is parallel to a projection axis.</exception>
    public static IReadOnlyList<Vector> ProjectPoints(IEnumerable<Vector> points, Plane workplane)
    {
        var projectedPoints = new List<Vector>();
        foreach (var pnt in points)
        {
            var pntToTarget = (workplane.Origin - pnt).Normalized();
            var projectionAxis = workplane.FromLocalCoords(pntToTarget).Z < 0
                ? -new Axis(pnt, workplane.ZDir)
                : new Axis(pnt, workplane.ZDir);
            var intersection = workplane.Intersect(projectionAxis);
            if (intersection is Axis)
                throw new InvalidOperationException("working_plane and projection_axis are parallel");
            if (intersection is Vector v)
                projectedPoints.Add(workplane.ToLocalCoords(v));
        }
        return projectedPoints;
    }

    /// <inheritdoc cref="ProjectPoints(IEnumerable{Vector}, Plane)"/>
    public static IReadOnlyList<Vector> ProjectPoints(IEnumerable<Vertex> points, Plane workplane) =>
        ProjectPoints(points.Select(p => p.ToVector()), workplane);

    /// <inheritdoc cref="ProjectPoints(IEnumerable{Vector}, Plane)"/>
    public static IReadOnlyList<Vector> ProjectPoints(Vector point, Plane workplane) => ProjectPoints([point], workplane);

    /// <inheritdoc cref="ProjectPoints(IEnumerable{Vector}, Plane)"/>
    public static IReadOnlyList<Vector> ProjectPoints(Vertex point, Plane workplane) => ProjectPoints([point.ToVector()], workplane);

    // ---- scale ----

    /// <summary>
    /// The objects scaled by a factor, about <paramref name="about"/> or, by default, each object's location. A compound
    /// of one object is unwrapped one level (build123d's <c>unwrap(fully=False)</c>).
    /// </summary>
    /// <returns>A Part, Sketch or Curve when the objects are all 3D, 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given.</exception>
    public static Compound Scale(IEnumerable<Shape> objects, double by = 1, Vector? about = null) =>
        ScaleCore(objects, o => o.Scale(by, about));

    /// <summary>
    /// The objects scaled by a factor along each axis, about <paramref name="about"/> or, by default, each object's
    /// location. Scaling unevenly may change the type of the geometry (a line or circle to a B-spline).
    /// </summary>
    /// <inheritdoc cref="Scale(IEnumerable{Shape}, double, Vector?)"/>
    public static Compound Scale(IEnumerable<Shape> objects, (double X, double Y, double Z) by, Vector? about = null) =>
        ScaleCore(objects, o => o.Scale(by, about));

    /// <inheritdoc cref="Scale(IEnumerable{Shape}, double, Vector?)"/>
    public static Compound Scale(Shape obj, double by = 1, Vector? about = null) => Scale([obj], by, about);

    /// <inheritdoc cref="Scale(IEnumerable{Shape}, ValueTuple{double, double, double}, Vector?)"/>
    public static Compound Scale(Shape obj, (double X, double Y, double Z) by, Vector? about = null) => Scale([obj], by, about);

    private static Compound ScaleCore(IEnumerable<Shape> objects, Func<Shape, Shape> scale)
    {
        // Phase 5: in builder mode the objects default to the builder's object, and the result replaces it by default
        if (objects is null)
            throw new ArgumentException("objects must be provided");
        var objectList = objects.ToList();
        return (Compound)OfInputDimension(objectList, objectList.Select(scale)).Unwrap(fully: false);
    }

    // ---- split ----

    /// <summary>
    /// The objects split by a plane (by default Plane.XZ), keeping the pieces on top of it, beneath it, or both.
    /// </summary>
    /// <returns>A Part, Sketch or Curve when the objects are all 3D, 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given.</exception>
    public static Compound Split(IEnumerable<Shape> objects, Plane? bisectBy = null, Keep keep = Keep.Top)
    {
        var plane = bisectBy ?? Plane.XZ;
        return SplitCore(objects, keep, o => o.Split(plane, keep), o => o.SplitBoth(plane));
    }

    /// <summary>The objects split by a face or shell, keeping the pieces on top of it, beneath it, or both.</summary>
    /// <inheritdoc cref="Split(IEnumerable{Shape}, Plane?, Keep)"/>
    public static Compound Split(IEnumerable<Shape> objects, Shape2D bisectBy, Keep keep = Keep.Top) =>
        SplitCore(objects, keep, o => o.Split(bisectBy, keep), o => o.SplitBoth(bisectBy));

    /// <inheritdoc cref="Split(IEnumerable{Shape}, Plane?, Keep)"/>
    public static Compound Split(Shape obj, Plane? bisectBy = null, Keep keep = Keep.Top) => Split([obj], bisectBy, keep);

    /// <inheritdoc cref="Split(IEnumerable{Shape}, Shape2D, Keep)"/>
    public static Compound Split(Shape obj, Shape2D bisectBy, Keep keep = Keep.Top) => Split([obj], bisectBy, keep);

    private static Compound SplitCore(IEnumerable<Shape> objects, Keep keep, Func<Shape, ShapeList<Shape>> split,
                                      Func<Shape, (ShapeList<Shape> Top, ShapeList<Shape> Bottom)> splitBoth)
    {
        // Phase 5: in builder mode the objects default to the builder's object and the result replaces it by default;
        // BuildSheet splits faces of its sheet
        if (objects is null)
            throw new ArgumentException("objects must be provided");
        var objectList = objects.ToList();
        var newObjects = new List<Shape>();
        foreach (var obj in objectList)
        {
            // A side with nothing on it adds nothing
            if (keep == Keep.Both)
            {
                var (top, bottom) = splitBoth(obj);
                newObjects.AddRange(top);
                newObjects.AddRange(bottom);
            }
            else
                newObjects.AddRange(split(obj));
        }
        return OfInputDimension(objectList, newObjects);
    }

    // ---- sweep ----

    /// <summary>
    /// Sections swept along a path. Faces (and the faces of sketches and solids) sweep into solids, returned as a Part;
    /// edges and wires sweep into faces, returned in a Sketch that holds each section's faces as a shell. With
    /// <paramref name="multisection"/> the faces' outer wires are the sections of one solid along the path.
    /// </summary>
    /// <param name="sections">The cross sections.</param>
    /// <param name="path">The path to follow: an edge, wire or curve.</param>
    /// <param name="multisection">Sweep one solid through all the sections.</param>
    /// <param name="isFrenet">Use the Frenet frame of the path.</param>
    /// <param name="transition">How a section turns a corner of the path.</param>
    /// <param name="normal">A fixed normal for the sections.</param>
    /// <param name="binormal">An edge or wire guiding the rotation of the sections along the path.</param>
    /// <param name="clean">Remove extraneous internal structure.</param>
    /// <returns>A Part for face sections, otherwise a Sketch.</returns>
    /// <exception cref="ArgumentException">The path is empty, or there are no sections.</exception>
    public static Compound Sweep(IEnumerable<Shape> sections, Shape path, bool multisection = false, bool isFrenet = false,
                                 Transition transition = Transition.Transformed, Vector? normal = null, Shape1D? binormal = null,
                                 bool clean = true)
    {
        // Phase 5: in builder mode the path defaults to the pending edges and the sections to the pending faces, and the
        // result is combined with mode
        if (path is null)
            throw new ArgumentException("path must be provided");
        if (path.IsEmpty)
            throw new ArgumentException("Cannot sweep along an empty path");
        var pathWire = path as Wire ?? new Wire(path.Edges());
        return SweepCore(sections, pathWire, multisection, isFrenet, transition, normal, binormal, clean);
    }

    /// <summary>Sections swept along a path of edges, which must join into a wire.</summary>
    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, Shape, bool, bool, Transition, Vector?, Shape1D?, bool)"/>
    /// <exception cref="ArgumentException">The edges don't join into a wire, or there are no sections.</exception>
    public static Compound Sweep(IEnumerable<Shape> sections, IEnumerable<Edge> path, bool multisection = false, bool isFrenet = false,
                                 Transition transition = Transition.Transformed, Vector? normal = null, Shape1D? binormal = null,
                                 bool clean = true)
    {
        Wire pathWire;
        try
        {
            pathWire = new Wire(path);
        }
        catch (ArgumentException e)
        {
            throw new ArgumentException("Unable to build path from edges", e);
        }
        return SweepCore(sections, pathWire, multisection, isFrenet, transition, normal, binormal, clean);
    }

    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, Shape, bool, bool, Transition, Vector?, Shape1D?, bool)"/>
    public static Compound Sweep(Shape section, Shape path, bool multisection = false, bool isFrenet = false,
                                 Transition transition = Transition.Transformed, Vector? normal = null, Shape1D? binormal = null,
                                 bool clean = true) =>
        Sweep([section], path, multisection, isFrenet, transition, normal, binormal, clean);

    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, IEnumerable{Edge}, bool, bool, Transition, Vector?, Shape1D?, bool)"/>
    public static Compound Sweep(Shape section, IEnumerable<Edge> path, bool multisection = false, bool isFrenet = false,
                                 Transition transition = Transition.Transformed, Vector? normal = null, Shape1D? binormal = null,
                                 bool clean = true) =>
        Sweep([section], path, multisection, isFrenet, transition, normal, binormal, clean);

    private static Compound SweepCore(IEnumerable<Shape> sections, Wire pathWire, bool multisection, bool isFrenet,
                                      Transition transition, Vector? normal, Shape1D? binormal, bool clean)
    {
        var sectionList = sections?.Where(s => s is not null).ToList() ?? [];
        if (sectionList.Count > 0 && sectionList.All(s => s.IsEmpty))
            // Nothing swept along anything is nothing, one dimension up
            return (Compound)Shape.MakeComposite([], (sectionList[0].Dim ?? 0) + 1);
        if (sectionList.Count == 0)
            throw new ArgumentException("No sections provided");

        var edgeList = new List<Edge>();
        var faceList = new List<Face>();
        foreach (var sec in sectionList)
        {
            if (sec is Curve or Wire or Edge)
                edgeList.AddRange(sec.Edges());
            else
                faceList.AddRange(sec.Faces());
        }

        // Sweep to create solids
        var newSolids = new List<Shape>();
        if (faceList.Count > 0)
        {
            object? binormalMode = binormal switch
            {
                null when normal is { } n => n,
                Edge e => new Wire(e),
                _ => binormal,
            };
            if (multisection)
                newSolids.Add(Solid.SweepMulti(faceList.Select(f => f.OuterWire()), pathWire, true, isFrenet, binormalMode));
            else
                newSolids.AddRange(faceList.Select(face => Solid.Sweep(face, pathWire, makeSolid: true, isFrenet: isFrenet,
                                                                       mode: binormalMode, transition: transition)));
        }

        // Sweep to create faces, kept together in the shell each section sweeps out
        var newShells = new List<Shape>();
        if (edgeList.Count > 0)
            foreach (var sec in sectionList)
                newShells.Add(Shell.Sweep(sec, pathWire, transition));

        if (clean)
        {
            newSolids = [.. newSolids.Select(s => s.Clean())];
            newShells = [.. newShells.Select(s => s.Clean())];
        }

        if (newSolids.Count > 0)
            return new Part(newSolids);
        return new Sketch(newShells);
    }
}
