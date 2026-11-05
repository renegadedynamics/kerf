
namespace Kerf.Modeling;

/// <summary>
/// The operations of build123d (<c>extrude</c>, <c>fillet</c>, <c>mirror</c>, …) as static methods, for algebra mode:
/// bring them in with <c>using static Kerf.Modeling.Operations;</c> and write <c>Extrude(face, 5)</c>.
/// </summary>
/// <remarks>
/// <para>
/// Every operation returns a new shape and leaves its inputs as they were. Where build123d returns a Part, Sketch or
/// Curve according to the dimension of its inputs, the result is typed <see cref="Compound"/> and is one of those
/// classes at run time.
/// </para>
/// <para>
/// Builder mode (phase 5) adds the builder's pending objects as defaults for the inputs and a mode that
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
    public static ModelShape BoundingBox(IEnumerable<Shape> objects)
    {
        // Phase 5: in builder mode the objects default to the builder's object and the boxes are combined with mode
        if (objects is null)
            throw new ArgumentNullException(nameof(objects));
        var objectList = objects.ToList();

        if (objectList.Count > 0 && objectList.All(o => o.IsEmpty))
            return NothingOf(objectList[0].Dim);

        if (objectList.All(o => o.Dim == 2))
        {
            var newFaces = new List<Shape>();
            foreach (var obj in objectList)
            {
                var bbox = obj.Bounds();
                Point3D[] vertices =
                [
                    new Point3D(bbox.Min.X, bbox.Min.Y, 0), new Point3D(bbox.Min.X, bbox.Max.Y, 0), new Point3D(bbox.Max.X, bbox.Max.Y, 0), new Point3D(bbox.Max.X, bbox.Min.Y, 0),
                    new Point3D(bbox.Min.X, bbox.Min.Y, 0),
                ];
                newFaces.Add(Face.Make(Wire.MakePolygon(vertices)));
            }
            return new Sketch(newFaces);
        }

        var newObjects = new List<Shape>();
        foreach (var obj in objectList)
        {
            if (obj.TryAs<Vertex>(out _))
                continue;
            var bbox = obj.Bounds();
            newObjects.Add(Solid.MakeBox(bbox.Size.X, bbox.Size.Y, bbox.Size.Z, new Plane(bbox.Min)));
        }
        return new Part(newObjects);
    }

    /// <inheritdoc cref="BoundingBox(IEnumerable{Shape})"/>
    public static ModelShape BoundingBox(Shape obj) => BoundingBox([obj]);

    /// <inheritdoc cref="BoundingBox(Shape)"/>
    public static ModelShape BoundingBox(Part obj) => obj.IsEmpty ? new Part() : BoundingBox([obj.Shape]);

    /// <inheritdoc cref="BoundingBox(Shape)"/>
    public static ModelShape BoundingBox(Sketch obj) => obj.IsEmpty ? new Sketch() : BoundingBox([obj.Shape]);

    /// <inheritdoc cref="BoundingBox(IEnumerable{Shape})"/>
    public static ModelShape BoundingBox<T>(IEnumerable<T> objects) where T : IShape => BoundingBox(Shapes(objects));

    /// <inheritdoc cref="Mirror(IEnumerable{Shape}, Plane?)"/>
    public static ModelShape Mirror<T>(IEnumerable<T> objects, Plane? about = null) where T : IShape => Mirror(Shapes(objects), about);

    /// <inheritdoc cref="Offset(IEnumerable{Shape}, double, OffsetOptions?)"/>
    public static Result<ModelShape> Offset<T>(IEnumerable<T> objects, double amount, OffsetOptions? options = null) where T : IShape =>
        Offset(Shapes(objects), amount, options);

    /// <inheritdoc cref="Project(IEnumerable{Shape}, Plane)"/>
    public static ModelShape Project<T>(IEnumerable<T> objects, Plane workplane) where T : IShape => Project(Shapes(objects), workplane);

    /// <inheritdoc cref="Scale(IEnumerable{Shape}, double, Point3D?)"/>
    public static ModelShape Scale<T>(IEnumerable<T> objects, double by = 1, Point3D? about = null) where T : IShape => Scale(Shapes(objects), by, about);

    /// <inheritdoc cref="Scale(IEnumerable{Shape}, Vector3D, Point3D?)"/>
    public static ModelShape Scale<T>(IEnumerable<T> objects, Vector3D by, Point3D? about = null) where T : IShape => Scale(Shapes(objects), by, about);

    /// <inheritdoc cref="Split(IEnumerable{Shape}, Plane?, Keep)"/>
    public static ModelShape Split<T>(IEnumerable<T> objects, Plane? bisectBy = null, Keep keep = Keep.Top) where T : IShape => Split(Shapes(objects), bisectBy, keep);

    /// <inheritdoc cref="Split(IEnumerable{Shape}, IShape2D, Keep)"/>
    public static ModelShape Split<T>(IEnumerable<T> objects, IShape2D bisectBy, Keep keep = Keep.Top) where T : IShape => Split(Shapes(objects), bisectBy, keep);

    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, Shape, SweepOptions?)"/>
    public static Result<ModelShape> Sweep<T>(IEnumerable<T> sections, Shape path, SweepOptions? options = null) where T : IShape =>
        Sweep(Shapes(sections), path, options);

    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, IEnumerable{Edge}, SweepOptions?)"/>
    public static Result<ModelShape> Sweep<T>(IEnumerable<T> sections, IEnumerable<Edge> path, SweepOptions? options = null) where T : IShape =>
        Sweep(Shapes(sections), path, options);

    /// <inheritdoc cref="Loft(IEnumerable{Shape}, bool, bool)"/>
    public static Result<Part> Loft<T>(IEnumerable<T> sections, bool ruled = false, bool clean = true) where T : IShape => Loft(Shapes(sections), ruled, clean);

    /// <inheritdoc cref="Revolve(IEnumerable{Shape}, Axis?, double, bool)"/>
    public static Result<Part> Revolve<T>(IEnumerable<T> profiles, Axis? axis = null, double revolutionArc = 360.0, bool clean = true) where T : IShape =>
        Revolve(Shapes(profiles), axis, revolutionArc, clean);

    /// <inheritdoc cref="MakeFace(IEnumerable{Shape})"/>
    public static Sketch MakeFace<T>(IEnumerable<T> edges) where T : IShape => MakeFace(Shapes(edges));

    /// <inheritdoc cref="Trace(IEnumerable{Shape}, double)"/>
    public static Sketch Trace<T>(IEnumerable<T> lines, double lineWidth = 1) where T : IShape => Trace(Shapes(lines), lineWidth);

    private static IEnumerable<Shape> Shapes<T>(IEnumerable<T> shapes) where T : IShape
    {
        ArgumentNullException.ThrowIfNull(shapes);
        return shapes.Select(s => s is null ? throw new ArgumentNullException(nameof(shapes), "A shape is null.") : s.AsShape());
    }

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
    public static Result<Part> Chamfer(IEnumerable<Edge> edges, double length, double? length2 = null, double? angle = null, Face? reference = null) =>
        Failures.Try(() => (Part)ChamferCore(TargetOf(edges, "chamfer"), edges.Select(e => (Shape)e), length, length2, angle, reference));

    /// <summary>The edges of <paramref name="target"/> chamfered (see <see cref="Chamfer(IEnumerable{Edge}, double, double?, double?, Face?)"/>).</summary>
    public static Result<Part> Chamfer(Shape target, IEnumerable<Edge> edges, double length, double? length2 = null, double? angle = null, Face? reference = null) =>
        Failures.Try(() => (Part)ChamferCore(target, edges.Select(e => (Shape)e), length, length2, angle, reference));

    /// <inheritdoc cref="Chamfer(IEnumerable{Edge}, double, double?, double?, Face?)"/>
    public static Result<Part> Chamfer(Shape target, Edge edge, double length, double? length2 = null, double? angle = null, Face? reference = null) =>
        Chamfer(target, [edge], length, length2, angle, reference);

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
    public static Result<ModelShape> Chamfer(IEnumerable<Vertex> vertices, double length, double? length2 = null, double? angle = null, Edge? reference = null) =>
        Failures.Try(() => ChamferCore(TargetOf(vertices, "chamfer"), vertices.Select(v => (Shape)v), length, length2, angle, reference));

    /// <summary>The vertices of <paramref name="target"/> chamfered (see <see cref="Chamfer(IEnumerable{Vertex}, double, double?, double?, Edge?)"/>).</summary>
    public static Result<ModelShape> Chamfer(Shape target, IEnumerable<Vertex> vertices, double length, double? length2 = null, double? angle = null, Edge? reference = null) =>
        Failures.Try(() => ChamferCore(target, vertices.Select(v => (Shape)v), length, length2, angle, reference));

    /// <inheritdoc cref="Chamfer(IEnumerable{Vertex}, double, double?, double?, Edge?)"/>
    public static Result<ModelShape> Chamfer(Shape target, Vertex vertex, double length, double? length2 = null, double? angle = null, Edge? reference = null) =>
        Chamfer(target, [vertex], length, length2, angle, reference);

    private static ModelShape ChamferCore(Shape target, IEnumerable<Shape> objects, double length, double? length2, double? angle, Shape? reference)
    {
        using var log = HistoryLog.Open();
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
        if (reference.HasValue && !(hasLength2 || hasAngle))
            throw new ArgumentException("reference can only be used in conjunction with length2 or angle");
        var distance2 = length2 ?? length;

        // Phase 5: in builder mode the target is the builder's object, and the result replaces it
        if (objects is null)
            throw new ArgumentException("No objects provided");
        var objectList = objects.ToList();
        Edge? referenceEdge = reference is { IsNull: false } re && re.IsEdge() ? (Edge)re : null;
        Face? referenceFace = reference is { IsNull: false } rf && rf.IsFace() ? (Face)rf : null;

        if (target.IsShell())
        {
            var newFaces = EditSheetFaces(target, objectList, "chamfer",
                                          (face, inFace) => face.Chamfer2D(length, distance2, inFace, referenceEdge));
            var sheet = Shell.MakeSheet(newFaces);
            return new Sketch((Shape)sheet).MadeBy(ModelHistory.Of([sheet]));
        }

        switch (target.Dim)
        {
            case 3:
            {
                if (!objectList.All(o => o.IsEdge()))
                    throw new ArgumentException("3D chamfer operation takes only Edges");
                var newPart = TopoUtil.As3D(target).Chamfer(length, distance2, objectList.Select(o => (Edge)o), referenceFace);
                return new Part([newPart]).MadeBy(ModelHistory.Of([newPart]));
            }
            case 2:
            {
                if (!objectList.All(o => o.IsVertex()))
                    throw new ArgumentException("2D chamfer operation takes only Vertices");
                var newFaces = new List<Face>();
                foreach (var face in target.Faces())
                {
                    var inFace = face.Vertices().Where(v => objectList.Contains(v, ShapeComparer.Same)).ToList();
                    newFaces.Add(inFace.Count > 0 ? face.Chamfer2D(length, distance2, inFace, referenceEdge) : face);
                }
                return new Sketch(newFaces).MadeBy(ModelHistory.Of(newFaces));
            }
            case 1:
            {
                var wire = WireTargetOf(target);
                if (!objectList.All(o => o.IsVertex()))
                    throw new ArgumentException("1D fillet operation takes only Vertices");
                var made = wire.Chamfer2D(length, distance2, InnerVertices(wire, objectList), referenceEdge);
                return new Curve((Shape)made).MadeBy(ModelHistory.Of([made]));
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
    public static Result<Part> Fillet(IEnumerable<Edge> edges, double radius) =>
        Failures.Try(() => (Part)FilletCore(TargetOf(edges, "fillet"), edges.Select(e => (Shape)e), radius));

    /// <summary>The edges of <paramref name="target"/> filleted (see <see cref="Fillet(IEnumerable{Edge}, double)"/>).</summary>
    public static Result<Part> Fillet(Shape target, IEnumerable<Edge> edges, double radius) =>
        Failures.Try(() => (Part)FilletCore(target, edges.Select(e => (Shape)e), radius));

    /// <inheritdoc cref="Fillet(IEnumerable{Edge}, double)"/>
    public static Result<Part> Fillet(Shape target, Edge edge, double radius) => Fillet(target, [edge], radius);

    /// <summary>
    /// The 2D object (a Sketch), sheet (a Shell) or 1D object (a Wire) the vertices were taken from, with the corners at
    /// the vertices filleted. The vertices at the ends of an open wire are skipped.
    /// </summary>
    /// <exception cref="ArgumentException">No vertices are given, or they weren't taken from a 1D, 2D or sheet object.</exception>
    public static Result<ModelShape> Fillet(IEnumerable<Vertex> vertices, double radius) =>
        Failures.Try(() => FilletCore(TargetOf(vertices, "fillet"), vertices.Select(v => (Shape)v), radius));

    /// <summary>The vertices of <paramref name="target"/> filleted (see <see cref="Fillet(IEnumerable{Vertex}, double)"/>).</summary>
    public static Result<ModelShape> Fillet(Shape target, IEnumerable<Vertex> vertices, double radius) =>
        Failures.Try(() => FilletCore(target, vertices.Select(v => (Shape)v), radius));

    /// <inheritdoc cref="Fillet(IEnumerable{Vertex}, double)"/>
    public static Result<ModelShape> Fillet(Shape target, Vertex vertex, double radius) => Fillet(target, [vertex], radius);

    private static ModelShape FilletCore(Shape target, IEnumerable<Shape> objects, double radius)
    {
        using var log = HistoryLog.Open();
        // Phase 5: in builder mode the target is the builder's object, and the result replaces it
        if (objects is null)
            throw new ArgumentException("No objects provided");
        var objectList = objects.ToList();

        if (target.IsShell())
        {
            var sheet = Shell.MakeSheet(EditSheetFaces(target, objectList, "fillet", (face, inFace) => face.Fillet2D(radius, inFace)));
            return new Sketch((Shape)sheet).MadeBy(ModelHistory.Of([sheet]));
        }

        switch (target.Dim)
        {
            case 3:
            {
                if (!objectList.All(o => o.IsEdge()))
                    throw new ArgumentException("3D fillet operation takes only Edges");
                var newPart = TopoUtil.As3D(target).Fillet(radius, objectList.Select(o => (Edge)o));
                return new Part([newPart]).MadeBy(ModelHistory.Of([newPart]));
            }
            case 2:
            {
                if (!objectList.All(o => o.IsVertex()))
                    throw new ArgumentException("2D fillet operation takes only Vertices");
                var newFaces = new List<Face>();
                foreach (var face in target.Faces())
                {
                    var inFace = face.Vertices().Where(v => objectList.Contains(v, ShapeComparer.Same)).ToList();
                    newFaces.Add(inFace.Count > 0 ? face.Fillet2D(radius, inFace) : face);
                }
                return new Sketch(newFaces).MadeBy(ModelHistory.Of(newFaces));
            }
            case 1:
            {
                var wire = WireTargetOf(target);
                if (!objectList.All(o => o.IsVertex()))
                    throw new ArgumentException("1D fillet operation takes only Vertices");
                var made = wire.Fillet2D(radius, InnerVertices(wire, objectList));
                return new Curve((Shape)made).MadeBy(ModelHistory.Of([made]));
            }
            default:
                throw new ArgumentException("Invalid object dimension");
        }
    }

    /// <summary>The shape the first of the objects was taken out of, which a chamfer or fillet rebuilds.</summary>
    /// <summary>The shape a selection was taken out of, which a chamfer or fillet rebuilds.</summary>
    private static Shape TargetOf<T>(IEnumerable<T> objects, string operation) where T : IShape
    {
        if (objects is null)
            throw new ArgumentException("No objects provided");
        if (objects is ShapeList<T> { Origin: { IsEmpty: false } origin })
            return origin.AsShape();
        throw new ArgumentException($"Nothing to {operation}: select from a shape, or give the shape as the first argument");
    }

    /// <summary>
    /// The faces of a sheet, those with the given vertices on their free boundary edited (chamfered or filleted in 2D).
    /// </summary>
    private static List<Face> EditSheetFaces(Shape target, List<Shape> objectList, string operation, Func<Face, List<Vertex>, Face> edit)
    {
        // build123d's messages name BuildSheet, which handles sheets in builder mode
        if (!objectList.All(o => o.IsVertex()))
            throw new ArgumentException($"A sheet's {operation} takes only vertices.");
        var targetFaces = target.Faces();
        foreach (var vertex in objectList)
        {
            var supportingFaces = targetFaces.Where(f => f.Vertices().Any(v => v.SameAs(vertex))).ToList();
            if (supportingFaces.Count != 1)
                throw new ArgumentException($"A sheet's {operation} vertices must be on the free boundary of one face.");
            if (supportingFaces[0].GeomType != GeomType.Plane)
                throw new ArgumentException($"Only a sheet's planar faces can take a {operation}.");
        }
        var newFaces = new List<Face>();
        foreach (var face in targetFaces)
        {
            var inFace = face.Vertices().Where(v => objectList.Contains(v, ShapeComparer.Same)).ToList();
            newFaces.Add(inFace.Count > 0 ? edit(face, inFace) : face);
        }
        return newFaces;
    }

    /// <summary>The wire a 1D chamfer or fillet rebuilds: a line object as a plain wire, or the first wire of a curve.</summary>
    private static Wire WireTargetOf(Shape target) => target.IsWire() ? (Wire)target : target.Wires()[0];

    /// <summary>The vertices without those at the ends of an open wire, which can't be chamfered or filleted.</summary>
    private static List<Vertex> InnerVertices(Wire wire, List<Shape> vertices)
    {
        if (wire.IsClosed)
            return [.. vertices.Select(v => (Vertex)v)];
        var start = wire.PositionAt(0);
        var end = wire.PositionAt(1);
        return [.. vertices.Select(v => (Vertex)v).Where(v =>
            !(TopologyUtils.IsCloseB((v.Point - start).Magnitude, 0.0) || TopologyUtils.IsCloseB((v.Point - end).Magnitude, 0.0)))];
    }

    // ---- mirror ----

    /// <summary>The objects mirrored about a plane (by default Plane.XZ).</summary>
    /// <returns>A Part, Sketch or Curve when the objects are all 3D, 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given.</exception>
    public static ModelShape Mirror(IEnumerable<Shape> objects, Plane? about = null)
    {
        // Phase 5: in builder mode the objects default to the builder's object and the result is combined with mode;
        // BuildSheet mirrors faces into its sheet
        if (objects is null)
            throw new ArgumentNullException(nameof(objects));
        var objectList = objects.ToList();
        var plane = about ?? Plane.XZ;
        var mirrored = objectList.Select(o => o.Mirror(plane)).ToList();
        return OfInputDimension(objectList, mirrored);
    }

    /// <inheritdoc cref="Mirror(IEnumerable{Shape}, Plane?)"/>
    public static ModelShape Mirror(Shape obj, Plane? about = null) => Mirror([obj], about);

    /// <summary>
    /// A compound of the results, as a Part, Sketch or Curve when the inputs are all of that dimension (build123d's
    /// <c>all(obj._dim == n for obj in object_list)</c>, true of no inputs).
    /// </summary>
    private static ModelShape OfInputDimension(IReadOnlyList<Shape> inputs, IEnumerable<Shape> results)
    {
        var compound = Compound.Make(results);
        if (inputs.All(o => o.Dim == 3))
            return new Part(compound);
        if (inputs.All(o => o.Dim == 2))
            return new Sketch(compound);
        if (inputs.All(o => o.Dim == 1))
            return new Curve(compound);
        return new Assembly(compound);
    }

    /// <summary>Nothing, as the object of a dimension.</summary>
    private static ModelShape NothingOf(int? dimension) => dimension switch
    {
        1 => new Curve(),
        2 => new Sketch(),
        3 => new Part(),
        _ => new Assembly(TopoUtil.EmptyCompound()),
    };

    // ---- offset ----

    /// <summary>
    /// The objects offset by <paramref name="amount"/>, outwards when positive: edges and wires (joined into one wire) are
    /// offset in their plane, faces have their outer wire offset out and their holes in, and solids are offset, with the
    /// faces among <see cref="OffsetOptions.Openings"/> left open (a hollow box with no lid). <see cref="OffsetOptions.Kind"/> shapes the
    /// corners.
    /// </summary>
    /// <param name="objects">Edges, wires, faces, solids or compounds of them.</param>
    /// <param name="amount">The offset distance: positive outwards, negative inwards.</param>
    /// <param name="options">Faces left open, the corners, the side and closing of an edge's offset; null for the defaults.</param>
    /// <returns>A Part, Sketch or Curve when the objects are all 3D, 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="objects"/> is null.</exception>
    /// <exception cref="ArgumentException">One of the objects can't be offset.</exception>
    public static Result<ModelShape> Offset(IEnumerable<Shape> objects, double amount, OffsetOptions? options = null) =>
        Failures.Try(() => OffsetOrThrow(objects, amount, options));

    internal static ModelShape OffsetOrThrow(IEnumerable<Shape> objects, double amount, OffsetOptions? options)
    {
        options ??= OffsetOptions.Default;
        var (openings, kind, side, closed, minEdgeLength) = ((IEnumerable<Face>)options.Openings, options.Kind, options.Side, options.Closed, options.MinEdgeLength);
        // Phase 5: in builder mode the objects default to the builder's object, and the result replaces it by default
        if (objects is null)
            throw new ArgumentNullException(nameof(objects));
        var objectList = objects.ToList();

        var edges = new List<Edge>();
        var faces = new List<Face>();
        var solids = new List<Solid>();
        foreach (var obj in objectList)
        {
            switch (TopoUtil.Typed(obj))
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
                    throw new ArgumentException($"Unsupported type {TopoUtil.KindName(obj)} for {obj.Describe()}");
            }
        }

        var newFaces = new List<Shape>();
        foreach (var face in faces)
        {
            var outerWire = AsWire(face.OuterWire().Offset2D(amount, kind).AsShape());
            if (minEdgeLength is { } outerMin)
                outerWire = outerWire.FixDegenerateEdges(outerMin);
            var innerWires = new List<Wire>();
            foreach (var innerWire in face.InnerWires())
            {
                try
                {
                    var offsetWire = AsWire(innerWire.Offset2D(-amount, kind).AsShape());
                    innerWires.Add(minEdgeLength is { } innerMin ? offsetWire.FixDegenerateEdges(innerMin) : offsetWire);
                }
                catch (Exception e) when (Failures.IsKernelFailure(e) || e is InvalidOperationException)
                {
                    // The hole closed up
                }
            }

            // Inner wires may go beyond the outer wire, so they're cut away as faces
            var newFace = Face.Make(outerWire);
            if ((newFace.NormalAt() - face.NormalAt()).Magnitude > 0.001)
                newFace = (Face)(-newFace);
            if (innerWires.Count > 0)
            {
                var subtraction = newFace.Cut(innerWires.Select(w => Face.Make(w)));
                // build123d would fail here on a cut into several pieces (it asks the face, not the result, to unwrap
                // itself); the pieces are kept instead
                if (subtraction.IsCompound())
                    newFaces.AddRange(subtraction.Faces().Select(f => (Shape)f));
                else
                    newFaces.Add(subtraction);
            }
            else
                newFaces.Add(newFace);
        }

        var newWires = new List<Shape>();
        if (edges.Count > 0)
        {
            // The line itself is offset, not a copy made from its ends: its location says which plane it was drawn on
            var offsetLine = edges.Count == 1 && edges[0].GeomType == GeomType.Line
                ? edges[0].Offset2D(amount, kind, side, closed)
                : Wire.Make(edges).Offset2D(amount, kind, side, closed);
            newWires.Add(minEdgeLength is { } min ? AsWire(offsetLine.AsShape()).FixDegenerateEdges(min) : offsetLine.AsShape());
        }

        var openingList = openings?.ToList() ?? [];
        var newSolids = new List<Shape>();
        foreach (var solid in solids)
        {
            var solidFaces = solid.Faces();
            var openingsInThisSolid = openingList.Where(f => solidFaces.Contains(f, ShapeComparer<Face>.Same)).ToList();
            newSolids.Add(solid.Offset3D(openingsInThisSolid, amount, kind: kind).Fix());
        }

        return OfInputDimension(objectList, [.. newWires, .. newFaces, .. newSolids]);
    }

    /// <inheritdoc cref="Offset(IEnumerable{Shape}, double, OffsetOptions?)"/>
    public static Result<ModelShape> Offset(Shape obj, double amount, OffsetOptions? options = null) => Offset([obj], amount, options);

    /// <summary>The part offset: its solids, with <see cref="OffsetOptions.Openings"/> left open.</summary>
    public static Result<Part> Offset(Part obj, double amount, OffsetOptions? options = null) => Offset([(Shape)obj], amount, options).Map(c => c.As<Part>());

    /// <summary>The sketch offset: its faces' outer wires out and their holes in.</summary>
    public static Result<Sketch> Offset(Sketch obj, double amount, OffsetOptions? options = null) => Offset([(Shape)obj], amount, options).Map(c => c.As<Sketch>());

    /// <summary>The curve offset in its plane.</summary>
    public static Result<Curve> Offset(Curve obj, double amount, OffsetOptions? options = null) => Offset([(Shape)obj], amount, options).Map(c => c.As<Curve>());

    /// <summary>A 2D offset as a wire: a single-edge offset comes back as its edge.</summary>
    private static Wire AsWire(Shape offset) => offset.IsWire() ? (Wire)offset : Wire.Make((Edge)offset);

    // ---- project ----

    /// <summary>
    /// Edges, wires and faces projected onto a workplane, along its normal, in the workplane's local coordinates. As in
    /// build123d's algebra mode, the faces of the projection are kept, so edges and wires project to an empty Curve.
    /// Points are projected with <see cref="ProjectPoints(IEnumerable{Point3D}, Plane)"/>.
    /// </summary>
    /// <returns>A Sketch or Curve when the objects are all 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given, or one of them is a vertex.</exception>
    public static ModelShape Project(IEnumerable<Shape> objects, Plane workplane)
    {
        // Phase 5: in builder mode the workplane defaults to the builder's, BuildLine and BuildSketch keep the projection
        // in their local coordinates, and BuildPart projects its pending objects onto its part (a target)
        if (objects is null)
            throw new ArgumentException("No object to project");
        var objectList = objects.ToList();
        if (objectList.Any(o => !o.IsNull && o.IsVertex()))
            throw new ArgumentException("Vertices are projected with ProjectPoints");
        if (objectList.Count > 0 && objectList.All(o => o.IsEmpty))
            // Nothing projects to nothing, of the same dimension
            return NothingOf(objectList[0].Dim);

        // The size of the object determines the size of the target projection screen, as the screen is normal to the
        // direction of parallel projection
        var objectSize = Compound.Make(objectList).Bounds(optimal: false).Diagonal;
        var target = Face.MakeRect(3 * objectSize, 3 * objectSize, workplane);
        if (target.IsEmpty)
            throw new ArgumentException("A target object could not be determined");

        var projectedShapes = new List<Shape>();
        foreach (var obj in objectList.Where(o => o.IsFace()).Concat(objectList.Where(o => o.IsEdge() || o.IsWire())))
        {
            var objToScreen = (target.Center() - obj.Center()).Normalized();
            var direction = workplane.FromLocalCoords(objToScreen.ToPoint()).Z < 0 ? -workplane.ZDir : workplane.ZDir;
            ShapeList<Face> projection = TopoUtil.Typed(obj) switch
            {
                Face face => face.ProjectToShape(target, direction).Faces(),
                Edge edge => edge.ProjectToShape(target, direction).Faces(),
                _ => ((Wire)obj).ProjectToShape(target, direction).Faces(),
            };
            projectedShapes.AddRange(projection.Select(f => (Shape)f));
        }

        var result = Compound.Make(projectedShapes);
        if (objectList.All(o => o.Dim == 2))
            return new Sketch(result);
        if (objectList.All(o => o.Dim == 1))
            return new Curve(result);
        return new Assembly(result);
    }

    /// <inheritdoc cref="Project(IEnumerable{Shape}, Plane)"/>
    public static ModelShape Project(Shape obj, Plane workplane) => Project([obj], workplane);

    /// <summary>
    /// Points projected onto a workplane along its normal, in the workplane's local coordinates (build123d's
    /// <c>project</c> of points, which returns a list of vectors).
    /// </summary>
    /// <exception cref="InvalidOperationException">The workplane is parallel to a projection axis.</exception>
    public static IReadOnlyList<Point3D> ProjectPoints(IEnumerable<Point3D> points, Plane workplane)
    {
        var projectedPoints = new List<Point3D>();
        foreach (var pnt in points)
        {
            var pntToTarget = (workplane.Origin - pnt).Normalized();
            var projectionAxis = workplane.FromLocalCoords(pntToTarget.ToPoint()).Z < 0
                ? -new Axis(pnt, workplane.ZDir)
                : new Axis(pnt, workplane.ZDir);
            var intersection = workplane.Intersect(projectionAxis);
            if (intersection is Axis)
                throw new InvalidOperationException("The working plane and the projection axis are parallel.");
            if (intersection is Point3D v)
                projectedPoints.Add(workplane.ToLocalCoords(v));
        }
        return projectedPoints;
    }

    /// <inheritdoc cref="ProjectPoints(IEnumerable{Point3D}, Plane)"/>
    public static IReadOnlyList<Point3D> ProjectPoints(IEnumerable<Vertex> points, Plane workplane) =>
        ProjectPoints(points.Select(p => p.Point), workplane);

    /// <inheritdoc cref="ProjectPoints(IEnumerable{Point3D}, Plane)"/>
    public static IReadOnlyList<Point3D> ProjectPoints(Point3D point, Plane workplane) => ProjectPoints([point], workplane);

    /// <inheritdoc cref="ProjectPoints(IEnumerable{Point3D}, Plane)"/>
    public static IReadOnlyList<Point3D> ProjectPoints(Vertex point, Plane workplane) => ProjectPoints([point.Point], workplane);

    // ---- scale ----

    /// <summary>
    /// The objects scaled by a factor, about <paramref name="about"/> or, by default, each object's location. A compound
    /// of one object is unwrapped one level (build123d's <c>unwrap(fully=False)</c>).
    /// </summary>
    /// <returns>A Part, Sketch or Curve when the objects are all 3D, 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given.</exception>
    public static ModelShape Scale(IEnumerable<Shape> objects, double by = 1, Point3D? about = null) =>
        ScaleCore(objects, o => o.Scale(by, about));

    /// <summary>
    /// The objects scaled by a factor along each axis, about <paramref name="about"/> or, by default, each object's
    /// location. Scaling unevenly may change the type of the geometry (a line or circle to a B-spline).
    /// </summary>
    /// <inheritdoc cref="Scale(IEnumerable{Shape}, double, Point3D?)"/>
    public static ModelShape Scale(IEnumerable<Shape> objects, Vector3D by, Point3D? about = null) =>
        ScaleCore(objects, o => o.Scale(by, about));

    /// <inheritdoc cref="Scale(IEnumerable{Shape}, double, Point3D?)"/>
    public static ModelShape Scale(Shape obj, double by = 1, Point3D? about = null) => Scale([obj], by, about);

    /// <inheritdoc cref="Scale(IEnumerable{Shape}, Vector3D, Point3D?)"/>
    public static ModelShape Scale(Shape obj, Vector3D by, Point3D? about = null) => Scale([obj], by, about);

    private static ModelShape ScaleCore(IEnumerable<Shape> objects, Func<Shape, Shape> scale)
    {
        // Phase 5: in builder mode the objects default to the builder's object, and the result replaces it by default
        if (objects is null)
            throw new ArgumentNullException(nameof(objects));
        var objectList = objects.ToList();
        return OfInputDimension(objectList, objectList.Select(scale)).Unwrapped();
    }

    // ---- split ----

    /// <summary>
    /// The objects split by a plane (by default Plane.XZ), keeping the pieces on top of it, beneath it, or both.
    /// </summary>
    /// <returns>A Part, Sketch or Curve when the objects are all 3D, 2D or 1D; otherwise a Compound.</returns>
    /// <exception cref="ArgumentException">No objects are given.</exception>
    public static ModelShape Split(IEnumerable<Shape> objects, Plane? bisectBy = null, Keep keep = Keep.Top)
    {
        var plane = bisectBy ?? Plane.XZ;
        return SplitCore(objects, keep, o => o.Split(plane, keep), o => o.SplitBoth(plane));
    }

    /// <summary>The objects split by a face or shell, keeping the pieces on top of it, beneath it, or both.</summary>
    /// <inheritdoc cref="Split(IEnumerable{Shape}, Plane?, Keep)"/>
    public static ModelShape Split(IEnumerable<Shape> objects, IShape2D bisectBy, Keep keep = Keep.Top) =>
        SplitCore(objects, keep, o => o.Split(bisectBy.AsShape(), keep), o => o.SplitBoth(bisectBy.AsShape()));

    /// <inheritdoc cref="Split(IEnumerable{Shape}, Plane?, Keep)"/>
    public static ModelShape Split(Shape obj, Plane? bisectBy = null, Keep keep = Keep.Top) => Split([obj], bisectBy, keep);

    /// <inheritdoc cref="Split(IEnumerable{Shape}, IShape2D, Keep)"/>
    public static ModelShape Split(Shape obj, IShape2D bisectBy, Keep keep = Keep.Top) => Split([obj], bisectBy, keep);

    private static ModelShape SplitCore(IEnumerable<Shape> objects, Keep keep, Func<Shape, ShapeList<Shape>> split,
                                      Func<Shape, (ShapeList<Shape> Top, ShapeList<Shape> Bottom)> splitBoth)
    {
        // Phase 5: in builder mode the objects default to the builder's object and the result replaces it by default;
        // BuildSheet splits faces of its sheet
        if (objects is null)
            throw new ArgumentNullException(nameof(objects));
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
    /// <see cref="SweepOptions.Multisection"/> the faces' outer wires are the sections of one solid along the path.
    /// </summary>
    /// <param name="sections">The cross sections.</param>
    /// <param name="path">The path to follow: an edge, wire or curve.</param>
    /// <param name="options">One solid through all sections, the frame, corners, a normal or binormal, and the clean.</param>
    /// <returns>A Part for face sections, otherwise a Sketch.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="path"/> is null.</exception>
    /// <exception cref="ArgumentException">The path is empty, or there are no sections.</exception>
    public static Result<ModelShape> Sweep(IEnumerable<Shape> sections, Shape path, SweepOptions? options = null) =>
        Failures.Try(() => SweepOrThrow(sections, path, options));

    internal static ModelShape SweepOrThrow(IEnumerable<Shape> sections, Shape path, SweepOptions? options)
    {
        // Phase 5: in builder mode the path defaults to the pending edges and the sections to the pending faces, and the
        // result is combined with mode
        if (path.IsEmpty)
            throw new ArgumentException("Can't sweep along an empty path.", nameof(path));
        var pathWire = path.IsWire() ? (Wire)path : Wire.Make(path.Edges());
        return SweepCore(sections, pathWire, options ?? SweepOptions.Default);
    }

    /// <summary>Sections swept along a path of edges, which must join into a wire.</summary>
    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, Shape, SweepOptions?)"/>
    /// <exception cref="ArgumentException">The edges don't join into a wire, or there are no sections.</exception>
    public static Result<ModelShape> Sweep(IEnumerable<Shape> sections, IEnumerable<Edge> path, SweepOptions? options = null) =>
        Failures.Try(() => SweepOrThrow(sections, path, options));

    internal static ModelShape SweepOrThrow(IEnumerable<Shape> sections, IEnumerable<Edge> path, SweepOptions? options)
    {
        Wire pathWire;
        try
        {
            pathWire = Wire.Make(path);
        }
        catch (ArgumentException e) when (!Diagnostic.IsKernelFailure(e))
        {
            throw new ArgumentException("The path's edges don't join into a wire.", nameof(path), e);
        }
        return SweepCore(sections, pathWire, options ?? SweepOptions.Default);
    }

    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, Shape, SweepOptions?)"/>
    public static Result<ModelShape> Sweep(Shape section, Shape path, SweepOptions? options = null) => Sweep([section], path, options);

    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, IEnumerable{Edge}, SweepOptions?)"/>
    public static Result<ModelShape> Sweep(Shape section, IEnumerable<Edge> path, SweepOptions? options = null) => Sweep([section], path, options);

    /// <summary>A face or sketch swept along a path into a Part.</summary>
    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, Shape, SweepOptions?)"/>
    public static Result<Part> Sweep(Face section, Shape path, SweepOptions? options = null) =>
        Sweep([(Shape)section], path, options).Map(c => c.As<Part>());

    /// <summary>A shell swept along a path gives a <see cref="Part"/>.</summary>
    public static Result<Part> Sweep(Shell section, Shape path, SweepOptions? options = null) =>
        Sweep([(Shape)section], path, options).Map(c => c.As<Part>());

    /// <summary>As the other forms of <c>Sweep</c>.</summary>
    public static Result<Part> Sweep(Sketch section, Shape path, SweepOptions? options = null) =>
        Sweep([(Shape)section], path, options).Map(c => c.As<Part>());

    /// <summary>An edge, wire or curve swept along a path into a Sketch of faces.</summary>
    /// <inheritdoc cref="Sweep(IEnumerable{Shape}, Shape, SweepOptions?)"/>
    public static Result<Sketch> Sweep(Edge section, Shape path, SweepOptions? options = null) =>
        Sweep([(Shape)section], path, options).Map(c => c.As<Sketch>());

    /// <summary>A wire swept along a path gives a <see cref="Sketch"/>.</summary>
    public static Result<Sketch> Sweep(Wire section, Shape path, SweepOptions? options = null) =>
        Sweep([(Shape)section], path, options).Map(c => c.As<Sketch>());

    /// <summary>As the other forms of <c>Sweep</c>.</summary>
    public static Result<Sketch> Sweep(BaseEdgeObject section, Shape path, SweepOptions? options = null) => Sweep((Curve)section, path, options);

    /// <summary>A line object swept along a path gives a <see cref="Sketch"/>.</summary>
    public static Result<Sketch> Sweep(BaseLineObject section, Shape path, SweepOptions? options = null) => Sweep((Curve)section, path, options);

    /// <summary>A curve swept along a path gives a <see cref="Sketch"/>.</summary>
    public static Result<Sketch> Sweep(Curve section, Shape path, SweepOptions? options = null) =>
        Sweep([(Shape)section], path, options).Map(c => c.As<Sketch>());

    private static ModelShape SweepCore(IEnumerable<Shape> sections, Wire pathWire, SweepOptions options)
    {
        var (multisection, isFrenet, transition, normal, binormal, clean) =
            (options.Multisection, options.IsFrenet, options.Transition, options.Normal, options.Binormal, options.Clean);
        var sectionList = sections?.ToList() ?? [];
        if (sectionList.Count > 0 && sectionList.All(s => s.IsEmpty))
            // Nothing swept along anything is nothing, one dimension up
            return NothingOf((sectionList[0].Dim ?? 0) + 1);
        if (sectionList.Count == 0)
            throw new ArgumentException("No sections provided");

        var edgeList = new List<Edge>();
        var faceList = new List<Face>();
        foreach (var sec in sectionList)
        {
            if (sec.Dim == 1)
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
                not null when binormal.AsShape().IsEdge() => Wire.Make((Edge)binormal.AsShape()),
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
