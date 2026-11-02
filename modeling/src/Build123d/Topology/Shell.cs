using Build123d.Occt;

namespace Build123d;

/// <summary>A connected set of faces: the boundary of a solid, or an open surface such as a sheet.</summary>
public partial class Shell : Shape2D
{
    /// <summary>The empty Shell.</summary>
    public Shell() : base(null) { }

    /// <summary>A shell wrapping an OCCT shell.</summary>
    public Shell(TopoDS_Shell shell, string label = "", Color? color = null) : base(shell, label, color) { }

    /// <summary>A shell of one face.</summary>
    public Shell(Face face, string label = "", Color? color = null) : base(MakeFromFaces([face], Constants.Tolerance), label, color) { }

    /// <summary>
    /// A shell of faces sewn where their edges coincide. Edges further apart than <paramref name="tolerance"/> stay
    /// separate, so faces whose shared edges were built independently may need a tolerance above their approximation error.
    /// </summary>
    /// <exception cref="ArgumentException">The tolerance isn't positive, or the faces don't sew into one shell.</exception>
    public Shell(IEnumerable<Face> faces, string label = "", Color? color = null, double tolerance = Constants.Tolerance)
        : base(MakeFromFaces(faces.ToList(), tolerance), label, color) { }

    private static TopoDS_Shell MakeFromFaces(IReadOnlyList<Face> faces, double tolerance)
    {
        if (tolerance <= 0)
            throw new ArgumentException("tolerance must be positive");
        if (faces.Count == 1 && faces[0].IsEmpty)
            throw new ArgumentException("Can't create a Shell from empty Face");
        return TopoUtil.MakeShell(faces.Select(f => f.TopoFace), tolerance);
    }

    internal TopoDS_Shell TopoShell => (TopoDS_Shell)TopoShape;

    internal override double Order => 2.5;

    internal override Shape MakeZero() => new Shell();

    /// <summary>The volume enclosed by the shell if it is manifold, otherwise zero.</summary>
    public override double Volume
    {
        get
        {
            if (!IsManifold)
                return 0.0;
            using var fix = new ShapeFix_Solid();
            using var solid = fix.SolidFromShell(TopoShell);
            using var props = new GProp_GProps();
            MassProperties(solid, props);
            return props.Mass();
        }
    }

    /// <summary>The centre of mass of the shell (computed, as build123d does, from its linear properties).</summary>
    public override Vector Center()
    {
        NeedsGeometry("center");
        using var props = new GProp_GProps();
        BRepGProp.LinearProperties(TopoShape, props);
        using var c = props.CentreOfMass();
        return new Vector(c);
    }

    /// <summary>The location on the face of the shell nearest <paramref name="surfacePoint"/>.</summary>
    public Location LocationAt(Vector surfacePoint, Vector? xDir = null) =>
        Faces().SortBy(f => f.DistanceTo(surfacePoint))[0].LocationAt(surfacePoint, xDir);

    /// <summary>This shell sewn with other 2D shapes (adding to a shell always sews, so the result is a Shell).</summary>
    public static Shell operator +(Shell a, Shape? b) => (Shell)a.Add(b);

    // ---- Factories ----

    /// <summary>A wire extruded into a shell.</summary>
    public static Shell Extrude(Wire obj, Vector direction) => new(TopoDS.Shell(TopoUtil.Extrude(obj.TopoShape, direction)));

    /// <summary>
    /// A loft through wires (open or closed), optionally starting or ending with a vertex; ruled (stepped) or smooth.
    /// </summary>
    public static Shell MakeLoft(IEnumerable<Shape> objs, bool ruled = false) => new(TopoDS.Shell(TopoUtil.MakeLoft(objs, false, ruled)));

    /// <summary>A 1D profile revolved through <paramref name="angle"/> degrees about an axis.</summary>
    public static Shell Revolve(Shape profile, double angle, Axis axis)
    {
        var wire = new Wire(profile.Edges());
        using var ax1 = axis.ToGpAx1();
        using var builder = new BRepPrimAPI_MakeRevol(wire.TopoShape, ax1, angle * Constants.DegToRad, true);
        return new Shell(TopoDS.Shell(builder.Shape()));
    }

    /// <summary>A 1D profile swept along a 1D path.</summary>
    public static Shell Sweep(Shape profile, Shape path, Transition transition = Transition.Transformed)
    {
        var profileWire = new Wire(profile.Edges());
        var pathWire = new Wire(new Wire(path.Edges()).OrderEdges());
        using var builder = new BRepOffsetAPI_MakePipeShell(pathWire.TopoWire);
        builder.Add(profileWire.TopoShape, false, false);
        builder.SetTransitionMode(Face.TransitionMode(transition));
        builder.Build();
        var result = new Shell(TopoDS.Shell(builder.Shape()));
        return SkipClean.Clean ? result.Clean() : result;
    }

    // ---- Sheet metal ----

    /// <summary>The cylindrical faces: on a sheet metal shell, the bends.</summary>
    public ShapeList<Face> Bends() => Faces().FilterBy(GeomType.Cylinder);

    /// <summary>The planar faces: on a sheet metal shell, the base, walls and whatever the bends join.</summary>
    public ShapeList<Face> Flats() => Faces().FilterBy(GeomType.Plane);

    /// <summary>The straight edges between two flats that still lie in one plane: the lines the sheet can be folded on.</summary>
    public ShapeList<Edge> FoldLines() =>
        new(Edges().Where(e => e.GeomType == GeomType.Line && TwoD.CoplanarPair(e, this) is not null));

    /// <summary>The free edges of the sheet's flats: what a flange or hem consumes (a bend's free edges aren't rims).</summary>
    public ShapeList<Edge> Rims() =>
        new(Edges().Where(e => OneD.ConnectedFaces(e, this) is [var only] && new Face(only).GeomType == GeomType.Plane));

    /// <summary>
    /// Faces sewn into a sheet metal reference shell: planar and cylindrical faces sewn into one connected, manifold, valid
    /// shell. With <paramref name="mergeCoplanar"/> touching coplanar faces are joined first; by default a seam between
    /// coplanar faces is kept, as on a sheet it may be a fold line.
    /// </summary>
    /// <exception cref="ArgumentException">A face isn't planar or cylindrical, or the faces don't sew into a valid manifold shell.</exception>
    public static Shell MakeSheet(IEnumerable<Face> faces, bool mergeCoplanar = false)
    {
        var list = faces.ToList();
        if (mergeCoplanar)
            list = TwoD.MergeCoplanarFaces(list).Faces;
        return SewnSheet(list, list.Select(f => ShapeHistory.Of([f])).OfType<ShapeHistory>().ToList());
    }

    /// <summary>
    /// <see cref="MakeSheet(IEnumerable{Face}, bool)"/> joining only the touching coplanar pairs that include one of
    /// <paramref name="fresh"/>: what has just arrived merges into what it touches, while seams already in the sheet stay.
    /// </summary>
    public static Shell MakeSheet(IEnumerable<Face> faces, IEnumerable<Face> fresh)
    {
        var list = TwoD.MergeCoplanarFaces(faces.ToList(), fresh).Faces;
        return SewnSheet(list, list.Select(f => ShapeHistory.Of([f])).OfType<ShapeHistory>().ToList());
    }

    /// <summary>
    /// This sheet trimmed by cutters, keeping its flats and bends. A solid cuts every face it passes through, so a cutout
    /// may cross a bend; a face removes area only where it is coplanar with a planar sheet face.
    /// </summary>
    /// <exception cref="ArgumentException">A face cutter is coplanar with no planar sheet face.</exception>
    public Shell CutSheet(params IEnumerable<Shape> cutters)
    {
        var cutterList = cutters.ToList();
        var remaining = Faces().ToList();
        var records = new List<ShapeHistory>();
        var solids = cutterList.Where(c => c.Dim == 3).ToList();
        var planes = cutterList.OfType<Face>().ToList();
        if (solids.Count > 0)
        {
            var trimmed = new List<Face>();
            foreach (var face in remaining)
            {
                var result = face.Cut(solids);
                trimmed.AddRange(result.IsEmpty ? [] : result.Faces());
                if (ShapeHistory.Of([result]) is { } record)
                    records.Add(record);
            }
            remaining = trimmed;
        }
        if (planes.Count > 0)
        {
            var trimmed = new List<Face>();
            var used = new HashSet<Face>(ReferenceEqualityComparer.Instance);
            foreach (var face in remaining)
            {
                var matching = planes.Where(c => face.GeomType == GeomType.Plane && c.GeomType == GeomType.Plane
                                                 && face.IsCoplanar(new Plane(c))).ToList();
                used.UnionWith(matching);
                if (matching.Count == 0)
                {
                    trimmed.Add(face);
                    continue;
                }
                var result = face.Cut(matching);
                trimmed.AddRange(result.IsEmpty ? [] : result.Faces());
                if (ShapeHistory.Of([result]) is { } record)
                    records.Add(record);
            }
            if (used.Count != planes.Distinct(ReferenceEqualityComparer.Instance).Count())
                throw new ArgumentException(
                    "A Face cutter must be coplanar with a planar sheet face - use a Solid to cut across bends or curved faces");
            remaining = trimmed;
        }
        return SewnSheet(remaining, records);
    }

    /// <summary>
    /// Faces sewn into a sheet, the sheet invariant checked, and the sewing recorded after the records of whatever
    /// booleans made the faces.
    /// </summary>
    private static Shell SewnSheet(List<Face> faces, List<ShapeHistory> records)
    {
        if (faces.Count == 0)
            return new Shell();
        foreach (var face in faces)
        {
            if (face.GeomType is not (GeomType.Plane or GeomType.Cylinder))
                throw new ArgumentException("a sheet has only planar and cylindrical faces");
            if (face.GeomType == GeomType.Cylinder && face.Radius is not > 0)
                throw new ArgumentException("a sheet's cylindrical faces need a positive radius");
        }
        using var sewing = new BRepBuilderAPI_Sewing();
        foreach (var face in faces)
            sewing.Add(face.TopoShape);
        sewing.Perform();
        var sewn = TopoUtil.Downcast(sewing.SewedShape());
        var shell = sewn switch
        {
            TopoDS_Face f => new Shell(new Face(f)),   // one face is a sheet of one face
            TopoDS_Shell s => new Shell(s),
            _ => throw new ArgumentException("Sheet faces must sew into one connected shell"),
        };
        if (!shell.IsValid)
            throw new ArgumentException("Sheet faces produced an invalid shell");
        if (shell.Edges().Any(e => OneD.ConnectedFaces(e, shell).Count > 2))
            throw new ArgumentException("Sheet faces produced non-manifold topology");
        var record = records.Aggregate(new ShapeHistory(), (r, earlier) => r.Merge(earlier))
            .Merge(ShapeHistory.FromSewing(sewing, faces.Select(f => f.TopoShape)));
        return (Shell)shell.MadeBy(record);
    }
}

/// <summary>The module-level functions of build123d's topology/two_d.py.</summary>
public static class TwoD
{
    /// <summary>
    /// The wires grouped into faces: each group the outer wire then its holes, assuming outer wires come before inner ones
    /// and there are no islands within holes.
    /// </summary>
    public static IReadOnlyList<IReadOnlyList<Wire>> SortWiresByBuildOrder(IReadOnlyList<Wire> wires)
    {
        if (wires.Count < 2)
            return [wires];
        // This may make a compound of faces
        var faces = new Face(wires[0], wires.Skip(1));
        return [.. faces.Faces().Select(f => (IReadOnlyList<Wire>)[f.OuterWire(), .. f.InnerWires()])];
    }

    /// <summary>The pieces of an edge split at a vertex (<c>BOPAlgo_Splitter</c>).</summary>
    internal static List<TopoDS_Edge> SplitEdgeAtVertex(Edge edge, Vertex splitVertex)
    {
        using var splitter = new BOPAlgo_Splitter();
        splitter.SetNonDestructive(true);   // leave the inputs' tolerances alone
        splitter.AddArgument(edge.TopoShape);
        splitter.AddTool(splitVertex.TopoShape);
        splitter.Perform();
        using var result = splitter.Shape();
        var edges = new List<TopoDS_Edge>();
        using var explorer = new TopExp_Explorer(result, TopAbs_ShapeEnum.TopAbs_EDGE);
        for (; explorer.More(); explorer.Next())
            edges.Add(TopoDS.Edge(explorer.Current()));
        return edges;
    }

    /// <summary>The two coplanar flats an edge lies between, if that is what it does.</summary>
    internal static (Face, Face)? CoplanarPair(Edge edge, Shape sheet)
    {
        var beside = OneD.ConnectedFaces(edge, sheet).Select(f => new Face(f)).ToList();
        if (beside.Count != 2 || beside.Any(f => f.GeomType != GeomType.Plane))
            return null;
        var normals = beside.Select(f => f.NormalAt(f.Center())).ToList();
        if (normals[0].Cross(normals[1]).Length > Constants.Tolerance)
            return null;
        return (beside[0], beside[1]);
    }

    /// <summary>The coplanar flat across a straight edge of <paramref name="face"/>, if there is one.</summary>
    internal static Face? FoldPartnerOf(Edge edge, Face face, Shape sheet)
    {
        if (edge.GeomType != GeomType.Line || CoplanarPair(edge, sheet) is not var (a, b))
            return null;
        return !a.IsSame(face) ? a : !b.IsSame(face) ? b : null;
    }

    /// <summary>
    /// Touching coplanar faces fused together, and the records of the fuses. With <paramref name="fresh"/> only pairs with one
    /// of those faces in them are joined.
    /// </summary>
    internal static (List<Face> Faces, List<ShapeHistory> Records) MergeCoplanarFaces(List<Face> faces, IEnumerable<Face>? fresh = null)
    {
        var merged = faces.ToList();
        var freshList = fresh?.ToList();
        var movable = merged.Select(f => freshList is null || freshList.Any(o => f.IsSame(o))).ToList();
        var records = new List<ShapeHistory>();
        bool changed = true;
        while (changed)
        {
            changed = false;
            for (int i = 0; i < merged.Count && !changed; i++)
            {
                var first = merged[i];
                if (first.GeomType != GeomType.Plane)
                    continue;
                for (int j = i + 1; j < merged.Count; j++)
                {
                    var second = merged[j];
                    if (second.GeomType != GeomType.Plane || !(movable[i] || movable[j])
                        || !first.IsCoplanar(new Plane(second)) || first.DistanceTo(second) > Constants.Tolerance)
                        continue;
                    if (first.Fuse(second) is Face fused)
                    {
                        merged[i] = fused;
                        movable[i] = true;
                        merged.RemoveAt(j);
                        movable.RemoveAt(j);
                        if (fused.History is not null)
                            records.Add(fused.History);
                        changed = true;
                        break;
                    }
                }
            }
        }
        return (merged, records);
    }
}
