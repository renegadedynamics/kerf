using Kerf.Construction;
using Kerf.Healing;
using Kerf.Measurement;

namespace Kerf.Modeling;

/// <summary>A connected set of faces: the boundary of a solid, or an open surface such as a sheet.</summary>
public static partial class ShellExtensions
{
    extension(Shell self)
    {

        internal Shell TopoShell => (Shell)self;

        /// <summary>The volume enclosed by the shell if it is manifold, otherwise zero.</summary>
        internal double VolumeOfShell
        {
            get
            {
                if (!self.IsManifold)
                    return 0.0;
                var solid = Failures.Get(self.MakeSolid(), () => "solid from shell").Shape;
                return ShapeExtensions.MassPropertiesOf(solid)!.Mass;
            }
        }

        /// <summary>The centre of mass of the shell (computed, as build123d does, from its linear properties).</summary>
        internal Point3D CenterOfShell()
        {
            self.NeedsGeometry("center");
            return self.MassProperties(Dimension.Length).CentreOfMass;
        }

        /// <summary>The location on the face of the shell nearest <paramref name="surfacePoint"/>.</summary>
        public Location LocationAt(Point3D surfacePoint, Vector3D? xDir = null) =>
            self.Faces().OrderBy(f => f.DistanceTo(surfacePoint)).First().LocationAt(surfacePoint, xDir);

        // ---- Sheet metal ----

        /// <summary>The cylindrical faces: on a sheet metal shell, the bends.</summary>
        public ShapeList<Face> Bends() => self.Faces().Where(GeomType.Cylinder);

        /// <summary>The planar faces: on a sheet metal shell, the base, walls and whatever the bends join.</summary>
        public ShapeList<Face> Flats() => self.Faces().Where(GeomType.Plane);

        /// <summary>The straight edges between two flats that still lie in one plane: the lines the sheet can be folded on.</summary>
        public ShapeList<Edge> FoldLines() =>
            new(self.Edges().Where(e => e.GeomType == GeomType.Line && TwoD.CoplanarPair(e, self) is not null));

        /// <summary>The free edges of the sheet's flats: what a flange or hem consumes (a bend's free edges aren't rims).</summary>
        public ShapeList<Edge> Rims() =>
            new(self.Edges().Where(e => OneD.ConnectedFaces(e, self) is [var only] && only.GeomType == GeomType.Plane));

        /// <summary>
        /// This sheet trimmed by cutters, keeping its flats and bends. A solid cuts every face it passes through, so a cutout
        /// may cross a bend; a face removes area only where it is coplanar with a planar sheet face.
        /// </summary>
        /// <exception cref="ArgumentException">A face cutter is coplanar with no planar sheet face.</exception>
        public Shell CutSheet(params IEnumerable<Shape> cutters)
        {
            var cutterList = cutters.ToList();
            var remaining = self.Faces().ToList();
            var records = new List<ModelHistory>();
            var solids = cutterList.Where(c => c.Dim == 3).ToList();
            var planes = cutterList.OfKind<Face>().ToList();
            if (solids.Count > 0)
            {
                var trimmed = new List<Face>();
                foreach (var face in remaining)
                {
                    var result = face.Cut(solids);
                    trimmed.AddRange(result.IsEmpty ? [] : result.Faces());
                    if (ModelHistory.Of([result]) is { } record)
                        records.Add(record);
                }
                remaining = trimmed;
            }
            if (planes.Count > 0)
            {
                var trimmed = new List<Face>();
                var used = new HashSet<Face>(ShapeComparer<Face>.Same);
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
                    if (ModelHistory.Of([result]) is { } record)
                        records.Add(record);
                }
                if (used.Count != planes.Distinct(ShapeComparer<Face>.Same).Count())
                    throw new ArgumentException(
                        "A Face cutter must be coplanar with a planar sheet face - use a Solid to cut across bends or curved faces");
                remaining = trimmed;
            }
            return ShellExtensions.SewnSheet(remaining, records);
        }
    }

    extension(Shell)
    {

        // ---- Factories ----

        /// <summary>A wire extruded into a shell.</summary>
        public static Shell Extrude(Wire obj, Vector3D direction) => (Shell)TopoUtil.Extrude(obj, direction);

        /// <summary>
        /// A loft through wires (open or closed), optionally starting or ending with a vertex; ruled (stepped) or smooth.
        /// </summary>
        public static Shell MakeLoft(IEnumerable<Shape> objects, bool ruled = false) => (Shell)TopoUtil.MakeLoft(objects, false, ruled);

        /// <summary>A 1D profile revolved through <paramref name="angle"/> degrees about an axis.</summary>
        public static Shell Revolve(Shape profile, double angle, Axis axis)
        {
            var wire = Wire.Make(profile.Edges());
            var ax1 = axis.ToAxis3D();
            // On a copy of the profile, as build123d revolves (BRepPrimAPI_MakeRevol with copy)
            var revolved = Failures.Get(wire.Revolve(ax1, angle * Constants.DegToRad, copyProfile: true), () => "revolve");
            return (Shell)revolved.Shape;
        }

        /// <summary>A 1D profile swept along a 1D path.</summary>
        public static Shell Sweep(Shape profile, Shape path, Transition transition = Transition.Transformed)
        {
            var profileWire = Wire.Make(profile.Edges());
            var pathWire = Wire.Make(Wire.Make(path.Edges()).OrderEdges());
            // BRepOffsetAPI_MakePipeShell's defaults (corrected Frenet), not closed into a solid
            var options = new SweepOptions { MakeSolid = false, Transition = FaceExtensions.TransitionMode(transition) };
            var built = Failures.Get(Sweeps.Sweep(profileWire, pathWire, options), () => "sweep");
            var result = (Shell)built.Shape;
            return SkipClean.Clean ? result.Clean() : result;
        }

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
            return ShellExtensions.SewnSheet(list, list.Select(f => ModelHistory.Of([f])).OfType<ModelHistory>().ToList());
        }

        /// <summary>
        /// <see cref="MakeSheet(IEnumerable{Face}, bool)"/> joining only the touching coplanar pairs that include one of
        /// <paramref name="fresh"/>: what has just arrived merges into what it touches, while seams already in the sheet stay.
        /// </summary>
        public static Shell MakeSheet(IEnumerable<Face> faces, IEnumerable<Face> fresh)
        {
            var list = TwoD.MergeCoplanarFaces(faces.ToList(), fresh).Faces;
            return ShellExtensions.SewnSheet(list, list.Select(f => ModelHistory.Of([f])).OfType<ModelHistory>().ToList());
        }
    }



    extension(Shell)
    {
        /// <summary>A shell of one face.</summary>
        public static Shell Make(Face face) => ShellExtensions.MakeFromFaces([face], Constants.Tolerance);
    }


    extension(Shell)
    {
        /// <summary>
        /// A shell of faces sewn where their edges coincide. Edges further apart than <paramref name="tolerance"/> stay
        /// separate, so faces whose shared edges were built independently may need a tolerance above their approximation error.
        /// </summary>
        /// <exception cref="ArgumentException">The tolerance isn't positive, or the faces don't sew into one shell.</exception>
        public static Shell Make(IEnumerable<Face> faces, double tolerance = Constants.Tolerance) => ShellExtensions.MakeFromFaces(faces.ToList(), tolerance);
    }


    private static Shell MakeFromFaces(IReadOnlyList<Face> faces, double tolerance)
    {
        if (tolerance <= 0)
            throw new ArgumentException("tolerance must be positive");
        if (faces.Count == 1 && faces[0].IsEmpty)
            throw new ArgumentException("Can't create a Shell from empty Face");
        return TopoUtil.MakeShell(faces.Select(f => f), tolerance);
    }

    /// <summary>
    /// Faces sewn into a sheet, the sheet invariant checked, and the sewing recorded after the records of whatever
    /// booleans made the faces.
    /// </summary>
    private static Shell SewnSheet(List<Face> faces, List<ModelHistory> records)
    {
        if (faces.Count == 0)
            return default(Shell);
        foreach (var face in faces)
        {
            if (face.GeomType is not (GeomType.Plane or GeomType.Cylinder))
                throw new ArgumentException("a sheet has only planar and cylindrical faces");
            if (face.GeomType == GeomType.Cylinder && face.Radius is not > 0)
                throw new ArgumentException("a sheet's cylindrical faces need a positive radius");
        }
        // BRepBuilderAPI_Sewing's defaults; its history records the faces it rebuilt, the edges and vertices it merged, and
        // the faces it dropped
        var sewing = Failures.Get(HealingOperations.Sew(faces.Select(f => (Shape)f)), () => "sew");
        var sewn = TopoUtil.Downcast(sewing.Shape);
        var shell = sewn.IsFace() ? Shell.Make((Face)sewn)   // one face is a sheet of one face
            : sewn.IsShell() ? (Shell)sewn
            : throw new ArgumentException("Sheet faces must sew into one connected shell");
        if (!shell.IsValid)
            throw new ArgumentException("Sheet faces produced an invalid shell");
        if (shell.Edges().Any(e => OneD.ConnectedFaces(e, shell).Count > 2))
            throw new ArgumentException("Sheet faces produced non-manifold topology");
        var record = records.Aggregate(new ModelHistory(), (r, earlier) => r.Merge(earlier))
            .Merge(ModelHistory.FromHistory(sewing.History, faces.Select(f => (Shape)f)));
        return (Shell)shell.MadeBy(record);
    }
}

/// <summary>What faces and shells share internally, from build123d's topology/two_d.py.</summary>
internal static class TwoD
{
    /// <summary>
    /// The wires grouped into faces: each group the outer wire then its holes, assuming outer wires come before inner ones
    /// and there are no islands within holes.
    /// </summary>
    internal static IReadOnlyList<IReadOnlyList<Wire>> SortWiresByBuildOrder(IReadOnlyList<Wire> wires)
    {
        if (wires.Count < 2)
            return [wires];
        // This may make a compound of faces
        var faces = Face.Make(wires[0], wires.Skip(1));
        return [.. faces.Faces().Select(f => (IReadOnlyList<Wire>)[f.OuterWire(), .. f.InnerWires()])];
    }

    /// <summary>The pieces of an edge split at a vertex (<c>BOPAlgo_Splitter</c>).</summary>
    internal static List<Edge> SplitEdgeAtVertex(Edge edge, Vertex splitVertex)
    {
        // Non-destructive, as all the modelling layer's Booleans: the inputs' tolerances are left alone
        var result = Failures.Get(TopoUtil.Boolean(BooleanKind.Split, [edge], [splitVertex]), () => "split").Shape;
        return [.. result.Explore<Edge>()];
    }

    /// <summary>The two coplanar flats an edge lies between, if that is what it does.</summary>
    internal static (Face, Face)? CoplanarPair(Edge edge, Shape sheet)
    {
        var beside = OneD.ConnectedFaces(edge, sheet).Select(f => f).ToList();
        if (beside.Count != 2 || beside.Any(f => f.GeomType != GeomType.Plane))
            return null;
        var normals = beside.Select(f => f.NormalAt(f.Center())).ToList();
        if (normals[0].Cross(normals[1]).Magnitude > Constants.Tolerance)
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
    internal static (List<Face> Faces, List<ModelHistory> Records) MergeCoplanarFaces(List<Face> faces, IEnumerable<Face>? fresh = null)
    {
        var merged = faces.ToList();
        var freshList = fresh?.ToList();
        var movable = merged.Select(f => freshList is null || freshList.Any(o => f.IsSame(o))).ToList();
        var records = new List<ModelHistory>();
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
                    if (first.Fuse(second).TryAs(out Face fused))
                    {
                        merged[i] = fused;
                        movable[i] = true;
                        merged.RemoveAt(j);
                        movable.RemoveAt(j);
                        if (fused.History is { } record)
                            records.Add(record);
                        changed = true;
                        break;
                    }
                }
            }
        }
        return (merged, records);
    }
}
