using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Occt;

namespace Build123d;

/// <summary>
/// A collection of shapes grouped into one: edges, faces, solids or a mix. With <see cref="AssemblyChildren"/> it is an
/// assembly, an immutable tree whose OCCT compound is built from its children, each placed relative to the compound.
/// </summary>
/// <remarks>
/// Unlike build123d's, a child holds no reference to its parent: an assembly is navigated from its root, and a node's
/// global location is that of the path to it from the root it is reached through.
/// </remarks>
public partial class Compound : Shape3D
{
    /// <summary>The empty Compound.</summary>
    public Compound() : base(null) { }

    /// <summary>A compound wrapping an OCCT compound (or comp-solid).</summary>
    public Compound(global::Kerf.Topology.Shape compound, string label = "", Color? color = null) : base(compound, label, color) { }

    /// <summary>A compound of shapes; empty shapes add nothing, so a compound of only those is empty.</summary>
    public Compound(IEnumerable<Shape> shapes, string label = "", Color? color = null)
        : base(TopoUtil.MakeCompound(shapes.Where(s => !s.IsEmpty).Select(s => s.TopoShape)), label, color) { }

    /// <summary>An assembly of children, placed relative to the compound.</summary>
    public static Compound Assembly(IEnumerable<Shape> children, string label = "", Color? color = null)
    {
        var list = children.ToList();
        return new Compound(list, label, color) { AssemblyChildren = list };
    }

    /// <summary>The assembly children: empty for a plain compound.</summary>
    public IReadOnlyList<Shape> AssemblyChildren { get; private set; } = [];

    /// <summary>A copy of this compound as an assembly of other children, rebuilt from them at this compound's location.</summary>
    public Compound WithAssemblyChildren(IEnumerable<Shape> children)
    {
        var list = children.ToList();
        var location = TopoShape.Location();
        var rebuilt = TopoUtil.MakeCompound(list.Select(c => c.TopoShape)).Located(location);
        var copy = (Compound)Rewrapped(rebuilt);
        copy.AssemblyChildren = list;
        return copy;
    }

    internal override double Order => 4.0;

    internal override Shape MakeZero() => new Compound();

    /// <summary>The dimension of the shapes in the compound, or null if they differ.</summary>
    internal override int? Dim => TopoUtil.Dim(TopoShape);

    /// <summary>The volume of the solids and shells directly in the compound.</summary>
    public override double Volume =>
        ElementsOfType<Solid>().Sum(s => s.Volume) + ElementsOfType<Shell>().Sum(s => s.Volume);

    /// <summary>A shell extruded into a compound of solids.</summary>
    public static Compound Extrude(Shell obj, Vector direction) => new(ShapeCasts.Compound(TopoUtil.Extrude(obj.TopoShape, direction)));

    // ---- Children ----

    /// <summary>The shapes directly inside the compound (its OCCT children).</summary>
    public IEnumerable<Shape> Children()
    {
        if (IsEmpty)
            yield break;
        var it = new Iterator(TopoShape);
        for (; it.More(); it.Next())
            yield return CastLike(it.Value());
    }

    /// <summary>Iterates the shapes directly inside the compound.</summary>
    public IEnumerator<Shape> GetEnumerator() => Children().GetEnumerator();

    /// <summary>The number of shapes directly inside the compound.</summary>
    public int Count => IsEmpty ? 0 : Children().Count();

    /// <summary>This compound and the compounds directly inside it.</summary>
    public override ShapeList<Compound> Compounds()
    {
        if (IsEmpty || !TopoShape.IsCompound())
            return [];
        return [.. Children().OfType<Compound>().Where(c => c.TopoShape.IsCompound()), this];
    }

    /// <summary>
    /// The shapes of one kind directly inside this compound and the compounds directly inside it (build123d's
    /// <c>get_type</c>): not, as <see cref="Shape.Faces"/> does, the faces of its solids.
    /// </summary>
    public List<T> ElementsOfType<T>() where T : Shape
    {
        var type = typeof(T) switch
        {
            var t when t == typeof(Vertex) => global::Kerf.Topology.ShapeType.Vertex,
            var t when t == typeof(Edge) => global::Kerf.Topology.ShapeType.Edge,
            var t when t == typeof(Wire) => global::Kerf.Topology.ShapeType.Wire,
            var t when t == typeof(Face) => global::Kerf.Topology.ShapeType.Face,
            var t when t == typeof(Shell) => global::Kerf.Topology.ShapeType.Shell,
            var t when t == typeof(Solid) => global::Kerf.Topology.ShapeType.Solid,
            var t when t == typeof(Compound) => global::Kerf.Topology.ShapeType.Compound,
            _ => throw new ArgumentException($"Can't extract {typeof(T).Name}s from a compound"),
        };
        var results = new List<T>();
        foreach (var comp in Compounds())
        {
            var it = new Iterator();
            it.Initialize(comp.TopoShape);
            for (; it.More(); it.Next())
            {
                var child = it.Value();
                if (child.ShapeType() == type)
                    results.Add((T)Cast(TopoUtil.Downcast(child)));
            }
        }
        return results;
    }

    /// <summary>
    /// The immediate elements of this compound in global coordinates, taking this compound as the root: an assembly's
    /// children moved by its location (each keeping its own subtree), or a plain compound's OCCT children as they are.
    /// </summary>
    internal List<Shape> GlobalElements()
    {
        if (AssemblyChildren.Count == 0)
            return [.. Children()];
        var location = Location;
        return [.. AssemblyChildren.Select(c => c.Moved(location))];
    }

    /// <summary>The parts of this assembly in pre-order: the nodes with their own geometry, each at its global location.</summary>
    internal List<(Shape Part, Shape Placed)> Parts()
    {
        var parts = new List<(Shape, Shape)>();
        void Walk(Shape node, Location global)
        {
            if (node is Compound { AssemblyChildren.Count: > 0 } assembly)
            {
                foreach (var child in assembly.AssemblyChildren)
                    VisitChild(child, global);
            }
        }
        void VisitChild(Shape child, Location parentGlobal)
        {
            var global = parentGlobal * child.Location;
            if (!(child is Compound { AssemblyChildren.Count: > 0 }))
                parts.Add((child, Cast(child.TopoShape.Located(global.Wrapped))));
            Walk(child, global);
        }
        Walk(this, IsEmpty ? new Location() : Location);
        return parts;
    }

    /// <summary>
    /// Whether any parts of this assembly overlap: each pair, at their global locations, is checked for a common volume
    /// greater than <paramref name="tolerance"/>. Returns the first such pair and the volume they share.
    /// </summary>
    public (bool Intersect, (Shape? A, Shape? B) Shapes, double Volume) DoChildrenIntersect(bool includeParent = false, double tolerance = 1e-5)
    {
        var parts = Parts();
        var children = parts.Select(p => p.Part).ToList();
        var placed = parts.Select(p => p.Placed).ToList();
        if (includeParent)
        {
            children.Insert(0, this);
            placed.Insert(0, placed.Count > 0 ? new Compound(placed) : this);
        }
        var boxes = placed.Select(p => Solid.FromBoundingBox(p.BoundingBox())).ToList();
        for (int i = 0; i < children.Count; i++)
            for (int j = i + 1; j < children.Count; j++)
            {
                // Bounding boxes first, then the shapes themselves, which may be complex
                if (boxes[i].Intersect(boxes[j]).Count == 0)
                    continue;
                var common = placed[i].Intersect(placed[j]);
                if (common.Count == 0)
                    continue;
                var volume = common.Solids().Sum(s => s.Volume);
                if (volume > tolerance)
                    return (true, (children[i], children[j]), volume);
            }
        return (false, (null, null), 0.0);
    }

    /// <summary>
    /// The compound without unnecessary compound wrappers: a compound of one shape becomes that shape (or, without
    /// <paramref name="fully"/>, a plain Part, Sketch, Curve or Compound around it).
    /// </summary>
    public Shape Unwrap(bool fully = true)
    {
        var children = Children().ToList();
        if (children.Count != 1)
            return this;
        var single = CopyAttributesTo(children[0]);
        if (single is not Compound inner)
            return fully ? single : this;
        var unwrapped = inner.Unwrap(fully);
        if (!fully)
            // A custom object's own class can't be rebuilt from a bare shape, so the wrapper is a plain one
            unwrapped = this switch
            {
                Part => new Part(unwrapped.TopoShape),
                Sketch => new Sketch(unwrapped.TopoShape),
                Curve => new Curve(unwrapped.TopoShape),
                _ => new Compound(unwrapped.TopoShape),
            };
        return CopyAttributesTo(unwrapped);
    }

    // ---- Operators ----

    /// <summary>
    /// This compound combined with others (build123d's <c>+</c>): connected 1D shapes join into a curve (or wire);
    /// otherwise the pieces are fused, within <see cref="Constants.Tolerance"/>, into a composite of this one's dimension.
    /// </summary>
    public override Shape Add(params IEnumerable<Shape?> other)
    {
        if (Dim == 1)
        {
            var curve = IsEmpty ? new Curve() : new Curve(TopoShape);
            var sum1d = curve.Add(other);
            var result1d = sum1d is Edge ? new Curve([sum1d]).MadeBy(ShapeHistory.Of([sum1d])) : sum1d;
            return CopyAttributesTo(result1d);
        }
        var brought = Operands(other);
        if (brought.Count == 0)
            return this;
        var summands = GetTopLevelShapes().Concat(brought).ToList();
        if (summands.Count <= 1)
            return MakeComposite(summands.Take(1), Dim);
        var fuse = new Fuse();
        fuse.SetFuzzyValue(Constants.Tolerance);
        summands[0] = CopyAttributesTo(summands[0]);
        var result = BoolOp(summands.Take(1), summands.Skip(1), fuse);
        // The fuse ran this compound's first piece against the rest; for the record, all of this compound was there
        // before, and what was added was brought in
        if (result.History is not null)
            result = result.MadeBy(result.History.WithInputs(GetTopLevelShapes().Select(s => s.TopoShape), brought.Select(s => s.TopoShape)));
        return result is Compound ? result : MakeComposite([result], Dim);
    }

    /// <summary>This compound with others cut away, always as a compound.</summary>
    public override Shape Subtract(params IEnumerable<Shape?> other)
    {
        var difference = base.Subtract(other);
        if (ReferenceEquals(difference, this))
            return this;
        if (difference is not Compound)
            difference = MakeComposite([difference]);
        return CopyAttributesTo(difference);
    }

    /// <summary>What this compound has in common with others, always as a compound.</summary>
    public override Shape CommonWith(params IEnumerable<object> others)
    {
        var intersection = base.CommonWith(others);
        if (intersection is not Compound)
            intersection = MakeComposite([intersection]);
        return CopyAttributesTo(intersection);
    }

    /// <summary>The centre of mass of the compound's contents (by default) or of its bounding box.</summary>
    public override Vector Center(CenterOf centerOf)
    {
        NeedsGeometry("center");
        switch (centerOf)
        {
            case CenterOf.Geometry:
                throw new ArgumentException("Center of GEOMETRY is not supported for this object");
            case CenterOf.Mass:
            {
                var props = new Properties();
                switch (TopoUtil.UnwrappedShapeType(this))
                {
                    case global::Kerf.Topology.ShapeType.Vertex:
                        throw new NotSupportedException("The centre of mass of a compound of vertices isn't defined");
                    case global::Kerf.Topology.ShapeType.Edge or global::Kerf.Topology.ShapeType.Wire:
                        MassPropertiesBuilder.LinearProperties(TopoShape, props);
                        break;
                    case global::Kerf.Topology.ShapeType.Face or global::Kerf.Topology.ShapeType.Shell:
                        MassPropertiesBuilder.SurfaceProperties(TopoShape, props);
                        break;
                    default:
                        MassPropertiesBuilder.VolumeProperties(TopoShape, props);
                        break;
                }
                var c = props.CentreOfMass();
                return new Vector(c);
            }
            default:
                return BoundingBox().Center();
        }
    }

    internal override ShapeList<Shape>? IntersectCore(object other, double tolerance, bool includeTouched)
    {
        // Distributed over the elements: Compound([a, b]) ∩ s = (a ∩ s) ∪ (b ∩ s)
        var shape = AsShape(other);
        var selfElements = GlobalElements();
        if (selfElements.Count == 0)
            return null;
        var otherElements = shape is Compound c ? c.GlobalElements() : [shape];
        var results = new List<Shape>();
        foreach (var a in selfElements)
            foreach (var b in otherElements)
                if (a.IntersectCore(b, tolerance, includeTouched) is { } found)
                    results.AddRange(found);
        var unique = results.Distinct().ToList();
        return unique.Count > 0 ? new ShapeList<Shape>(unique) : null;
    }

    /// <summary>The boundary contacts of the compound's elements (in global coordinates) with another shape.</summary>
    public override ShapeList<Shape> Touch(Shape other, double tolerance = 1e-6) =>
        new(GlobalElements().SelectMany(e => e.Touch(other, tolerance)).Distinct());

    /// <inheritdoc/>
    public override string ToString() =>
        $"{GetType().Name}" + (Label.Length > 0 ? $", label({Label})" : "") + $", #children({AssemblyChildren.Count})";
}

/// <summary>A compound of 1D shapes: edges and wires.</summary>
public partial class Curve : Compound
{
    /// <summary>The empty Curve.</summary>
    public Curve() { }

    /// <summary>A curve of the given edges and wires.</summary>
    public Curve(IEnumerable<Shape> shapes, string label = "", Color? color = null) : base(shapes, label, color) { }

    /// <summary>A curve wrapping an OCCT compound.</summary>
    public Curve(global::Kerf.Topology.Shape compound, string label = "", Color? color = null) : base(compound, label, color) { }

    internal override int? Dim => 1;

    internal override Shape MakeZero() => new Curve();

    /// <summary>The curve joined with other 1D shapes, as an edge or wire joins (build123d's <c>+</c>).</summary>
    public override Shape Add(params IEnumerable<Shape?> other) => OneD.Add1D(this, other);

    /// <summary>The wires made from the curve's edges.</summary>
    public override ShapeList<Wire> Wires(Select select = Select.All) => Wire.Combine(Edges(select));

    /// <summary>The point at a parameter along the curve, which must be continuous (build123d's <c>@</c>).</summary>
    public Vector PositionAt(double position) => new Wire(Edges()).PositionAt(position);

    /// <summary>The tangent at a parameter along the curve, which must be continuous (build123d's <c>%</c>).</summary>
    public static Vector operator %(Curve curve, double position) => new Wire(curve.Edges()).TangentAt(position);

    /// <summary>The location at a parameter along the curve, which must be continuous (build123d's <c>^</c>).</summary>
    public static Location operator ^(Curve curve, double position) => new Wire(curve.Edges()).LocationAt(position);
}

/// <summary>A compound of 2D shapes: faces.</summary>
public partial class Sketch : Compound
{
    /// <summary>The empty Sketch.</summary>
    public Sketch() { }

    /// <summary>A sketch of the given faces.</summary>
    public Sketch(IEnumerable<Shape> shapes, string label = "", Color? color = null) : base(shapes, label, color) { }

    /// <summary>A sketch wrapping an OCCT compound.</summary>
    public Sketch(global::Kerf.Topology.Shape compound, string label = "", Color? color = null) : base(compound, label, color) { }

    internal override int? Dim => 2;

    internal override Shape MakeZero() => new Sketch();
}

/// <summary>A compound of 3D shapes: solids.</summary>
public partial class Part : Compound
{
    /// <summary>The empty Part.</summary>
    public Part() { }

    /// <summary>A part of the given solids.</summary>
    public Part(IEnumerable<Shape> shapes, string label = "", Color? color = null) : base(shapes, label, color) { }

    /// <summary>A part wrapping an OCCT compound.</summary>
    public Part(global::Kerf.Topology.Shape compound, string label = "", Color? color = null) : base(compound, label, color) { }

    internal override int? Dim => 3;

    internal override Shape MakeZero() => new Part();
}
