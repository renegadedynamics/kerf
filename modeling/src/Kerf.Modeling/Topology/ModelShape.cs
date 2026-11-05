using Kerf.Booleans;
using Kerf.Geometry;

namespace Kerf.Modeling;

/// <summary>
/// The base of the modelling layer's objects, <see cref="Part"/>, <see cref="Sketch"/>, <see cref="Curve"/> and
/// <see cref="Assembly"/>: a kernel shape with what a topological shape can't carry, its label, colour and construction
/// flag, and the record of the operation that made it.
/// </summary>
/// <remarks>
/// An object is immutable, and goes wherever a shape goes: it converts implicitly to <see cref="Kerf.Topology.Shape"/>, and
/// the extension members on shapes are on it too. Its operations keep its class, so <c>box.Moved(loc)</c> is a <c>Box</c>
/// (see <see cref="ModelShapeOperations"/>).
/// </remarks>
public abstract partial class ModelShape : IShape3D, IEquatable<ModelShape>
{
    private Shape _shape;
    // The root Location the history describes: the shape's Location when it was recorded. A relocated copy shares
    // the record, so selection maps its sub-shapes back by the difference between this and its own.
    private Placement? _historyAt;

    private protected ModelShape(Shape? shape, string label = "", Color? color = null)
    {
        _shape = shape is { } s ? TopoUtil.Downcast(s) : TopoUtil.EmptyCompound();
        Label = label;
        Color = color;
    }

    /// <summary>The kernel shape this object holds.</summary>
    public Shape Shape => _shape;

    /// <inheritdoc/>
    public Shape AsShape() => _shape;

    /// <summary>The kernel shape this object holds.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="model"/> is null.</exception>
    public static implicit operator Shape(ModelShape model)
    {
        ArgumentNullException.ThrowIfNull(model);
        return model._shape;
    }

    /// <summary>The user-assigned label.</summary>
    public string Label { get; private set; }

    /// <summary>The object's colour, if it has one.</summary>
    public Color? Color { get; private set; }

    /// <summary>Whether the object is construction geometry, used to place others rather than as part of the result.</summary>
    public bool ForConstruction { get; private set; }

    /// <summary>
    /// What the operation that made this object did to its inputs' sub-shapes, when the kernel reported it
    /// (booleans, clean, fillets and chamfers). Used by <see cref="Select.Last"/> and <see cref="Select.New"/>.
    /// </summary>
    public ModelHistory? History { get; private set; }

    /// <summary>The dimension of the object's contents: 3 for a part, 2 for a sketch, 1 for a curve, or null if mixed.</summary>
    internal virtual int? Dimension => TopoUtil.Dim(_shape);

    /// <summary>The zero of this class: nothing of a Box is a Part.</summary>
    internal abstract ModelShape MakeZero();

    // ---- Copies ----

    /// <summary>
    /// A copy of this object around an already placed kernel shape: the same class and attributes, holding
    /// <paramref name="shape"/> rather than a copy of the geometry. The history is kept, as a relocated copy shares it.
    /// </summary>
    internal ModelShape Placed(Shape shape)
    {
        var copy = (ModelShape)MemberwiseClone();
        copy._shape = shape;
        return copy;
    }

    /// <summary>A copy of this object around new geometry: the same class and attributes, without a history.</summary>
    internal ModelShape Rewrapped(Shape shape)
    {
        var copy = (ModelShape)MemberwiseClone();
        copy._shape = TopoUtil.Downcast(shape);
        copy.History = null;
        copy._historyAt = null;
        return copy;
    }

    /// <summary>A copy of this object noting the record of how it was made.</summary>
    internal ModelShape MadeBy(ModelHistory? history)
    {
        var copy = (ModelShape)MemberwiseClone();
        copy.History = history;
        copy._historyAt = copy._shape.Placement;
        return copy;
    }

    internal ModelShape WithLabelCore(string label)
    {
        var copy = (ModelShape)MemberwiseClone();
        copy.Label = label;
        return copy;
    }

    internal ModelShape WithColorCore(Color? color)
    {
        var copy = (ModelShape)MemberwiseClone();
        copy.Color = color;
        return copy;
    }

    internal ModelShape WithForConstructionCore(bool forConstruction)
    {
        var copy = (ModelShape)MemberwiseClone();
        copy.ForConstruction = forConstruction;
        return copy;
    }

    /// <summary>
    /// <paramref name="target"/> with this object's attributes (label, colour, construction flag) where its own are unset.
    /// The history isn't copied: it says how this object was made, not the target.
    /// </summary>
    internal T CopyAttributesTo<T>(T target) where T : ModelShape
    {
        var copy = (T)target.MemberwiseClone();
        if (copy.Label.Length == 0)
            copy.Label = Label;
        copy.Color ??= Color;
        copy.ForConstruction |= ForConstruction;
        return copy;
    }

    /// <summary>
    /// The record, brought to where the object is now. A relocated object shares the record of the object it was placed
    /// from, which names the sub-shapes where that one was; an operation that extends the record (<c>Clean</c>) needs a
    /// copy chained forward to here.
    /// </summary>
    internal ModelHistory? HistoryHere()
    {
        if (History is null)
            return null;
        var here = _shape.Placement;
        if (_historyAt is not { } at || TopoUtil.IsSamePlacement(at, here))
            return History;
        var recorded = _shape.Located(at);
        return new ModelHistory(History.Before, History.Brought)
            .Merge(History)
            .Merge(ModelHistory.FromRelocation(recorded, _shape));
    }

    /// <summary>This object as the algebra's class <typeparamref name="T"/> (a Part, Sketch or Curve): itself if it is one, else that class around its compound, with its attributes and history.</summary>
    internal T As<T>() where T : ModelShape
    {
        if (this is T typed)
            return typed;
        ModelShape wrapped;
        Shape held = this.IsEmpty ? TopoUtil.EmptyCompound() : _shape.IsCompound() ? _shape : TopoUtil.MakeCompound([_shape]);
        wrapped = typeof(T) == typeof(Part) ? new Part(held) : typeof(T) == typeof(Sketch) ? new Sketch(held) : new Curve(held);
        return (T)CopyAttributesTo(wrapped).MadeBy(History);
    }

    /// <summary>This object without an unnecessary compound around one compound: the same class if it can be rebuilt, else a plain one.</summary>
    internal ModelShape Unwrapped()
    {
        if (this.IsEmpty || !_shape.IsCompound() || _shape.ChildCount != 1)
            return this;
        var inner = _shape.ChildShapes()[0];
        if (!inner.IsCompound())
            return this;
        return Placed(inner).Unwrapped();
    }

    // ---- Selection ----

    /// <summary>All the vertices in this object, or those <paramref name="select"/> picks.</summary>
    public ShapeList<Vertex> Vertices(Select select = Select.All) => SelectFrom(this.SelectVertices(), select);

    /// <summary>All the non-degenerate edges in this object, or those <paramref name="select"/> picks.</summary>
    public ShapeList<Edge> Edges(Select select = Select.All) => SelectFrom(this.SelectEdges(), select);

    /// <summary>All the wires in this object, or those <paramref name="select"/> picks.</summary>
    public virtual ShapeList<Wire> Wires(Select select = Select.All) => SelectFrom(this.SelectWires(), select);

    /// <summary>All the faces in this object, or those <paramref name="select"/> picks.</summary>
    public ShapeList<Face> Faces(Select select = Select.All) => SelectFrom(this.SelectFaces(), select);

    /// <summary>All the shells in this object, or those <paramref name="select"/> picks.</summary>
    public ShapeList<Shell> Shells(Select select = Select.All) => SelectFrom(this.SelectShells(), select);

    /// <summary>All the solids in this object, or those <paramref name="select"/> picks.</summary>
    public ShapeList<Solid> Solids(Select select = Select.All) => SelectFrom(this.SelectSolids(), select);

    /// <summary>Narrows a list of this object's sub-shapes by what the last operation did.</summary>
    private protected ShapeList<T> SelectFrom<T>(ShapeList<T> shapes, Select select) where T : IShape
    {
        if (select == Select.All)
            return shapes;
        var record = History ?? throw new InvalidOperationException(
            $"Select.{select} needs a record of how this shape was made, and this one has none - "
            + "it is the result of no recorded operation");
        // The record names the sub-shapes where the shape was when it was recorded. If the shape has been
        // relocated since, map each sub-shape back there to look it up.
        var here = _shape.Placement;
        Func<T, Shape> key;
        if (_historyAt is not { } at || TopoUtil.IsSamePlacement(at, here))
            key = s => s.AsShape();
        else
        {
            var back = at.Multiplied(here.Inverted());
            key = s => s.AsShape().Moved(back);
        }
        return select switch
        {
            Select.Last => shapes.Where(s => record.IsLast(key(s))),
            Select.New => shapes.Where(s => record.IsNew(key(s))),
            _ => throw new ArgumentOutOfRangeException(nameof(select)),
        };
    }

    // ---- Equality ----

    /// <summary>
    /// Whether two objects are the same: they hold the same data at the same place, either orientation. Empty objects of
    /// one class are all the same nothing.
    /// </summary>
    public bool Equals(ModelShape? other)
    {
        if (other is null)
            return false;
        bool empty = this.IsEmpty, otherEmpty = other.IsEmpty;
        if (empty || otherEmpty)
            return empty && otherEmpty && GetType() == other.GetType();
        return _shape.IsSame(other._shape);
    }

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is ModelShape s && Equals(s);

    /// <summary>A hash consistent with <see cref="Equals(ModelShape?)"/>.</summary>
    public override int GetHashCode() => this.IsEmpty ? GetType().GetHashCode() : ShapeComparer.Same.GetHashCode(_shape);

    /// <summary>Whether two objects are the same (see <see cref="Equals(ModelShape?)"/>).</summary>
    public static bool operator ==(ModelShape? a, ModelShape? b) => a is null ? b is null : a.Equals(b);

    /// <summary>Whether two objects aren't the same.</summary>
    public static bool operator !=(ModelShape? a, ModelShape? b) => !(a == b);

    /// <summary>The class name, label and location, for debugging.</summary>
    public override string ToString() =>
        $"{GetType().Name}" + (Label.Length > 0 ? $", label({Label})" : "") + (this.IsEmpty ? "()" : $" at {this.Location}");
}

/// <summary>A compound of 1D shapes: edges and wires. The curve objects (<see cref="Line"/>, <see cref="Polyline"/>…) hold one edge or wire.</summary>
public partial class Curve : ModelShape, IShape1D
{
    /// <summary>The empty Curve.</summary>
    public Curve() : base(null) { }

    /// <summary>A curve of the given edges and wires.</summary>
    public Curve(IEnumerable<Shape> shapes, string label = "", Color? color = null) : base(TopoUtil.MakeCompoundOf(shapes), label, color) { }

    /// <summary>A curve of the given edges.</summary>
    public Curve(IEnumerable<Edge> edges, string label = "", Color? color = null) : base(TopoUtil.MakeCompoundOf(edges.Select(e => (Shape)e)), label, color) { }

    /// <summary>A curve of the given wires.</summary>
    public Curve(IEnumerable<Wire> wires, string label = "", Color? color = null) : base(TopoUtil.MakeCompoundOf(wires.Select(w => (Shape)w)), label, color) { }

    /// <summary>A curve holding a kernel shape: a compound, edge or wire.</summary>
    public Curve(Shape shape, string label = "", Color? color = null) : base(shape, label, color) { }

    internal override int? Dimension => 1;

    internal override ModelShape MakeZero() => new Curve();

    /// <summary>The wires made from the curve's edges.</summary>
    public override ShapeList<Wire> Wires(Select select = Select.All) =>
        Shape.Type == ShapeType.Wire && select == Select.All ? base.Wires(select) : Wire.Combine(Edges(select));

    private Wire AsWire() => Shape.TryAs(out Wire wire) && !Shape.IsNull ? wire : Wire.Make(Edges());

    /// <summary>The point at a parameter along the curve, which must be continuous.</summary>
    public Vector PositionAt(double position) => Shape.TryAs(out Edge edge) && !Shape.IsNull ? edge.PositionAt(position) : AsWire().PositionAt(position);

    /// <summary>The tangent at a parameter along the curve, which must be continuous.</summary>
    public Vector TangentAt(double position) => Shape.TryAs(out Edge edge) && !Shape.IsNull ? edge.TangentAt(position) : AsWire().TangentAt(position);

    /// <summary>The location at a parameter along the curve, which must be continuous.</summary>
    public Location LocationAt(double position) => Shape.TryAs(out Edge edge) && !Shape.IsNull ? edge.LocationAt(position) : AsWire().LocationAt(position);
}

/// <summary>A compound of 2D shapes: faces.</summary>
public partial class Sketch : ModelShape
{
    /// <summary>The empty Sketch.</summary>
    public Sketch() : base(null) { }

    /// <summary>A sketch of the given shapes.</summary>
    public Sketch(IEnumerable<Shape> shapes, string label = "", Color? color = null) : base(TopoUtil.MakeCompoundOf(shapes), label, color) { }

    /// <summary>A sketch of the given faces.</summary>
    public Sketch(IEnumerable<Face> faces, string label = "", Color? color = null) : base(TopoUtil.MakeCompoundOf(faces.Select(f => (Shape)f)), label, color) { }

    /// <summary>A sketch holding a kernel shape.</summary>
    public Sketch(Shape shape, string label = "", Color? color = null) : base(shape, label, color) { }

    internal override int? Dimension => 2;

    internal override ModelShape MakeZero() => new Sketch();
}

/// <summary>A compound of 3D shapes: solids.</summary>
public partial class Part : ModelShape
{
    /// <summary>The empty Part.</summary>
    public Part() : base(null) { }

    /// <summary>A part of the given shapes.</summary>
    public Part(IEnumerable<Shape> shapes, string label = "", Color? color = null) : base(TopoUtil.MakeCompoundOf(shapes), label, color) { }

    /// <summary>A part of the given solids.</summary>
    public Part(IEnumerable<Solid> solids, string label = "", Color? color = null) : base(TopoUtil.MakeCompoundOf(solids.Select(s => (Shape)s)), label, color) { }

    /// <summary>A part holding a kernel shape.</summary>
    public Part(Shape shape, string label = "", Color? color = null) : base(shape, label, color) { }

    internal override int? Dimension => 3;

    internal override ModelShape MakeZero() => new Part();
}

/// <summary>
/// An immutable tree of objects whose kernel compound is built from its children, each placed relative to the assembly. It
/// has a label and colour of its own. A leaf is an assembly with no children holding a shape of any kind.
/// </summary>
/// <remarks>
/// Unlike build123d's, a child holds no reference to its parent: an assembly is navigated from its root, and a node's
/// global location is that of the path to it from the root it is reached through.
/// </remarks>
public class Assembly : ModelShape
{
    /// <summary>An assembly of children, placed relative to it.</summary>
    public Assembly(IEnumerable<ModelShape> children, string label = "", Color? color = null) : this(children.ToList(), label, color) { }

    private Assembly(List<ModelShape> children, string label, Color? color)
        : base(TopoUtil.MakeCompoundOf(children.Select(c => c.Shape)), label, color) => Children = children;

    /// <summary>An assembly with no children around a kernel shape: an assembly's node of any kind of shape.</summary>
    public Assembly(Shape shape, string label = "", Color? color = null) : base(shape, label, color) => Children = [];

    /// <summary>The children: empty for a leaf.</summary>
    public IReadOnlyList<ModelShape> Children { get; private set; }

    internal override ModelShape MakeZero() => new Assembly([], "", null);

    /// <summary>A copy of this assembly with other children, rebuilt from them at this assembly's location.</summary>
    public Assembly WithChildren(IEnumerable<ModelShape> children)
    {
        var list = children.ToList();
        var rebuilt = TopoUtil.MakeCompound(list.Select(c => c.Shape)).Located(Shape.Placement);
        var copy = (Assembly)Rewrapped(rebuilt);
        copy.Children = list;
        return copy;
    }

    /// <summary>The immediate elements of this assembly in global coordinates, taking it as the root.</summary>
    internal List<Shape> GlobalElements()
    {
        if (Children.Count == 0)
            return Shape.ChildShapes();
        var location = this.Location;
        return [.. Children.Select(c => ((Shape)c).Moved(location))];
    }

    /// <summary>The parts of this assembly in pre-order: the nodes with their own geometry, each at its global location.</summary>
    internal List<(ModelShape Part, Shape Placed)> Parts()
    {
        var parts = new List<(ModelShape, Shape)>();
        void Walk(ModelShape node, Location global)
        {
            if (node is Assembly { Children.Count: > 0 } assembly)
                foreach (var child in assembly.Children)
                    VisitChild(child, global);
        }
        void VisitChild(ModelShape child, Location parentGlobal)
        {
            var global = parentGlobal * child.Location;
            if (child is not Assembly { Children.Count: > 0 })
                parts.Add((child, child.Shape.Located(global.Wrapped)));
            Walk(child, global);
        }
        Walk(this, this.IsEmpty ? new Location() : this.Location);
        return parts;
    }

    /// <summary>
    /// Whether any parts of this assembly overlap: each pair, at their global locations, is checked for a common volume
    /// greater than <paramref name="tolerance"/>. Returns the first such pair and the volume they share.
    /// </summary>
    public (bool Intersect, (ModelShape? A, ModelShape? B) Shapes, double Volume) DoChildrenIntersect(bool includeParent = false, double tolerance = 1e-5)
    {
        var parts = Parts();
        var children = parts.Select(p => p.Part).ToList();
        var placed = parts.Select(p => p.Placed).ToList();
        if (includeParent)
        {
            children.Insert(0, this);
            placed.Insert(0, placed.Count > 0 ? Compound.Make(placed) : this);
        }
        var boxes = placed.Select(p => Solid.FromBoundingBox(p.BoundingBox())).ToList();
        for (int i = 0; i < children.Count; i++)
            for (int j = i + 1; j < children.Count; j++)
            {
                // Bounding boxes first, then the shapes themselves, which may be complex
                if (boxes[i].Intersect(boxes[j].AsShape()).Count == 0)
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

    /// <inheritdoc/>
    public override string ToString() =>
        $"{GetType().Name}" + (Label.Length > 0 ? $", label({Label})" : "") + $", #children({Children.Count})";
}
